"""End-to-end independent wire packets through the actual reader and session."""
import math
import threading
import unittest

from imu_desktop.core.angles import JOINTS
from imu_desktop.core.math3d import IDENTITY, axis_angle, multiply
from imu_desktop.core.serial import SerialReader
from imu_desktop.core.session import SessionController
from test_protocol_alignment import timed_payload, wire


def quality_payload(*args, **kwargs):
    """Current QUALITY wire data, retaining independent clock/quaternion fixtures."""
    payload = timed_payload(*args, **kwargs)
    payload[1] = 11
    payload[36:40] = (1).to_bytes(4, 'little')
    return payload


class ReplayPort:
    def __init__(self, data, stop, fragment=11):
        self.data, self.stop, self.fragment = bytearray(data), stop, fragment
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, size):
        if not self.data:
            self.stop.set()
            return b""
        size = min(size, self.fragment)
        result = bytes(self.data[:size])
        del self.data[:size]
        return result

    def close(self):
        self.closed = True


class ReplayTests(unittest.TestCase):
    def test_binary_bench_reader_needs_only_the_two_selected_nodes(self):
        frames = []
        reader = SerialReader(frames.append)
        reader.nodes = (2, 4)
        stream = bytearray()
        for seq in range(21):
            for node in reader.nodes:
                t = 1000000+seq*10000
                stream.extend(wire(quality_payload(node, t,
                    offset_us=node*100000, seq=seq, q=IDENTITY), event=t+300))
        stop = threading.Event()
        reader._read_loop('REPLAY_TWO_NODES', ReplayPort(stream, stop), stop)
        self.assertEqual(len(frames), 21)
        self.assertEqual(set(frames[-1].orientations), {2, 4})

    def test_four_node_binary_to_signed_angles_and_duplicate_suppression(self):
        payloads = []
        c = SessionController(payloads.append, lambda _: None, lambda _: None)
        self.addCleanup(c.close)
        c._active = c._has_current = True
        c.kinematics.sensor_to_segment = {n: IDENTITY for n in range(1, 5)}
        c.kinematics.calibration_kind = "measured_profile"
        c._functional = set(JOINTS)
        stream = bytearray()
        expected = []
        for seq in range(101):
            t = 1_000_000+seq*10000
            hip = 20*math.sin(seq/100*2*math.pi)
            knee = 30+10*math.sin(seq/100*2*math.pi)
            ankle = 15*math.sin(seq/100*2*math.pi)
            q = {1: IDENTITY, 2: axis_angle((0, 0, 1), hip)}
            q[3] = multiply(q[2], axis_angle((0, 0, 1), -knee))
            q[4] = multiply(q[3], axis_angle((0, 0, 1), ankle))
            expected.append((hip, knee, ankle))
            for node in range(1, 5):
                packet = wire(quality_payload(node, t, offset_us=node*100000, seq=seq, q=q[node]), event=t+300)
                stream.extend(packet)
                if seq % 10 == 0:
                    stream.extend(packet)
        stop = threading.Event()
        port = ReplayPort(stream, stop)
        c.reader._read_loop("REPLAY", port, stop)
        rows = c.rows_for_export()
        self.assertEqual(len(rows), 101)
        squared = []
        for row, truth in zip(rows, expected):
            squared.extend((row[joint]-value)**2 for joint, value in zip(JOINTS, truth))
        self.assertLess(math.sqrt(sum(squared)/len(squared)), .01)
        self.assertTrue(port.closed)

    def test_serial_error_closes_port_and_reports_disconnect(self):
        statuses, connections = [], []
        reader = SerialReader(lambda _: None, statuses.append, connections.append)
        class BrokenPort:
            in_waiting = 1
            closed = False
            def read(self, _):
                raise OSError("test disconnect")
            def close(self):
                self.closed = True
        port = BrokenPort()
        reader._read_loop("REPLAY", port, threading.Event())
        self.assertTrue(port.closed)
        self.assertFalse(connections[-1])
        self.assertTrue(any("Mất kết nối" in s for s in statuses))


if __name__ == "__main__":
    unittest.main()
