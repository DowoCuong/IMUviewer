import math
import struct
import unittest
import zlib

from imu_desktop.core.alignment import SampleSynchronizer
from imu_desktop.core.math3d import IDENTITY, axis_angle, distance_deg
from imu_desktop.core.protocol import SensorSample, StreamDecoder


def wire(payload, event=0):
    body = struct.pack("<HI", len(payload), event)+bytes(payload)
    return b"IMU1"+body+struct.pack("<I", zlib.crc32(body) & 0xffffffff)


def timed_payload(node=1, time_us=1_010_000, offset_us=500_000, seq=1,
                  q=IDENTITY, ppm=0, hub_anchor=None, session=12):
    payload = bytearray(64)
    struct.pack_into("<BBBBIII", payload, 0, 1, 9, node, 3, session, seq, seq)
    if hub_anchor is None:
        hub_anchor = time_us//40000*40000
    local_anchor = hub_anchor+offset_us
    local_time = local_anchor+round((time_us-hub_anchor)*(1+ppm/1e6))
    struct.pack_into("<Q4hBB2xI", payload, 16, local_time,
                     *(round(v*16384) for v in q), 255, 2, 2000)
    struct.pack_into("<IIiIII", payload, 40, local_anchor & 0xffffffff,
                     hub_anchor & 0xffffffff, ppm, 30, 77, 1100)
    return payload


def sample(node, t, q=IDENTITY, sequence=None, session=1, usable=True):
    return SensorSample(node, t, q, int(t) if sequence is None else sequence, session,
                        ("test_shared", 1), usable, 20, {})


class ProtocolTests(unittest.TestCase):
    def test_binary_fragmentation_at_every_byte(self):
        data = wire(timed_payload(q=axis_angle((0, 0, 1), 40)))
        decoder, result = StreamDecoder(), []
        for byte in data:
            result.extend(decoder.feed(bytes([byte])))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].time_us, 1_010_000)
        self.assertLess(distance_deg(result[0].quaternion, axis_angle((0, 0, 1), 40)), .01)
        self.assertTrue(result[0].usable)

    def test_corrupt_crc_and_noise_resynchronize(self):
        good = wire(timed_payload())
        bad = bytearray(good)
        bad[35] ^= 1
        decoder = StreamDecoder()
        result = decoder.feed(b"noise"+bad+good)
        self.assertEqual(len(result), 1)
        self.assertEqual(decoder.counts["crc_errors"], 1)

    def test_clock_offset_and_frequency_are_mapped_not_pc_arrival(self):
        decoder = StreamDecoder()
        for node, offset in ((1, 500_000), (2, 87_000_000)):
            result = decoder.feed(wire(timed_payload(node=node, time_us=1_035_000, offset_us=offset, ppm=100), event=node*9))
            self.assertAlmostEqual(result[0].time_us, 1_035_000, delta=1)

    def test_hub_and_node_low32_rollover(self):
        decoder = StreamDecoder()
        anchor = (1 << 32)-20000
        a = decoder.feed(wire(timed_payload(time_us=anchor+10000, hub_anchor=anchor)))[0]
        b = decoder.feed(wire(timed_payload(time_us=anchor+50000, hub_anchor=anchor+40000, seq=2)))[0]
        self.assertAlmostEqual(b.time_us-a.time_us, 40000)

    def test_old_binary_data_is_logged_but_never_assumed_time_aligned(self):
        p = timed_payload()[:40]
        p[1] = 1
        result = StreamDecoder().feed(wire(p))[0]
        self.assertIsNone(result.time_us)
        self.assertIsNotNone(result.quaternion)

    def test_stale_paused_or_uncalibrated_sample_is_unusable(self):
        for offset, value in ((3, 7), (3, 11), (32, 0), (33, 1)):
            p = timed_payload()
            p[offset] = value
            result = StreamDecoder().feed(wire(p))
            self.assertEqual(len(result), 1)
            self.assertFalse(result[0].usable)

    def test_bad_norm_clock_fit_and_node_are_rejected(self):
        cases = []
        p = timed_payload(q=(0, 0, 0, 0)); cases.append(p)
        p = timed_payload(); p[2] = 99; cases.append(p)
        p = timed_payload(); struct.pack_into("<i", p, 48, 201); cases.append(p)
        p = timed_payload(); struct.pack_into("<I", p, 52, 51); cases.append(p)
        for p in cases:
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(wire(p)), [])
            self.assertEqual(decoder.counts["invalid_records"], 1)

    def test_text_quaternion_and_explicit_legacy_convention(self):
        a = StreamDecoder("quaternion").feed(b"QUAT,1,12.5,1,0,0,0\n")[0]
        self.assertEqual(a.time_us, 12500)
        b = StreamDecoder("euler_zyx").feed(b"IMU,1,12.5,0,30,0\n")[0]
        self.assertLess(distance_deg(b.quaternion, axis_angle((1, 0, 0), 30)), 1e-10)

    def test_empty_nonfinite_and_oversized_text_are_bounded(self):
        decoder = StreamDecoder("quaternion")
        for line in (b"QUAT,1,1,1,,0,0\n", b"QUAT,1,inf,1,0,0,0\n", b"QUAT,1,1,nan,0,0,0\n"):
            self.assertEqual(decoder.feed(line), [])
        decoder.feed(b"x"*100000)
        self.assertLessEqual(len(decoder.buffer), 512)


class SynchronizationTests(unittest.TestCase):
    def test_missing_sensor_never_produces_fictitious_zero_angles(self):
        sync = SampleSynchronizer()
        for t in range(0, 100000, 10000):
            for node in (1, 2, 3):
                self.assertEqual(sync.add(sample(node, t)), [])

    def test_asynchronous_rotations_interpolate_to_same_time(self):
        sync, frames = SampleSynchronizer(), []
        records = []
        for node, offset in enumerate((0, 2000, 4000, 6000), 1):
            for t in range(offset, 100001, 10000):
                q = axis_angle((0, 0, 1), t/1000)
                records.append(sample(node, t, tuple(-v for v in q) if t//10000 % 2 else q))
        for record in sorted(records, key=lambda r: r.time_us):
            frames.extend(sync.add(record))
        self.assertGreater(len(frames), 5)
        for frame in frames:
            for q in frame.orientations.values():
                self.assertLess(distance_deg(q, axis_angle((0, 0, 1), frame.time_us/1000)), 1e-6)

    def test_retransmission_and_out_of_order_do_not_create_new_samples(self):
        sync = SampleSynchronizer(nodes=(1,))
        self.assertEqual(len(sync.add(sample(1, 10000))), 1)
        self.assertEqual(sync.add(sample(1, 10000)), [])
        self.assertEqual(sync.add(sample(1, 9000)), [])
        self.assertEqual(sync.counts["retransmissions"], 1)
        self.assertEqual(sync.counts["out_of_order"], 1)

    def test_gap_and_bad_status_are_not_interpolated_across(self):
        sync = SampleSynchronizer(nodes=(1, 2))
        sync.add(sample(1, 0)); sync.add(sample(2, 0))
        sync.add(sample(1, 10000)); sync.add(sample(2, 10000))
        sync.add(sample(1, 20000, usable=False))
        self.assertEqual(sync.add(sample(2, 50000)), [])
        result = sync.add(sample(1, 50000))
        self.assertEqual([r.time_us for r in result], [50000])

    def test_node_reboot_resets_all_buffers(self):
        sync = SampleSynchronizer(nodes=(1, 2))
        sync.add(sample(1, 10000)); sync.add(sample(2, 10000))
        generation = sync.generation
        self.assertEqual(sync.add(sample(1, 0, session=2)), [])
        self.assertGreater(sync.generation, generation)
        self.assertFalse(sync.buffers[2])


if __name__ == "__main__":
    unittest.main()
