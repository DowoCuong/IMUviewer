import csv
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import zipfile

from imu_desktop.core.bench import BenchController
from imu_desktop.core.protocol import StreamDecoder, encode_frame
from imu_desktop.core.serial import SerialReader
from test_bench import OPTIONS
from test_radio_bench import packet, measurement


class RadioOnlyTests(unittest.TestCase):
    def test_stop_leaves_serial_close_to_reader_without_racing_inflight_read(self):
        reader = SerialReader(lambda _: None)
        reading = threading.Event()
        closed = threading.Event()
        calls = []
        class Connection:
            in_waiting = 0
            def read(self, count):
                reading.set()
                reader._stop_event.wait(1)
                self.assert_still_open = not closed.is_set()
                return b''
            def close(self):
                calls.append(threading.current_thread().name)
                closed.set()
        connection = Connection()
        thread = threading.Thread(target=reader._read_loop, args=('FAKE',connection), name='test-reader')
        reader._serial, reader._thread = connection, thread
        thread.start()
        self.assertTrue(reading.wait(1))
        reader.stop()
        self.assertTrue(connection.assert_still_open)
        self.assertEqual(calls, ['test-reader'])
        self.assertFalse(thread.is_alive())

    def test_hub_only_receives_four_nodes_exports_raw_but_never_scores_rmse(self):
        bench = BenchController(lambda _: None)
        try:
            with patch.object(bench.encoder, 'start') as encoder, \
                 patch.object(bench.motor, 'start') as motor, \
                 patch.object(bench.imu, 'start', return_value=(True, 'IMU', 115200)) as imu:
                bench.connect(dict(OPTIONS, encoder_port=''))
                encoder.assert_not_called(); motor.assert_not_called()
                imu.assert_called_once_with('IMU', 115200)
                self.assertEqual(bench.imu.nodes, (1, 2))
            decoder = StreamDecoder()
            for node in range(1, 5):
                p = measurement(node, 0, 100000) if node <= 2 else packet(node, 0, 100000)
                if node == 2:
                    p[32] = 0xf3  # Real observed fault: accelerometer calibration fell to 0.
                    p[33] = 1
                bench._log('imu', decoder.feed(encode_frame(p, 100400))[0].record)
            self.assertEqual(bench.radio_bench.snapshot()['nodes'][1]['calibration_raw'], 0xf3)
            self.assertEqual(bench.radio_bench.snapshot()['nodes'][1]['sensor_state'], 1)
            self.assertEqual(bench.n, 0)
            self.assertIsNone(bench.snapshot()['metrics']['rmse_deg'])
            for action in (bench.capture_zero, bench.start_trial):
                with self.assertRaisesRegex(ValueError, 'encoder'): action()
            with tempfile.TemporaryDirectory() as d:
                path = Path(d)/'radio.csv'
                result = bench.export(path)
                self.assertEqual(result['mode'], 'radio_bench')
                with zipfile.ZipFile(path.with_suffix('.session.zip')) as z:
                    self.assertEqual(list(csv.DictReader(io.StringIO(z.read('paired.csv').decode('utf-8-sig')))), [])
                    self.assertEqual([r['packets'] for r in json.loads(z.read('radio_bench.json'))['nodes']], [1]*4)
                    self.assertIn(b'"synthetic": true', z.read('raw.jsonl'))
        finally:
            bench.close()

    def test_invalid_ports_and_motor_without_encoder_do_not_open_any_device(self):
        bench = BenchController(lambda _: None)
        try:
            for values in (dict(OPTIONS, imu_port=''), dict(OPTIONS, encoder_port='imu'),
                           dict(OPTIONS, encoder_port='', motor_port='UNO')):
                with patch('imu_desktop.core.bench.serial.Serial') as port:
                    with self.assertRaises(ValueError): bench.connect(values)
                    port.assert_not_called()
        finally:
            bench.close()


if __name__ == '__main__': unittest.main()
