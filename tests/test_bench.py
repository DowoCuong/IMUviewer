import csv
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.bench import BenchController, parse_encoder, ReceiptClock, twist_degrees
from imu_desktop.core.math3d import axis_angle, multiply


OPTIONS = dict(imu_port='IMU', encoder_port='MEGA', motor_port='', parent=1, child=2,
               channel=1, counts_per_rev=2500, sign=1, axis='y', max_gap_ms=30)


class DecoderTests(unittest.TestCase):
    def test_current_and_legacy_counts_keep_negative_and_full_turns(self):
        sample = parse_encoder('ENC,7,10000,-2500,5001')
        self.assertEqual(sample.counts, (-2500, 5001))
        self.assertEqual(sample.unit_us, 1)
        legacy = parse_encoder('100,359.856,0.144,-1,2501')
        self.assertEqual(legacy.counts, (-1, 2501))
        self.assertEqual(legacy.unit_us, 1000)

    def test_bad_records_never_become_zero_angles(self):
        for line in ('', 'ms,encoder1_deg,encoder2_deg,encoder1_count,encoder2_count',
                     'ENC,1,2,3', 'ENC,1,2,3,4,5', 'ENC,1,-1,0,0',
                     'ENC,1,2,nan,0', '1,nan,0,1,2', '1,0,0,1.5,2',
                     'ENC,1,2,2147483648,0', 'ENC,4294967296,2,0,0'):
            self.assertIsNone(parse_encoder(line), line)

    def test_receipt_clock_freezes_without_fitting_to_motion(self):
        clock = ReceiptClock()
        clock.observe(100, 1000)
        clock.observe(200, 1080)
        self.assertEqual(clock.offset_us, 880)
        clock.frozen = True
        clock.observe(300, 1100)
        self.assertEqual(clock.mapped(300), 1180)
        with self.assertRaises(ValueError):
            clock.observe(10, 1200)

    def test_twist_parent_axis_neutral_mount_and_sign_equivalence(self):
        neutral = axis_angle((1, 2, 3), 160)
        for angle in (-120, -88, 0, 88, 120, 179):
            relative = multiply(axis_angle((0, 1, 0), angle), neutral)
            for q in (relative, tuple(-x for x in relative)):
                value, off_axis = twist_degrees(q, neutral, 'y')
                self.assertAlmostEqual(value, angle, places=8)
                self.assertLess(off_axis, 1e-8)

    def test_swing_is_reported_and_singularity_is_rejected(self):
        value, off_axis = twist_degrees(axis_angle((1, 0, 0), 20), (1, 0, 0, 0), 'y')
        self.assertAlmostEqual(value, 0)
        self.assertAlmostEqual(off_axis, 20)
        with self.assertRaises(ValueError):
            twist_degrees(axis_angle((1, 0, 0), 180), (1, 0, 0, 0), 'y')


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.updates = []
        self.bench = BenchController(self.updates.append)
        self.addCleanup(self.bench.close)
        self.bench.options = dict(OPTIONS)
        self.bench.connected = True
        self.host_us = 10**12
        self.clock = patch('imu_desktop.core.bench.time.monotonic', side_effect=lambda: self.host_us/1e6)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.mount = axis_angle((1, 1, 2), 145)
        self.i = 0

    def inject(self, count=0, frame=True, encoder=True, relative=None):
        self.host_us = 10**12+self.i*10000
        if encoder:
            self.bench.on_encoder(f'ENC,{self.i},{1000000+self.i*10000},{250+count},{1000+2*count}', self.host_us)
        if frame:
            global_q = axis_angle((1, 2, 3), self.i*.1)
            local = relative or multiply(axis_angle((0, 1, 0), count*360/2500), self.mount)
            self.bench.on_imu(JointFrame(20000000+self.i*10000,
                {1: global_q, 2: multiply(global_q, local)}, 20, 10000, 'hub_read_completion'))
        self.i += 1

    def zero(self):
        for _ in range(301):
            self.inject()
        self.bench.capture_zero()

    def test_two_node_replay_static_zero_full_turns_and_direct_metrics(self):
        self.zero()
        self.bench.start_trial()
        for count in range(-100, 3001, 25):
            self.inject(count)
        self.bench.stop_trial()
        self.assertEqual(self.bench.n, 125)
        self.assertLess(self.bench.snapshot()['metrics']['rmse_deg'], 1e-9)
        self.assertAlmostEqual(self.bench.rows[-1]['encoder'], 432)
        self.assertAlmostEqual(self.bench.rows[-1]['imu_deg'], 432)

    def test_interpolates_reference_without_extrapolation_or_stale_hold(self):
        self.zero()
        self.bench.start_trial()
        self.inject(0)
        n = len(self.bench.rows)
        self.inject(1, encoder=False)
        self.assertEqual(len(self.bench.rows), n)
        self.inject(2)
        self.assertEqual(len(self.bench.rows), n+2)
        self.assertAlmostEqual(self.bench.rows[-2]['encoder'], .144)
        self.assertEqual(self.bench.rows[-2]['encoder_span_ms'], 20)

    def test_reference_gap_rejects_affected_samples(self):
        self.zero()
        self.bench.start_trial()
        self.inject()
        n = len(self.bench.rows)
        for _ in range(5):
            self.inject(5, encoder=False)
        self.assertEqual(len(self.bench.rows), n)
        self.inject(5)
        self.assertGreater(self.bench.counts['encoder_gaps'], 0)
        self.assertEqual(len(self.bench.rows), n+1)

    def test_zero_rejects_motion_and_cannot_change_mid_trial(self):
        for i in range(301):
            self.inject(i)
        with self.assertRaisesRegex(ValueError, 'chuyển động'):
            self.bench.capture_zero()
        self.bench.connected = False
        self.bench._reset_trial()
        self.bench.connected = True
        self.i = 0
        self.zero()
        self.bench.start_trial()
        self.inject(1)
        with self.assertRaises(ValueError):
            self.bench.capture_zero()

    def test_timer_wrap_and_reboot_are_distinct(self):
        self.bench.on_encoder('ENC,1,4294967280,0,0', self.host_us)
        self.bench.on_encoder('ENC,2,16,1,1', self.host_us+32)
        self.assertEqual(self.bench.encoder_extended, 4294967312)
        self.assertEqual(self.bench.counts['device_failures'], 0)
        self.bench.on_encoder('ENC,0,1,0,0', self.host_us+100)
        self.assertEqual(self.bench.counts['device_failures'], 1)

    def test_watchdog_stops_recording_and_requests_motor_s(self):
        self.zero()
        self.bench.start_trial()
        self.inject(1)
        self.bench.motor.connection = object()
        with patch.object(self.bench.motor, 'send') as send:
            self.host_us += 600000
            self.bench.check_health()
        self.bench.motor.connection = None
        send.assert_called_once_with('S')
        self.assertFalse(self.bench.recording)
        self.assertIsNotNone(self.bench.calibration)  # Preserve calibration for exported failed trial.

    def test_export_contains_exact_direct_metrics_calibration_and_raw(self):
        self.zero()
        self.bench.start_trial()
        for count in (0, 10, 20):
            self.inject(count, relative=multiply(axis_angle((0, 1, 0), count*360/2500+2), self.mount))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'paired.csv'
            with self.assertRaises(ValueError):
                self.bench.export(path)
            self.bench.stop_trial()
            report = self.bench.export(path)
            self.assertAlmostEqual(report['metrics']['rmse_deg'], 2)
            self.assertAlmostEqual(report['metrics']['bias_deg'], 2)
            self.assertIn('Chưa xác nhận', report['timing'])
            with zipfile.ZipFile(path.with_suffix('.session.zip')) as archive:
                self.assertEqual(set(archive.namelist()), {'paired.csv', 'metadata.json', 'raw.jsonl', 'radio_bench.json'})
                data = list(csv.DictReader(io.StringIO(archive.read('paired.csv').decode('utf-8-sig'))))
                self.assertEqual(len(data), 3)
                self.assertIsNotNone(json.loads(archive.read('metadata.json'))['calibration'])

    def test_connect_checks_ports_before_open_and_does_not_move_motors(self):
        self.bench.connected = False
        for values in (dict(OPTIONS, encoder_port='imu'), dict(OPTIONS, child=1), dict(OPTIONS, axis='xy')):
            with patch('imu_desktop.core.bench.serial.Serial') as serial:
                with self.assertRaises(ValueError):
                    self.bench.connect(values)
                serial.assert_not_called()
        with patch.object(self.bench.encoder, 'start'), patch.object(self.bench.motor, 'start'), \
             patch.object(self.bench.imu, 'start', return_value=(True, 'IMU', 115200)), \
             patch.object(self.bench.motor, 'send') as send:
            self.bench.connect(dict(OPTIONS, motor_port='UNO'))
            send.assert_not_called()
            self.bench.on_motor('READY: M a b; C a b; S', self.host_us)
            self.bench.motor_command('M', 90, -45)
            send.assert_called_once_with('M 90.000000 -45.000000')
            with self.assertRaises(ValueError):
                self.bench.motor_command('C', 90, 0)

    def test_motor_limits_and_commands_are_checked(self):
        self.bench.motor_state = 'idle'
        with patch.object(self.bench.motor, 'send') as send:
            for args in [('X', 0, 0), ('M', math.nan, 0), ('C', 0, 0), ('M', 361, 0)]:
                with self.assertRaises(ValueError):
                    self.bench.motor_command(*args)
            send.assert_not_called()

    def test_second_encoder_and_declared_negative_imu_direction(self):
        self.bench.options.update(channel=2, sign=-1)
        self.zero()
        self.bench.start_trial()
        for count in (0, 10, 20, 30):
            self.inject(count, relative=multiply(axis_angle((0, 1, 0), -2*count*360/2500), self.mount))
        self.assertLess(self.bench.snapshot()['metrics']['rmse_deg'], 1e-9)

    def test_mega_resolution_mismatch_blocks_trial(self):
        self.bench.on_encoder('ENCODER,2,10000,100', self.host_us)
        self.assertEqual(self.bench.counts['device_failures'], 1)
        with self.assertRaises(ValueError):
            self.bench.start_trial()

    def test_continuing_raw_data_without_valid_pairs_also_stops_trial(self):
        self.zero()
        self.bench.start_trial()
        self.inject(0)
        self.bench.encoder_clock.offset_us += 1000000
        for _ in range(60):
            self.inject(1)
        self.bench.check_health()
        self.assertFalse(self.bench.recording)
        self.assertEqual(self.bench.counts['device_failures'], 1)

    def test_uno_restart_retains_explanation_and_blocks_new_motion(self):
        self.bench.motor_state = 'cycling'
        self.bench.on_motor('READY: M a b; C a b; S', self.host_us)
        self.assertEqual(self.bench.motor_state, 'unknown')
        self.assertIn('khởi động lại', self.bench.message)
        with self.assertRaises(ValueError):
            self.bench.motor_command('M', 10, 0)


if __name__ == '__main__':
    unittest.main()
