import math
import unittest

from imu_desktop.core.angles import AngleProcessor, clamp, norm_deg, parse_serial_line, relative_angles


class AngleMathTests(unittest.TestCase):
    def test_original_normalization_boundary_contract(self):
        for source, expected in [(0, 0), (180, 180), (181, -179), (540, 180), (-181, 179), (-540, -180)]:
            self.assertEqual(norm_deg(source), expected)

    def test_normalization_rejects_nonfinite_without_hanging(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                norm_deg(value)
        self.assertLessEqual(abs(norm_deg(1e300)), 180)

    def test_clamp(self):
        self.assertEqual(clamp(-1, 0, 10), 0)
        self.assertEqual(clamp(11, 0, 10), 10)
        self.assertEqual(clamp(4.5, 0, 10), 4.5)
        with self.assertRaises(ValueError):
            clamp(math.nan, 0, 10)

    def test_planar_helper_requires_complete_sensors_and_keeps_sign(self):
        self.assertEqual(relative_angles({1: 10, 2: 40, 3: 100, 4: 70}), (30, 60, -30, 0))
        with self.assertRaises(ValueError):
            relative_angles({1: 0})


class SerialParserTests(unittest.TestCase):
    def test_valid_imu_record_preserves_timestamp_precision(self):
        self.assertEqual(parse_serial_line("IMU,2,123.9,1.0,-2.5,88.0"), ("imu", 2, 123.9, 1, -2.5, 88))

    def test_malformed_fields_and_extra_columns_are_rejected(self):
        for line in ("", "OTHER,1,1,0,0,0", "IMU,1,broken,0,0,0", "IMU,1,1,0,0",
                     "IMU,1,1,0,,12,34", "IMU,1,1,0,0,0,extra", "IMU,99,1,0,0,0",
                     "IMU,1,-1,0,0,0", "IMU,1,inf,0,0,0", "IMU,1,1,0,nan,0"):
            with self.subTest(line=line):
                self.assertIsNone(parse_serial_line(line))


class ProcessorTests(unittest.TestCase):
    def test_signed_angles_and_ankle_neutral_are_not_clipped(self):
        for value in (-180, -50, -20, -1, 0, 1, 20, 35, 150):
            sample = AngleProcessor().process(0, value, value, value)
            self.assertEqual((sample.hip, sample.knee, sample.ankle), (value, value, value))

    def test_negative_maximum_is_an_observed_value(self):
        p = AngleProcessor()
        sample = p.process(0, -10, -20, -30)
        self.assertEqual((sample.max_hip, sample.max_knee, sample.max_ankle), (-10, -20, -30))

    def test_filter_has_same_response_for_same_elapsed_time(self):
        results = []
        for dt in (1, 5, 10, 20):
            p = AngleProcessor(cutoff_hz=5)
            p.process(0, 0, 0, 0)
            for t in range(dt, 101, dt):
                sample = p.process(t, 20, -20, 10)
            results.append(sample.hip)
        for result in results:
            self.assertAlmostEqual(result, 20*(1-math.exp(-math.pi)), places=10)

    def test_new_session_reset_clears_filter_and_maxima(self):
        p = AngleProcessor(cutoff_hz=5)
        p.process(1000, 20, 100, 100)
        p.reset()
        sample = p.process(0, 0, 0, 0)
        self.assertEqual((sample.hip, sample.knee, sample.ankle), (0, 0, 0))

    def test_reset_max_preserves_filter_but_not_fictitious_zero_maximum(self):
        p = AngleProcessor(cutoff_hz=5)
        p.process(0, -10, -20, -30)
        p.reset_max_angles()
        self.assertIsNone(p.snapshot()["maxHip"])
        sample = p.process(10, -5, -10, -15)
        self.assertEqual(sample.max_hip, -5)
        self.assertLess(sample.hip, -5)

    def test_duplicate_backward_or_nonfinite_samples_do_not_change_state(self):
        p = AngleProcessor()
        p.process(10, 1, 2, 3)
        for t, value in ((10, 4), (9, 4), (20, math.nan), (20, math.inf)):
            snapshot = p.snapshot()
            with self.assertRaises(ValueError):
                p.process(t, value, 0, 0)
            self.assertEqual(p.snapshot(), snapshot)

    def test_filter_resets_after_gap(self):
        p = AngleProcessor(cutoff_hz=5)
        p.process(0, 0, 0, 0)
        self.assertEqual(p.process(500, 20, 30, 40).hip, 20)


if __name__ == "__main__":
    unittest.main()
