import csv
import math
from pathlib import Path
import tempfile
import unittest

from tools.validate_encoder_archive import analyze_pair


class ArchiveValidationTests(unittest.TestCase):
    def write_trial(self, directory):
        path = Path(directory)/'trial.csv'
        rows = []
        for i, angle in enumerate((0, 30, 60, 88)):
            half = math.radians(angle)/2
            row = {'Timestamp': f'2026-05-28 12:00:00.{i*20:03}', 'Encoder_Valid': '1',
                   'Relative_Angle': -90 if angle == 88 else -angle,
                   'Relative_Angle_Zeroed': -90 if angle == 88 else -angle,
                   'Encoder_Angle': angle, 'Encoder_Angle_Zeroed': angle,
                   'Encoder_Age_ms': 1000 if i == 3 else 1, 'Master_Seq': i,
                   'Parent_IMU': '1', 'Child_IMU': '2', 'Encoder_Timestamp': 'unverified'}
            for label, q in [('Parent', (1, 0, 0, 0)), ('Child', (math.cos(half), 0, -math.sin(half), 0))]:
                row.update({label+'_Q_'+axis: value for axis, value in zip('WXYZ', q)})
            rows.append(row)
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_primary_preserves_stale_reference_and_clipped_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_trial(directory)
            before = path.read_bytes()
            report, pairs = analyze_pair(path, -1)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(report['primary_stored_zeroed']['count'], 4)
            self.assertEqual(report['primary_stored_zeroed']['rmse_deg'], 1)
            self.assertEqual(report['age_sensitivity']['excluded_count'], 1)
            self.assertEqual(report['age_sensitivity']['metrics']['rmse_deg'], 0)
            self.assertEqual(pairs[-1]['measured'], 90)

    def test_quaternion_replay_uses_only_imu_for_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            report, pairs = analyze_pair(self.write_trial(directory), -1)
            self.assertLess(report['quaternion_replay']['metrics']['rmse_deg'], 1e-10)
            self.assertEqual(report['quaternion_replay']['premature_90_degree_samples'], 1)
            self.assertAlmostEqual(pairs[-1]['quaternion_zeroed_measured'], 88)
            self.assertGreater(report['literal_stored_without_direction_mapping']['rmse_deg'], 50)

    def test_direction_is_explicit_not_fitted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_trial(directory)
            for sign in (0, 2):
                with self.assertRaises(ValueError):
                    analyze_pair(path, sign)
            report, _ = analyze_pair(path, 1)
            self.assertGreater(report['primary_stored_zeroed']['rmse_deg'], 50)


if __name__ == '__main__':
    unittest.main()
