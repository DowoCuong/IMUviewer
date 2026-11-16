import csv
import math
import tempfile
import unittest
from pathlib import Path

from imu_desktop.core.gait import analyze_cycles
from imu_desktop.validate import metrics, validate_csv


class GaitTests(unittest.TestCase):
    def make_rows(self):
        return [{"t_ms": t, "hip": 10*math.sin(t*math.pi/500),
                 "knee": 30+20*math.sin(t*math.pi/500),
                 "ankle": -5+10*math.sin(t*math.pi/500), "calibration_id": 1}
                for t in range(0, 2001, 10)]

    def events(self):
        return [{"t_ms": 0, "event": "IC"}, {"t_ms": 600, "event": "TO"},
                {"t_ms": 1000, "event": "IC"}, {"t_ms": 1600, "event": "TO"},
                {"t_ms": 2000, "event": "IC"}]

    def test_verified_events_produce_phase_timing_signed_rom_and_mean_sd(self):
        report = analyze_cycles(self.make_rows(), self.events())
        self.assertEqual(report["count"], 2)
        self.assertEqual(report["cycles"][0]["stance_s"], .6)
        self.assertEqual(report["cycles"][0]["swing_s"], .4)
        self.assertEqual(report["cycles"][0]["rom_deg"]["ankle"], 20)
        self.assertEqual(len(report["curves"]["hip"]["mean"]), 101)
        self.assertLess(max(report["curves"]["hip"]["sd"]), 1e-10)

    def test_incomplete_cycle_has_no_invented_contact(self):
        report = analyze_cycles(self.make_rows(), [{"t_ms": 0, "event": "IC"}, {"t_ms": 1000, "event": "IC"}])
        self.assertEqual(report["count"], 0)
        self.assertEqual(len(report["rejected"]), 1)

    def test_gap_and_calibration_change_reject_affected_cycle(self):
        rows = [r for r in self.make_rows() if not 200 < r["t_ms"] < 400]
        report = analyze_cycles(rows, self.events())
        self.assertEqual(report["count"], 1)
        rows = self.make_rows()
        for row in rows:
            if row["t_ms"] > 500:
                row["calibration_id"] = 2
        self.assertEqual(analyze_cycles(rows, self.events())["count"], 1)

    def test_bad_event_order_nonfinite_or_unknown_event_fails(self):
        for events in ([{"t_ms": 10, "event": "IC"}, {"t_ms": 0, "event": "TO"}],
                       [{"t_ms": math.nan, "event": "IC"}], [{"t_ms": 0, "event": "HEEL"}]):
            with self.assertRaises(ValueError):
                analyze_cycles(self.make_rows(), events)


class EncoderValidationTests(unittest.TestCase):
    def test_direct_metrics_keep_bias_and_population_error_sd(self):
        result = metrics([1, 3, 5], [0, 2, 3])
        self.assertAlmostEqual(result["rmse_deg"]**2,
                               result["bias_deg"]**2+result["error_sd_deg"]**2)
        self.assertEqual(result["max_abs_deg"], 2)

    def test_nonfinite_data_is_rejected(self):
        with self.assertRaises(ValueError):
            metrics([1, math.nan], [1, 2])

    def test_direct_result_is_preserved_when_lag_fit_is_requested(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"encoder.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["t_ms", "knee", "encoder"])
                for t in range(0, 3001, 5):
                    writer.writerow([t, 2+30*math.sin((t-20)/1000*2*math.pi), 30*math.sin(t/1000*2*math.pi)])
            direct = validate_csv(path)
            fitted = validate_csv(path, lag_search_ms=50)
            self.assertEqual(direct["direct"], fitted["direct"])
            self.assertEqual(fitted["diagnostic_fit"]["lag_ms"], 20)
            self.assertAlmostEqual(fitted["diagnostic_fit"]["additional_offset_deg"], 2)
            self.assertLess(fitted["diagnostic_fit"]["metrics"]["rmse_deg"], 1e-10)


if __name__ == "__main__":
    unittest.main()
