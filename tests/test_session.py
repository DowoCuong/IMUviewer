import csv
import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.angles import JOINTS
from imu_desktop.core.math3d import IDENTITY, axis_angle
from imu_desktop.core.session import SessionController


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.payloads = []
        self.controller = SessionController(self.payloads.append, lambda _: None, lambda _: None)
        self.addCleanup(self.controller.close)

    def activate(self):
        c = self.controller
        c._active = c._has_current = True
        c.kinematics.sensor_to_segment = {n: IDENTITY for n in range(1, 5)}
        c.kinematics.calibration_kind = "measured_profile"
        c._functional = set(JOINTS)
        return c

    def test_csv_retains_leading_angle_columns_and_raw_bundle(self):
        c = self.activate()
        c._handle_record({"node": 1, "qw": 16384, "completed_us": 12345})
        c._handle_frame(JointFrame(10000, {1: IDENTITY, 2: axis_angle((0, 0, 1), -12.34567),
            3: IDENTITY, 4: axis_angle((0, 0, 1), -10)}, 30, 10000, "test"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"samples.csv"
            ok, message = c.write_csv(path)
            self.assertTrue(ok, message)
            data = path.read_bytes()
            self.assertTrue(data.startswith(b"\xef\xbb\xbf"))
            rows = list(csv.DictReader(io.StringIO(data.decode("utf-8-sig"))))
            self.assertEqual(list(rows[0])[:4], ["t_ms", "hip", "knee", "ankle"])
            self.assertAlmostEqual(float(rows[0]["hip"]), -12.34567)
            self.assertAlmostEqual(float(rows[0]["ankle"]), -10)
            with zipfile.ZipFile(path.with_suffix(".session.zip")) as bundle:
                self.assertEqual(json.loads(bundle.read("raw.jsonl"))["qw"], 16384)
                self.assertEqual(json.loads(bundle.read("metadata.json"))["measurement_filter_hz"], 0)

    def test_rom_keeps_negative_angles(self):
        rows = [{"t_ms": 1, "hip": -10, "knee": -5, "ankle": -20},
                {"t_ms": 2, "hip": 30, "knee": 50, "ankle": 15}]
        summary = self.controller._summary(rows)
        self.assertEqual((summary["romHip"], summary["romKnee"], summary["romAnkle"]), (40, 55, 35))

    def test_second_stop_retains_last_session(self):
        c = self.activate()
        c._current_rows = [{"t_ms": 0, "hip": -10, "knee": 20, "ankle": -5}]
        first = c.stop()
        self.assertEqual(c.stop(), first)
        self.assertEqual(len(c.rows_for_export()), 1)

    def test_current_session_is_exported_before_previous_session(self):
        c = self.activate()
        c._last_rows = [{"t_ms": 1, "hip": 1, "knee": 2, "ankle": 3}]
        c._current_rows = [{"t_ms": 2, "hip": 10, "knee": 20, "ankle": 30}]
        self.assertEqual(c.rows_for_export()[0]["hip"], 10)

    def test_uncalibrated_preview_never_becomes_a_measured_csv_sample(self):
        c = self.controller
        c._active = c._has_current = True
        c._handle_frame(JointFrame(0, {n: IDENTITY for n in range(1, 5)}, 20, 0, "test"))
        self.assertTrue(self.payloads[-1]["preview"])
        self.assertEqual(c.rows_for_export(), [])

    def test_max_reset_does_not_add_a_sample_or_reset_pose(self):
        c = self.activate()
        c._handle_frame(JointFrame(10000, {n: IDENTITY for n in range(1, 5)}, 20, 0, "test"))
        c.reset_max()
        self.assertEqual(len(c.rows_for_export()), 1)
        self.assertTrue(self.payloads[-1]["metricsOnly"])
        self.assertEqual(self.payloads[-1]["ankle"], 0)

    def test_per_sample_path_does_not_scan_or_copy_history(self):
        class AppendOnly(list):
            def __iter__(self):
                raise AssertionError("History scanned in the acquisition callback")
        c = self.activate()
        c._current_rows = AppendOnly([{}]*100000)
        c._handle_frame(JointFrame(10000, {n: IDENTITY for n in range(1, 5)}, 20, 0, "test"))
        self.assertEqual(len(c._current_rows), 100001)
        c._current_rows = []

    def test_failed_start_preserves_previous_export_and_clears_old_filter(self):
        c = self.controller
        c._last_rows = [{"t_ms": 1, "hip": 1, "knee": 2, "ankle": 3}]
        c.processor.process(1000, 20, 100, 100)
        c.reader.start = lambda *_: (False, "unavailable", 115200)
        self.assertFalse(c.start(None, 115200)[0])
        self.assertEqual(c.rows_for_export()[0]["knee"], 2)
        self.assertIsNone(c.processor.snapshot()["smoothKnee"])

    def test_empty_export_has_no_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"empty.csv"
            self.assertFalse(self.controller.write_csv(path)[0])
            self.assertFalse(path.exists())

    def test_failed_restart_keeps_previous_raw_capture_metadata(self):
        c = self.activate()
        c.reader.mode = "quaternion"
        c._health = {"decoder": {"crc_errors": 3}}
        c._handle_record({"test": "previous"})
        c._current_rows = [{"t_ms": 0, "hip": 1, "knee": 2, "ankle": 3}]
        c.stop()
        c.reader.start = lambda *_: (False, "unavailable", 115200)
        self.assertFalse(c.start(None, 115200, "euler_zyx")[0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"saved.csv"
            self.assertTrue(c.write_csv(path)[0])
            with zipfile.ZipFile(path.with_suffix(".session.zip")) as bundle:
                metadata = json.loads(bundle.read("metadata.json"))
            self.assertEqual(metadata["input_protocol"], "quaternion")
            self.assertEqual(metadata["capture_health"]["decoder"]["crc_errors"], 3)


if __name__ == "__main__":
    unittest.main()
