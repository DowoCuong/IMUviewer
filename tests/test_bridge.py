import json
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
try:
    from PySide6.QtWidgets import QApplication, QWidget
    from imu_desktop.bridge import DesktopBridge
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, "Install the desktop requirements for Qt tests")
class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        class Page:
            def __init__(self):
                self.calls = []
            def runJavaScript(self, source):
                self.calls.append(source)
        self.page, self.parent = Page(), QWidget()
        self.bridge = DesktopBridge(self.page, self.parent)
        self.addCleanup(self.bridge.shutdown)

    def test_dialog_uses_widget_parent_and_export_failure_is_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch("imu_desktop.bridge.QFileDialog.getSaveFileName", return_value=(str(Path(directory)/"a.csv"), "")) as dialog:
                result = self.bridge.export_csv()
            self.assertIs(dialog.call_args.args[0], self.parent)
            self.assertFalse(result["ok"])

    def test_ui_batch_keeps_samples_in_order_and_metrics_reset_is_separate(self):
        for t in range(4):
            self.bridge._on_sample({"t": t, "hip": -t})
        self.assertEqual(self.page.calls, [])
        self.bridge._flush_samples()
        data = json.loads(self.page.calls[-1].split("window.handleBatch(", 1)[1].removesuffix(");"))
        self.assertEqual([row["t"] for row in data], [0, 1, 2, 3])
        self.bridge._on_sample({"metricsOnly": True, "maxHip": None})
        self.assertIn("window.handleSample", self.page.calls[-1])

    def test_json_boundary_rejects_nonfinite_numbers(self):
        with self.assertRaises(ValueError):
            self.bridge._run_js("window.handleSample", {"hip": math.nan})

    def test_empty_neutral_capture_returns_error_instead_of_exception(self):
        self.assertFalse(self.bridge.capture_neutral()["ok"])


if __name__ == "__main__":
    unittest.main()
