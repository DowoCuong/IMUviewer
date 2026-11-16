"""Exercise the real local WebEngine page and QWebChannel without hardware."""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox --use-angle=swiftshader --enable-unsafe-swiftshader")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtWidgets import QApplication
from imu_desktop.main import MainWindow
from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.math3d import axis_angle, multiply


def main() -> int:
    app = QApplication.instance() or QApplication([])
    window = MainWindow()
    window.resize(1440, 1000)
    window.show()
    controller = window.bridge.controller
    controller.reader.start = lambda *_: (True, "SYNTHETIC", 115200)
    output = ROOT/"exports"/"verification"
    output.mkdir(parents=True, exist_ok=True)

    def wait(predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            app.processEvents()
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError("Desktop smoke timed out")

    def js(source):
        values = []
        window.page.runJavaScript("JSON.stringify((0,eval)("+json.dumps(source)+"))", 0, values.append)
        wait(lambda: bool(values))
        return json.loads(values[0]) if values[0] else None

    def inject(t, joint=None, angle=0):
        upright = axis_angle((1, 0, 0), 90)
        poses = {n: upright for n in range(1, 5)}
        if joint:
            parent = {"hip": 1, "knee": 2, "ankle": 3}[joint]
            for node in range(parent+1, 5):
                poses[node] = multiply(upright, axis_angle((0, 0, 1), -angle if joint == "knee" else angle))
        controller._handle_frame(JointFrame(t, poses, 30, 10000, "hub_read_completion"))

    try:
        wait(lambda: js("Boolean(window.handleBatch && document.getElementById('portDescription').textContent !== 'Chưa tìm thấy cổng kết nối.')"))
        js("document.getElementById('startButton').click(); true")
        wait(lambda: controller._active)
        for i in range(251):
            inject(i*10000)
        window.bridge._flush_samples()
        js("document.getElementById('neutralButton').click(); true")
        wait(lambda: controller.kinematics.calibrated)
        cursor = 2_500_000
        for joint in ("hip", "knee", "ankle"):
            js(f"document.getElementById('calibrationJoint').value={json.dumps(joint)}; document.getElementById('functionalButton').click(); true")
            wait(lambda: controller._collect_joint == joint)
            for i in range(501):
                cursor += 10000
                inject(cursor, joint, 30*math.sin(i/200*2*math.pi))
            window.bridge._flush_samples()
            js("document.getElementById('functionalButton').click(); true")
            wait(lambda: joint in controller._functional)
        for i in range(101):
            cursor += 10000
            inject(cursor, "knee", 30+20*math.sin(i/100*2*math.pi))
        window.bridge._flush_samples()
        wait(lambda: js("Number(document.getElementById('sampleCount').textContent.replaceAll(',', '')) >= 100"))
        state = js("({errors:window.__startupErrors, count:document.getElementById('sampleCount').textContent, knee:document.getElementById('liveKnee').textContent, calibration:document.getElementById('calibrationState').textContent, sceneError:window.__sceneError || null, overflow:document.documentElement.scrollWidth > innerWidth})")
        assert not state["errors"], state
        assert abs(float(state["knee"])-30) < .01, state
        assert not state["overflow"], state
        ok, message = controller.write_csv(output/"desktop-smoke.csv")
        assert ok, message
        js("document.getElementById('resetMaxButton').click(); true")
        wait(lambda: js("document.getElementById('maxKnee').textContent === '—'"))
        assert abs(float(js("document.getElementById('liveKnee').textContent"))-30) < .01
        paint_deadline = time.monotonic()+.5
        wait(lambda: time.monotonic() > paint_deadline)
        window.view.grab().save(str(output/"desktop-smoke.png"))
        js("document.getElementById('stopButton').click(); true")
        wait(lambda: not controller._active)
        state["measured_samples"] = len(controller.rows_for_export())
        (output/"desktop-smoke.json").write_text(json.dumps(state, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(state))
    except Exception:
        print('PAGE DIAGNOSTICS:', js("JSON.stringify({url:location.href,ready:document.readyState,errors:window.__startupErrors,bridge:typeof QWebChannel,handler:typeof window.handleBatch,text:document.body?.innerText?.slice(0,500)})"))
        window.view.grab().save(str(output/"desktop-failure.png"))
        raise
    finally:
        window.close()
        app.processEvents()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
