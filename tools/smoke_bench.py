"""Real Qt/WebChannel encoder bench smoke. All serial devices and time are fake."""
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('QTWEBENGINE_CHROMIUM_FLAGS', '--no-sandbox --disable-gpu')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PySide6.QtCore import QUrl
from PySide6.QtWidgets import QApplication
from imu_desktop.main import MainWindow
from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.math3d import axis_angle


def main():
    app = QApplication.instance() or QApplication([])
    window = MainWindow()
    bench = window.bridge.bench
    output = ROOT/'exports/verification/bench'
    output.mkdir(parents=True, exist_ok=True)
    commands = []
    class FakeMotor:
        def write(self, data):
            commands.append(data.decode('ascii').strip())
            return len(data)
        def close(self):
            pass
    bench.imu.start = lambda *_: (True, 'FAKE_IMU', 115200)
    bench.encoder.start = lambda *_: None
    bench.motor.start = lambda *_: setattr(bench.motor, 'connection', FakeMotor())
    now = [time.monotonic()]
    def wait(predicate, timeout=20):
        deadline = time.perf_counter()+timeout
        while time.perf_counter() < deadline:
            app.processEvents()
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('Bench UI timed out')
    def js(source):
        values = []
        window.page.runJavaScript('JSON.stringify((0,eval)('+json.dumps(source)+'))', 0, values.append)
        wait(lambda: bool(values))
        return json.loads(values[0]) if values[0] else None
    ports = [{'device': n, 'description': 'Thiết bị mô phỏng'} for n in ('IMU', 'MEGA', 'UNO')]
    window.resize(1440, 1000)
    with patch('imu_desktop.bridge.available_serial_ports', return_value=ports), \
         patch('imu_desktop.core.bench.time.monotonic', side_effect=lambda: now[0]), \
         patch('imu_desktop.bridge.QFileDialog.getSaveFileName', return_value=(str(output/'paired.csv'), '')):
        try:
            window.page.load(QUrl.fromLocalFile(str(ROOT/'imu_desktop/ui/bench.html')))
            window.show()
            wait(lambda: js("document.getElementById('benchEncoderPort')?.options.length === 4"))
            js("document.getElementById('benchImuPort').value='IMU';document.getElementById('benchEncoderPort').value='MEGA';document.getElementById('benchMotorPort').value='UNO';document.getElementById('benchConnect').click();true")
            wait(lambda: bench.connected)
            assert not commands, commands
            bench.on_motor('READY: M a b; C a b; S', now[0]*1e6)
            i = 0
            def inject(count):
                nonlocal i
                now[0] += .01
                bench.on_encoder(f'ENC,{i},{1000000+i*10000},{count},0', now[0]*1e6)
                bench.on_imu(JointFrame(20000000+i*10000,
                    {1:(1,0,0,0),2:axis_angle((0,1,0),count*360/2500)},20,10000,'hub_read_completion'))
                i += 1
            for _ in range(301):
                inject(0)
            window.bridge._flush_samples()
            js("document.getElementById('benchZero').click();true")
            wait(lambda: bench.calibration is not None)
            js("document.getElementById('benchStart').click();true")
            wait(lambda: bench.recording)
            js("document.getElementById('motorMove').click();true")
            wait(lambda: len(commands) == 1)
            assert commands[0] == 'M 90.000000 0.000000', commands
            for k in range(301):
                inject(round(300*math.sin(k*math.pi/150)))
            window.bridge._flush_samples()
            wait(lambda: js("document.getElementById('benchSamples').textContent.includes('301')"))
            assert bench.n == 301
            assert bench.snapshot()['metrics']['rmse_deg'] < 1e-7
            js("document.getElementById('benchStop').click();true")
            wait(lambda: not bench.recording)
            assert commands[-1] == 'S'
            bench.on_motor('STOP', now[0]*1e6)
            window.bridge._flush_samples()
            wait(lambda: js("document.getElementById('benchBadge').textContent==='Đã kết nối'"))
            js("document.getElementById('benchExport').click();true")
            wait(lambda: (output/'paired.session.zip').exists())
            with (output/'paired.csv').open(encoding='utf-8-sig', newline='') as stream:
                assert len(list(csv.DictReader(stream))) == 301
            sizes = []
            for width, height in ((1440, 1000), (1050, 700)):
                window.resize(width, height)
                for _ in range(50):
                    app.processEvents()
                    time.sleep(.01)
                state = js("({overflow:document.documentElement.scrollWidth>innerWidth, badge:document.getElementById('benchBadge').textContent, rmse:document.getElementById('benchRmse').textContent, samples:document.getElementById('benchSamples').textContent, chartWidth:document.getElementById('benchAngles').width, chartHeight:document.getElementById('benchAngles').height})")
                assert not state['overflow'] and state['chartWidth']>0 and state['chartHeight']>0, state
                assert state['badge'] == 'Đã kết nối', state
                sizes.append(state)
                window.view.grab().save(str(output/f'bench-{width}.png'))
            js("document.getElementById('benchDisconnect').click();true")
            wait(lambda: not bench.connected)
            report = {'hardware_opened':False, 'paired_rows':bench.n, 'metrics':bench.snapshot()['metrics'],
                      'motor_commands':commands, 'viewports':sizes}
            js("document.getElementById('benchClear').click();true")
            wait(lambda: js("document.getElementById('benchSamples').textContent.startsWith('0 ') && document.getElementById('benchImu').textContent==='—'"))
            assert not bench.rows and bench.calibration is None
            report['clear_resets_values_and_rows'] = True
            (output/'ui-smoke.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
            print(json.dumps(report))
        finally:
            window.close()
            app.processEvents()


if __name__ == '__main__':
    main()
