"""Check all six page links in one Qt window; serial access is forbidden."""
import json
import os
from pathlib import Path
import re
import sys
import time
from unittest.mock import patch

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['QTWEBENGINE_DISABLE_SANDBOX'] = '1'
os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = '--disable-gpu'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication
from imu_desktop import __version__
from imu_desktop.main import MainWindow

out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
QSettings.setDefaultFormat(QSettings.IniFormat)
QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, str(out/'settings'))
app = QApplication([])


def wait(condition, timeout=20):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        app.processEvents()
        if condition():
            return
        time.sleep(.01)
    raise AssertionError('Navigation timed out')


with patch('serial.Serial', side_effect=AssertionError('Real COM access forbidden')) as serial:
    window = MainWindow()
    bridge = window.bridge
    window.show()

    def js(code):
        values = []
        window.page.runJavaScript('JSON.stringify((0,eval)('+json.dumps(code)+'))', 0, values.append)
        wait(lambda: bool(values))
        return json.loads(values[0]) if values[0] else None

    results = []
    try:
        handlers = {'index.html': 'handleBatch', 'bench.html': 'handleBenchBatch', 'motion.html': 'handleMotionBatch'}
        titles = {name: re.search(r'<title>(.*?)</title>', (ROOT/'imu_desktop/ui'/name).read_text(encoding='utf-8')).group(1)
                  for name in handlers}
        for target in ('index.html', 'motion.html', 'bench.html', 'index.html', 'bench.html', 'motion.html', 'index.html'):
            if results:
                js("document.querySelector('header a[href=\""+target+"\"]').click(); true")
            wait(lambda: window.page.url().fileName() == target)
            wait(lambda: js("document.readyState==='complete' && location.href.endsWith('/"+target+"') && document.title==="+
                            json.dumps(titles[target])+" && typeof window."+handlers[target]+" === 'function'"))
            assert window.bridge is bridge
            assert __version__ in window.windowTitle()
            for width in (1050, 1440):
                window.resize(width, 900)
                painted = time.monotonic()+.2
                wait(lambda: time.monotonic() >= painted)
                state = js("({title:document.title,links:[...document.querySelectorAll('header a')].map(a=>a.getAttribute('href')),overflow:document.documentElement.scrollWidth>innerWidth,errors:window.__startupErrors||[]})")
                assert state['title'] == titles[target]
                assert set(handlers)-{target} <= set(state['links']), state
                assert not state['errors'] and not state['overflow'], state
                window.grab().save(str(out/(target+f'-{width}.png')))
            results.append(dict(page=target, **state))
        assert not serial.called
        (out/'navigation.json').write_text(json.dumps(dict(passed=True, version=__version__,
            pages=results, one_window_one_bridge=True, serial_opened=False), indent=2), encoding='utf-8')
        print('PASS: six page transitions, one window/bridge, no COM access')
    finally:
        window.close()
        app.processEvents()
