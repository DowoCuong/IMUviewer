from __future__ import annotations

import logging
import argparse
import json
import os
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

from PySide6.QtCore import QSettings, QStandardPaths, QUrl, QTimer
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings
from PySide6.QtWebEngineWidgets import QWebEngineView

from . import __version__
from .bridge import DesktopBridge


def resource_path(relative: str) -> Path:
    if getattr(sys, "frozen", False):
        base = Path(getattr(sys, "_MEIPASS")) / "imu_desktop"
    else:
        base = Path(__file__).resolve().parent
    return base / relative


def configure_logging(directory=None) -> Path:
    data_root = Path(directory) if directory else Path(QStandardPaths.writableLocation(QStandardPaths.AppLocalDataLocation))
    data_root.mkdir(parents=True, exist_ok=True)
    log_path = data_root / "imu-joint-viewer.log"
    handler = RotatingFileHandler(log_path, maxBytes=1_000_000, backupCount=3, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.basicConfig(level=logging.INFO, handlers=[handler])
    return log_path


class MainWindow(QMainWindow):
    def __init__(self, initial_page: str = "index.html") -> None:
        super().__init__()
        self.setWindowTitle(f"IMU Viewer {__version__}")
        self.resize(1440, 900)
        self.setMinimumSize(1050, 700)

        self._settings = QSettings("OpenSourceTools", "IMUJointViewer")
        self._restore_geometry()

        self.view = QWebEngineView(self)
        self.page = LocalPage(self.view)
        self.channel = QWebChannel(self.page)
        self.bridge = DesktopBridge(self.page, self)
        self.channel.registerObject("desktop", self.bridge)
        self.page.setWebChannel(self.channel)
        self.page.settings().setAttribute(
            QWebEngineSettings.LocalContentCanAccessFileUrls, True
        )
        self.page.settings().setAttribute(
            QWebEngineSettings.LocalContentCanAccessRemoteUrls, False
        )
        self.view.setPage(self.page)
        self.setCentralWidget(self.view)
        self.page.titleChanged.connect(
            lambda title: self.setWindowTitle(f"IMU Viewer {__version__} · {title}")
        )

        self.page.loadFinished.connect(self._on_load_finished)
        if initial_page not in ("index.html", "bench.html", "motion.html"):
            raise ValueError("Trang khởi động không hợp lệ")
        index_path = resource_path("ui/"+initial_page)
        logging.info("Loading interface from %s (exists=%s)", index_path, index_path.exists())
        self.page.load(QUrl.fromLocalFile(str(index_path)))

    def _restore_geometry(self) -> None:
        geometry = self._settings.value("window/geometry")
        state = self._settings.value("window/state")
        if geometry is not None:
            self.restoreGeometry(geometry)
        if state is not None:
            self.restoreState(state)

    def _on_load_finished(self, ok: bool) -> None:
        if not ok:
            logging.error("The local interface failed to load")

    def closeEvent(self, event: Any) -> None:
        self.bridge.shutdown()
        self._settings.setValue("window/geometry", self.saveGeometry())
        self._settings.setValue("window/state", self.saveState())
        self._settings.sync()
        super().closeEvent(event)


class LocalPage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level: Any, message: str, line: int, source: str) -> None:
        logging.warning("Interface %s:%s %s", source, line, message)


def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument('--legacy-quality',action='store_true',help='Explicit legacy quality compatibility')
    parser.add_argument('--bench',action='store_true')
    parser.add_argument('--motion',action='store_true')
    parser.add_argument('--verify-ui',metavar='OUTPUT_DIRECTORY')
    options=parser.parse_args()
    if options.legacy_quality: os.environ['IMU_ALLOW_LEGACY_QUALITY']='1'
    if options.verify_ui:
        os.environ['QT_QPA_PLATFORM']='offscreen'
        os.environ['QTWEBENGINE_DISABLE_SANDBOX']='1'
        os.environ['QTWEBENGINE_CHROMIUM_FLAGS']='--disable-gpu'
        QSettings.setDefaultFormat(QSettings.IniFormat)
        QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, str(Path(options.verify_ui)/'settings'))
    configure_logging(options.verify_ui)
    application = QApplication([sys.argv[0]])
    application.setApplicationName("IMU Joint Viewer")
    application.setApplicationDisplayName("Đo góc IMU")
    application.setOrganizationName("OpenSourceTools")
    application.setApplicationVersion(__version__)

    window = MainWindow("bench.html" if options.bench else ("motion.html" if options.motion else "index.html"))
    window.show()
    if options.verify_ui:
        output=Path(options.verify_ui)
        def checked(value):
            state=json.loads(value or '{}')
            passed=(state.get('cards')==7 if options.motion else bool(state.get('title'))) and not state.get('errors')
            state['passed']=passed
            (output/'portable-ui.json').write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
            window.grab().save(str(output/'portable-ui.png'))
            window.bridge.shutdown()
            application.exit(0 if passed else 1)
        QTimer.singleShot(5000,lambda:window.page.runJavaScript("JSON.stringify({title:document.title,cards:document.querySelectorAll('.reading').length,errors:window.__startupErrors||[]})",0,checked))
        QTimer.singleShot(15000,lambda:application.exit(2))
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
