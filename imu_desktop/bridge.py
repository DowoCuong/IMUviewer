from __future__ import annotations

import json
import zipfile
from collections import deque
from pathlib import Path
from typing import Any

from PySide6.QtCore import QObject, QSettings, QStandardPaths, QTimer, Signal, Slot
from PySide6.QtWidgets import QFileDialog, QWidget

from .core.serial import available_serial_ports
from .core.session import SessionController
from .core.bench import BenchController
from .core.motion import MotionSession


def operator_error(exc: Exception) -> str:
    if isinstance(exc, KeyError):
        return f"Thiếu thông tin {exc}. Kiểm tra lại tệp hoặc cấu hình đã chọn."
    if isinstance(exc, json.JSONDecodeError):
        return "Tệp cấu hình không đúng định dạng JSON. Hãy chọn lại tệp hiệu chuẩn đã lưu."
    if isinstance(exc, (TypeError, OverflowError)):
        return "Có giá trị không đúng định dạng. Kiểm tra lại các ô nhập và tệp dữ liệu."
    if isinstance(exc, OSError):
        return "Không truy cập được tệp hoặc thiết bị. Kiểm tra đường dẫn, dây cắm và ứng dụng đang dùng cổng kết nối."
    if any(text in str(exc) for text in ("invalid literal", "could not convert")):
        return "Có giá trị cấu hình không hợp lệ. Kiểm tra lại các ô nhập và tệp đã chọn."
    return str(exc)


class DesktopBridge(QObject):
    sample_received = Signal(object)
    status_received = Signal(str)
    summary_received = Signal(object)
    connection_received = Signal(bool)
    health_received = Signal(object)
    bench_received = Signal(object)
    motion_received = Signal(object)

    def __init__(self, page: Any, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._page = page
        self._settings = QSettings("OpenSourceTools", "IMUJointViewer")
        self._pending_samples: list[dict[str, Any]] = []
        self._pending_bench: list[dict[str, Any]] = []
        self._pending_motion = []
        self.motion = MotionSession(self.motion_received.emit)
        self.motion_received.connect(self._on_motion)
        self.bench = BenchController(self.bench_received.emit)
        self.bench_received.connect(self._on_bench)
        self.controller = SessionController(
            on_sample=self.sample_received.emit,
            on_status=self.status_received.emit,
            on_summary=self.summary_received.emit,
            on_connection_change=self.connection_received.emit,
            on_health=self.health_received.emit,
        )
        self.sample_received.connect(self._on_sample)
        self.status_received.connect(self._on_status)
        self.summary_received.connect(self._on_summary)
        self.connection_received.connect(self._on_connection_change)
        self.health_received.connect(self._on_health)
        self._display_timer = QTimer(self)
        self._display_timer.setInterval(33)
        self._display_timer.timeout.connect(self._flush_samples)
        self._display_timer.start()

    @Slot(result="QVariantList")
    def list_ports(self) -> list[dict[str, str]]:
        return available_serial_ports()

    @Slot(result="QVariantMap")
    def initial_settings(self) -> dict[str, Any]:
        return {
            "port": str(self._settings.value("serial/port", "")),
            "baud": int(self._settings.value("serial/baud", 115200)),
            "running": self.controller.is_running,
            "protocol": str(self._settings.value("serial/protocol", "binary")),
        }

    @Slot(str, int, str, result="QVariantMap")
    def start_session(self, port: str, baud: int, protocol: str = "binary") -> dict[str, Any]:
        if self.bench.connected or self.motion.reader.is_running:
            return {"ok": False, "message": "Ngắt kết nối phép thử encoder trước khi bắt đầu đo góc khớp."}
        selected_port = port.strip()
        try:
            baud_value = int(baud)
            if baud_value <= 0:
                raise ValueError
        except (TypeError, ValueError):
            return {"ok": False, "message": "Tốc độ truyền phải là số nguyên dương."}

        self._settings.setValue("serial/port", selected_port)
        self._settings.setValue("serial/baud", baud_value)
        self._settings.setValue("serial/protocol", protocol)
        self._pending_samples.clear()
        ok, message = self.controller.start(selected_port or None, baud_value, protocol)
        return {"ok": ok, "message": message}

    @Slot(result="QVariantMap")
    def stop_session(self) -> dict[str, Any]:
        summary = self.controller.stop()
        self._flush_samples()
        return {"ok": True, "message": "Đã dừng đo.", "summary": summary}

    @Slot(result="QVariantMap")
    def reset_maxima(self) -> dict[str, Any]:
        self.controller.reset_max()
        return {"ok": True, "message": "Đã xóa thống kê góc lớn nhất."}

    @Slot(result="QVariantMap")
    def export_csv(self) -> dict[str, Any]:
        default_name = self.controller.default_export_name()
        documents = QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation)
        destination, _selected_filter = QFileDialog.getSaveFileName(
            self._dialog_parent(),
            "Xuất dữ liệu góc khớp",
            str(Path(documents) / default_name),
            "Tệp CSV (*.csv)",
        )
        if not destination:
            return {"ok": False, "cancelled": True, "message": "Đã hủy xuất dữ liệu."}

        ok, message = self.controller.write_csv(Path(destination))
        return {"ok": ok, "message": message}

    def _dialog_parent(self) -> QWidget | None:
        return self.parent() if isinstance(self.parent(), QWidget) else None

    @Slot(result="QVariantMap")
    def capture_neutral(self) -> dict[str, Any]:
        try:
            report = self.controller.capture_neutral()
            return {"ok": True, "message": "Đã lấy mốc đứng thẳng. Tiếp tục hiệu chuẩn trục từng khớp.", "report": report}
        except (ValueError, KeyError) as exc:
            return {"ok": False, "message": operator_error(exc)}

    @Slot(str, bool, result="QVariantMap")
    def functional_calibration(self, joint: str, finish: bool) -> dict[str, Any]:
        try:
            report = self.controller.functional_capture(joint, finish)
            message = "Đã xác định trục khớp" if finish else "Bắt đầu gấp theo chiều dương, rồi gấp/duỗi lặp lại khoảng 5 giây"
            return {"ok": True, "message": message, "report": report}
        except (ValueError, KeyError) as exc:
            return {"ok": False, "message": operator_error(exc)}

    @Slot(bool, result="QVariantMap")
    def calibration_profile(self, save: bool) -> dict[str, Any]:
        dialog = QFileDialog.getSaveFileName if save else QFileDialog.getOpenFileName
        path, _ = dialog(self._dialog_parent(), "Tệp hiệu chuẩn", "", "Tệp hiệu chuẩn JSON (*.json)")
        if not path:
            return {"ok": False, "cancelled": True, "message": "Đã hủy"}
        try:
            operation = self.controller.export_profile if save else self.controller.import_profile
            operation(Path(path))
            return {"ok": True, "message": "Đã lưu tệp hiệu chuẩn" if save else "Đã mở tệp hiệu chuẩn; chỉ dùng khi cách gắn cảm biến không đổi"}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {"ok": False, "message": operator_error(exc)}

    @Slot(result="QVariantMap")
    def import_gait_events(self) -> dict[str, Any]:
        path, _ = QFileDialog.getOpenFileName(self._dialog_parent(), "Mốc chạm đất và nhấc chân đã xác nhận", "", "Tệp sự kiện CSV (*.csv)")
        if not path:
            return {"ok": False, "cancelled": True, "message": "Đã hủy"}
        try:
            result = self.controller.import_gait_events(Path(path))
            self._run_js("window.handleGait", result)
            return {"ok": True, "message": f"Đã phân tích {result['count']} chu kỳ; loại {len(result['rejected'])} chu kỳ"}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {"ok": False, "message": operator_error(exc)}

    @Slot()
    def shutdown(self) -> None:
        self._display_timer.stop()
        self.bench.close()
        self.motion.close()
        self.controller.close()

    @Slot(object)
    def _on_bench(self, payload: dict) -> None:
        self._pending_bench.append(payload)
        if len(self._pending_bench) > 512:
            self._pending_bench = self._pending_bench[-512:]

    @Slot(str, str, result="QVariantMap")
    def bench_action(self, action: str, arguments: str = "{}") -> dict:
        try:
            values = json.loads(arguments)
            if not isinstance(values, dict):
                raise ValueError("Cấu hình phép thử không đúng định dạng")
            if action == "connect":
                if self.controller.is_running or self.motion.reader.is_running:
                    raise ValueError("Dừng đo góc khớp trước khi kết nối phép thử encoder")
                self.bench.connect(values)
            elif action == "disconnect":
                if self.bench.motor_state in ('moving', 'cycling', 'stopping'):
                    raise ValueError("Gửi lệnh S và đợi motor báo dừng xong rồi mới ngắt kết nối Uno")
                self.bench.disconnect()
            elif action == "zero":
                self.bench.capture_zero()
            elif action == "start":
                self.bench.start_trial()
            elif action == "stop":
                self.bench.stop_trial()
            elif action == "clear":
                self.bench.clear_trial()
            elif action == "motor":
                self.bench.motor_command(str(values['command']), float(values.get('angle1', 0)), float(values.get('angle2', 0)))
            elif action != "state":
                raise ValueError("Thao tác phép thử không được hỗ trợ")
            return {"ok": True, "state": self.bench.snapshot()}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {"ok": False, "message": operator_error(exc)}

    @Slot(result="QVariantMap")
    def bench_export(self) -> dict:
        path, _ = QFileDialog.getSaveFileName(self._dialog_parent(), "Xuất dữ liệu đối chiếu IMU và encoder",
            str(Path(QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation))/"imu_encoder.csv"), "Tệp CSV (*.csv)")
        if not path:
            return {"ok": False, "cancelled": True, "message": "Đã hủy xuất dữ liệu"}
        try:
            metadata = self.bench.export(Path(path))
            if metadata['mode'] == 'radio_bench':
                return {"ok": True, "message": "Đã lưu dữ liệu gốc và thống kê đường truyền trong tệp .session.zip; chưa có mẫu RMSE"}
            return {"ok": True, "message": f"Đã lưu CSV đối chiếu, dữ liệu gốc và {metadata['metrics']['count']} mẫu đo"}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {"ok": False, "message": operator_error(exc)}

    @Slot(object)
    def _on_sample(self, payload: dict[str, Any]) -> None:
        if payload.get("metricsOnly"):
            self._run_js("window.handleSample", payload)
        else:
            self._pending_samples.append(payload)
            if len(self._pending_samples) > 1024:
                self._pending_samples = self._pending_samples[-1024:]

    def _flush_samples(self) -> None:
        self.bench.check_health()
        if self._pending_motion:
            batch, self._pending_motion = self._pending_motion, []
            self._run_js('window.handleMotionBatch', batch)
        if self._pending_bench:
            batch, self._pending_bench = self._pending_bench, []
            self._run_js("window.handleBenchBatch", batch)
        if self._pending_samples:
            batch, self._pending_samples = self._pending_samples, []
            self._run_js("window.handleBatch", batch)

    @Slot(str)
    def _on_status(self, message: str) -> None:
        self._run_js("window.handleStatus", message)

    @Slot(object)
    def _on_health(self, health: dict[str, Any]) -> None:
        self._run_js("window.handleHealth", health)

    @Slot(object)
    def _on_summary(self, payload: dict[str, Any]) -> None:
        self._flush_samples()
        self._run_js("window.handleSummary", payload)

    @Slot(bool)
    def _on_connection_change(self, connected: bool) -> None:
        if not connected and self.controller.is_running:
            return  # Ignore a queued disconnect from the previous reader.
        self._run_js("window.handleConnection", connected)

    def _run_js(self, function_name: str, argument: Any) -> None:
        encoded = json.dumps(argument, ensure_ascii=False, allow_nan=False)
        self._page.runJavaScript(f"typeof {function_name} === 'function' && {function_name}({encoded});")

    @Slot(object)
    def _on_motion(self, payload):
        self._pending_motion.append(payload)
        if len(self._pending_motion)>1024:
            self._pending_motion=self._pending_motion[-1024:]

    @Slot(str, str, result='QVariantMap')
    def motion_action(self, action, arguments='{}'):
        try:
            values=json.loads(arguments)
            if not isinstance(values,dict):
                raise ValueError('Cấu hình phải là đối tượng JSON')
            if action=='configure':
                self.motion.configure(values['model_id'],values['side'])
            elif action=='connect':
                if self.controller.is_running or self.bench.connected:
                    raise ValueError('Dừng màn hình gait / ngắt bench trước khi kết nối')
                self.motion.start(values.get('port'),int(values.get('baud',115200)))
            elif action=='stop':
                self.motion.stop()
            elif action=='neutral':
                return {'ok':True,'report':self.motion.capture_neutral()}
            elif action=='functional':
                return {'ok':True,'report':self.motion.functional(values['name'],values.get('finish',False),values.get('validate',False))}
            elif action in ('save_profile','load_profile','export','replay'):
                if self.motion.reader.is_running and action in ('export','replay'):
                    raise ValueError('Dừng thu nhận trước khi xuất hoặc replay')
                save=action in ('save_profile','export')
                dialog=QFileDialog.getSaveFileName if save else QFileDialog.getOpenFileName
                filt='CSV (*.csv)' if action=='export' else ('Session (*.zip)' if action=='replay' else 'JSON (*.json)')
                name='motion.csv' if action=='export' else ''
                path,_=dialog(self._dialog_parent(),'Dữ liệu đo và hiệu chuẩn',name,filt)
                if not path:
                    return {'ok':False,'cancelled':True}
                if action=='export':
                    self.motion.export(Path(path))
                elif action=='save_profile':
                    if not self.motion.engine.calibrated:
                        raise ValueError('Chưa có hiệu chuẩn để lưu')
                    Path(path).write_text(json.dumps(self.motion.profile(),indent=2,allow_nan=False),encoding='utf-8')
                elif action=='load_profile':
                    with self.motion.lock:
                        self.motion.load_profile(json.loads(Path(path).read_text(encoding='utf-8-sig')))
                else:
                    from .core.replay_motion import replay
                    replayed=replay(path)
                    self.motion.close(); self.motion=replayed
                    self.motion.emit=self.motion_received.emit
                    tail=[{k:row[k] for k in ('joint','time_us','t_ms','calibration_id','angles','quality','measured')}
                          for row in deque(self.motion.rows,maxlen=3000)]
                    return {'ok':True,'message':'Replay hoàn tất; dữ liệu phần mềm, không phải đo mới',
                            'state':self.motion.metadata(),'latest':self.motion.latest,'tail':tail}
            elif action!='state':
                raise ValueError('Thao tác không được hỗ trợ')
            return {'ok':True,'state':self.motion.metadata(),'ready':self.motion.ready(),
                    'connected':self.motion.reader.is_running}
        except (OSError,ValueError,KeyError,TypeError,zipfile.BadZipFile) as exc:
            return {'ok':False,'message':operator_error(exc)}
