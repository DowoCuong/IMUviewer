from __future__ import annotations

import csv
import io
import json
import math
import os
import tempfile
import threading
import zipfile
from collections import deque
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .alignment import JointFrame
from .angles import AngleProcessor, COMPONENTS, JOINTS, JointKinematics
from .calibration import functional_calibration, neutral_calibration
from .gait import analyze_cycles
from .serial import SerialReader
from .recording import RowLog

ANGLE_COLUMNS = [f"{joint}_{component}" for joint in JOINTS for component in COMPONENTS]
CSV_COLUMNS = ["t_ms", *JOINTS, *ANGLE_COLUMNS, "calibration_id", "clock_uncertainty_us",
               "interpolation_gap_us", "sample_time_basis"]


class SessionController:
    def __init__(self, on_sample: Callable[[dict[str, Any]], None],
                 on_status: Callable[[str], None], on_summary: Callable[[dict[str, Any]], None],
                 on_connection_change: Callable[[bool], None] | None = None,
                 on_health: Callable[[dict[str, Any]], None] | None = None) -> None:
        self.processor, self.kinematics = AngleProcessor(), JointKinematics()
        self._on_sample, self._on_status, self._on_summary = on_sample, on_status, on_summary
        self._on_connection_change = on_connection_change or (lambda _connected: None)
        self._on_health = on_health or (lambda _health: None)
        self._lock = threading.RLock()
        self._current_rows = RowLog()
        self._last_rows = RowLog()
        self._raw = tempfile.TemporaryFile(mode="w+b")
        self._last_raw = None
        self._profiles: list[dict[str, Any]] = []
        self._last_profiles: list[dict[str, Any]] = []
        self._last_context: dict[str, Any] = {}
        self._events: list[dict[str, Any]] = []
        self._gait: dict[str, Any] | None = None
        self._has_current = False
        self._active = False
        self._closed = False
        self._recent: deque[JointFrame] = deque(maxlen=1000)
        self._functional_frames: list[JointFrame] = []
        self._collect_joint: str | None = None
        self._functional: set[str] = set()
        self._origin_us: float | None = None
        self._calibration_id = 0
        self._last_payload: dict[str, Any] | None = None
        self._health: dict[str, Any] = {}
        self._minima: dict[str, float] = {}
        self._maxima: dict[str, float] = {}
        self.reader = SerialReader(self._handle_frame, on_status,
            on_connection_change=self._connection_changed, on_record=self._handle_record,
            on_health=self._handle_health, on_reset=self._device_reset)

    @property
    def is_running(self) -> bool:
        return self.reader.is_running

    def start(self, port: str | None, baud: int, mode: str = "binary") -> tuple[bool, str]:
        if self._closed:
            return False, "Bộ quản lý phiên đo đã đóng"
        if mode not in ("binary", "quaternion", "euler_zyx"):
            return False, "Định dạng dữ liệu đầu vào không được hỗ trợ"
        self.stop(emit=False)
        with self._lock:
            self._raw.close()
            self._raw = tempfile.TemporaryFile(mode="w+b")
            if isinstance(self._current_rows, RowLog):
                self._current_rows.close()
            self._current_rows, self._profiles = RowLog(), []
            self._events, self._gait = [], None
            self._recent.clear()
            self._origin_us, self._last_payload = None, None
            self._health = {}
            self._calibration_id = 0
            self._minima, self._maxima = {}, {}
            self.processor.reset()
            self._invalidate_calibration()
            self.reader.mode, self.reader.latency_us = mode, {}
            self._has_current, self._active = True, True
        ok, selected, actual_baud = self.reader.start(port, baud)
        if not ok:
            with self._lock:
                self._active, self._has_current = False, False
            return False, selected
        return True, f"Đã kết nối {selected}, tốc độ {actual_baud}. Hãy lấy mốc đứng thẳng."

    def _connection_changed(self, connected: bool) -> None:
        self._on_connection_change(connected)

    def stop(self, emit: bool = True) -> dict[str, Any]:
        self.reader.stop()
        with self._lock:
            if self._has_current:
                self._last_context = self._metadata()
                if self._last_raw is not None:
                    self._last_raw.close()
                if isinstance(self._last_rows, RowLog):
                    self._last_rows.close()
                self._last_rows, self._current_rows = self._current_rows, RowLog()
                self._last_profiles, self._profiles = self._profiles, []
                self._last_raw, self._raw = self._raw, tempfile.TemporaryFile(mode="w+b")
                self._has_current = False
            self._active = False
            self._collect_joint = None
            summary = self._summary(self._last_rows)
        if emit:
            self._on_summary(summary)
            self._on_status(f"Đã dừng; giữ lại {summary['count']} mẫu đo")
        return summary

    def close(self) -> None:
        if self._closed:
            return
        self.stop(emit=False)
        self._raw.close()
        if self._last_raw is not None:
            self._last_raw.close()
        for rows in (self._current_rows, self._last_rows):
            if isinstance(rows, RowLog):
                rows.close()
        self._closed = True

    def _invalidate_calibration(self) -> None:
        with self._lock:
            self.kinematics.reset()
            self._functional.clear()
            self._recent.clear()
            self._collect_joint, self._functional_frames = None, []
            self.processor.reset()

    def _device_reset(self) -> None:
        self._profile_identities={}
        self._invalidate_calibration()
        with self._lock:
            self._active = False
        # Keep collecting raw/health; quantitative trial remains inactive.
        self._on_status("Đồng hồ hoặc thiết bị vừa khởi động lại. Bắt đầu phiên mới và hiệu chuẩn lại.")

    def capture_neutral(self) -> dict[str, Any]:
        with self._lock:
            if not self._recent:
                raise ValueError("Đợi đủ bốn cảm biến đã hiệu chuẩn và có thời gian chung")
            latest = self._recent[-1].time_us
            frames = [f for f in self._recent if f.time_us >= latest-2_500_000]
            report = neutral_calibration(self.kinematics, frames)
            self._functional.clear()
            self.processor.reset()
            self._save_profile_change(report)
        return report

    def functional_capture(self, joint: str, finish: bool = False) -> dict[str, Any]:
        with self._lock:
            if joint not in JOINTS or not self.kinematics.calibrated:
                raise ValueError("Lấy mốc đứng thẳng trước khi ghi trục gấp/duỗi")
            if not finish:
                if self._collect_joint is not None:
                    raise ValueError("Hoàn tất lần ghi trục khớp hiện tại trước")
                self._collect_joint, self._functional_frames = joint, []
                return {"collecting": joint}
            if self._collect_joint != joint:
                raise ValueError("Chưa ghi chuyển động hiệu chuẩn cho khớp này")
            frames, self._functional_frames = self._functional_frames, []
            self._collect_joint = None
            report = functional_calibration(self.kinematics, joint, frames)
            self._functional.add(joint)
            if self._functional == set(JOINTS):
                self.kinematics.calibration_kind = "functional"
            self._save_profile_change(report)
            self.processor.reset()
            return report

    def _save_profile_change(self, report: dict[str, Any]) -> None:
        self._calibration_id += 1
        self._profiles.append({"id": self._calibration_id, "utc": datetime.now(timezone.utc).isoformat(),
                               "profile": self.kinematics.profile(), "report": report,
                               "latency_us": dict(self.reader.latency_us)})

    def export_profile(self, path: Path) -> None:
        with self._lock:
            if not self.kinematics.calibrated:
                raise ValueError("Chưa có hiệu chuẩn để lưu")
            profile = self.kinematics.profile()
            profile['sensor_identities']=getattr(self,'_sensor_identities',{}).copy()
            profile["latency_us"] = {str(k): v for k, v in self.reader.latency_us.items()}
        path.write_text(json.dumps(profile, indent=2, allow_nan=False)+"\n", encoding="utf-8")

    def import_profile(self, path: Path) -> None:
        profile = json.loads(path.read_text(encoding="utf-8-sig"))
        if profile.get('sensor_identities') and getattr(self,'_sensor_identities',{}) and profile['sensor_identities']!=self._sensor_identities:
            raise ValueError('Profile thuộc epoch cảm biến khác; hiệu chuẩn lại')
        self._profile_identities=profile.get('sensor_identities',{}).copy()
        engine = JointKinematics()
        engine.load_profile(profile)
        latency = {int(k): float(v) for k, v in profile.get("latency_us", {}).items()}
        if any(k not in range(1, 5) or not math.isfinite(v) or abs(v) > 100000 for k, v in latency.items()):
            raise ValueError("Giá trị hiệu chỉnh độ trễ cố định không hợp lệ")
        with self._lock:
            if self._current_rows:
                raise ValueError("Mở tệp hiệu chuẩn trước khi bắt đầu ghi góc đo")
            self.kinematics = engine
            self._functional = set(JOINTS) if engine.calibration_kind in ("functional", "measured_profile") else set()
            self.reader.latency_us = latency
            self.reader.reset_alignment()
            self._recent.clear()
            self._origin_us = None
            self._save_profile_change({"imported": str(path)})
            self.processor.reset()

    def _handle_record(self, record: dict[str, Any]) -> None:
        if 'sensor_epoch' in record and not record.get('epoch_stale'):
            if not hasattr(self,'_sensor_identities'): self._sensor_identities={}
            self._sensor_identities[str(record['node'])]=[record['session'],record['sensor_epoch']]
            expected=getattr(self,'_profile_identities',{}).get(str(record['node']))
            if expected and expected!=self._sensor_identities[str(record['node'])]: self._device_reset()
        data = (json.dumps(record, separators=(",", ":"), allow_nan=False)+"\n").encode("utf-8")
        with self._lock:
            self._raw.seek(0, io.SEEK_END)
            self._raw.write(data)

    def _handle_health(self, health: dict[str, Any]) -> None:
        with self._lock:
            self._health = health
        self._on_health(health)

    def _handle_frame(self, frame: JointFrame) -> None:
        with self._lock:
            if not self._active:
                return
            if self._origin_us is None:
                self._origin_us = frame.time_us
            self._recent.append(frame)
            if self._collect_joint:
                if len(self._functional_frames) < 6000:
                    self._functional_frames.append(frame)
                else:
                    self._collect_joint = None
                    self._on_status("Ghi trục khớp đã quá 60 giây; hãy thực hiện lại ngắn hơn")
            try:
                angles = self.kinematics.compute(frame.orientations)
            except ValueError as exc:
                self._on_status(f"Mẫu góc khớp bị loại: {exc}")
                return
            t_ms = (frame.time_us-self._origin_us)/1000
            measured = self.kinematics.calibrated and self._functional == set(JOINTS) and self._collect_joint is None
            if measured:
                display = self.processor.process(t_ms, *(angles[j] for j in JOINTS))
                row = {"t_ms": t_ms, **angles, "calibration_id": self._calibration_id,
                       "clock_uncertainty_us": frame.clock_uncertainty_us,
                       "interpolation_gap_us": frame.interpolation_gap_us,
                       "sample_time_basis": frame.time_basis}
                self._current_rows.append(row)
                for key in ANGLE_COLUMNS+list(JOINTS):
                    self._minima[key] = min(self._minima.get(key, angles[key]), angles[key])
                    self._maxima[key] = max(self._maxima.get(key, angles[key]), angles[key])
                displayed = {joint: getattr(display, joint) for joint in JOINTS}
            else:
                displayed = {joint: angles[joint] for joint in JOINTS}
            payload = {"t": t_ms, **displayed, "components": angles, "preview": not measured,
                "sampleCount": len(self._current_rows), "calibration": self.kinematics.calibration_kind,
                "functional": sorted(self._functional), "collecting": self._collect_joint,
                "clockUncertaintyUs": frame.clock_uncertainty_us,
                "interpolationGapUs": frame.interpolation_gap_us, "timeBasis": frame.time_basis,
                **{f"max{j.title()}": self.processor.max_angles[j] for j in JOINTS},
                **{f"rom{j.title()}": self._maxima.get(j, 0)-self._minima.get(j, 0) for j in JOINTS}}
            self._last_payload = payload
        self._on_sample(payload)

    def reset_max(self) -> None:
        with self._lock:
            self.processor.reset_max_angles()
            payload = dict(self._last_payload or {})
            payload.update({f"max{j.title()}": None for j in JOINTS})
            payload["metricsOnly"] = True
        self._on_sample(payload)

    def rows_for_export(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._current_rows if self._has_current else self._last_rows or self._current_rows)

    def import_gait_events(self, path: Path) -> dict[str, Any]:
        if self._active:
            raise ValueError("Dừng đo trước khi mở các mốc chạm đất và nhấc chân")
        with path.open(newline="", encoding="utf-8-sig") as stream:
            events = list(csv.DictReader(stream))
        result = analyze_cycles(self.rows_for_export(), events)
        with self._lock:
            self._events, self._gait = events, result
            if not self._has_current:
                self._last_context.update(events=events, gait=result)
        return result

    def _metadata(self) -> dict[str, Any]:
        return {"sensor_identities":getattr(self,"_sensor_identities",{}),"strict_quality":self.reader.strict_quality, "quality_policy":__import__("imu_desktop.core.quality",fromlist=["POLICY"]).POLICY,"schema": 2, "quaternion_order": "wxyz",
                "sample_time_basis": "Mapped read completion; BNO internal fusion epoch is unverified",
                "measurement_filter_hz": 0, "display_filter_hz": self.processor.cutoff_hz,
                "joint_sequence": "ZXY", "profiles": self._profiles,
                "input_protocol": self.reader.mode, "capture_health": self._health,
                "time_origin_us": self._origin_us, "gait": self._gait, "events": self._events,
                "accuracy": "Software tests do not establish in-vivo RMSE"}

    def write_csv(self, destination: Path) -> tuple[bool, str]:
        destination = destination.resolve()
        if self.reader.is_running:
            return False, 'Dừng đo trước khi xuất dữ liệu'
        with self._lock:
            rows = self._current_rows if self._has_current else self._last_rows
            raw = self._raw if self._has_current else self._last_raw
            raw_size = raw.seek(0, io.SEEK_END) if raw is not None else 0
            metadata = json.dumps(self._metadata() if self._has_current else self._last_context,
                                  indent=2, allow_nan=False)
        if not rows and not raw_size:
            return False, "Chưa có dữ liệu để xuất."
        temporary_paths: list[Path] = []
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            for suffix in (".csv", ".zip"):
                with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=suffix, delete=False) as stream:
                    temporary_paths.append(Path(stream.name))
            with temporary_paths[0].open('w', encoding='utf-8-sig', newline='') as output:
                writer = csv.DictWriter(output, fieldnames=CSV_COLUMNS, extrasaction='ignore')
                writer.writeheader()
                for row in rows:
                    writer.writerow({key: f'{value:.6f}' if isinstance(value, float) else value
                                     for key, value in row.items()})
            with zipfile.ZipFile(temporary_paths[1], "w", zipfile.ZIP_DEFLATED) as archive:
                archive.write(temporary_paths[0], "angles.csv")
                archive.writestr("metadata.json", metadata)
                with archive.open("raw.jsonl", "w") as output:
                    position = 0
                    while position < raw_size:
                        with self._lock:
                            raw.seek(position)
                            chunk = raw.read(min(1024*1024, raw_size-position))
                        if not chunk:
                            raise OSError("Dữ liệu gốc thay đổi trong khi xuất")
                        output.write(chunk)
                        position += len(chunk)
            bundle = destination.with_suffix(".session.zip")
            os.replace(temporary_paths[1], bundle)
            os.replace(temporary_paths[0], destination)
        except (OSError, ValueError) as exc:
            return False, f"Không xuất được dữ liệu: {exc}"
        finally:
            for path in temporary_paths:
                path.unlink(missing_ok=True)
        return True, f"Đã lưu {len(rows)} mẫu đo và dữ liệu gốc vào {destination}"

    @staticmethod
    def _range(rows: list[dict[str, Any]], key: str) -> float:
        if isinstance(rows, RowLog):
            return rows.maximum.get(key, 0)-rows.minimum.get(key, 0)
        return max(row[key] for row in rows)-min(row[key] for row in rows) if rows else 0.0

    @staticmethod
    def _summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
        return {"count": len(rows), **{f"rom{j.title()}": SessionController._range(rows, j) for j in JOINTS}}

    def default_export_name(self) -> str:
        return f"imu_{datetime.now():%Y%m%d_%H%M%S}_{len(self._current_rows if self._has_current else self._last_rows)}rows.csv"
