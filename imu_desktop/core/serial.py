from __future__ import annotations

import os
import threading
import time
from collections.abc import Callable
from dataclasses import replace
from typing import Any

import serial
from serial.tools import list_ports

from .alignment import JointFrame, SampleSynchronizer
from .protocol import StreamDecoder


def available_serial_ports() -> list[dict[str, str]]:
    return [{"device": item.device, "description": item.description or "Cổng kết nối"}
            for item in list_ports.comports()]


def auto_detect_serial_port() -> str | None:
    ports = list_ports.comports()
    markers = ("USB", "ACM", "CP210", "CH340", "UART", "SERIAL")
    for item in ports:
        if any(marker in (item.description or "").upper() for marker in markers):
            return item.device
    return ports[0].device if ports else None


class SerialReader:
    def __init__(self, on_sample: Callable[[JointFrame], None],
                 on_status: Callable[[str], None] | None = None,
                 on_connection_change: Callable[[bool], None] | None = None,
                 on_record: Callable[[dict[str, Any]], None] | None = None,
                 on_health: Callable[[dict[str, Any]], None] | None = None,
                 on_reset: Callable[[], None] | None = None,
                 sample_handler: Callable | None = None) -> None:
        self._on_sample = on_sample
        self._on_status = on_status or (lambda _message: None)
        self._on_connection_change = on_connection_change or (lambda _connected: None)
        self._on_record = on_record or (lambda _record: None)
        self._on_health = on_health or (lambda _health: None)
        self._on_reset = on_reset or (lambda: None)
        self.sample_handler = sample_handler
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self.mode = "binary"
        self.strict_quality = os.environ.get("IMU_ALLOW_LEGACY_QUALITY") != "1"
        self.nodes = (1, 2, 3, 4)
        self.latency_us: dict[int, float] = {}
        self._alignment_reset = threading.Event()

    def reset_alignment(self) -> None:
        self._alignment_reset.set()

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def start(self, port: str | None = None, baud: int = 115200) -> tuple[bool, str, int]:
        self.stop()
        if self.is_running:
            return False, "Bộ nhận dữ liệu trước chưa dừng", baud
        selected = port or os.environ.get("SERIAL_PORT") or auto_detect_serial_port()
        if not selected:
            return False, "Chưa tìm thấy cổng kết nối. Cắm bộ thu rồi làm mới danh sách.", baud
        try:
            connection = serial.Serial(selected, baud, timeout=0.05)
        except (OSError, serial.SerialException, ValueError) as exc:
            return False, f"Không mở được cổng {selected}. Kiểm tra dây cắm và đóng ứng dụng khác đang dùng cổng này. Chi tiết: {exc}", baud
        stop = threading.Event()
        with self._lock:
            self._serial, self._stop_event = connection, stop
            self._thread = threading.Thread(target=self._read_loop,
                args=(selected, connection, stop), name="imu-serial-reader", daemon=True)
            self._on_connection_change(True)
            self._thread.start()
        return True, selected, baud

    def stop(self) -> None:
        with self._lock:
            self._stop_event.set()
            connection, thread = self._serial, self._thread
        # Reads have a 50 ms timeout. Let the reader own its close before joining;
        # closing here races its finally block inside pyserial's Windows handles.
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            if self._thread is thread and (thread is None or not thread.is_alive()):
                if connection is not None:
                    try:
                        connection.close()
                    except (OSError, serial.SerialException):
                        pass
                self._serial, self._thread = None, None

    def _read_loop(self, port: str, connection: serial.Serial,
                   stop: threading.Event | None = None) -> None:
        stop = stop or self._stop_event
        decoder, synchronizer = StreamDecoder(self.mode,self.strict_quality), SampleSynchronizer(self.nodes)
        seen: dict[int, float] = {}
        trial_broken=False
        node_health={}
        last_health = 0.0
        self._on_status(f"Đang nhận dữ liệu {self.mode} từ {port}; chất lượng "+('STRICT' if self.strict_quality else 'TƯƠNG THÍCH CŨ: chất lượng chưa biết'))
        try:
            while not stop.is_set():
                if self._alignment_reset.is_set():
                    synchronizer.reset()
                    self._alignment_reset.clear()
                raw = connection.read(min(max(connection.in_waiting, 1), 4096))
                now = time.monotonic()
                for sample in decoder.feed(raw):
                    record = dict(sample.record, host_monotonic_s=now)
                    self._on_record(record)
                    if sample.record.get('type')==12 and not sample.record.get('epoch_stale'):
                        node_health[str(sample.node)]=dict(sample.record)
                        seen[sample.node]=now
                    if sample.record.get('epoch_changed'):
                        trial_broken=True; synchronizer.reset(); self._on_reset()
                        self._on_status('Fusion đổi epoch: phép đo đã dừng, tiếp tục thu raw; bắt đầu phiên mới và hiệu chuẩn lại')
                    if sample.quaternion is not None:
                        seen[sample.node] = now
                    if sample.time_us is not None:
                        sample = replace(sample, time_us=sample.time_us-self.latency_us.get(sample.node, 0.0))
                    if self.sample_handler is not None:
                        self.sample_handler(sample)
                        continue
                    if trial_broken: continue
                    generation = synchronizer.generation
                    frames = synchronizer.add(sample)
                    if synchronizer.generation != generation:
                        self._on_reset()
                        self._on_status("Thiết bị hoặc bộ thu vừa khởi động lại; cần hiệu chuẩn lại")
                    for frame in frames:
                        self._on_sample(frame)
                if now-last_health >= 0.25:
                    self._on_health({"nodes": {str(n): now-seen[n] < 0.2 if n in seen else False
                                              for n in range(1, 5)},
                        "protocol": self.mode, "quality_mode":"strict" if self.strict_quality else "legacy_explicit", "quality_nodes":node_health, "decoder": dict(decoder.counts),
                        "alignment": dict(synchronizer.counts)})
                    last_health = now
        except (OSError, serial.SerialException) as exc:
            if not stop.is_set():
                self._on_status(f"Mất kết nối thiết bị: {exc}")
        except Exception as exc:
            # A programming/callback failure must stop acquisition visibly.
            self._on_status(f"Đã dừng thu dữ liệu: {type(exc).__name__}: {exc}")
        finally:
            decoder.counts["pending_bytes_at_end"] = len(decoder.buffer)
            now = time.monotonic()
            self._on_health({"nodes": {str(n): now-seen[n] < .2 if n in seen else False for n in range(1, 5)},
                             "protocol": self.mode, "quality_mode":"strict" if self.strict_quality else "legacy_explicit", "quality_nodes":node_health, "decoder": dict(decoder.counts),
                             "alignment": dict(synchronizer.counts)})
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass
            with self._lock:
                if self._serial is connection:
                    self._serial, self._thread = None, None
            self._on_connection_change(False)
            self._on_status("Đã dừng nhận dữ liệu")
