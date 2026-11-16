"""Two-IMU / encoder bench. USB receipt anchors are approximate, not hardware sync.

Encoder counts are the reference. Motor targets are commands, never measurements.
Zero comes from a stationary pre-trial capture; no fit to the moving trial.
"""
from __future__ import annotations

from collections import Counter, deque
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import io
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time
from typing import Callable
import zipfile

import serial

from .alignment import JointFrame
from .math3d import conjugate, distance_deg, mean_quaternion, multiply, normalize
from .serial import SerialReader
from .radio_bench import RadioBench
from ..validate import metrics

TIMING_NOTE = ('Ghép giờ gần đúng theo thời điểm máy tính nhận dữ liệu USB, cố định khi lấy mốc 0°. '
               'Chưa xác nhận lệch đồng hồ và trễ USB/BNO; chưa đồng bộ bằng phần cứng.')


@dataclass(frozen=True)
class EncoderReading:
    tick: int
    unit_us: int
    counts: tuple[int, int]
    sequence: int | None = None


def parse_encoder(line: str) -> EncoderReading | None:
    parts = line.strip().split(',')
    try:
        if len(parts) == 5 and parts[0] == 'ENC':
            seq, tick, a, b = map(int, parts[1:])
            unit = 1
        elif len(parts) == 5:
            tick = int(parts[0])
            angles = [float(x) for x in parts[1:3]]
            if not all(math.isfinite(x) and 0 <= x <= 360 for x in angles):
                return None
            a, b = map(int, parts[3:])
            seq, unit = None, 1000
        else:
            return None
        if not 0 <= tick < 2**32 or seq is not None and not 0 <= seq < 2**32:
            return None
        if any(not -(2**31) <= count < 2**31 for count in (a, b)):
            return None
        return EncoderReading(tick, unit, (a, b), seq)
    except (ValueError, OverflowError):
        return None


class ReceiptClock:
    """Lower-envelope receipt offset before zero; fixed offset during trial.

    Does not estimate absolute latency or promise a clock uncertainty bound.
    """
    def __init__(self):
        self.offset_us: float | None = None
        self.frozen = False
        self.last_us: float | None = None

    def observe(self, device_us: float, host_us: float) -> None:
        if not all(math.isfinite(x) for x in (device_us, host_us)):
            raise ValueError('Thời gian mẫu không hợp lệ')
        if self.last_us is not None and device_us <= self.last_us:
            raise ValueError('Đồng hồ thiết bị lặp lại hoặc vừa khởi động lại')
        offset = host_us-device_us
        if not self.frozen:
            self.offset_us = offset if self.offset_us is None else min(self.offset_us, offset)
        self.last_us = device_us

    def mapped(self, device_us: float) -> float:
        if self.offset_us is None:
            raise ValueError('Chưa có mốc đồng hồ')
        return device_us+self.offset_us


class LineDevice:
    """One bounded line reader for the existing Mega or Uno serial protocol."""
    def __init__(self, name: str, on_line: Callable, on_lost: Callable):
        self.name, self.on_line, self.on_lost = name, on_line, on_lost
        self.connection = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.RLock()

    def start(self, port: str, baud: int = 115200) -> None:
        self.close()
        if self.thread is not None and self.thread.is_alive():
            raise ValueError('Bộ nhận dữ liệu trước đang dừng')
        connection = serial.Serial(port, baud, timeout=.05, write_timeout=.25)
        self.connection = connection
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, args=(connection, self.stop_event), daemon=True,
                                       name=self.name+'-reader')
        self.thread.start()

    def send(self, command: str) -> None:
        with self.lock:
            if self.connection is None:
                raise ValueError(self.name+' chưa kết nối')
            data = (command+'\n').encode('ascii')
            if self.connection.write(data) != len(data):
                raise OSError('Lệnh gửi qua cổng kết nối chưa đầy đủ')

    def _run(self, connection, stop) -> None:
        buffer = bytearray()
        try:
            while not stop.is_set():
                buffer.extend(connection.read(min(max(connection.in_waiting, 1), 4096)))
                while b'\n' in buffer:
                    line, _, tail = buffer.partition(b'\n')
                    buffer = bytearray(tail)
                    if len(line) > 512:
                        continue
                    try:
                        text = line.decode('ascii').strip()
                    except UnicodeError:
                        continue
                    self.on_line(text, time.monotonic()*1e6)
                if len(buffer) > 512:
                    buffer.clear()
        except Exception as exc:
            if not stop.is_set():
                self.on_lost(self.name, str(exc))
        finally:
            connection.close()
            with self.lock:
                if self.connection is connection:
                    self.connection = None

    def close(self) -> None:
        self.stop_event.set()
        with self.lock:
            connection = self.connection
            self.connection = None
        if connection is not None:
            connection.close()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=2)


def twist_degrees(relative, neutral, axis: str) -> tuple[float, float]:
    # Left delta expresses rotation about the proximal SENSOR frame axes.
    q = normalize(multiply(relative, conjugate(neutral)))
    index = 'xyz'.index(axis)+1
    projection = q[index]
    size = math.hypot(q[0], projection)
    if size < 1e-8:
        raise ValueError('Không phân biệt được góc quanh trục đã chọn; kiểm tra lại trục quay')
    twist = [q[0]/size, 0., 0., 0.]
    twist[index] = projection/size
    angle = math.remainder(math.degrees(2*math.atan2(projection, q[0])), 360)
    return angle, distance_deg(tuple(twist), q)


class BenchController:
    def __init__(self, on_update: Callable[[dict], None]):
        self.on_update = on_update
        self.lock = threading.RLock()
        self.connected = self.recording = False
        self.motor_state = 'disconnected'
        self._closed = False
        self.options = {}
        self.rows = []
        self.raw = tempfile.TemporaryFile(mode='w+b')
        self.imu = SerialReader(self.on_imu, self._imu_status,
                                on_record=lambda r: self._log('imu', r),
                                on_health=self._radio_health,
                                on_reset=self._epoch_reset,
                                on_connection_change=self._imu_connection)
        self.encoder = LineDevice('encoder', self.on_encoder, self._lost)
        self.motor = LineDevice('motor', self.on_motor, self._lost)
        self._reset_trial()

    def _epoch_reset(self):
        with self.lock:
            self.recording=False; self.calibration=None; self.pending.clear(); self.latest=None
            self.counts['device_failures']+=1
            self.message='Fusion/clock đổi epoch: phép đo dừng, tiếp tục thu raw; tạo phiên mới và lấy mốc lại'
        self._log('epoch_reset',self.message); self._publish()

    def _reset_trial(self):
        self.sensor_identities={}
        self.radio_bench = RadioBench()
        self.rows = []
        self.calibration = None
        self.counts = Counter()
        self.imu_clock, self.encoder_clock = ReceiptClock(), ReceiptClock()
        self.pending, self.references, self.recent = deque(maxlen=500), deque(maxlen=500), deque(maxlen=500)
        self.encoder_tick = self.encoder_extended = self.encoder_unit = self.encoder_seq = None
        self.origin_us = self.last_paired_us = self.last_angle = None
        self.unwrapped_angle = 0.
        self.n = 0
        self.mean_error = self.m2 = self.square_error = self.abs_error = self.max_error = 0.
        self.latest = None
        self.message = 'Kết nối bộ thu IMU và Mega; giữ cơ cấu đứng yên trước khi lấy mốc 0°.'
        self.last_imu_receipt = self.last_encoder_receipt = None
        self.last_measurement_receipt = self.recording_started_us = None

    def _log(self, source, data):
        with self.lock:
            if self.raw.closed:
                return
            if source == 'imu':
                self.radio_bench.add(data)
                if 'sensor_epoch' in data and not data.get('epoch_stale'):
                    if not hasattr(self,'sensor_identities'): self.sensor_identities={}
                    self.sensor_identities[str(data['node'])]=[data['session'],data['sensor_epoch']]
            self.raw.seek(0, io.SEEK_END)
            self.raw.write((json.dumps({'source': source, 'host_monotonic_us': time.monotonic()*1e6,
                                        'data': data}, allow_nan=False)+'\n').encode('utf-8'))

    def _radio_health(self, health):
        with self.lock:
            self.radio_bench.decoder = dict(health.get('decoder', {}))
            snapshot = self.radio_bench.snapshot()
        self.on_update({'kind': 'radio', 'radio': snapshot})

    def snapshot(self):
        with self.lock:
            return {'kind': 'state', 'connected': self.connected, 'recording': self.recording,
                    'calibrated': self.calibration is not None and not self.counts['device_failures'], 'motor_state': self.motor_state,
                    'message': self.message, 'timing': TIMING_NOTE, 'counts': dict(self.counts),
                    'metrics': self._metrics(), 'latest': self.latest, 'options': dict(self.options),
                    'radio': self.radio_bench.snapshot()}

    def _publish(self):
        self.on_update(self.snapshot())

    def _metrics(self):
        return {'count': self.n, 'rmse_deg': math.sqrt(self.square_error/self.n) if self.n else None,
                'bias_deg': self.mean_error if self.n else None,
                'error_sd_deg': math.sqrt(max(0., self.m2/self.n)) if self.n else None,
                'mae_deg': self.abs_error/self.n if self.n else None,
                'max_abs_deg': self.max_error if self.n else None}

    def connect(self, options: dict):
        if self._closed:
            raise ValueError('Bộ điều khiển phép thử đã đóng')
        parent, child = int(options['parent']), int(options['child'])
        channel, cpr = int(options['channel']), int(options['counts_per_rev'])
        sign = int(options['sign'])
        gap = float(options.get('max_gap_ms', 30))
        ports = [str(options[k]).strip() for k in ('imu_port', 'encoder_port')]
        motor_port = str(options.get('motor_port', '')).strip()
        selected_ports = [p for p in ports+[motor_port] if p]
        if not ports[0] or len({p.casefold() for p in selected_ports}) != len(selected_ports):
            raise ValueError('Chọn các cổng riêng biệt cho bộ thu IMU, Mega và Uno')
        if motor_port and not ports[1]:
            raise ValueError('Chọn cổng encoder trước khi kết nối motor cho phép thử')
        if parent == child or parent not in range(1, 5) or child not in range(1, 5):
            raise ValueError('Chọn hai IMU khác nhau trong các số 1 đến 4')
        if channel not in (1, 2) or not 1 <= cpr <= 10000000 or sign not in (-1, 1) or options['axis'] not in 'xyz' or len(options['axis']) != 1:
            raise ValueError('Cấu hình encoder hoặc trục quay không hợp lệ')
        if not math.isfinite(gap) or not 10 <= gap <= 200:
            raise ValueError('Khoảng nội suy encoder phải từ 10 đến 200 ms')
        mode = str(options.get('protocol', 'binary'))
        if mode not in ('binary', 'quaternion', 'euler_zyx'):
            raise ValueError('Định dạng dữ liệu IMU không được hỗ trợ')
        if self.connected:
            raise ValueError('Ngắt kết nối trước khi đổi cấu hình phép thử')
        if self.rows:
            raise ValueError('Xuất và xóa phép thử trước trước khi kết nối lại')
        self.disconnect()
        with self.lock:
            self._reset_trial()
            self.raw.close()
            self.raw = tempfile.TemporaryFile(mode='w+b')
            self.options = dict(options, parent=parent, child=child, channel=channel,
                                sign=sign, counts_per_rev=cpr, max_gap_ms=gap,
                                imu_port=ports[0], encoder_port=ports[1], motor_port=motor_port)
            self.connected = True
            self.motor_state = 'starting' if motor_port else 'disconnected'
        try:
            if ports[1]:
                self.encoder.start(ports[1])
            if motor_port:
                self.motor.start(motor_port)
            self.imu.nodes, self.imu.mode = (parent, child), mode
            ok, message, _ = self.imu.start(ports[0], 115200)
            if not ok:
                raise ValueError(message)
        except Exception:
            self.disconnect()
            raise
        self.message = 'Đã kết nối. Đợi đủ dữ liệu mới, giữ yên 3 giây rồi lấy mốc 0°.'
        if not ports[1]:
            self.message = 'Đang kiểm tra đường truyền bốn node. Xem bảng bên dưới; cần thêm encoder để đo góc đối chiếu và RMSE.'
        self._log('setup', self.options)
        self._publish()

    def disconnect(self):
        with self.lock:
            self.connected = self.recording = False
        if self.motor.connection is not None:
            try:
                self.motor.send('S')
                self._log('motor_command', 'S (disconnect)')
            except (ValueError, OSError, serial.SerialException):
                pass
        self.imu.stop()
        self.encoder.close()
        self.motor.close()
        self.motor_state = 'disconnected'
        self._publish()

    def close(self):
        if self._closed:
            return
        self.disconnect()
        self.raw.close()
        self._closed = True

    def _imu_status(self, message):
        self.message = message
        self._publish()

    def _imu_connection(self, connected):
        if not connected and self.connected:
            self._lost('imu', 'Kết nối đã đóng')

    def _lost(self, source, reason):
        with self.lock:
            self.recording = False
            label = {'imu': 'IMU', 'encoder': 'Encoder', 'motor': 'Motor', 'stream': 'Dữ liệu'}.get(source, source)
            self.message = f'{label}: {reason}; đã dừng phép thử. Xuất dữ liệu rồi ngắt và kết nối lại.'
            self.counts['device_failures'] += 1
        if source == 'motor':
            self.motor_state = 'unknown'
        elif self.motor.connection is not None:
            try:
                self.motor.send('S')
                self.motor_state = 'stopping'
            except (ValueError, OSError, serial.SerialException):
                self.motor_state = 'unknown'
        self._log('failure', self.message)
        self._publish()

    def on_encoder(self, line: str, host_us: float):
        self._log('encoder', {'line': line, 'receipt_us': host_us})
        with self.lock:
            if line.startswith(('ms,', 'ZERO,')):
                if self.encoder_tick is not None:
                    self._lost('encoder', 'Encoder vừa khởi động lại hoặc bị đổi mốc đếm')
                return
            if line.startswith('ENCODER,'):
                fields = line.split(',')
                if self.encoder_tick is not None:
                    self._lost('encoder', 'Mega vừa khởi động lại hoặc đổi định dạng dữ liệu')
                elif len(fields) != 4 or fields[2] != str(self.options.get('counts_per_rev')):
                    self._lost('encoder', 'Số xung mỗi vòng không khớp với cấu hình trong Mega')
                return
            reading = parse_encoder(line)
            if reading is None:
                self.counts['encoder_nondata_lines'] += 1
                return
            if not self.connected or self.counts['device_failures']:
                return
            if self.encoder_tick is not None:
                delta = ((reading.tick-self.encoder_tick+2**31) % 2**32)-2**31
                if reading.unit_us != self.encoder_unit or delta < 0:
                    self._lost('encoder', 'Đồng hồ encoder vừa khởi động lại')
                    return
                if delta == 0:
                    self.counts['encoder_repeats'] += 1
                    return
                self.encoder_extended += delta*reading.unit_us
            else:
                self.encoder_extended = reading.tick*reading.unit_us
            self.encoder_tick, self.encoder_unit = reading.tick, reading.unit_us
            if reading.sequence is not None and self.encoder_seq is not None:
                seq_delta = ((reading.sequence-self.encoder_seq+2**31) % 2**32)-2**31
                if seq_delta <= 0:
                    self._lost('encoder', 'Số thứ tự mẫu encoder vừa bắt đầu lại')
                    return
                self.counts['encoder_sequence_gaps'] += seq_delta-1
            self.encoder_seq = reading.sequence
            self.last_encoder_receipt = host_us
            self.encoder_clock.observe(self.encoder_extended, host_us)
            self.references.append({'device_us': self.encoder_extended, 'counts': reading.counts,
                                    'host_us': host_us})
            self.counts['encoder_samples'] += 1
            self._pair()

    def on_imu(self, frame: JointFrame):
        with self.lock:
            if not self.connected or self.counts['device_failures']:
                return
            try:
                self.last_imu_receipt = time.monotonic()*1e6
                self.imu_clock.observe(frame.time_us, self.last_imu_receipt)
            except ValueError as exc:
                self._lost('imu', str(exc))
                return
            relative = normalize(multiply(conjugate(frame.orientations[self.options['parent']]),
                                           frame.orientations[self.options['child']]))
            self.pending.append({'device_us': frame.time_us, 'relative': relative,
                                 'clock_uncertainty_us': frame.clock_uncertainty_us,
                                 'imu_span_ms': frame.interpolation_gap_us/1000})
            self.counts['imu_frames'] += 1
            self._pair()

    def _pair(self):
        while self.pending and self.references:
            imu = self.pending[0]
            timestamp = self.imu_clock.mapped(imu['device_us'])
            while len(self.references) > 1 and self.encoder_clock.mapped(self.references[1]['device_us']) <= timestamp:
                self.references.popleft()
            left = self.references[0]
            a = self.encoder_clock.mapped(left['device_us'])
            if timestamp < a:
                self.pending.popleft()
                self.counts['before_reference'] += 1
                continue
            if timestamp == a:
                right, b, fraction = left, a, 0.
            elif len(self.references) < 2:
                break
            else:
                right = self.references[1]
                b = self.encoder_clock.mapped(right['device_us'])
                fraction = (timestamp-a)/(b-a)
            self.pending.popleft()
            if b-a > self.options['max_gap_ms']*1000:
                self.counts['encoder_gaps'] += 1
                continue
            if self.last_paired_us is not None and timestamp <= self.last_paired_us:
                self.counts['clock_anchor_adjustments'] += 1
                continue
            self.last_paired_us = timestamp
            channel = self.options['channel']-1
            count = left['counts'][channel]+fraction*(right['counts'][channel]-left['counts'][channel])
            reference = count*360/self.options['counts_per_rev']
            paired = dict(imu, host_time_us=timestamp, reference=reference,
                          encoder_device_us=left['device_us']+fraction*(right['device_us']-left['device_us']),
                          encoder_span_ms=(b-a)/1000, encoder_count=count)
            self.recent.append(paired)
            self.counts['paired_samples'] += 1
            self._consume(paired)

    def capture_zero(self):
        with self.lock:
            if not self.options.get('encoder_port'):
                raise ValueError('Kết nối encoder để lấy mốc và đo RMSE; hiện chỉ kiểm tra đường truyền')
            if not self.connected or self.recording or self.rows or self.counts['device_failures']:
                raise ValueError('Kết nối và lấy mốc 0° trước khi ghi phép thử mới')
            if self.motor_state not in ('idle', 'disconnected'):
                raise ValueError('Đợi Uno báo motor đã dừng trước khi lấy mốc 0°')
            if not self.recent or time.monotonic()*1e6-self.recent[-1]['host_time_us'] > 200000:
                raise ValueError('Chưa có cặp dữ liệu mới; kiểm tra IMU và tốc độ xuất dữ liệu của Mega')
            end = self.recent[-1]['host_time_us']
            frames = [p for p in self.recent if end-p['host_time_us'] <= 2500000]
            if len(frames) < 20 or end-frames[0]['host_time_us'] < 2000000:
                raise ValueError('Giữ cơ cấu đứng yên ít nhất 3 giây')
            if any(b['host_time_us']-a['host_time_us'] > 30000 for a, b in zip(frames, frames[1:])):
                raise ValueError('Dữ liệu lấy mốc bị gián đoạn; hãy dùng firmware Mega 100 Hz')
            q = mean_quaternion(p['relative'] for p in frames)
            spread = max(distance_deg(q, p['relative']) for p in frames)
            encoder_spread = max(p['reference'] for p in frames)-min(p['reference'] for p in frames)
            if spread > 1 or encoder_spread > 1:
                raise ValueError('Cơ cấu đang chuyển động; giữ yên hai IMU và encoder')
            self.calibration = {'sensor_identities':getattr(self,'sensor_identities',{}).copy(), 'relative_quaternion': q, 'encoder_zero_deg': statistics.mean(p['reference'] for p in frames),
                                'imu_motion_deg': spread, 'encoder_motion_deg': encoder_spread,
                                'utc': datetime.now(timezone.utc).isoformat()}
            self.imu_clock.frozen = self.encoder_clock.frozen = True
            self.last_angle = None
            self.unwrapped_angle = 0.
            self.message = 'Đã lấy mốc 0°. Bắt đầu ghi phép thử rồi mới cho motor quay.'
            self._log('zero_capture', self.calibration)
        self._publish()

    def check_health(self):
        with self.lock:
            if not self.recording:
                return
            now = time.monotonic()*1e6
            last_valid = max(self.recording_started_us or 0, self.last_measurement_receipt or 0)
            if any(t is None or now-t > 500000 for t in (self.last_imu_receipt, self.last_encoder_receipt, last_valid)):
                self._lost('stream', 'Không nhận đủ dữ liệu IMU/encoder mới trong 500 ms')

    def start_trial(self):
        with self.lock:
            if not self.options.get('encoder_port'):
                raise ValueError('Kết nối encoder trước khi ghi phép thử RMSE')
            if not self.connected or not self.calibration or self.counts['device_failures']:
                raise ValueError('Kết nối thiết bị và lấy mốc 0° khi đứng yên trước')
            if self.recording or self.rows:
                raise ValueError('Xuất và xóa phép thử trước trước khi bắt đầu phép thử mới')
            if not self.recent or time.monotonic()*1e6-self.recent[-1]['host_time_us'] > 200000:
                raise ValueError('Cần dữ liệu IMU và encoder mới')
            self.origin_us = None
            self.recording = True
            self.recording_started_us = time.monotonic()*1e6
            self.message = 'Đang ghi sai số trực tiếp; đồng bộ USB vẫn còn gần đúng.'
        self._publish()

    def stop_trial(self):
        self.recording = False
        if self.motor_state in ('moving', 'cycling', 'stopping'):
            self.motor_command('S', 0, 0)
        self.message = 'Đã dừng phép thử. Xuất CSV và dữ liệu gốc trước khi xóa.'
        self._publish()

    def clear_trial(self):
        if self.connected:
            raise ValueError('Ngắt kết nối trước khi xóa phép thử')
        with self.lock:
            self._reset_trial()
            self.raw.seek(0)
            self.raw.truncate()
        self._publish()

    def _consume(self, pair):
        if self.calibration is None:
            if self.counts['paired_samples'] % 10 == 0:
                self._publish()
            return
        try:
            angle, off_axis = twist_degrees(pair['relative'], self.calibration['relative_quaternion'], self.options['axis'])
        except ValueError:
            self.counts['axis_singularities'] += 1
            return
        if self.last_angle is None:
            self.unwrapped_angle = angle
        else:
            self.unwrapped_angle += math.remainder(angle-self.last_angle, 360)
        self.last_angle = angle
        measured = self.options['sign']*self.unwrapped_angle
        reference = pair['reference']-self.calibration['encoder_zero_deg']
        if self.origin_us is None:
            origin = pair['host_time_us']
        else:
            origin = self.origin_us
        row = {'t_ms': (pair['host_time_us']-origin)/1000, 'imu_deg': measured, 'encoder': reference,
               'error_deg': measured-reference, 'off_axis_deg': off_axis,
               'imu_device_us': pair['device_us'], 'encoder_device_us': pair['encoder_device_us'],
               'host_time_us': pair['host_time_us'], 'encoder_count': pair['encoder_count'],
               'encoder_span_ms': pair['encoder_span_ms'], 'imu_span_ms': pair['imu_span_ms'],
               'imu_clock_fit_us': pair['clock_uncertainty_us'], 'time_basis': 'usb_receipt_anchors_approximate'}
        self.latest = row
        self.last_measurement_receipt = time.monotonic()*1e6
        if not self.recording:
            if self.counts['paired_samples'] % 10 == 0:
                self._publish()
            return
        if self.origin_us is None:
            self.origin_us = pair['host_time_us']
        self.rows.append(row)
        error = row['error_deg']
        self.n += 1
        delta = error-self.mean_error
        self.mean_error += delta/self.n
        self.m2 += delta*(error-self.mean_error)
        self.square_error += error*error
        self.abs_error += abs(error)
        self.max_error = max(self.max_error, abs(error))
        self.on_update({'kind': 'sample', 'row': row, 'metrics': self._metrics()})

    def on_motor(self, line, host_us):
        self._log('motor', {'line': line, 'receipt_us': host_us})
        with self.lock:
            if line.startswith('READY'):
                if self.motor_state != 'starting':
                    self._lost('motor', 'Uno vừa khởi động lại; chưa xác nhận lại vị trí motor')
                    return
                self.motor_state = 'idle'
            elif line.startswith('STOPPING'):
                self.motor_state = 'stopping'
            elif line.startswith('CYCLE'):
                self.motor_state = 'cycling'
            elif line == 'STOP' or line.startswith('OK'):
                self.motor_state = 'idle'
            elif line.startswith('ERR'):
                self.motor_state = 'unknown'
            descriptions = {
                'idle': 'Motor đã dừng, có thể nhận lệnh mới.',
                'stopping': 'Motor đang dừng mềm; khi quay lặp sẽ về gốc chu kỳ.',
                'cycling': 'Motor đang quay lặp. Gửi S để về gốc rồi dừng.',
                'moving': 'Motor đang thực hiện một lần quay.',
                'unknown': 'Uno báo lỗi lệnh. Gửi S rồi đợi xác nhận dừng; phản hồi gốc được lưu trong tệp dữ liệu.'}
            self.message = descriptions.get(self.motor_state, 'Đã nhận phản hồi từ Uno.')
        self._publish()

    def motor_command(self, command: str, angle1: float, angle2: float):
        command = command.upper()
        with self.lock:
            if command not in ('M', 'C', 'S'):
                raise ValueError('Lệnh motor được hỗ trợ: M, C hoặc S')
            if command != 'S':
                if not self.connected or self.motor_state != 'idle' or self.counts['device_failures']:
                    raise ValueError('Đợi Uno báo sẵn sàng hoặc đã dừng trước khi cho motor quay')
                if not all(math.isfinite(v) and abs(v) <= 360 for v in (angle1, angle2)):
                    raise ValueError('Mỗi lệnh quay của phép thử giới hạn trong ±360°')
                if command == 'C' and angle1 == angle2 == 0:
                    raise ValueError('Cần ít nhất một motor có góc quay lặp khác 0°')
                text = f'{command} {angle1:.6f} {angle2:.6f}'
            else:
                text = 'S'
            self.motor.send(text)
            self._log('motor_command', text)
            self.motor_state = 'stopping' if command == 'S' else ('moving' if command == 'M' else 'cycling')
        self._publish()

    def export(self, destination: Path):
        with self.lock:
            if self.recording:
                raise ValueError('Dừng phép thử trước khi xuất dữ liệu')
            if not self.rows and not self.radio_bench.nodes:
                raise ValueError('Chưa có mẫu đối chiếu hoặc gói vô tuyến để xuất')
            metadata = {'strict_quality':self.imu.strict_quality,'quality_policy':__import__('imu_desktop.core.quality',fromlist=['POLICY']).POLICY,'schema': 1, 'mode': 'encoder_bench' if self.rows else 'radio_bench', 'options': self.options,
                        'radio_bench': self.radio_bench.snapshot(),
                        'calibration': self.calibration, 'counts': dict(self.counts),
                        'metrics': metrics([r['imu_deg'] for r in self.rows], [r['encoder'] for r in self.rows]) if len(self.rows) >= 2 else self._metrics(),
                        'timing': TIMING_NOTE, 'imu_offset_us': self.imu_clock.offset_us,
                        'encoder_offset_us': self.encoder_clock.offset_us,
                        'angle_method': 'Signed swing-twist about selected proximal sensor axis; continuous unwrap; stationary zero',
                        'processing': 'No smoothing, clipping, moving-trial offset/lag fit or reference-derived repair',
                        'reference': 'Encoder counts, not commanded stepper position; resolution is not accuracy'}
            contents = io.StringIO(newline='')
            writer = csv.DictWriter(contents, fieldnames=list(self.rows[0]) if self.rows else ['t_ms', 'imu_deg', 'encoder', 'error_deg'])
            writer.writeheader()
            writer.writerows(self.rows)
            text = contents.getvalue()
            raw_size = self.raw.seek(0, io.SEEK_END)
        destination = destination.resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        temps = []
        try:
            for suffix in ('.csv', '.zip'):
                with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=suffix, delete=False) as f:
                    temps.append(Path(f.name))
            temps[0].write_text(text, encoding='utf-8-sig', newline='')
            with zipfile.ZipFile(temps[1], 'w', zipfile.ZIP_DEFLATED) as archive:
                archive.writestr('paired.csv', text.encode('utf-8-sig'))
                archive.writestr('metadata.json', json.dumps(metadata, indent=2, allow_nan=False))
                archive.writestr('radio_bench.json', json.dumps(metadata['radio_bench'], indent=2, allow_nan=False))
                with archive.open('raw.jsonl', 'w') as output:
                    position = 0
                    while position < raw_size:
                        with self.lock:
                            self.raw.seek(position)
                            chunk = self.raw.read(min(raw_size-position, 1024*1024))
                        if not chunk:
                            raise OSError('Dữ liệu gốc thay đổi trong khi xuất')
                        output.write(chunk)
                        position += len(chunk)
            os.replace(temps[1], destination.with_suffix('.session.zip'))
            os.replace(temps[0], destination)
        finally:
            for path in temps:
                path.unlink(missing_ok=True)
        return metadata
