"""Bounded stream decoder for IMU1 firmware and explicit text protocols.

TIMED_DATA (type 9) retains local read-completion time and appends the
node-to-hub affine clock fit. This maps clock domains, not BNO fusion epochs.
"""
from __future__ import annotations

import math
import struct
import zlib
from collections import Counter
from dataclasses import dataclass
from typing import Any

from .angles import parse_serial_line
from .math3d import Quaternion, from_euler_zyx, normalize

MAGIC = b"IMU1"
LENGTHS = {1: 40, 2: 40, 3: 32, 4: 32, 5: 40, 6: 64, 7: 64, 8: 24, 9: 64, 10: 64, 11: 64, 12: 64}
MASK32 = 0xffffffff


def signed_delta(value: int, anchor: int) -> int:
    return ((value-anchor+0x80000000) & MASK32)-0x80000000


@dataclass(frozen=True)
class SensorSample:
    node: int
    time_us: float | None
    quaternion: Quaternion | None
    sequence: int
    session: int
    domain: tuple[str, int]
    usable: bool
    clock_uncertainty_us: float
    record: dict[str, Any]


def kind_legacy(sample):
    return sample.record.get('type') in (1,2,9) or sample.record.get('protocol')!='binary'

def encode_frame(payload: bytes, event_us: int = 0) -> bytes:
    body = struct.pack("<HI", len(payload), event_us & MASK32)+payload
    return MAGIC+body+struct.pack("<I", zlib.crc32(body) & MASK32)


class StreamDecoder:
    def __init__(self, mode: str = "binary", strict_quality: bool = False) -> None:
        if mode not in ("binary", "quaternion", "euler_zyx"):
            raise ValueError("Định dạng dữ liệu đầu vào không được hỗ trợ")
        self.mode = mode
        self.strict_quality = strict_quality
        from .quality import EpochTracker
        self.epochs = EpochTracker()
        self.buffer = bytearray()
        self.counts: Counter[str] = Counter()
        self.hub_high_water: dict[int, int] = {}

    def feed(self, data: bytes) -> list[SensorSample]:
        self.buffer.extend(data)
        result = []
        if self.mode != "binary":
            while b"\n" in self.buffer:
                line, _, tail = self.buffer.partition(b"\n")
                self.buffer = bytearray(tail)
                try:
                    result.append(self._text(line.decode("ascii")))
                except (ValueError, UnicodeError, OverflowError):
                    self.counts["invalid_records"] += 1
            if len(self.buffer) > 512:
                self.counts["oversize_lines"] += 1
                self.buffer.clear()
            return result
        while self.buffer:
            position = self.buffer.find(MAGIC)
            if position < 0:
                keep = next((n for n in (3, 2, 1) if self.buffer.endswith(MAGIC[:n])), 0)
                self.counts["discarded_bytes"] += len(self.buffer)-keep
                self.buffer = self.buffer[-keep:] if keep else bytearray()
                break
            if position:
                self.counts["discarded_bytes"] += position
                del self.buffer[:position]
            if len(self.buffer) < 10:
                break
            length, event = struct.unpack_from("<HI", self.buffer, 4)
            if not 16 <= length <= 64:
                self.counts["invalid_lengths"] += 1
                del self.buffer[0]
                continue
            total = length+14
            if len(self.buffer) < total:
                break
            expected, = struct.unpack_from("<I", self.buffer, 10+length)
            if zlib.crc32(self.buffer[4:10+length]) & MASK32 != expected:
                self.counts["crc_errors"] += 1
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[10:10+length])
            del self.buffer[:total]
            try:
                sample=self._binary(payload,event)
                changed,stale=self.epochs.observe(sample)
                sample.record.update(epoch_changed=changed,epoch_stale=stale)
                if stale:
                    from dataclasses import replace
                    sample=replace(sample,usable=False)
                    sample.record['quality_reason']='old_epoch'
                if self.strict_quality and kind_legacy(sample):
                    from dataclasses import replace
                    sample=replace(sample,usable=False)
                    sample.record['quality_reason']='legacy_quality_unknown'
                result.append(sample)
            except (ValueError, struct.error, OverflowError):
                self.counts["invalid_records"] += 1
        return result

    def _binary(self, payload: bytes, event: int) -> SensorSample:
        version, kind, node, flags, session, tx, sequence = struct.unpack_from("<BBBBIII", payload)
        if version != 1 or LENGTHS.get(kind) != len(payload) or flags & 0xf0 or node > 4:
            raise ValueError("Phần đầu gói dữ liệu nhị phân không hợp lệ")
        record: dict[str, Any] = dict(protocol="binary", type=kind, node=node,
            flags=flags, session=session, tx_seq=tx, read_seq=sequence,
            receiver_event_us_low32=event, payload_hex=payload.hex())
        if kind == 10:
            generated, local_slot, hub_slot, tx_start, frame = struct.unpack_from('<Q4I', payload, 16)
            local, hub, ppm, uncertainty, hub_session, marker = struct.unpack_from('<IIiIII', payload, 40)
            if node not in (3, 4) or flags != 2 or sequence or marker != 0x54534554 or abs(ppm) > 200 or uncertainty > 50:
                raise ValueError('Gói tạo tải vô tuyến không hợp lệ')
            record.update(synthetic=True, generated_us=generated, scheduled_local_us=local_slot,
                          scheduled_hub_us=hub_slot, tx_start_local_us=tx_start, frame_hub_us=frame,
                          local_anchor_us=local, hub_anchor_us=hub, clock_ppm=ppm,
                          clock_uncertainty_us=uncertainty, hub_session=hub_session)
            # Transport-only: even selecting node 3/4 cannot produce an angle.
            return SensorSample(node, None, None, sequence, session, ('radio_test', hub_session), False, uncertainty, record)
        if kind==12:
            stamp,epoch,revision,health,reason,cal,state,ca,ha,sys_status,sys_error,mode,page=struct.unpack_from('<QIH10B',payload,16)
            if (flags!=2 if node else flags!=0 or page not in (2,3)) or health>2 or reason>4 or state>3 or page>3:
                raise ValueError('Invalid health packet')
            record.update(sensor_epoch=epoch,sw_revision=revision,health=health,health_reason=reason,
                          calibration=cal,state=state,cal_age_ms=ca,health_age_ms=ha,sys_status=sys_status,
                          sys_error=sys_error,opr_mode=mode,health_page=page,health_time_local_us=stamp,
                          health_counters=list(struct.unpack_from('<6I',payload,40)),capability='quality_v1')
            if node==0:
                record.pop('sensor_epoch',None)
                record['hub_health']=True
            return SensorSample(node,None,None,sequence,session,('metadata',0),False,math.inf,record)
        if kind not in (1, 2, 9, 11):
            return SensorSample(node, None, None, sequence, session, ("metadata", 0), False, math.inf, record)
        if node not in range(1, 5) or (kind!=11 and bool(flags & 1) != (kind != 2)) or payload[33] > 3 or (kind!=11 and any(payload[34:36])):
            raise ValueError("Trạng thái mẫu đo không hợp lệ")
        completed, w, x, y, z, calibration, state, age = struct.unpack_from("<Q4hBB2xI", payload, 16)
        record.update(completed_us=completed, qw=w, qx=x, qy=y, qz=z,
                      calibration=calibration, state=state, beacon_age_us=age)
        quaternion = None
        if flags & 1:
            raw = tuple(value/16384 for value in (w, x, y, z))
            if not 0.8 <= math.hypot(*raw) <= 1.2:
                raise ValueError("Độ dài quaternion BNO055 không hợp lệ")
            quaternion = normalize(raw)
        time_us, uncertainty = None, math.inf
        domain = ("unmapped", session)
        if kind in (9,11):
            local_anchor, hub_anchor, ppm, uncertainty, hub_session, duration = struct.unpack_from("<IIiIII", payload, 40)
            if not flags & 2 or abs(ppm) > 200 or uncertainty > 50 or duration > 30000:
                raise ValueError("Thông tin quy đổi đồng hồ không hợp lệ")
            previous = self.hub_high_water.get(hub_session, hub_anchor)
            hub_extended = previous+signed_delta(hub_anchor, previous & MASK32)
            self.hub_high_water[hub_session] = max(previous, hub_extended)
            local_delta = signed_delta(completed & MASK32, local_anchor) if flags&1 else 0
            if abs(local_delta) > 120000:
                raise ValueError("Thông tin quy đổi đồng hồ đã quá cũ")
            time_us = hub_extended+local_delta/(1+ppm/1_000_000) if flags&1 else None
            domain = ("hub_read_completion", hub_session)
            record.update(local_anchor_us=local_anchor, hub_anchor_us=hub_anchor,
                          clock_ppm=ppm, clock_uncertainty_us=uncertainty,
                          hub_session=hub_session, i2c_duration_us=duration)
        usable = bool(flags & 1) and not bool(flags & (4 | 8)) and state == 2 and calibration == 255
        if kind==11:
            ca,ha=payload[34:36]
            record.pop('beacon_age_us',None)
            record.update(sensor_epoch=age,cal_age_ms=ca,health_age_ms=ha,capability='quality_v1')
            # Conservative extra age until hub RX: includes the pre-serialization
            # sample wait, so it cannot make quality appear younger.
            transit=max(0.,signed_delta(event,hub_anchor)-local_delta/(1+ppm/1_000_000))/1000 if time_us is not None else 0.
            record['quality_to_hub_upper_ms']=transit
            usable=usable and ca<=100 and ha<=100 and max(ca,ha)+transit<=100
            record['quality_reason']='good' if usable else ('quality_expired_or_unknown' if ca>100 or ha>100 or max(ca,ha)+transit>100 else 'sensor_not_ready')
        elif kind in (1,2,9):
            record.update(capability='legacy',quality_reason='legacy_quality_unknown')
        record["mapped_time_us"] = time_us
        return SensorSample(node, time_us, quaternion, sequence, session, domain, usable, uncertainty, record)

    def _text(self, line: str) -> SensorSample:
        if self.mode == "euler_zyx":
            parsed = parse_serial_line(line)
            if parsed is None:
                raise ValueError("Dữ liệu Euler không hợp lệ")
            _, node, timestamp, yaw, roll, pitch = parsed
            q = from_euler_zyx(yaw, roll, pitch)
        else:
            parts = [p.strip() for p in line.strip().split(",")]
            if len(parts) != 7 or parts[0].upper() != "QUAT" or not all(parts):
                raise ValueError("Dữ liệu cần có dạng QUAT,node,t_ms,w,x,y,z")
            node = int(parts[1])
            timestamp, *values = map(float, parts[2:])
            if node not in range(1, 5) or not math.isfinite(timestamp) or timestamp < 0:
                raise ValueError("Thời gian hoặc mã cảm biến không hợp lệ")
            if not 0.8 <= math.hypot(*values) <= 1.2:
                raise ValueError("Quaternion dạng chữ phải có độ dài gần bằng 1")
            q = normalize(values)
        record = dict(protocol=self.mode, node=node, completed_us=timestamp*1000,
                      qw=q[0], qx=q[1], qy=q[2], qz=q[3], raw_line=line.strip())
        # Text contract explicitly requires a shared acquisition clock.
        return SensorSample(node, timestamp*1000, q, int(round(timestamp*1000)), 0,
                            ("text_shared_clock", 0), not self.strict_quality, 0.0, dict(record,quality_reason="legacy_quality_unknown"))
