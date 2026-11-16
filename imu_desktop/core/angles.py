"""Signed joint kinematics; numerical results never use display clamps."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from .math3d import (IDENTITY, Quaternion, axis_angle, cardan_zxy, conjugate,
                     multiply, normalize)

JOINTS = {"hip": (1, 2), "knee": (2, 3), "ankle": (3, 4)}
COMPONENTS = ("flexion", "abduction", "rotation")


def norm_deg(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("Góc phải là số hữu hạn")
    result = math.remainder(value, 360.0)
    return math.copysign(180.0, value) if abs(result) == 180 else result


def clamp(value: float, lower: float, upper: float) -> float:
    if not all(math.isfinite(v) for v in (value, lower, upper)) or lower > upper:
        raise ValueError("Giới hạn góc không hợp lệ")
    return max(lower, min(upper, value))


def parse_serial_line(line: str) -> tuple[str, int, float, float, float, float] | None:
    """Strict legacy Euler parser; empty fields must never shift columns."""
    parts = [part.strip() for part in line.strip().split(",")]
    if len(parts) != 6 or parts[0].upper() != "IMU" or any(not p for p in parts):
        return None
    try:
        node = int(parts[1])
        timestamp, yaw, roll, pitch = map(float, parts[2:])
        if node not in range(1, 5) or timestamp < 0 or not all(
                math.isfinite(v) for v in (timestamp, yaw, roll, pitch)):
            return None
        return ("imu", node, timestamp, yaw, roll, pitch)
    except (ValueError, OverflowError):
        return None


def relative_angles(sensor_rolls: Mapping[int, float], sensor2_pitch: float = 0.0
                    ) -> tuple[float, float, float, float]:
    """Explicit planar helper; acquisition uses quaternion relative orientations."""
    if any(node not in sensor_rolls for node in range(1, 5)):
        raise ValueError("Cần đủ bốn cảm biến")
    return (*(norm_deg(sensor_rolls[b]-sensor_rolls[a]) for a, b in JOINTS.values()),
            sensor2_pitch)


class JointKinematics:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.sensor_to_segment: dict[int, Quaternion] = {}
        self.joint_frames = {joint: IDENTITY for joint in JOINTS}
        self.signs = {"hip": (1, 1, 1), "knee": (-1, 1, 1), "ankle": (1, 1, 1)}
        self.calibration_kind = "uncalibrated"

    @property
    def calibrated(self) -> bool:
        return all(node in self.sensor_to_segment for node in range(1, 5))

    def calibrate_neutral(self, sensors: Mapping[int, Quaternion]) -> None:
        """Known upright neutral pose; functional alignment still identifies heading."""
        if set(sensors) != {1, 2, 3, 4}:
            raise ValueError("Lấy mốc đứng thẳng cần đủ bốn cảm biến")
        upright = axis_angle((1, 0, 0), 90)
        self.sensor_to_segment = {
            node: normalize(multiply(conjugate(normalize(q)), upright))
            for node, q in sensors.items()}
        self.joint_frames = {joint: IDENTITY for joint in JOINTS}
        self.calibration_kind = "neutral_reference"

    def segments(self, sensors: Mapping[int, Quaternion]) -> dict[int, Quaternion]:
        if set(sensors) != {1, 2, 3, 4}:
            raise ValueError("Thiếu hướng của đoạn cơ thể")
        return {node: normalize(multiply(normalize(q), self.sensor_to_segment.get(node, IDENTITY)))
                for node, q in sensors.items()}

    def compute(self, sensors: Mapping[int, Quaternion]) -> dict[str, float]:
        segments = self.segments(sensors)
        result = {}
        for joint, (parent, child) in JOINTS.items():
            relative = normalize(multiply(conjugate(segments[parent]), segments[child]))
            basis = self.joint_frames[joint]
            relative = normalize(multiply(multiply(conjugate(basis), relative), basis))
            values = cardan_zxy(relative)
            for component, value, sign in zip(COMPONENTS, values, self.signs[joint]):
                result[f"{joint}_{component}"] = sign*value
            result[joint] = result[f"{joint}_flexion"]
        return result

    def profile(self) -> dict[str, Any]:
        return {"schema": 1, "quaternion_order": "wxyz", "sequence": "ZXY",
                "axes": "x anterior, y superior, z right", "kind": self.calibration_kind,
                "sensor_to_segment": {str(k): list(v) for k, v in self.sensor_to_segment.items()},
                "joint_frames": {k: list(v) for k, v in self.joint_frames.items()},
                "signs": {k: list(v) for k, v in self.signs.items()}}

    def load_profile(self, profile: dict[str, Any]) -> None:
        if profile.get("schema") != 1 or profile.get("quaternion_order") != "wxyz" or profile.get("sequence") != "ZXY":
            raise ValueError("Tệp hiệu chuẩn không được hỗ trợ")
        corrections = {int(k): normalize(v) for k, v in profile["sensor_to_segment"].items()}
        if set(corrections) != {1, 2, 3, 4}:
            raise ValueError("Tệp hiệu chuẩn phải có cảm biến 1 đến 4")
        bases = {joint: normalize(profile["joint_frames"][joint]) for joint in JOINTS}
        signs = {joint: tuple(profile["signs"][joint]) for joint in JOINTS}
        if any(len(s) != 3 or any(v not in (-1, 1) for v in s) for s in signs.values()):
            raise ValueError("Chiều góc mỗi khớp phải gồm ba giá trị +1 hoặc −1")
        self.sensor_to_segment, self.joint_frames, self.signs = corrections, bases, signs
        self.calibration_kind = str(profile.get("kind", "measured_profile"))


@dataclass(frozen=True)
class ProcessedSample:
    t_ms: float
    hip: float
    knee: float
    ankle: float
    max_hip: float
    max_knee: float
    max_ankle: float


class AngleProcessor:
    """Optional time-based display filter; raw signed values remain available."""
    def __init__(self, cutoff_hz: float = 0.0, reset_gap_ms: float = 100.0) -> None:
        if not math.isfinite(cutoff_hz) or cutoff_hz < 0 or not math.isfinite(reset_gap_ms) or reset_gap_ms <= 0:
            raise ValueError("Tần số lọc không hợp lệ")
        self.cutoff_hz, self.reset_gap_ms = cutoff_hz, reset_gap_ms
        self.reset()

    def reset(self) -> None:
        self._last: dict[str, float] = {}
        self._time: float | None = None
        self.reset_max_angles()

    def reset_max_angles(self) -> None:
        self.max_angles: dict[str, float | None] = {joint: None for joint in JOINTS}

    def process(self, t_ms: float, raw_hip: float, raw_knee: float,
                raw_ankle: float, pitch2: float = 0.0) -> ProcessedSample:
        values = dict(zip(JOINTS, (raw_hip, raw_knee, raw_ankle)))
        if not all(math.isfinite(v) for v in (t_ms, *values.values())):
            raise ValueError("Mẫu đo có giá trị không hợp lệ")
        if self._time is not None and t_ms <= self._time:
            raise ValueError("Thời gian các mẫu phải tăng liên tục")
        dt = t_ms-self._time if self._time is not None else None
        alpha = 1.0 if not self.cutoff_hz or dt is None or dt > self.reset_gap_ms else -math.expm1(-2*math.pi*self.cutoff_hz*dt/1000)
        output = {}
        for joint, value in values.items():
            previous = self._last.get(joint, value)
            output[joint] = value if alpha == 1.0 else previous+alpha*norm_deg(value-previous)
            old_max = self.max_angles[joint]
            self.max_angles[joint] = value if old_max is None else max(old_max, value)
        self._last, self._time = output, t_ms
        return ProcessedSample(t_ms, *output.values(), *(self.max_angles[j] for j in JOINTS))

    def snapshot(self) -> dict[str, Any]:
        return {**{f"smooth{j.title()}": self._last.get(j) for j in JOINTS},
                **{f"max{j.title()}": self.max_angles[j] for j in JOINTS}}
