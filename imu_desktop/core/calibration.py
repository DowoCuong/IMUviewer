"""Reference-pose averaging and functional flexion-axis calibration."""
from __future__ import annotations

import math
from collections.abc import Sequence

from .alignment import JointFrame
from .angles import JOINTS, JointKinematics
from .math3d import (angular_velocity, conjugate, cross, distance_deg, dot,
                     from_matrix, mean_quaternion, principal_axis, rotate, unit)


def neutral_calibration(engine: JointKinematics, frames: Sequence[JointFrame],
                        minimum_duration_s: float = 2.0, max_motion_deg: float = 1.0) -> dict:
    if len(frames) < 20 or (frames[-1].time_us-frames[0].time_us)/1e6 < minimum_duration_s:
        raise ValueError("Giữ tư thế đứng thẳng chuẩn ít nhất 2 giây")
    if any(b.time_us-a.time_us > 30000 for a, b in zip(frames, frames[1:])):
        raise ValueError("Dữ liệu hiệu chuẩn bị gián đoạn")
    means, spreads = {}, {}
    for node in range(1, 5):
        means[node] = mean_quaternion(frame.orientations[node] for frame in frames)
        spreads[node] = max(distance_deg(means[node], frame.orientations[node]) for frame in frames)
    if max(spreads.values()) > max_motion_deg:
        raise ValueError("Chuyển động quá nhiều khi lấy mốc; giữ yên và thử lại")
    engine.calibrate_neutral(means)
    return {"duration_s": (frames[-1].time_us-frames[0].time_us)/1e6,
            "max_motion_deg": spreads, "kind": "neutral_reference"}


def functional_calibration(engine: JointKinematics, joint: str,
                           frames: Sequence[JointFrame]) -> dict:
    """Fit relative angular velocity in the proximal segment frame.

    Begin with positive flexion, then perform several flexion/extension cycles.
    The one-axis fit is a calibration maneuver, not a constraint on measured gait.
    """
    if not engine.calibrated or joint not in JOINTS:
        raise ValueError("Cần lấy mốc đứng thẳng trước")
    if len(frames) < 50 or frames[-1].time_us-frames[0].time_us < 2_000_000:
        raise ValueError("Ghi chuyển động gấp/duỗi ít nhất 2 giây")
    parent, child = JOINTS[joint]
    vectors = []
    for previous, current in zip(frames, frames[1:]):
        dt = (current.time_us-previous.time_us)/1e6
        if not 0 < dt <= 0.03:
            raise ValueError("Dữ liệu ghi trục khớp bị gián đoạn")
        a, b = engine.segments(previous.orientations), engine.segments(current.orientations)
        wp = rotate(a[parent], angular_velocity(a[parent], b[parent], dt))
        wc = rotate(a[child], angular_velocity(a[child], b[child], dt))
        relative = rotate(conjugate(a[parent]), tuple(y-x for x, y in zip(wp, wc)))
        if math.hypot(*relative) >= math.radians(10):
            vectors.append(relative)
    if len(vectors) < 30:
        raise ValueError("Khớp chuyển động chưa đủ; thực hiện vài lần gấp/duỗi rõ ràng")
    axis, concentration = principal_axis(vectors)
    if concentration < 0.95:
        raise ValueError("Chuyển động lệch khỏi một mặt phẳng quá nhiều để xác định trục")
    # The instructed first positive motion resolves the PCA sign ambiguity.
    sign = engine.signs[joint][0]
    first = vectors[:min(10, len(vectors))]
    direction = tuple(sum(v[i] for v in first)*sign for i in range(3))
    if dot(axis, direction) < 0:
        axis = tuple(-v for v in axis)
    y = (0.0, 1.0, 0.0)
    if abs(dot(axis, y)) > 0.35:
        raise ValueError("Trục gấp/duỗi không phù hợp với tư thế đứng thẳng đã lấy mốc")
    # Preserve the measured flexion axis, including its obliquity. Flattening it
    # onto the horizontal plane creates cross-talk for tilted anatomical axes.
    z = unit(axis)
    y = unit(tuple(v-dot(y, z)*w for v, w in zip(y, z)))
    x = unit(cross(y, z))
    basis = from_matrix(tuple(tuple(v[i] for v in (x, y, z)) for i in range(3)))
    engine.joint_frames[joint] = basis
    return {"joint": joint, "axis": list(axis), "concentration": concentration,
            "moving_samples": len(vectors)}
