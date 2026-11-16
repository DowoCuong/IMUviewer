"""Hamilton quaternions, scalar first; q_AB maps vectors from B into A.

Technical anatomical frames: x forward, y up/proximal, z right. Components
use intrinsic Z-X-Y: flexion, abduction, axial rotation.
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

Quaternion = tuple[float, float, float, float]
Vector = tuple[float, float, float]
IDENTITY: Quaternion = (1.0, 0.0, 0.0, 0.0)


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x*y for x, y in zip(a, b))


def unit(v: Sequence[float]) -> tuple[float, ...]:
    values = tuple(float(x) for x in v)
    if not values or not all(math.isfinite(x) for x in values):
        raise ValueError("Phép quay hoặc véc-tơ có giá trị không hợp lệ")
    length = math.hypot(*values)
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("Quaternion hoặc véc-tơ có độ dài bằng 0")
    return tuple(x/length for x in values)


def normalize(q: Sequence[float]) -> Quaternion:
    if len(q) != 4:
        raise ValueError("Quaternion phải gồm đủ w,x,y,z")
    return unit(q)  # type: ignore[return-value]


def conjugate(q: Quaternion) -> Quaternion:
    return (q[0], -q[1], -q[2], -q[3])


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def cross(a: Sequence[float], b: Sequence[float]) -> Vector:
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def rotate(q: Quaternion, v: Sequence[float]) -> Vector:
    return multiply(multiply(q, (0.0, *v)), conjugate(q))[1:]


def axis_angle(axis: Sequence[float], degrees: float) -> Quaternion:
    axis = unit(axis)
    half = math.radians(degrees)/2
    return (math.cos(half), *(math.sin(half)*x for x in axis))


def from_euler_zyx(yaw: float, roll: float, pitch: float) -> Quaternion:
    """Explicit legacy contract: Rz(yaw) Ry(pitch) Rx(roll), degrees."""
    return normalize(multiply(multiply(axis_angle((0, 0, 1), yaw),
                                        axis_angle((0, 1, 0), pitch)),
                              axis_angle((1, 0, 0), roll)))


def matrix(q: Quaternion) -> tuple[Vector, Vector, Vector]:
    w, x, y, z = normalize(q)
    return ((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)),
            (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)),
            (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))


def from_matrix(m: Sequence[Sequence[float]]) -> Quaternion:
    candidates = (1+m[0][0]+m[1][1]+m[2][2], 1+m[0][0]-m[1][1]-m[2][2],
                  1-m[0][0]+m[1][1]-m[2][2], 1-m[0][0]-m[1][1]+m[2][2])
    index = max(range(4), key=candidates.__getitem__)
    s = math.sqrt(max(0.0, candidates[index]))*2
    if s < 1e-12:
        raise ValueError("Ma trận quay không hợp lệ")
    if index == 0:
        q = (s/4, (m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s)
    elif index == 1:
        q = ((m[2][1]-m[1][2])/s, s/4, (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s)
    elif index == 2:
        q = ((m[0][2]-m[2][0])/s, (m[0][1]+m[1][0])/s, s/4, (m[1][2]+m[2][1])/s)
    else:
        q = ((m[1][0]-m[0][1])/s, (m[0][2]+m[2][0])/s, (m[1][2]+m[2][1])/s, s/4)
    return normalize(q)


def cardan_zxy(q: Quaternion) -> tuple[float, float, float]:
    """Signed technical angles; reject the singularity instead of inventing axes."""
    r = matrix(q)
    abduction = math.asin(max(-1.0, min(1.0, r[2][1])))
    if abs(math.cos(abduction)) < 1e-6:
        raise ValueError("Không tách được các góc khớp khi góc dạng/khép gần 90°")
    return tuple(math.degrees(x) for x in (
        math.atan2(-r[0][1], r[1][1]), abduction, math.atan2(-r[2][0], r[2][2])))


def slerp(a: Quaternion, b: Quaternion, fraction: float) -> Quaternion:
    if not 0 <= fraction <= 1:
        raise ValueError("Không thể nội suy ngoài khoảng dữ liệu đã đo")
    cosine = dot(a, b)
    if cosine < 0:
        b = tuple(-x for x in b)
        cosine = -cosine
    cosine = min(1.0, cosine)
    if cosine > 0.9995:
        return normalize(tuple(x+fraction*(y-x) for x, y in zip(a, b)))
    theta = math.acos(cosine)
    denominator = math.sin(theta)
    return normalize(tuple((math.sin((1-fraction)*theta)*x+math.sin(fraction*theta)*y)
                           / denominator for x, y in zip(a, b)))


def distance_deg(a: Quaternion, b: Quaternion) -> float:
    relative = normalize(multiply(conjugate(a), b))
    return math.degrees(2*math.atan2(math.hypot(*relative[1:]), abs(relative[0])))


def principal_axis(vectors: Iterable[Sequence[float]]) -> tuple[tuple[float, ...], float]:
    vectors = [tuple(v) for v in vectors]
    if not vectors:
        raise ValueError("Chưa có chuyển động hiệu chuẩn")
    n = len(vectors[0])
    covariance = [[sum(v[i]*v[j] for v in vectors) for j in range(n)] for i in range(n)]
    seed = max(range(n), key=lambda i: covariance[i][i])
    axis = tuple(float(i == seed) for i in range(n))
    for _ in range(80):
        candidate = unit(tuple(dot(row, axis) for row in covariance))
        if abs(dot(candidate, axis)) > 1-1e-12:
            axis = candidate
            break
        axis = candidate
    eigenvalue = dot(axis, tuple(dot(row, axis) for row in covariance))
    energy = sum(covariance[i][i] for i in range(n))
    return axis, eigenvalue/energy if energy else 0.0


def mean_quaternion(values: Iterable[Quaternion]) -> Quaternion:
    return normalize(principal_axis(values)[0])


def angular_velocity(previous: Quaternion, current: Quaternion, dt_s: float) -> Vector:
    if dt_s <= 0:
        raise ValueError("Khoảng thời gian giữa các mẫu phải lớn hơn 0")
    delta = normalize(multiply(conjugate(previous), current))
    if delta[0] < 0:
        delta = tuple(-x for x in delta)
    size = math.hypot(*delta[1:])
    if size < 1e-12:
        return (0.0, 0.0, 0.0)
    scale = 2*math.atan2(size, delta[0])/(size*dt_s)
    return tuple(x*scale for x in delta[1:])
