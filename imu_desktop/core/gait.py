"""Single-leg gait cycles from externally verified IC/TO events.

Orientation alone is not treated as a contact sensor. Event timestamps must
share the exported measurement t_ms clock (initial contact = IC, toe-off = TO).
"""
from __future__ import annotations

import bisect
import math
import statistics
from typing import Any

from .angles import COMPONENTS, JOINTS


def analyze_cycles(rows: list[dict[str, Any]], events: list[dict[str, Any]],
                   max_gap_ms: float = 30.0) -> dict[str, Any]:
    if not rows:
        raise ValueError("Chưa có góc đo")
    times = [float(row["t_ms"]) for row in rows]
    if not all(math.isfinite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Thời gian các mẫu đo phải tăng liên tục")
    parsed = [(float(e["t_ms"]), str(e["event"]).strip().upper()) for e in events]
    if any(not math.isfinite(t) or kind not in ("IC", "TO") for t, kind in parsed) or any(
            b[0] <= a[0] for a, b in zip(parsed, parsed[1:])):
        raise ValueError("Mốc IC/TO phải có thời gian hợp lệ, tăng dần và không trùng nhau")
    contacts = [t for t, kind in parsed if kind == "IC"]
    columns = [f"{joint}_{component}" for joint in JOINTS for component in COMPONENTS
               if all(f"{joint}_{component}" in row for row in rows)]
    columns = columns or list(JOINTS)
    if any(not math.isfinite(float(row[column])) for row in rows for column in columns):
        raise ValueError("Góc khớp không hợp lệ")
    cycles, rejected = [], []
    for start, end in zip(contacts, contacts[1:]):
        toe_offs = [t for t, kind in parsed if kind == "TO" and start < t < end]
        if len(toe_offs) != 1 or start < times[0] or end > times[-1]:
            rejected.append({"start_ms": start, "reason": "Thiếu dữ liệu hoặc thiếu mốc nhấc chân đã xác nhận"})
            continue
        left, right = max(0, bisect.bisect_right(times, start)-1), bisect.bisect_left(times, end)
        window = rows[left:right+1]
        if any(b["t_ms"]-a["t_ms"] > max_gap_ms for a, b in zip(window, window[1:])) or len(
                {r.get("calibration_id", 0) for r in window}) != 1:
            rejected.append({"start_ms": start, "reason": "Dữ liệu gián đoạn hoặc thay đổi hiệu chuẩn"})
            continue
        normalized = {}
        for column in columns:
            curve = []
            for percent in range(101):
                t = start+(end-start)*percent/100
                index = bisect.bisect_left(times, t)
                if times[index] == t:
                    value = float(rows[index][column])
                else:
                    fraction = (t-times[index-1])/(times[index]-times[index-1])
                    value = float(rows[index-1][column])+(float(rows[index][column])-float(rows[index-1][column]))*fraction
                if not math.isfinite(value):
                    raise ValueError("Góc khớp không hợp lệ")
                curve.append(value)
            normalized[column] = curve
        # ROM uses all measured points, including extrema between the 101 grid points.
        rom = {column: max([normalized[column][0], normalized[column][-1]]+
                          [float(r[column]) for r in window if start <= r["t_ms"] <= end])-
                       min([normalized[column][0], normalized[column][-1]]+
                          [float(r[column]) for r in window if start <= r["t_ms"] <= end])
               for column in columns}
        cycles.append({"start_ms": start, "end_ms": end, "stride_s": (end-start)/1000,
                       "stance_s": (toe_offs[0]-start)/1000, "swing_s": (end-toe_offs[0])/1000,
                       "stance_percent": (toe_offs[0]-start)/(end-start)*100,
                       "rom_deg": rom, "normalized": normalized})
    curves = {}
    for column in columns:
        curves[column] = {"mean": [statistics.mean(c["normalized"][column][i] for c in cycles)
                                    for i in range(101)],
                          "sd": [statistics.pstdev(c["normalized"][column][i] for c in cycles)
                                  for i in range(101)]} if cycles else {"mean": [], "sd": []}
    strides = [cycle["stride_s"] for cycle in cycles]
    return {"event_source": "external_IC_TO", "cycles": cycles, "count": len(cycles),
            "rejected": rejected, "phase_percent": list(range(101)), "curves": curves,
            "mean_stride_s": statistics.mean(strides) if strides else None,
            "stride_sd_s": statistics.pstdev(strides) if strides else None,
            "stride_rate_per_min": 60/statistics.mean(strides) if strides else None}
