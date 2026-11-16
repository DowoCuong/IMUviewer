"""Encoder validation without hidden offset fitting or per-cycle time warping.

python -m imu_desktop.validate trial.csv --measured knee --reference encoder
python -m imu_desktop.validate trial.csv --measured knee --reference encoder --lag-search-ms 100

CSV requires t_ms and the two angle columns, in degrees. Direct metrics are
always reported. Any fitted offset/lag is labeled diagnostic, not validation.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path


def metrics(measured: list[float], reference: list[float]) -> dict:
    if len(measured) != len(reference) or len(measured) < 2:
        raise ValueError("Need at least two paired samples of equal length")
    errors = [a-b for a, b in zip(measured, reference)]
    if not all(math.isfinite(x) for x in (*measured, *reference)):
        raise ValueError("Validation data must be finite")
    ordered = sorted(abs(e) for e in errors)
    return {"count": len(errors), "rmse_deg": math.sqrt(statistics.mean(e*e for e in errors)),
            "bias_deg": statistics.mean(errors), "error_sd_deg": statistics.pstdev(errors),
            "mae_deg": statistics.mean(ordered), "p95_abs_deg": ordered[math.ceil(.95*len(ordered))-1],
            "max_abs_deg": ordered[-1]}


def validate_csv(path: Path, measured_column: str = "knee", reference_column: str = "encoder",
                 lag_search_ms: int = 0, applied_offset_deg: float = 0,
                 applied_lag_ms: float = 0, max_gap_ms: float = 30) -> dict:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    t = [float(row["t_ms"]) for row in rows]
    measured = [float(row[measured_column]) for row in rows]
    reference = [float(row[reference_column]) for row in rows]
    if len(t) < 2 or not all(math.isfinite(x) for x in t) or any(b <= a for a, b in zip(t, t[1:])):
        raise ValueError("Timestamps must be finite and strictly increasing")
    if not all(math.isfinite(x) for x in (applied_offset_deg, applied_lag_ms, max_gap_ms)) or max_gap_ms <= 0:
        raise ValueError("Invalid validation settings")
    direct = metrics(measured, reference)

    def paired(lag: float, margin: float = 0) -> tuple[list[float], list[float]]:
        values, targets = [], []
        for timestamp, value in zip(t, measured):
            if timestamp < t[0]+margin or timestamp > t[-1]-margin:
                continue
            target_time = timestamp-lag
            index = bisect.bisect_left(t, target_time)
            if index == len(t) or target_time < t[0]:
                continue
            if t[index] == target_time:
                target = reference[index]
            elif t[index]-t[index-1] <= max_gap_ms:
                f = (target_time-t[index-1])/(t[index]-t[index-1])
                target = reference[index-1]+f*(reference[index]-reference[index-1])
            else:
                continue
            values.append(value-applied_offset_deg)
            targets.append(target)
        return values, targets

    result = {"direct": direct, "applied_offset_deg": applied_offset_deg,
              "applied_lag_ms": applied_lag_ms,
              "fixed_calibration": metrics(*paired(applied_lag_ms)),
              "diagnostic_fit": None,
              "note": "Direct metrics are uncorrected. A fit on this same trial is diagnostic only."}
    if not 0 <= lag_search_ms <= 2000:
        raise ValueError("Lag search must be 0..2000 ms")
    if lag_search_ms:
        candidates = []
        for lag in range(-lag_search_ms, lag_search_ms+1):
            a, b = paired(lag, lag_search_ms)
            if len(a) < 20:
                continue
            errors = [x-y for x, y in zip(a, b)]
            candidates.append((statistics.pstdev(errors), abs(lag), lag, a, b))
        if not candidates:
            raise ValueError("Insufficient duration for lag search")
        _, _, lag, a, b = min(candidates, key=lambda item: item[:2])
        bias = statistics.mean(x-y for x, y in zip(a, b))
        result["diagnostic_fit"] = {"lag_ms": lag, "additional_offset_deg": bias,
                                    "metrics": metrics([x-bias for x in a], b)}
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--measured", default="knee")
    parser.add_argument("--reference", default="encoder")
    parser.add_argument("--lag-search-ms", type=int, default=0)
    parser.add_argument("--offset-deg", type=float, default=0)
    parser.add_argument("--lag-ms", type=float, default=0)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    try:
        result = validate_csv(args.csv, args.measured, args.reference, args.lag_search_ms,
                              args.offset_deg, args.lag_ms)
        text = json.dumps(result, indent=2, allow_nan=False)
        if args.out:
            args.out.write_text(text+"\n", encoding="utf-8")
        print(text)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Validation failed: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
