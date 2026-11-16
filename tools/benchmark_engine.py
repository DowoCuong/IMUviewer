"""Informational acquisition callback benchmark; no hardware accuracy claim."""
import json
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.angles import JOINTS
from imu_desktop.core.math3d import IDENTITY
from imu_desktop.core.session import SessionController

results = []
for size in (1000, 10000, 100000, 500000):
    c = SessionController(lambda _: None, lambda _: None, lambda _: None)
    c._active = c._has_current = True
    c.kinematics.sensor_to_segment = {n: IDENTITY for n in range(1, 5)}
    c._functional = set(JOINTS)
    c._current_rows = [{}]*size
    durations = []
    for i in range(200):
        frame = JointFrame(i*10000, {n: IDENTITY for n in range(1, 5)}, 30, 10000, "benchmark")
        start = time.perf_counter_ns()
        c._handle_frame(frame)
        durations.append((time.perf_counter_ns()-start)/1e6)
    results.append({"history_rows": size, "median_ms": statistics.median(durations),
                    "p95_ms": sorted(durations)[189]})
    c._current_rows = []
    c.close()
output = ROOT/"exports"/"verification"/"benchmark.json"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps({"unit": "ms per synchronized frame", "results": results}, indent=2)+"\n", encoding="utf-8")
print(output.read_text())
