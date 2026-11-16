"""Repeatable software verification; optional firmware source and desktop smoke."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--firmware-source", type=Path)
parser.add_argument("--with-ui", action="store_true")
args = parser.parse_args()
cases = [("desktop_pytest", [sys.executable, "-m", "pytest", "-q"], ROOT),
         ("desktop_unittest", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-q"], ROOT),
         ("performance", [sys.executable, "tools/benchmark_engine.py"], ROOT)]
if args.firmware_source:
    cases.append(("firmware_unittest", [sys.executable, "-m", "unittest", "discover", "-s", ".", "-p", "test_imu*.py", "-q"], args.firmware_source))
if args.with_ui:
    cases.append(("desktop_webchannel_smoke", [sys.executable, "tools/smoke_desktop.py"], ROOT))
    cases.append(("encoder_bench_webchannel_smoke", [sys.executable, "tools/smoke_bench.py"], ROOT))
results = []
for name, command, directory in cases:
    completed = subprocess.run(command, cwd=directory, capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=180)
    result = {"name": name, "command": command, "cwd": str(directory),
              "exit_code": completed.returncode, "stdout": completed.stdout, "stderr": completed.stderr}
    results.append(result)
    print(name, "PASS" if completed.returncode == 0 else "FAIL", flush=True)
    if completed.returncode:
        print(completed.stdout+completed.stderr, flush=True)
node = shutil.which("node")
if node:
    completed = subprocess.run([node, "--input-type=module", "--check"],
        input=(ROOT/"imu_desktop/ui/app.js").read_bytes(), capture_output=True, timeout=15)
    results.append({"name": "javascript_module_syntax", "exit_code": completed.returncode,
                    "stderr": completed.stderr.decode("utf-8", errors="replace")})
    print("javascript_module_syntax", "PASS" if completed.returncode == 0 else "FAIL", flush=True)
    completed = subprocess.run([node, "--check", "imu_desktop/ui/bench.js"], cwd=ROOT,
                               capture_output=True, timeout=15)
    results.append({"name": "encoder_bench_javascript_syntax", "exit_code": completed.returncode,
                    "stderr": completed.stderr.decode("utf-8", errors="replace")})
report = {"utc": datetime.now(timezone.utc).isoformat(), "results": results,
          "all_passed": all(r["exit_code"] == 0 for r in results),
          "accuracy_scope": "Synthetic/software checks only; no new encoder or in-vivo data acquired"}
destination = ROOT/"exports/verification/verification.json"
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
print(destination)
raise SystemExit(0 if report["all_passed"] else 1)
