"""Identify a portable by its version, source hashes and complete file hashes."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from imu_desktop import __version__


def entry(path, root):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return dict(path=path.relative_to(root).as_posix(), bytes=path.stat().st_size,
                sha256=digest.hexdigest())


def main():
    portable = Path(sys.argv[1]).resolve()
    for relative in ('IMUJointViewer.exe', '_internal/imu_desktop/ui/index.html',
                     '_internal/imu_desktop/ui/bench.html', '_internal/imu_desktop/ui/motion.html'):
        if not (portable / relative).is_file():
            raise SystemExit(f'Missing portable component: {relative}')
    for name in ('README.md', 'THIRD_PARTY_NOTICES.md'):
        shutil.copy2(ROOT / name, portable / name)
    sources = [p for folder in ('imu_desktop', 'packaging') for p in (ROOT / folder).rglob('*')
               if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo')]
    sources += [ROOT / p for p in ('pyproject.toml', 'run_desktop.py', 'RUN_IMU.cmd')]
    report = dict(version=__version__, created_utc=datetime.now(timezone.utc).isoformat(),
                  entry_point='IMUJointViewer.exe', modes=['gait', 'upper_lower_limb', 'encoder_radio_bench'],
                  hardware_tested=False, serial_opened=False, motor_run=False,
                  sources=[entry(p, ROOT) for p in sorted(sources)],
                  files=[entry(p, portable) for p in sorted(portable.rglob('*'))
                         if p.is_file() and p.name != 'release-manifest.json'])
    (portable / 'release-manifest.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(f'Portable {__version__}: {len(report["files"])} files, three modes, manifest written')


if __name__ == '__main__':
    main()
