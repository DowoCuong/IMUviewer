"""Final read-only release audit, then write the consolidation report."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path('D:/imu-web-min-main')
OUT = Path(__file__).resolve().parent
BACKUP = ROOT/'backups/viewer-consolidation-20261116/source'
PORTABLE = ROOT/'dist/IMUJointViewer'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


manifest = json.loads((PORTABLE/'release-manifest.json').read_text())
current = json.loads((ROOT/'CURRENT.json').read_text(encoding='utf-8-sig'))
assert manifest['version'] == current['version'] == '2.4.0'
for row in manifest['files']:
    file = PORTABLE/row['path']
    assert file.stat().st_size == row['bytes'] and sha(file) == row['sha256'], file
for row in manifest['sources']:
    assert sha(ROOT/row['path']) == row['sha256'], row
engine = list((BACKUP/'imu_desktop/core').glob('*.py'))
assert engine
assert all(p.read_bytes() == (ROOT/'imu_desktop/core'/p.name).read_bytes() for p in engine)
ui = json.loads((OUT/'portable-checks.json').read_text(encoding='utf-8'))
assert all(row['exit_code'] == 0 and row['state']['passed'] for row in ui)
assert json.loads((OUT/'test-result.json').read_text())['exit_code'] == 0
assert '160 passed' in (OUT/'tests-release.log').read_text()
archives = json.loads((OUT/'archive-transfers.json').read_text(encoding='utf-8-sig'))
assert len(archives) == 7
assert [p.name for p in ROOT.glob('dist*') if p.is_dir()] == ['dist']
probe = subprocess.run([sys.executable, str(OUT/'inventory.py'), '--probe',
                        str(PORTABLE/'IMUJointViewer.exe')], capture_output=True, text=True, check=True)
modules = json.loads(probe.stdout)
assert not modules['source_code_differences']
changes = []
for before in BACKUP.rglob('*'):
    if not before.is_file():
        continue
    relative = before.relative_to(BACKUP)
    after = ROOT/relative
    if after.is_file() and sha(before) != sha(after):
        changes.append(relative.as_posix())
report = dict(version=manifest['version'], current=current, tests_passed=160,
              portable_modes_verified=[r['mode'] for r in ui],
              portable_files_verified=len(manifest['files']),
              portable_bytes=sum(row['bytes'] for row in manifest['files']),
              source_engine_files_unchanged=len(engine), packaged_code_matches_source=True,
              modules=modules['modules'], source_changes=changes,
              archived_packages=archives, backup_source=str(BACKUP),
              hardware_tested=False, serial_opened=False, flashed=False, motor_run=False,
              ui_limit='Offscreen Qt checks; WebGL may use the fallback. No hardware accuracy claim.')
(OUT/'consolidation.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k: report[k] for k in ('version', 'tests_passed', 'portable_modes_verified',
      'portable_files_verified', 'portable_bytes', 'source_engine_files_unchanged',
      'packaged_code_matches_source', 'source_changes')}, indent=2))
