"""Check the final portable and preserve the UI change evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path('D:/imu-web-min-main')
OUT = Path(__file__).resolve().parent
PORTABLE = ROOT/'dist/IMUJointViewer'
BACKUP = ROOT/'backups/ui-refresh-20261116/source'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


manifest = json.loads((PORTABLE/'release-manifest.json').read_text())
current = json.loads((ROOT/'CURRENT.json').read_text(encoding='utf-8-sig'))
assert manifest['version'] == current['version'] == '2.4.1'
for record in manifest['files']:
    assert sha(PORTABLE/record['path']) == record['sha256'], record['path']
for record in manifest['sources']:
    assert sha(ROOT/record['path']) == record['sha256'], record['path']
engine = list((BACKUP/'imu_desktop/core').glob('*.py'))
assert all(p.read_bytes() == (ROOT/'imu_desktop/core'/p.name).read_bytes() for p in engine)
ui = json.loads((OUT/'motion-ui/ui-smoke.json').read_text(encoding='utf-8'))
assert ui['passed'] and ui['cards'] == 7 and ui['joints'] == 3
portable_checks = json.loads((OUT/'portable-checks.json').read_text())
assert all(r['exit_code'] == 0 and r['state']['passed'] for r in portable_checks)
assert '161 passed' in (OUT/'tests.log').read_text()
for name in ('index.html', 'bench.html', 'motion.html'):
    text = (PORTABLE/'_internal/imu_desktop/ui'/name).read_text(encoding='utf-8')
    assert 'Đo định lượng yêu cầu firmware chất lượng mới' not in text
assert 'data-model-id="leg"' not in (PORTABLE/'_internal/imu_desktop/ui/index.html').read_text(encoding='utf-8')
probe = subprocess.run([sys.executable, str(ROOT/'exports/verification/viewer-consolidation-20261116/inventory.py'),
                        '--probe', str(PORTABLE/'IMUJointViewer.exe')], capture_output=True, text=True, check=True)
assert not json.loads(probe.stdout)['source_code_differences']
report = dict(version='2.4.1', tests_passed=161, portable_files_verified=len(manifest['files']),
              engine_files_unchanged=len(engine), ui=ui, portable_checks=portable_checks,
              source_backup=str(BACKUP), portable_rollback=current['previous_portable'],
              serial_opened=False, hardware_tested=False, motor_run=False,
              limitations=['Offscreen checks use the no-WebGL fallback; real GPU mannequin appearance not hardware-validated.',
                           'Mannequin rotations are illustrative and do not calculate measurement values.'])
(OUT/'verification.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k: report[k] for k in ('version', 'tests_passed', 'portable_files_verified',
      'engine_files_unchanged', 'portable_rollback')}, indent=2))
