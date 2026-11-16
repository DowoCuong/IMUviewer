"""Read-only inventory of old portables; compare packaged code without executing it."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT = Path('D:/imu-web-min-main')
sys.path.insert(0, str(ROOT / '.venv310/Lib/site-packages'))
from PyInstaller.archive.readers import CArchiveReader


def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def signature(c):
    if isinstance(c, types.CodeType):
        return (c.co_code.hex(), c.co_names, c.co_varnames, c.co_freevars, c.co_cellvars,
                c.co_argcount, c.co_kwonlyargcount, c.co_flags,
                tuple(signature(x) for x in c.co_consts))
    return repr(c)


def probe(exe):
    archive = CArchiveReader(str(exe))
    pyz = archive.open_embedded_archive(next(n for n in archive.toc if n.endswith('.pyz')))
    result = dict(modules=[], source_code_differences=[])
    for name in sorted(n for n in pyz.toc if n.startswith('imu_desktop')):
        code = pyz.extract(name)
        file = ROOT / (name.replace('.', '/') + '.py')
        if not file.exists():
            file = ROOT / name.replace('.', '/') / '__init__.py'
        result['modules'].append(name)
        if not file.exists() or signature(code) != signature(compile(file.read_bytes(), code.co_filename, 'exec')):
            result['source_code_differences'].append(name)
    return result


if len(sys.argv) == 3 and sys.argv[1] == '--probe':
    print(json.dumps(probe(Path(sys.argv[2]))))
else:
    entries = []
    for directory in sorted(ROOT.glob('dist*')):
        exe = directory / 'IMUJointViewer/IMUJointViewer.exe'
        if not exe.exists():
            continue
        runtime = '3.12' if (exe.parent / '_internal/python312.dll').exists() else '3.10'
        python = ('C:/ncs/toolchains/dcbdc366a1/opt/bin/python.exe' if runtime == '3.12'
                  else str(ROOT / '.venv310/Scripts/python.exe'))
        code = subprocess.run([python, str(Path(__file__).resolve()), '--probe', str(exe)],
                              capture_output=True, text=True, check=True)
        ui = exe.parent / '_internal/imu_desktop/ui'
        differences, absent = [], []
        for source in (ROOT / 'imu_desktop/ui').rglob('*'):
            if not source.is_file():
                continue
            rel = source.relative_to(ROOT / 'imu_desktop/ui')
            packaged = ui / rel
            if not packaged.exists():
                absent.append(rel.as_posix())
            elif sha(source) != sha(packaged):
                differences.append(rel.as_posix())
        entries.append(dict(path=str(directory), python=runtime, exe_sha256=sha(exe),
                            ui_differences=differences, ui_absent=absent, **json.loads(code.stdout)))
    report = dict(root=str(ROOT), packages=entries, source_is_git_checkout=(ROOT / '.git').exists())
    (Path(__file__).parent / 'inventory.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    for row in entries:
        print(Path(row['path']).name, 'code differences:', row['source_code_differences'],
              'UI differing/missing:', len(row['ui_differences']), len(row['ui_absent']))
