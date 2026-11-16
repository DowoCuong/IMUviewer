"""First-run readiness must reject unsupported Python and missing/wrong packages."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
from unittest.mock import patch

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('source_runtime_check', ROOT/'packaging/check_source_runtime.py')
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)


def test_current_pinned_environment_is_ready():
    runtime.check_runtime(ROOT/'requirements-desktop.txt')


@pytest.mark.parametrize('python_version,pointer_bytes', [((3, 9), 8), ((3, 14), 8), ((3, 12), 4)])
def test_unsupported_interpreter_is_not_reused(python_version, pointer_bytes):
    with patch.object(runtime.sys, 'version_info', python_version), patch.object(runtime.struct, 'calcsize', return_value=pointer_bytes):
        with pytest.raises(RuntimeError, match='64-bit'):
            runtime.check_interpreter()


def test_mismatched_dependency_requires_setup():
    with patch.object(runtime.importlib.metadata, 'version', return_value='0.0.0'):
        with pytest.raises(RuntimeError, match='PySide6'):
            runtime.check_runtime(ROOT/'requirements-desktop.txt')


def test_missing_dependency_requires_setup():
    with patch.object(runtime.importlib.metadata, 'version', side_effect=runtime.importlib.metadata.PackageNotFoundError('PySide6')):
        with pytest.raises(runtime.importlib.metadata.PackageNotFoundError):
            runtime.check_runtime(ROOT/'requirements-desktop.txt')


@pytest.mark.skipif(os.name != 'nt', reason='Windows first-run launcher')
def test_corrupt_bootstrap_download_is_rejected_before_execution(tmp_path):
    root = tmp_path/'fresh source & check'
    (root/'packaging').mkdir(parents=True)
    for name in ('run_source.ps1', 'check_source_runtime.py'):
        shutil.copy2(ROOT/'packaging'/name, root/'packaging'/name)
    shutil.copy2(ROOT/'requirements-desktop.txt', root/'requirements-desktop.txt')
    launcher = str(root/'packaging/run_source.ps1').replace("'", "''")
    command = """
function global:Invoke-WebRequest {
    param($Uri, $OutFile, [switch]$UseBasicParsing, $TimeoutSec)
    [IO.File]::WriteAllText($OutFile, 'corrupt archive')
}
& '""" + launcher + "'"
    powershell = Path(os.environ['SystemRoot'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
    result = subprocess.run([str(powershell), '-NoLogo', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                             '-Command', command], capture_output=True, text=True, errors='replace', timeout=30)
    assert result.returncode == 1, result.stdout + result.stderr
    assert 'SHA256' in result.stdout
    assert (root/'.runtime/uv-0.11.3.zip').read_text() == 'corrupt archive'
    assert not (root/'.runtime/uv/uv.exe').exists()
