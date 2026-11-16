"""Check an interpreter and the pinned desktop dependencies without opening COM."""
import importlib.metadata
from pathlib import Path
import struct
import sys


def check_interpreter():
    if not ((3, 10) <= sys.version_info[:2] < (3, 13)) or struct.calcsize('P') != 8:
        raise RuntimeError('Can Python 3.10-3.12 64-bit.')


def check_runtime(requirements):
    check_interpreter()
    for line in Path(requirements).read_text(encoding='utf-8').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        package, expected = line.strip().split('==')
        if importlib.metadata.version(package) != expected:
            raise RuntimeError('Sai phien ban thu vien: ' + package)
    # Metadata alone cannot detect missing Qt DLLs or an incomplete wheel install.
    from PySide6 import QtWebEngineWidgets  # noqa: F401
    import serial  # noqa: F401


if __name__ == '__main__':
    try:
        if sys.argv[1:] == ['--interpreter-only']:
            check_interpreter()
        else:
            check_runtime(sys.argv[1])
    except Exception as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
