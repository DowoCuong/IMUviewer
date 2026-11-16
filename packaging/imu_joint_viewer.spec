# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

ROOT = Path(SPECPATH).parent

analysis = Analysis(
    [str(ROOT / "run_desktop.py")],
    pathex=[str(ROOT)],
    binaries=[],
    datas=[(str(ROOT / "imu_desktop" / "ui"), "imu_desktop/ui")],
    hiddenimports=[
        "PySide6.QtWebChannel",
        "PySide6.QtWebEngineCore",
        "PySide6.QtWebEngineWidgets",
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "flask",
        "flask_login",
        "flask_socketio",
        "langchain",
        "chromadb",
        "llama_cpp",
        "sentence_transformers",
        "torch",
        "transformers",
    ],
    noarchive=False,
)

pyz = PYZ(analysis.pure)

exe = EXE(
    pyz,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="IMUJointViewer",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch="x86_64",
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    name="IMUJointViewer",
)
