# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['dac_scan_gui.py'],
    pathex=['tools'],
    binaries=[('D:\\anaconda\\envs\\dielectric_env\\Library\\bin\\tcl86t.dll', '.'), ('D:\\anaconda\\envs\\dielectric_env\\Library\\bin\\tk86t.dll', '.'), ('D:\\anaconda\\envs\\dielectric_env\\Library\\bin\\ffi.dll', '.'), ('D:\\anaconda\\envs\\dielectric_env\\Library\\bin\\libbz2.dll', '.'), ('D:\\anaconda\\envs\\dielectric_env\\Library\\bin\\libcrypto-3-x64.dll', '.'), ('D:\\anaconda\\envs\\dielectric_env\\Library\\bin\\liblzma.dll', '.')],
    datas=[],
    hiddenimports=['serial.tools.list_ports'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='DAC_Wireless_Scan_GUI_OneFile',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
