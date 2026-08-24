@echo off
setlocal EnableExtensions
cd /d "%~dp0"

if not exist "SensorWaveformViewer_AC_DC_v2.exe" goto missing
start "" /D "%~dp0" "SensorWaveformViewer_AC_DC_v2.exe"
exit /b 0

:missing
echo [ERROR] SensorWaveformViewer_AC_DC_v2.exe was not found.
echo Keep the BAT and EXE files in the same directory.
pause
exit /b 1
