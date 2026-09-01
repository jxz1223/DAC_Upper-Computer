@echo off
setlocal EnableExtensions
cd /d "%~dp0"

for %%F in (*.exe) do (
    start "" "%%~fF"
    exit /b 0
)

:missing
echo [ERROR] Sensor waveform viewer executable was not found.
echo Keep this BAT file and the Chinese EXE file in the same directory.
pause
exit /b 1
