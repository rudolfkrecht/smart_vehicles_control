@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 run_simulator.py
) else (
    python run_simulator.py
)
if errorlevel 1 (
    echo.
    echo The simulator could not start.
    echo Install Python from https://www.python.org/downloads/
    echo and select "Add Python to PATH".
    pause
)

