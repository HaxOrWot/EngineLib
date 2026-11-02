@echo off
setlocal
title Physics Engine Analyser & Diagnostics Runner

echo ==============================================================================
echo     PHYSICS ENGINE AUTOMATED ANALYSER & VISUAL DIAGNOSTICS SUITE
echo ==============================================================================
echo.

cd /d "%~dp0"

echo [1/3] Checking Python and dependencies...
python -c "import pygame, PIL; print('[OK] Pygame and Pillow detected')"
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Required Python packages (pygame, pillow) are missing.
    pause
    exit /b 1
)

echo.
echo [2/3] Executing Automated Headless Physics Analysis Run...
echo Target: GameSim Demolition (Castle Impact Scenario)
echo Backend: Native physics_engine.dll (Rotational Dynamics, Torque, Friction)
echo.

python run_analysis.py --target gamesim --backend dll --frames 180

if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Analysis run encountered an error. Check traceback above.
    pause
    exit /b 1
)

echo.
echo [3/3] Opening Interactive Visual Diagnostic Report...
if exist "reports\gamesim_dll\report.html" (
    start "" "reports\gamesim_dll\report.html"
) else (
    echo Report generated at reports\gamesim_dll\report.html
)

echo.
echo ==============================================================================
echo  ANALYSIS COMPLETE! Review the browser dashboard for screenshots, replay GIF,
echo  physical anomalies, and concrete code recommendations.
echo ==============================================================================
pause
