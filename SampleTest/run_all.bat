@echo off
setlocal
cd /d "%~dp0"
title EngineLib (physics_engine.dll) - Sample Verification Suite

echo ==============================================================================
echo        ENGINELIB (physics_engine.dll) SAMPLE TEST RUNNER
echo ==============================================================================
echo.

echo [1/3] Running Python Sample (ctypes)...
echo ------------------------------------------------------------------------------
cd python
python sample.py
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Python sample failed!
    pause
    exit /b 1
)
cd ..

echo.
echo [2/3] Compiling and Running C99 Sample (GCC)...
echo ------------------------------------------------------------------------------
cd c
gcc -O2 sample.c -L. -lphysics_engine -o sample_c.exe
if %ERRORLEVEL% neq 0 (
    echo [ERROR] C compilation failed!
    pause
    exit /b 1
)
.\sample_c.exe
if %ERRORLEVEL% neq 0 (
    echo [ERROR] C execution failed!
    pause
    exit /b 1
)
cd ..

echo.
echo [3/3] Compiling and Running C++17 Sample (G++)...
echo ------------------------------------------------------------------------------
cd cpp
g++ -std=c++17 -O2 sample.cpp -L. -lphysics_engine -o sample_cpp.exe
if %ERRORLEVEL% neq 0 (
    echo [ERROR] C++ compilation failed!
    pause
    exit /b 1
)
.\sample_cpp.exe
if %ERRORLEVEL% neq 0 (
    echo [ERROR] C++ execution failed!
    pause
    exit /b 1
)
cd ..

echo.
echo ==============================================================================
echo  ALL SAMPLES EXECUTED SUCCESSFULLY! Python, C, and C++ bindings verified.
echo ==============================================================================
pause
