@echo off
cd /d "%~dp0"
echo Compiling EngineLib C++ Sample with G++...
g++ -std=c++17 -O2 sample.cpp -L. -lphysics_engine -o sample_cpp.exe
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Compilation failed!
    pause
    exit /b 1
)
echo Running sample_cpp.exe...
.\sample_cpp.exe
pause
