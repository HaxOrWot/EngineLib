@echo off
cd /d "%~dp0"
echo Compiling EngineLib C Sample with GCC...
gcc -O2 sample.c -L. -lphysics_engine -o sample_c.exe
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Compilation failed!
    pause
    exit /b 1
)
echo Running sample_c.exe...
.\sample_c.exe
pause
