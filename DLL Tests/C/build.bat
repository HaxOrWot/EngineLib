@echo off
echo [*] Copying latest physics_engine.dll...
copy /Y "..\..\PhysicsEngine\target\release\physics_engine.dll" . >nul

echo [*] Compiling main.c with MinGW GCC...
gcc main.c -I. -L. -lphysics_engine -O2 -o test_c.exe
if %ERRORLEVEL% NEQ 0 (
    echo [-] Compilation failed!
    exit /b %ERRORLEVEL%
)

echo [*] Running test_c.exe...
test_c.exe
