@echo off
echo [*] Copying latest physics_engine.dll...
copy /Y "..\..\PhysicsEngine\target\release\physics_engine.dll" . >nul

echo [*] Compiling main.cpp with MinGW G++...
g++ -std=c++17 main.cpp -I. -L. -lphysics_engine -O2 -o test_cpp.exe
if %ERRORLEVEL% NEQ 0 (
    echo [-] Compilation failed!
    exit /b %ERRORLEVEL%
)

echo [*] Running test_cpp.exe...
test_cpp.exe
