# Master DLL Test Suite Runner for Python, C, and C++
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "   Master Physics Engine DLL Test Runner" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan

$baseDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# 1. Python Test
Write-Host "`n>>> Running Python DLL Test..." -ForegroundColor Yellow
Push-Location "$baseDir\Python"
python test_physics.py
if ($LASTEXITCODE -ne 0) {
    Write-Host "[-] Python test failed!" -ForegroundColor Red
} else {
    Write-Host "[+] Python test passed." -ForegroundColor Green
}
Pop-Location

# 2. C Test
Write-Host "`n>>> Compiling & Running C DLL Test..." -ForegroundColor Yellow
Push-Location "$baseDir\C"
cmd /c build.bat
if ($LASTEXITCODE -ne 0) {
    Write-Host "[-] C test failed!" -ForegroundColor Red
} else {
    Write-Host "[+] C test passed." -ForegroundColor Green
}
Pop-Location

# 3. C++ Test
Write-Host "`n>>> Compiling & Running C++ DLL Test..." -ForegroundColor Yellow
Push-Location "$baseDir\C++"
cmd /c build.bat
if ($LASTEXITCODE -ne 0) {
    Write-Host "[-] C++ test failed!" -ForegroundColor Red
} else {
    Write-Host "[+] C++ test passed." -ForegroundColor Green
}
Pop-Location

Write-Host "`n=================================================================" -ForegroundColor Cyan
Write-Host "   All Test Suites Completed!" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
