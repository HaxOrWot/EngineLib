# Universal Physics Engine DLL Test Suite

This directory contains standalone integration tests in **Python**, **C**, and **C++** validating the native dynamic library `physics_engine.dll` across multiple host language environments.

---

## 📁 Directory Structure

```text
Z:\VSCode\GameLib\DLL Tests/
├── README.md                      # This guide
├── run_all_tests.ps1              # Master PowerShell script running all 3 language test suites
│
├── Python/                        # Python ctypes test harness
│   ├── test_physics.py            # Bouncing box test with ASCII trajectory graph + mid-air collision
│   └── physics_engine.dll         # Local copy of the compiled DLL
│
├── C/                             # Native C test harness
│   ├── physics_engine.h           # C-ABI header definitions
│   ├── main.c                     # Gravity, collision, and rebound test
│   ├── build.bat                  # Compiles with MinGW GCC and runs test_c.exe
│   ├── physics_engine.dll         # Local copy of the compiled DLL
│   └── test_c.exe                 # Compiled binary
│
└── C++/                           # Modern C++ (RAII) test harness
    ├── physics_engine.hpp         # RAII Physics::World class wrapper
    ├── main.cpp                   # Multi-body concurrent drop test
    ├── build.bat                  # Compiles with MinGW G++ and runs test_cpp.exe
    ├── physics_engine.dll         # Local copy of the compiled DLL
    └── test_cpp.exe               # Compiled binary
```

---

## 🚀 How to Run the Tests

### Option A: Run All Tests at Once
From PowerShell:
```powershell
Set-Location "Z:\VSCode\GameLib\DLL Tests"
.\run_all_tests.ps1
```

---

### Option B: Run Individual Language Tests

#### 1. Python (`Python/`)
```powershell
cd "Z:\VSCode\GameLib\DLL Tests\Python"
python test_physics.py
```
- **Features Tested**:
  - Dynamic DLL loading via `ctypes.CDLL`.
  - Earth gravity freefall and elastic floor rebound ($e = 0.85$).
  - Full ASCII trajectory plot drawn directly in the terminal.
  - Head-on mid-air collision between two moving bodies with equal and opposite momentum.

---

#### 2. C (`C/`)
```powershell
cd "Z:\VSCode\GameLib\DLL Tests\C"
.\build.bat
```
- **Features Tested**:
  - Direct linking against `physics_engine.dll` with MinGW `gcc`.
  - Precision C struct layout and pointer validation.
  - 120-step simulation tracking numerical convergence and lowest penetration depth.

---

#### 3. C++ (`C++/`)
```powershell
cd "Z:\VSCode\GameLib\DLL Tests\C++"
.\build.bat
```
- **Features Tested**:
  - Modern C++ RAII wrapper (`Physics::World`) with auto-cleanup on scope exit.
  - Multiple concurrent dynamic bodies dropped from different heights ($Y = 12.0\text{ m}$ and $Y = 6.0\text{ m}$).
  - Independent restitution tracking and simultaneous bounce confirmation.
