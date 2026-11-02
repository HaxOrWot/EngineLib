# EngineLib 🎮⚡

**EngineLib** is a modular, high-performance suite of universal cross-language game engine libraries built from the ground up in modern systems programming languages (**Rust** & **C++**). 

EngineLib operates on a **Pure Computational State & Mathematics Server** paradigm. Each subsystem compiles down to a native dynamic shared library (`.dll` on Windows, `.so` on Linux, `.dylib` on macOS) with a stable, zero-overhead **C-compatible ABI (Foreign Function Interface)**.

---

## 🏛️ Ecosystem Overview

EngineLib decouples heavy mathematical and spatial computation from host runtime environments, allowing developers to drive game logic in high-level languages like **Python, C# (Unity / Godot), Java**, or native **C / C++** with near-zero marshaling overhead:

```
+-----------------------------------------------------------------+
|                  Host Runtime / Game Framework                  |
|     Python (ctypes) | C# (.NET/Unity/Godot) | C++ | Java        |
+-----------------------------------------------------------------+
                                │
                    C-Compatible ABI (FFI)
                                │
                                ▼
+-----------------------------------------------------------------+
|                           EngineLib                             |
|                                                                 |
|   +───────────────────────────+   +─────────────────────────+   |
|   |       PhysicsEngine       |   |      GraphicsEngine     |   |
|   |  3D Rigid Body Kinematics |   | Low-Overhead Rendering  |   |
|   |  (Rust Native .DLL)       |   | (In Development)        |   |
|   +───────────────────────────+   +─────────────────────────+   |
+-----------------------------------------------------------------+
```

---

## 🚀 Engine Modules

### 1. `PhysicsEngine/` (Available Now)
A zero-dependency 3D Kinematic & Rigid Body physics simulation engine written in Rust:
- **Numerical Integrator**: Symplectic (Semi-Implicit) Euler solver ensuring energy stability over discrete timesteps.
- **Collision Pipeline**: 3D Axis-Aligned Bounding Box (AABB) overlap testing with minimum-penetration normal derivation.
- **Contact Solver**: Newtonian elastic restitution impulses coupled with Baumgarte positional stabilization and penetration slop ($5\text{ mm}$) to completely prevent terrain sinking.
- **Anti-Jitter Dynamics**: Clamps micro-bounce velocities to zero ($v < 0.30\text{ m/s}$) for resting contacts on static surfaces.
- **FFI Safety**: Protected against panics across the C-boundary via `std::panic::catch_unwind` and null-pointer guards.
- **Memory Optimized**: 64-byte aligned `RigidBody3D` records fitting into single CPU L1 cache lines.
- **Comprehensive Documentation**: Complete mathematical derivations, blueprints, and multi-language binding guides inside [`PhysicsEngine/docs/`](PhysicsEngine/docs/).

### 2. `GraphicsEngine/` (Roadmap)
A modern, lightweight rendering pipeline designed for headless and windowed spatial visualization.

---

## 🧪 Multi-Language DLL Testing Suite (`DLL Tests/`)

EngineLib includes standalone validation test harnesses demonstrating dynamic library linking and simulation across three host languages:

- **🐍 Python ([`DLL Tests/Python/`](DLL%20Tests/Python/))**:
  - Uses `ctypes.CDLL`.
  - Simulates an elastic cube dropping under Earth gravity with a real-time **terminal ASCII trajectory plot**.
  - Simulates head-on mid-air momentum exchange between opposing bodies.
- **⚡ Native C ([`DLL Tests/C/`](DLL%20Tests/C/))**:
  - Direct compilation and linking using MinGW GCC.
  - Verifies C struct alignment and low-level pointer safety.
- **🛡️ Modern C++ ([`DLL Tests/C++/`](DLL%20Tests/C++/))**:
  - Object-oriented RAII wrapper (`Physics::World`).
  - Concurrent multi-body freefall and independent restitution validation.

---

## 🛠️ Quick Start & Building

### Prerequisites
- **Rust Toolchain**: `rustup`, `rustc`, `cargo` (v1.80+)
- **C/C++ Compiler (Optional for C/C++ tests)**: GCC / Clang / MSVC
- **Python (Optional for Python tests)**: Python 3.8+

### 1. Build the Native Dynamic Library
```powershell
cd PhysicsEngine
cargo build --release
```
This produces `PhysicsEngine/target/release/physics_engine.dll` (or `.so` / `.dylib`).

### 2. Run Native Rust Tests
```powershell
cargo test
```

### 3. Run All Multi-Language DLL Integration Tests
```powershell
cd "../DLL Tests"
.\run_all_tests.ps1
```

---

## 📄 License
Licensed under either of:
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))
at your option.
