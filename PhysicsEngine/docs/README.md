# Universal Cross-Language 3D Physics Engine Architecture & Documentation

Welcome to the definitive architecture, design specification, and technical documentation for the **Universal 3D Kinematic & Rigid Body Physics Engine**.

Built in **Rust** as a zero-overhead, pure computational dynamic shared library (`.dll` / `.so` / `.dylib`), this engine exposes an FFI C-ABI interface enabling high-performance spatial simulation across any host runtime—including **Python, C#, C++, Java, Rust, and WebAssembly**.

---

## 🧭 Master Documentation Index

```
docs/
├── PROMPT.md                         # 🤖 Master autonomous prompt for AI-driven implementation
│
├── 01_architecture/                  # High-level architecture, memory model, and concurrency
│   ├── system_overview.md            # Client-server headless paradigm and lifecycle
│   ├── memory_model.md               # Heap safety, FFI boundaries, and cache locality
│   ├── concurrency_threading.md      # Thread safety, lock contention, and parallel stepping
│   └── edge_cases_and_stability.md   # Degenerate normals, micro-jitter, slop factor, tunneling
│
├── 02_math_and_physics/              # Physical simulation mathematics & algorithms
│   ├── numerical_integration.md      # Semi-implicit Euler, Verlet, timestep accumulation
│   ├── collision_detection.md        # AABB intersection, SAT, broadphase vs narrowphase
│   ├── collision_resolution.md       # Linear momentum, restitution impulse, Baumgarte stabilization
│   ├── friction_model.md             # Coulomb friction, tangential sliding drag, static vs dynamic
│   └── rotational_dynamics_roadmap.md# Roadmap for angular velocity, inertia tensors, quaternions
│
├── 03_ffi_and_api/                   # Foreign Function Interface contracts & specifications
│   ├── c_abi_spec.md                 # Complete C ABI symbols, struct memory layouts, signatures
│   ├── error_handling.md             # Panic boundaries, catch_unwind, error codes
│   └── batch_api_design.md           # Zero-copy flat buffer batch updates vs scalar calls
│
├── 04_language_bindings/             # Production-ready client integration harnesses
│   ├── python_ctypes.md              # Python ctypes wrapper & automated bounce validation
│   ├── csharp_unity.md               # C# P/Invoke, Unity / Godot engine integration
│   └── cpp_interop.md                # Native C header and modern C++ RAII wrapper
│
├── 05_engine_implementation_plan/    # Modular codebase organization and build configuration
│   ├── complete_code_templates.md    # Complete, drop-in Rust code blueprints for every module
│   ├── module_breakdown.md           # Modular directory tree (math, core, collision, ffi)
│   ├── cargo_configuration.md        # Release optimization (LTO, SIMD, panic=abort, cdylib)
│   └── roadmap_and_milestones.md     # Engineering phases from MVP to advanced solver
│
└── 06_testing_and_benchmarks/        # Verification, validation, and benchmarking
    ├── unit_and_integration_tests.md # Test suites: gravity, restitution, stacking, tunneling
    └── performance_benchmarks.md     # Stress testing 1,000 to 10,000 rigid bodies
```

---

## 🎯 Architectural Philosophy & Core Tenets

1. **Pure State & Mathematics Server**:
   - Zero graphics dependencies, zero windowing overhead, zero audio/input bloat.
   - The engine consumes raw state inputs ($\Delta t$, forces, body additions/removals) and yields updated transform matrices.
2. **Universal C-ABI Compatibility**:
   - Compiles as a `cdylib` with `#[repr(C)]` memory layouts.
   - Any language capable of loading dynamic libraries (`ctypes`, P/Invoke, JNI, FFM) can drive this engine with near-zero marshaling overhead.
3. **Deterministic Timestep Support**:
   - Decoupled from rendering frame rates. Designed for fixed-timestep integration ($\Delta t = 1/60s$) with accumulator loops to guarantee reproducible simulation states.
4. **Zero-Cost Abstractions with Memory Safety**:
   - Authored in safe Rust, isolating raw pointer dereferences strictly to FFI boundary validation and double-mutable borrow dispatch.
5. **Evolutionary Architecture**:
   - Designed to evolve from simple Axis-Aligned Bounding Box (AABB) linear dynamics into full 6-DOF (degrees of freedom) rotational rigid body mechanics with spatial acceleration structures.

---

## ⚡ Complete Execution Pipeline

```
+-------------------------------------------------------------+
|               Host Runtime (Client Environment)              |
|          Python (ctypes) | C# (.NET/Unity) | C++ / Java     |
+-------------------------------------------------------------+
                              │
                     C-Compatible ABI (FFI)
           (Pointers, Primitives, Fixed Memory Layouts)
                              │
                              ▼
+-------------------------------------------------------------+
|             Universal Rust Physics Engine (.DLL)             |
|                                                             |
|   +-----------------------------------------------------+   |
|   | 1. FFI Boundary Layer (catch_unwind, Pointer Guard) |   |
|   +-----------------------------------------------------+   |
|                              │                              |
|   +-----------------------------------------------------+   |
|   | 2. World State Manager (Thread-Safe Mutex / Bodies) |   |
|   +-----------------------------------------------------+   |
|                              │                              |
|   +-----------------------------------------------------+   |
|   | 3. Integrator (Semi-Implicit Symplectic Euler)      |   |
|   +-----------------------------------------------------+   |
|                              │                              |
|   +-----------------------------------------------------+   |
|   | 4. Collision Detection (AABB Overlap & Min-Pen Axis)|   |
|   +-----------------------------------------------------+   |
|                              │                              |
|   +-----------------------------------------------------+   |
|   | 5. Contact Solver (Baumgarte Correction + Impulses) |   |
|   +-----------------------------------------------------+   |
|                              │                              |
|   +-----------------------------------------------------+   |
|   | 6. Coulomb Friction Solver (Tangential Drag Clamp)  |   |
|   +-----------------------------------------------------+   |
+-------------------------------------------------------------+
```

---

## 🚀 How to Implement This Engine

If you are an AI assistant or software engineer tasked with implementing this physics engine:
1. Open and follow [PROMPT.md](file:///Z:/VSCode/GameLib/PhysicsEngine/docs/PROMPT.md) for the end-to-end execution directives and acceptance criteria.
2. Review [05_engine_implementation_plan/complete_code_templates.md](file:///Z:/VSCode/GameLib/PhysicsEngine/docs/05_engine_implementation_plan/complete_code_templates.md) for modular source code for every Rust file.
3. Verify the final build using the Python test suite in [04_language_bindings/python_ctypes.md](file:///Z:/VSCode/GameLib/PhysicsEngine/docs/04_language_bindings/python_ctypes.md).
