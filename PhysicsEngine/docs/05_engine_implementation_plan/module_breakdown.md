# Modular Codebase Breakdown & Structure

This document outlines the refactoring plan from a monolithic single-file implementation (`src/lib.rs`) into a modular, testable, and maintainable production Rust crate architecture.

---

## 1. Monolithic vs. Modular Architecture

The initial specification places all code in a single file:
```text
physics_engine/
├── Cargo.toml
└── src/
    └── lib.rs (contains math, structures, integration, AABB collision, impulse solver, and FFI)
```

While sufficient as a prototype, keeping all logic in `lib.rs` leads to:
- Tangled concerns (FFI pointer handling mixed with physical integration mathematics).
- Inability to write clean unit tests for collision detection independent of the global mutex.
- Code bloat as broadphase algorithms, rotational dynamics, and raycasting are introduced.

---

## 2. Target Modular Directory Structure

```text
physics_engine/
├── Cargo.toml
├── include/
│   └── physics_engine.h             # C/C++ API header
├── src/
│   ├── lib.rs                       # Root crate entry point & public module exports
│   │
│   ├── math/                        # Vector and mathematical foundations
│   │   ├── mod.rs
│   │   ├── vec3.rs                  # Vector3D struct, dot product, cross product, norms
│   │   └── aabb.rs                  # Bounding box bounds, half-extents, containment
│   │
│   ├── core/                        # Simulation state and entity management
│   │   ├── mod.rs
│   │   ├── body.rs                  # RigidBody3D struct, mass calculations
│   │   └── world.rs                 # PhysicsWorld container, entity storage, gravity
│   │
│   ├── dynamics/                    # Motion solvers & numerical integration
│   │   ├── mod.rs
│   │   └── integrator.rs            # Semi-implicit Euler & Verlet implementations
│   │
│   ├── collision/                   # Spatial indexing & geometric intersection
│   │   ├── mod.rs
│   │   ├── broadphase.rs            # Spatial hash grid & pair culling
│   │   ├── narrowphase.rs           # AABB-AABB intersection tests, contact normal derivation
│   │   └── manifold.rs              # Contact point, normal, and penetration depth records
│   │
│   ├── solver/                      # Contact and constraint resolution
│   │   ├── mod.rs
│   │   ├── contact.rs               # Baumgarte positional correction & velocity impulse
│   │   └── island.rs                # Sleeping & constraint island generation (future)
│   │
│   └── ffi/                         # Foreign Function Interface boundary layer
│       ├── mod.rs
│       ├── exports.rs               # #[no_mangle] extern "C" function definitions
│       ├── types.rs                 # #[repr(C)] FFI structs and enums
│       └── guard.rs                 # Panic boundaries and pointer validation helpers
│
└── tests/                           # Integration test suites
    ├── gravity_test.rs              # Validates acceleration under freefall
    ├── bounce_test.rs               # Validates restitution coefficient preservation
    └── stacking_test.rs             # Validates multi-body static stability
```

---

## 3. Module Responsibilities & Dependency Graph

```
                   [ ffi/exports.rs ]
                           │
                           ▼
                  [ core/world.rs ]
                 /        │        \
                ▼         ▼         ▼
  [ dynamics/integrator ] [ collision/ ] [ solver/contact ]
                \         │         /
                 ▼        ▼        ▼
                   [ math/vec3.rs ]
```

1. **`math/`**: Zero external dependencies. Implements vector arithmetic, operators (`+`, `-`, `*`), dot products, and distance formulas.
2. **`core/`**: Defines `RigidBody3D` and `PhysicsWorld`. Holds the state without knowing how physics steps are executed.
3. **`dynamics/`**: Pure functions that accept mutable slices of bodies and advance velocities and coordinates.
4. **`collision/`**: Pure geometric algorithms. Takes bodies and produces a list of `ContactManifold` items.
5. **`solver/`**: Consumes `ContactManifold` items and applies corrective impulses to participating bodies.
6. **`ffi/`**: The sole module containing `unsafe`, `catch_unwind`, raw pointer dereferences, and `#[no_mangle]`.
