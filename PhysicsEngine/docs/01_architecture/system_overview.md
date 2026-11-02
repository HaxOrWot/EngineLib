# System Overview: Architecture & Lifecycle

This document describes the high-level architecture, subsystem boundaries, client-server paradigm, and operational lifecycle of the 3D Physics Engine dynamic library.

---

## 1. Architectural Philosophy: The Headless State Server

The engine operates on a strict **Headless State & Mathematics Server** paradigm. It intentionally decouples physical mathematics from:
- Graphic rendering pipelines (OpenGL, Vulkan, DirectX, Metal).
- Windowing and input event management (SDL, GLFW, Win32).
- Audio and asset loading.

```
+--------------------+        Calls: init(), add_body(), step(dt)
|                    | ---------------------------------------------> +---------------------+
|    Host Engine     |                                                | Native Physics DLL  |
| (Game / Simulation)| <--------------------------------------------- |   (Rust cdylib)     |
|                    |        Reads: get_position(), transforms[]     +---------------------+
+--------------------+
```

### Key Responsibilities of the DLL
1. **State Retention**: Allocates and maintains all physical entity records (`RigidBody3D`) in native unmanaged heap memory.
2. **Deterministic Time Stepping**: Advances velocity, momentum, and position by discrete timesteps ($\Delta t$).
3. **Collision Detection**: Evaluates geometric intersections between bodies using broadphase culling followed by narrowphase penetration analysis.
4. **Impulse Dynamics**: Computes restorative and repulsive impulses to enforce non-penetration and bounce kinematics according to Newtonian mechanics.

---

## 2. Component Architecture Breakdown

The engine is structured into five cohesive internal domains:

```
[ FFI Ingress Layer ]
         │
         ▼
[ World State Container ] ─── (Bodies HashMap, Gravity, Config)
         │
         ├───► [ Integrator Pipeline ] ─────── (Numerical Euler Solver)
         │
         ├───► [ Spatial Broadphase ] ──────── (Spatial Hash / Brute-force Pair Culler)
         │
         ├───► [ Collision Narrowphase ] ───── (AABB Overlap & Normal Derivation)
         │
         └───► [ Impulse & Contact Solver ] ── (Baumgarte Correction & Elastic Bounce)
```

1. **FFI Layer (`ffi/`)**: Validates C-compatible pointers, catches unwinding panics, and translates external function calls into internal method dispatches.
2. **World State Manager (`core/world.rs`)**: Manages the life cycle of rigid bodies (creation, mutation, destruction, queries) and holds global simulation constants (e.g., gravity vector).
3. **Integrator (`dynamics/integrator.rs`)**: Updates linear kinematic equations ($a \to v \to x$) based on applied forces and gravity.
4. **Collision Pipeline (`collision/`)**:
   - **Broadphase**: Rapidly discards distant object pairs that cannot possibly collide.
   - **Narrowphase**: Performs precise 3D Axis-Aligned Bounding Box (AABB) intersection tests, determining contact normals and penetration depths.
5. **Contact Solver (`solver/contact.rs`)**: Solves non-penetration constraints using Baumgarte positional stabilization and applies momentum impulse scalars.

---

## 3. Engine Lifecycle States

A typical client session progresses through four discrete phases:

```
+-------------------+
|  1. Inactive /    |
|   Uninitialized   |
+-------------------+
          │
          │ init_physics_world(gx, gy, gz)
          ▼
+-------------------+
|  2. Provisioned   | <── add_rigid_body(...)
|     (Ready)       | <── apply_force(...)
+-------------------+
          │
          │ step_physics(dt)
          ▼
+-------------------+
|  3. Active Tick   | ─── Integrates kinematics, detects collisions,
|   (Simulating)    |     resolves impulses
+-------------------+
          │
          │ get_body_position(...) / batch read
          ▼
+-------------------+
|  4. Query / Read  | ─── Syncs positions to host game objects
+-------------------+
          │
          └─── Loop back to Phase 3 (Active Tick) per simulation frame
```

### Phase 1: Engine Initialization
The host runtime loads the dynamic library (`.dll` on Windows, `.so` on Linux) into its process space and calls `init_physics_world(gx, gy, gz)`.
- Global thread-safe mutex is locked.
- Internal hash map and entity counters are initialized.
- Default gravity vector is established (typically `gx=0.0, gy=-9.81, gz=0.0`).

### Phase 2: Scene Population
The host creates static entities (e.g., ground planes with `mass = 0.0`) and dynamic actors (cubes, spheres with `mass > 0.0`).
- Each call returns a unique 32-bit entity ID (`u32`).
- Inverse mass (`1.0 / mass`) is precomputed once to avoid costly divisions inside the physics loop.

### Phase 3: The Physics Tick Loop
Inside the game loop (ideally on a fixed timestep accumulator like 60 Hz or 120 Hz):
1. User input forces are applied via `apply_force(id, fx, fy, fz)`.
2. `step_physics(dt)` is invoked.
3. The engine performs numerical integration, pairwise intersection tests, positional corrections, and impulse application.

### Phase 4: State Synchronization
The host invokes position queries (e.g., `get_body_position`) or a batch buffer sync to extract native positions and update mesh transforms in the rendering engine.
