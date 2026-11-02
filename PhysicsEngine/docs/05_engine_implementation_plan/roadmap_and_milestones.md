# Engineering Roadmap & Milestones

This document establishes the phased engineering roadmap, functional milestones, and deliverables from MVP to advanced production-grade physics engine.

---

## 1. Roadmap Overview

```
[ Milestone 1: Core MVP ] ──────────► [ Milestone 2: Throughput & Scaling ]
- C-ABI DLL Export                     - Spatial Hashing Broadphase
- Linear Semi-Implicit Euler           - Zero-Copy Batch Transforms API
- Pairwise AABB Intersections          - Friction Dynamics (Coulomb Model)
- Baumgarte & Elastic Bounce           - Sleeping / Deactivation Subsystem

[ Milestone 3: 6-DOF Dynamics ] ────► [ Milestone 4: Production Polish ]
- Quaternion Orientations              - Multi-Threaded Rayon Island Stepping
- 3D Inertia Tensors                   - Raycasting & Spatial Queries API
- Angular Momentum & Torque            - Continuous Collision Detection (CCD)
- Contact Point Manifold Generation    - Full Bindings for Unity, Godot & Python
```

---

## 2. Milestone 1: The Core FFI MVP (Current Phase)
**Goal**: Deliver a stable, zero-dependency native shared library satisfying the original project specification.

- [x] Universal `Cargo.toml` with `cdylib` and `rlib` targets.
- [x] 3D Vector arithmetic and AABB spatial extents.
- [x] Semi-implicit Euler integration with gravity and force accumulation.
- [x] Pairwise AABB overlap testing with minimum penetration normal derivation.
- [x] Baumgarte anti-sinking positional correction and Newtonian restitution impulse resolution.
- [x] Thread-safe global `Mutex` state container.
- [x] Exported C-ABI symbols: `init_physics_world`, `add_rigid_body`, `get_body_position`, `apply_force`, `step_physics`.
- [x] Automated Python validation harness.

---

## 3. Milestone 2: High-Throughput Spatial Acceleration & Friction
**Goal**: Scale from 200 bodies to 5,000+ bodies at 60 Hz.

1. **Spatial Hash Grid Broadphase**:
   - Discard distant bodies in $O(N)$ average time instead of $O(N^2)$.
2. **Coulomb Friction Model**:
   - Implement static friction coefficient ($\mu_s$) and kinetic friction coefficient ($\mu_k$).
   - Apply friction impulses tangential to the contact normal plane.
3. **Batch Transforms Streaming**:
   - Implement `sync_all_transforms` to eliminate per-body FFI marshaling overhead.
4. **Body Sleeping / Inactivity**:
   - Bodies whose linear velocity remains below a velocity threshold ($\epsilon = 0.05\text{ m/s}$) for $> 0.5\text{ s}$ enter a sleep state, bypassing integration and collision loops.

---

## 4. Milestone 3: Full 6-DOF Rotational Rigid Body Solver
**Goal**: Support tumbling boxes, spinning projectiles, and realistic off-center impacts.

1. **Orientation Quaternions**:
   - Integrate angular velocity into unit quaternions without gimbal lock.
2. **Inertia Tensors**:
   - Precompute local diagonal tensors and transform into world-space coordinates per step.
3. **Offset Force & Torque**:
   - Expose `apply_torque(id, tx, ty, tz)` and `apply_force_at_point(id, fx, fy, fz, px, py, pz)`.
4. **Contact Manifolds**:
   - Generate exact contact points and lever arms ($\mathbf{r}_A, \mathbf{r}_B$) for off-center angular impulse application.

---

## 5. Milestone 4: Production Tooling & Raycasting
**Goal**: Commercial game engine readiness.

1. **Raycast Queries**:
   - Expose `raycast(origin, direction, max_dist, out_hit)` for weapon mechanics, line-of-sight checks, and ground detection.
2. **Multi-Threaded Island Solver**:
   - Group non-interacting bodies into independent islands and solve using Rayon worker threads.
3. **Continuous Collision Detection (CCD)**:
   - Prevent fast-moving small objects from tunneling through walls via swept volume testing.
