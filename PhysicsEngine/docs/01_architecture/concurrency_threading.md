# Concurrency & Threading Model

This document outlines the thread synchronization strategies, lock contention mitigation, and the roadmap for multi-threaded stepping in the physics engine.

---

## 1. Global State Synchronization: `Mutex<Option<PhysicsWorld>>`

Because the engine is loaded as a native C-compatible DLL, external callers invoke C-ABI functions without passing an engine handle instance. To maintain global engine state across function invocations, the engine encapsulates its simulation state in a thread-safe static container:

```rust
use std::sync::Mutex;

static WORLD: Mutex<Option<PhysicsWorld>> = Mutex::new(None);
```

### Safety Guarantees
- **Data Race Prevention**: Rust's standard library `Mutex<T>` enforces mutual exclusion. Multiple threads calling `step_physics` or `apply_force` concurrently cannot induce data races.
- **Poisoning Resilience**: If an internal thread panics while holding the lock, subsequent lock acquisitions return an `Err(PoisonError)`. The FFI wrapper safely intercepts this rather than corrupting host memory.

---

## 2. Lock Contention Analysis & Optimization

While a single global `Mutex` guarantees thread safety for simple host architectures, high-frequency queries (`get_body_position` called thousands of times per frame from render threads) can cause lock contention with the physics update thread (`step_physics`).

### Evolution Path: `Mutex` vs. `RwLock` vs. Engine Context Handles

| Synchronization Pattern | Read Performance | Write (`step_physics`) Performance | Safety / Complexity |
| :--- | :--- | :--- | :--- |
| **Global `Mutex<World>` (MVP)** | Serialized | Serialized | Simple, robust, zero deadlocks |
| **Global `RwLock<World>`** | Concurrent across render threads | Exclusive write lock during step | Low contention for heavy reads |
| **Opaque Pointer Context Handles** | Independent per-world parallelism | Isolated thread contexts | High; eliminates all global state |

### Moving to Opaque World Handles (Production Architecture)
In advanced game engine setups, host runtimes may want to simulate multiple independent physics scenes concurrently (e.g., UI physics vs. game world physics).

The next evolutionary step introduces explicit opaque world handles:

```c
// C-ABI Context Pattern
typedef struct PhysicsWorldOpaque* PhysicsWorldHandle;

PhysicsWorldHandle physics_world_create(float gx, float gy, float gz);
void physics_world_destroy(PhysicsWorldHandle handle);
void physics_world_step(PhysicsWorldHandle handle, float dt);
```

```rust
// Rust implementation
#[no_mangle]
pub extern "C" fn physics_world_create(gx: f32, gy: f32, gz: f32) -> *mut PhysicsWorld {
    let world = Box::new(PhysicsWorld::new(gx, gy, gz));
    Box::into_raw(world)
}

#[no_mangle]
pub unsafe extern "C" fn physics_world_destroy(handle: *mut PhysicsWorld) {
    if !handle.is_null() {
        drop(Box::from_raw(handle));
    }
}
```

---

## 3. Parallel Step Execution Roadmap

When simulating thousands of active bodies, `step_physics` can be parallelized across distinct stages:

```
[ Phase 1: Numerical Integration ]
    └── Embarrassingly Parallel: Rayon parallel iterator over world.bodies.par_iter_mut()

[ Phase 2: Broadphase Culling ]
    └── Parallel Grid Insertion & Pair Generation via Spatial Hash Cells

[ Phase 3: Narrowphase Intersection ]
    └── Rayon chunked pair evaluation

[ Phase 4: Constraint & Impulse Resolution ]
    └── Sequential or Island-based parallel solving (Graph coloring of contact islands)
```

1. **Integration**: Positions and velocities for independent bodies can be updated concurrently across CPU cores without synchronization.
2. **Contact Islands**: Groups of interacting resting/colliding bodies form independent subgraphs ("islands"). Each island can be solved on a separate thread pool worker with zero lock contention.
