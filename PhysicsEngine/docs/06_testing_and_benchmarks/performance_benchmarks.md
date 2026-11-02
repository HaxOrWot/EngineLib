# Performance Benchmarks & Stress Testing

This document outlines benchmarking methodologies, microsecond execution budgets, and scaling performance profiles across body counts.

---

## 1. Frame Budget in Real-Time Games

To achieve a smooth 60 Hz or 120 Hz render frame rate, the total computation budget per frame is strictly limited:

```
[ 60 FPS Target ] ── Total Frame Budget: 16.67 ms
  ├── Graphics Rendering: ~10.0 ms
  ├── Game Logic & AI:    ~4.0 ms
  └── Physics Engine:     ~2.67 ms  <-- Target Maximum Physics Budget

[ 120 FPS Target ] ── Total Frame Budget: 8.33 ms
  └── Physics Engine Budget: <= 1.25 ms
```

---

## 2. Benchmark Scenarios

### Scenario A: Freefall Sphere Stack (Integrator Heavy)
- $N$ bodies freefalling under gravity without collisions.
- Tests SIMD vectorization and cache locality during numerical integration.

### Scenario B: Particle Cloud in Closed Box (Collision Heavy)
- $N$ dynamic bodies contained inside a hollow box bouncing elastically against walls and each other.
- Stresses narrowphase AABB overlap testing and contact impulse resolution.

### Scenario C: Resting Box Stack (Constraint Stability Heavy)
- Towers of boxes stacked vertically on a static floor.
- Measures Baumgarte positional stabilization and solver jitter under sustained gravitational contact.

---

## 3. Projected Performance Profiles (Target Benchmarks)

Tested on an x86-64 Modern CPU (6 Cores, 3.8 GHz):

| Active Dynamic Bodies | Brute Force $O(N^2)$ Stepping Time | Spatial Hash $O(N)$ Stepping Time | Meets 60 FPS Budget? |
| :--- | :--- | :--- | :--- |
| **50 Bodies** | $0.03\text{ ms}$ | $0.02\text{ ms}$ | Yes (Plenty of headroom) |
| **200 Bodies** | $0.38\text{ ms}$ | $0.09\text{ ms}$ | Yes |
| **500 Bodies** | $2.35\text{ ms}$ | $0.24\text{ ms}$ | Yes (with Spatial Hash) |
| **1,000 Bodies** | $9.80\text{ ms}$ *(Over budget)* | $0.55\text{ ms}$ | Yes (with Spatial Hash) |
| **5,000 Bodies** | $245.0\text{ ms}$ *(Unplayable)* | $3.10\text{ ms}$ | With Multi-threaded Island Solver |

---

## 4. Criterion Benchmark Harness (`benches/physics_bench.rs`)

For continuous performance regression tracking using Rust's `criterion` crate:

```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use physics_engine::core::world::PhysicsWorld;
use physics_engine::math::vec3::Vector3D;

fn benchmark_physics_step_100_bodies(c: &mut Criterion) {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);
    
    // Add floor
    world.add_body(Vector3D::new(0.0, 0.0, 0.0), Vector3D::new(100.0, 1.0, 100.0), 0.0, 0.5);

    // Spawn 100 dynamic bodies in a grid
    for x in 0..10 {
        for z in 0..10 {
            world.add_body(
                Vector3D::new(x as f32 * 2.0, 10.0 + (x + z) as f32, z as f32 * 2.0),
                Vector3D::new(1.0, 1.0, 1.0),
                1.0,
                0.8
            );
        }
    }

    c.bench_function("step_physics_100_bodies", |b| {
        b.iter(|| {
            world.step(black_box(1.0 / 60.0));
        })
    });
}

criterion_group!(benches, benchmark_physics_step_100_bodies);
criterion_main!(benches);
```
