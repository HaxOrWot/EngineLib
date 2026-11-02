# Complete Implementation Code Blueprints

This document contains complete, production-ready, fully commented Rust source code templates for each file in the modular engine architecture. Any AI or developer can reference or directly translate these templates into the `src/` directory.

---

## 1. `src/math/vec3.rs`

```rust
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, Neg};

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3D {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    pub const UP: Self = Self { x: 0.0, y: 1.0, z: 0.0 };

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline(always)]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline(always)]
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline(always)]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline(always)]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    #[inline(always)]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > 1e-6 {
            self * (1.0 / len)
        } else {
            Self::ZERO
        }
    }

    #[inline(always)]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vector3D {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vector3D {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vector3D {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vector3D {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Mul<f32> for Vector3D {
    type Output = Self;
    #[inline(always)]
    fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

impl MulAssign<f32> for Vector3D {
    #[inline(always)]
    fn mul_assign(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
        self.z *= scalar;
    }
}

impl Div<f32> for Vector3D {
    type Output = Self;
    #[inline(always)]
    fn div(self, scalar: f32) -> Self {
        let inv = 1.0 / scalar;
        Self::new(self.x * inv, self.y * inv, self.z * inv)
    }
}

impl Neg for Vector3D {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
```

---

## 2. `src/core/body.rs`

```rust
use crate::math::vec3::Vector3D;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RigidBody3D {
    pub id: u32,
    pub position: Vector3D,
    pub velocity: Vector3D,
    pub acceleration: Vector3D,
    pub size: Vector3D,
    pub mass: f32,
    pub inverse_mass: f32,
    pub restitution: f32,
}

impl RigidBody3D {
    pub fn new(id: u32, position: Vector3D, size: Vector3D, mass: f32, restitution: f32) -> Self {
        let inverse_mass = if mass > 0.0001 { 1.0 / mass } else { 0.0 };
        let clamped_restitution = restitution.clamp(0.0, 1.0);

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
            size,
            mass,
            inverse_mass,
            restitution: clamped_restitution,
        }
    }

    #[inline(always)]
    pub fn is_static(&self) -> bool {
        self.inverse_mass == 0.0
    }

    #[inline(always)]
    pub fn apply_force(&mut self, force: Vector3D) {
        if !self.is_static() {
            self.acceleration += force * self.inverse_mass;
        }
    }
}
```

---

## 3. `src/dynamics/integrator.rs`

```rust
use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

pub struct Integrator;

impl Integrator {
    /// Advances body kinematics by timestep dt using Symplectic (Semi-Implicit) Euler
    pub fn integrate(body: &mut RigidBody3D, gravity: Vector3D, dt: f32) {
        if body.is_static() {
            return;
        }

        // 1. Advance velocity by acceleration + global gravity
        body.velocity += (body.acceleration + gravity) * dt;

        // 2. Advance position by newly computed velocity
        body.position += body.velocity * dt;

        // 3. Reset accumulated forces for next frame
        body.acceleration = Vector3D::ZERO;
    }
}
```

---

## 4. `src/collision/narrowphase.rs`

```rust
use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

pub struct ContactManifold {
    pub normal: Vector3D,
    pub penetration: f32,
}

pub struct Narrowphase;

impl Narrowphase {
    const EPSILON: f32 = 1e-6;

    /// Evaluates 3D AABB overlap between two rigid bodies
    pub fn test_aabb(body_a: &RigidBody3D, body_b: &RigidBody3D) -> Option<ContactManifold> {
        let delta = body_a.position - body_b.position;

        let overlap_x = (body_a.size.x + body_b.size.x) * 0.5 - delta.x.abs();
        if overlap_x <= 0.0 { return None; }

        let overlap_y = (body_a.size.y + body_b.size.y) * 0.5 - delta.y.abs();
        if overlap_y <= 0.0 { return None; }

        let overlap_z = (body_a.size.z + body_b.size.z) * 0.5 - delta.z.abs();
        if overlap_z <= 0.0 { return None; }

        // Find normal along shallowest penetration axis
        let (normal, penetration) = if overlap_x < overlap_y && overlap_x < overlap_z {
            let sign = if delta.x.abs() > Self::EPSILON { delta.x.signum() } else { 1.0 };
            (Vector3D::new(sign, 0.0, 0.0), overlap_x)
        } else if overlap_y < overlap_x && overlap_y < overlap_z {
            let sign = if delta.y.abs() > Self::EPSILON { delta.y.signum() } else { 1.0 };
            (Vector3D::new(0.0, sign, 0.0), overlap_y)
        } else {
            let sign = if delta.z.abs() > Self::EPSILON { delta.z.signum() } else { 1.0 };
            (Vector3D::new(0.0, 0.0, sign), overlap_z)
        };

        Some(ContactManifold { normal, penetration })
    }
}
```

---

## 5. `src/solver/contact.rs`

```rust
use crate::core::body::RigidBody3D;
use crate::collision::narrowphase::ContactManifold;

pub struct ContactSolver;

impl ContactSolver {
    const PENETRATION_SLOP: f32 = 0.005; // 5 mm allowance
    const BAUMGARTE_FACTOR: f32 = 0.60;   // Resolve 60% of penetration per frame
    const RESTING_VEL_THRESHOLD: f32 = 0.30; // Threshold to kill micro-bounce jitter

    pub fn resolve(body_a: &mut RigidBody3D, body_b: &mut RigidBody3D, manifold: &ContactManifold) {
        let total_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if total_inv_mass <= 0.0 {
            return; // Immovable terrain vs immovable terrain
        }

        let normal = manifold.normal;
        let penetration = manifold.penetration;

        // 1. Baumgarte Positional Correction (Anti-Sinking)
        let effective_penetration = (penetration - Self::PENETRATION_SLOP).max(0.0);
        if effective_penetration > 0.0 {
            let correction_scalar = (effective_penetration / total_inv_mass) * Self::BAUMGARTE_FACTOR;
            body_a.position += normal * (body_a.inverse_mass * correction_scalar);
            body_b.position -= normal * (body_b.inverse_mass * correction_scalar);
        }

        // 2. Relative Velocity & Normal Component
        let rel_vel = body_a.velocity - body_b.velocity;
        let vel_along_normal = rel_vel.dot(normal);

        // Do not resolve if bodies are separating
        if vel_along_normal >= 0.0 {
            return;
        }

        // 3. Elastic Restitution Impulse (with resting velocity clamping)
        let e = if vel_along_normal.abs() < Self::RESTING_VEL_THRESHOLD {
            0.0 // Eliminate jitter for resting contact
        } else {
            body_a.restitution.min(body_b.restitution)
        };

        let mut impulse_scalar = -(1.0 + e) * vel_along_normal;
        impulse_scalar /= total_inv_mass;

        let impulse = normal * impulse_scalar;
        body_a.velocity += impulse * body_a.inverse_mass;
        body_b.velocity -= impulse * body_b.inverse_mass;
    }
}
```

---

## 6. `src/core/world.rs`

```rust
use std::collections::HashMap;
use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;
use crate::dynamics::integrator::Integrator;
use crate::collision::narrowphase::Narrowphase;
use crate::solver::contact::ContactSolver;

pub struct PhysicsWorld {
    pub bodies: HashMap<u32, RigidBody3D>,
    pub next_id: u32,
    pub gravity: Vector3D,
}

impl PhysicsWorld {
    pub fn new(gx: f32, gy: f32, gz: f32) -> Self {
        Self {
            bodies: HashMap::new(),
            next_id: 1,
            gravity: Vector3D::new(gx, gy, gz),
        }
    }

    pub fn add_body(&mut self, pos: Vector3D, size: Vector3D, mass: f32, restitution: f32) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let body = RigidBody3D::new(id, pos, size, mass, restitution);
        self.bodies.insert(id, body);
        id
    }

    pub fn remove_body(&mut self, id: u32) -> bool {
        self.bodies.remove(&id).is_some()
    }

    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() {
            return;
        }

        // Phase 1: Numerical Integration
        for body in self.bodies.values_mut() {
            Integrator::integrate(body, self.gravity, dt);
        }

        // Phase 2 & 3: Collision Detection and Contact Resolution
        let keys: Vec<u32> = self.bodies.keys().copied().collect();
        let len = keys.len();

        for i in 0..len {
            for j in (i + 1)..len {
                let id_a = keys[i];
                let id_b = keys[j];

                // Fetch references safely via raw pointers to allow disjoint mutable borrows
                let ptr_a = self.bodies.get_mut(&id_a).unwrap() as *mut RigidBody3D;
                let ptr_b = self.bodies.get_mut(&id_b).unwrap() as *mut RigidBody3D;

                unsafe {
                    let body_a = &mut *ptr_a;
                    let body_b = &mut *ptr_b;

                    if let Some(manifold) = Narrowphase::test_aabb(body_a, body_b) {
                        ContactSolver::resolve(body_a, body_b, &manifold);
                    }
                }
            }
        }
    }
}
```

---

## 7. `src/ffi/exports.rs` & `src/lib.rs`

```rust
// In src/lib.rs
pub mod math;
pub mod core;
pub mod dynamics;
pub mod collision;
pub mod solver;
pub mod ffi;

pub use math::vec3::Vector3D;
pub use core::body::RigidBody3D;
pub use core::world::PhysicsWorld;

use std::sync::Mutex;
use std::panic::{catch_unwind, AssertUnwindSafe};

static WORLD: Mutex<Option<PhysicsWorld>> = Mutex::new(None);

#[no_mangle]
pub extern "C" fn init_physics_world(gx: f32, gy: f32, gz: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let mut world = WORLD.lock().unwrap();
        *world = Some(PhysicsWorld::new(gx, gy, gz));
    }));
}

#[no_mangle]
pub extern "C" fn add_rigid_body(
    px: f32, py: f32, pz: f32,
    sx: f32, sy: f32, sz: f32,
    mass: f32, restitution: f32,
) -> u32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        let mut world_lock = WORLD.lock().unwrap();
        if let Some(ref mut world) = *world_lock {
            return world.add_body(
                Vector3D::new(px, py, pz),
                Vector3D::new(sx, sy, sz),
                mass,
                restitution,
            );
        }
        0
    }));
    result.unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn get_body_position(id: u32, out_pos: *mut Vector3D) -> bool {
    if out_pos.is_null() {
        return false;
    }
    let result = catch_unwind(AssertUnwindSafe(|| {
        let world_lock = WORLD.lock().unwrap();
        if let Some(ref world) = *world_lock {
            if let Some(body) = world.bodies.get(&id) {
                std::ptr::write(out_pos, body.position);
                return true;
            }
        }
        false
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub extern "C" fn apply_force(id: u32, fx: f32, fy: f32, fz: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let mut world_lock = WORLD.lock().unwrap();
        if let Some(ref mut world) = *world_lock {
            if let Some(body) = world.bodies.get_mut(&id) {
                body.apply_force(Vector3D::new(fx, fy, fz));
            }
        }
    }));
}

#[no_mangle]
pub extern "C" fn step_physics(dt: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let mut world_lock = WORLD.lock().unwrap();
        if let Some(ref mut world) = *world_lock {
            world.step(dt);
        }
    }));
}
```
