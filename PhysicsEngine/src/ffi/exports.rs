use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Mutex;
use crate::core::world::PhysicsWorld;
use crate::ffi::types::{BodyTransform, RaycastHitFfi};
use crate::math::ray::Ray;
use crate::math::vec3::Vector3D;

pub static WORLD: Mutex<Option<PhysicsWorld>> = Mutex::new(None);

#[inline]
fn with_world_mut<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut PhysicsWorld) -> R,
{
    let mut lock = match WORLD.lock() {
        Ok(guard) => guard,
        Err(poisoned) => {
            eprintln!("[PHYSICS_ENGINE] Mutex poisoned; recovering inner state.");
            poisoned.into_inner()
        }
    };
    lock.as_mut().map(f)
}

#[inline]
fn with_world<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&PhysicsWorld) -> R,
{
    let lock = match WORLD.lock() {
        Ok(guard) => guard,
        Err(poisoned) => {
            eprintln!("[PHYSICS_ENGINE] Mutex poisoned; recovering inner state.");
            poisoned.into_inner()
        }
    };
    lock.as_ref().map(f)
}

#[no_mangle]
pub extern "C" fn init_physics_world(gx: f32, gy: f32, gz: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let mut lock = match WORLD.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        };
        *lock = Some(PhysicsWorld::new(gx, gy, gz));
    }));
}

#[no_mangle]
pub extern "C" fn add_rigid_body(
    px: f32, py: f32, pz: f32,
    sx: f32, sy: f32, sz: f32,
    mass: f32, restitution: f32,
) -> u32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.add_body(
                Vector3D::new(px, py, pz),
                Vector3D::new(sx, sy, sz),
                mass,
                restitution,
            )
        }).unwrap_or(0)
    }));
    result.unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn add_sphere_body(
    px: f32, py: f32, pz: f32,
    radius: f32,
    mass: f32,
    restitution: f32,
    friction: f32,
) -> u32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.add_sphere(
                Vector3D::new(px, py, pz),
                radius,
                mass,
                restitution,
                friction,
            )
        }).unwrap_or(0)
    }));
    result.unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn add_distance_joint(
    body_a: u32,
    body_b: u32,
    ax: f32, ay: f32, az: f32,
    bx: f32, by: f32, bz: f32,
    rest_len: f32,
    stiffness: f32,
    damping: f32,
) -> u32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.add_distance_joint(
                body_a,
                body_b,
                Vector3D::new(ax, ay, az),
                Vector3D::new(bx, by, bz),
                rest_len,
                stiffness,
                damping,
            )
        }).unwrap_or(0)
    }));
    result.unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn remove_joint(id: u32) -> bool {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| world.remove_joint(id)).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub extern "C" fn set_body_damping(id: u32, damping: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            if let Some(body) = world.get_body_mut(id) {
                body.linear_damping = damping.max(0.0);
            }
        });
    }));
}

#[no_mangle]
pub extern "C" fn set_world_substeps(substeps: u32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.substeps = substeps.clamp(1, 16);
        });
    }));
}

#[no_mangle]
pub extern "C" fn set_solver_iterations(iters: u32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.solver_iterations = iters.clamp(1, 20);
        });
    }));
}

#[no_mangle]
pub extern "C" fn set_body_friction(id: u32, friction: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            if let Some(body) = world.get_body_mut(id) {
                body.friction = friction.max(0.0);
            }
        });
    }));
}

#[no_mangle]
pub extern "C" fn is_body_sleeping(id: u32) -> bool {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world(|world| {
            world.get_body(id).map(|b| b.is_sleeping == 1).unwrap_or(false)
        }).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub extern "C" fn wake_body(id: u32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            if let Some(body) = world.get_body_mut(id) {
                body.wake();
            }
        });
    }));
}

#[no_mangle]
pub unsafe extern "C" fn get_body_position(id: u32, out_pos: *mut Vector3D) -> bool {
    if out_pos.is_null() {
        return false;
    }
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world(|world| {
            if let Some(body) = world.get_body(id) {
                std::ptr::write(out_pos, body.position);
                true
            } else {
                false
            }
        }).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub unsafe extern "C" fn get_body_velocity(id: u32, out_vel: *mut Vector3D) -> bool {
    if out_vel.is_null() {
        return false;
    }
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world(|world| {
            if let Some(body) = world.get_body(id) {
                std::ptr::write(out_vel, body.velocity);
                true
            } else {
                false
            }
        }).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub extern "C" fn set_body_velocity(id: u32, vx: f32, vy: f32, vz: f32) -> bool {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            if let Some(body) = world.get_body_mut(id) {
                body.velocity = Vector3D::new(vx, vy, vz);
                body.wake();
                true
            } else {
                false
            }
        }).unwrap_or(false)
    }));
    result.unwrap_or(false)
}


#[no_mangle]
pub extern "C" fn apply_force(id: u32, fx: f32, fy: f32, fz: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            if let Some(body) = world.get_body_mut(id) {
                body.apply_force(Vector3D::new(fx, fy, fz));
            }
        });
    }));
}

#[no_mangle]
pub extern "C" fn step_physics(dt: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.step(dt);
        });
    }));
}

#[no_mangle]
pub extern "C" fn remove_rigid_body(id: u32) -> bool {
    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| world.remove_body(id)).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub extern "C" fn remove_body(id: u32) -> bool {
    remove_rigid_body(id)
}


#[no_mangle]
pub extern "C" fn destroy_physics_world() {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let mut lock = match WORLD.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        };
        *lock = None;
    }));
}

#[no_mangle]
pub unsafe extern "C" fn raycast(
    ox: f32, oy: f32, oz: f32,
    dx: f32, dy: f32, dz: f32,
    max_dist: f32,
    out_hit: *mut RaycastHitFfi,
) -> bool {
    if out_hit.is_null() {
        return false;
    }

    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world(|world| {
            let ray = Ray::new(Vector3D::new(ox, oy, oz), Vector3D::new(dx, dy, dz));
            if let Some(hit) = world.raycast(&ray, max_dist) {
                std::ptr::write(
                    out_hit,
                    RaycastHitFfi {
                        point: hit.point,
                        normal: hit.normal,
                        distance: hit.distance,
                        body_id: hit.body_id,
                    },
                );
                true
            } else {
                false
            }
        }).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub unsafe extern "C" fn sync_all_transforms(
    out_buffer: *mut BodyTransform,
    buffer_capacity: u32,
    out_written_count: *mut u32,
) -> bool {
    if out_buffer.is_null() || out_written_count.is_null() {
        return false;
    }

    let result = catch_unwind(AssertUnwindSafe(|| {
        with_world(|world| {
            let mut count = 0u32;
            let capacity = buffer_capacity as usize;

            for (id, body) in world.bodies.iter() {
                if (count as usize) >= capacity {
                    break;
                }
                let dest = out_buffer.add(count as usize);
                std::ptr::write(
                    dest,
                    BodyTransform {
                        id: *id,
                        position: body.position,
                    },
                );
                count += 1;
            }

            std::ptr::write(out_written_count, count);
            true
        }).unwrap_or(false)
    }));
    result.unwrap_or(false)
}

#[no_mangle]
pub extern "C" fn disable_collision_pair(body_a: u32, body_b: u32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.disable_collision_pair(body_a, body_b);
        });
    }));
}

#[no_mangle]
pub extern "C" fn enable_collision_pair(body_a: u32, body_b: u32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        with_world_mut(|world| {
            world.enable_collision_pair(body_a, body_b);
        });
    }));
}

