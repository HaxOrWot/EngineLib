use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Mutex;
use crate::core::world::PhysicsWorld;
use crate::ffi::types::BodyTransform;
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
