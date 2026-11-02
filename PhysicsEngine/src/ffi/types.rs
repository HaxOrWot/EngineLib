use crate::math::vec3::Vector3D;

pub use crate::core::body::{RigidBody3D, ShapeType};
pub use crate::math::vec3::Vector3D as FfiVector3D;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BodyTransform {
    pub id: u32,
    pub position: Vector3D,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RaycastHitFfi {
    pub point: Vector3D,
    pub normal: Vector3D,
    pub distance: f32,
    pub body_id: u32,
}
