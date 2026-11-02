use crate::math::vec3::Vector3D;

pub use crate::core::body::RigidBody3D;
pub use crate::math::vec3::Vector3D as FfiVector3D;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BodyTransform {
    pub id: u32,
    pub position: Vector3D,
}
