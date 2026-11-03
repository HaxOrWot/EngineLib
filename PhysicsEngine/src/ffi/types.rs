use crate::math::vec3::Vector3D;

pub use crate::core::body::{RigidBody3D, ShapeType};
pub use crate::core::world::CollisionEvent;
pub use crate::math::quat::Quaternion;
pub use crate::math::vec3::Vector3D as FfiVector3D;
pub use crate::solver::joint::DistanceJoint;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BodyTransform {
    pub id: u32,
    pub position: Vector3D,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BodyTransformEx {
    pub id: u32,
    pub position: Vector3D,
    pub orientation: Quaternion,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RaycastHitFfi {
    pub point: Vector3D,
    pub normal: Vector3D,
    pub distance: f32,
    pub body_id: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DistanceJointFfi {
    pub id: u32,
    pub body_a: u32,
    pub body_b: u32,
    pub rest_length: f32,
}
