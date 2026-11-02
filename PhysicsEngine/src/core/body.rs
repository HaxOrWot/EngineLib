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
        let inv_mass = if mass > 0.0001 { 1.0 / mass } else { 0.0 };
        let clamped_restitution = restitution.clamp(0.0, 1.0);

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
            size,
            mass,
            inverse_mass: inv_mass,
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
