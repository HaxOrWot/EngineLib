use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DistanceJoint {
    pub id: u32,
    pub body_a: u32,
    pub body_b: u32,
    pub offset_a: Vector3D,
    pub offset_b: Vector3D,
    pub rest_length: f32,
    pub stiffness: f32,
    pub damping: f32,
}

impl DistanceJoint {
    pub fn new(
        id: u32,
        body_a: u32,
        body_b: u32,
        offset_a: Vector3D,
        offset_b: Vector3D,
        rest_length: f32,
        stiffness: f32,
        damping: f32,
    ) -> Self {
        Self {
            id,
            body_a,
            body_b,
            offset_a,
            offset_b,
            rest_length: rest_length.max(0.0),
            stiffness: if stiffness > 0.0 { stiffness.clamp(0.0, 1.0) } else { 0.9 },
            damping: damping.clamp(0.0, 1.0),
        }
    }

    /// Solves joint distance constraint and applies positional & velocity damping corrections
    pub fn solve(&self, body_a: &mut RigidBody3D, body_b: &mut RigidBody3D) {
        let total_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if total_inv_mass <= 0.0 {
            return;
        }

        // Compute world anchors
        let world_a = body_a.position + self.offset_a;
        let world_b = body_b.position + self.offset_b;

        let delta = world_a - world_b;
        let dist = delta.length();
        if dist < 1e-6 {
            return;
        }

        let normal = delta * (1.0 / dist);
        let error = dist - self.rest_length;

        // 1. Position-level constraint projection (XPBD)
        let correction_magnitude = (error / total_inv_mass) * self.stiffness;
        let correction = normal * correction_magnitude;

        body_a.position -= correction * body_a.inverse_mass;
        body_b.position += correction * body_b.inverse_mass;

        // 2. Velocity damping along constraint axis (prevents elastic oscillation)
        let rel_vel = body_a.velocity - body_b.velocity;
        let vel_along_axis = rel_vel.dot(normal);

        let damping_impulse = (vel_along_axis / total_inv_mass) * self.damping;
        let damping_vec = normal * damping_impulse;

        body_a.velocity -= damping_vec * body_a.inverse_mass;
        body_b.velocity += damping_vec * body_b.inverse_mass;

        // Auto-wake connected limbs under tension or motion
        if error.abs() > 0.02 || vel_along_axis.abs() > 0.05 {
            body_a.wake();
            body_b.wake();
        }
    }
}
