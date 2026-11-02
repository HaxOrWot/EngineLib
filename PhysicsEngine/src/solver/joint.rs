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
            stiffness: if stiffness > 0.0 { stiffness.clamp(0.0, 1.0) } else { 0.95 },
            damping: damping.clamp(0.0, 1.0),
        }
    }

    /// Solves joint distance constraint via XPBD position projection and rotational torque
    pub fn solve(&self, body_a: &mut RigidBody3D, body_b: &mut RigidBody3D, _dt: f32) {
        let linear_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if linear_inv_mass <= 0.0 {
            return;
        }

        let r_a = self.offset_a;
        let r_b = self.offset_b;

        // World anchors
        let world_a = body_a.position + r_a;
        let world_b = body_b.position + r_b;

        let delta = world_a - world_b;
        let dist = delta.length();
        if dist < 1e-6 {
            return;
        }

        let normal = delta * (1.0 / dist);
        let error = dist - self.rest_length;

        // Angular inertia contribution to joint effective mass
        let ang_a = if !body_a.is_static() && r_a.length_squared() > 1e-6 {
            let rn = r_a.cross(normal);
            let inv_i_rn = Vector3D::new(
                rn.x * body_a.inv_inertia.x,
                rn.y * body_a.inv_inertia.y,
                rn.z * body_a.inv_inertia.z,
            );
            inv_i_rn.cross(r_a).dot(normal)
        } else {
            0.0
        };

        let ang_b = if !body_b.is_static() && r_b.length_squared() > 1e-6 {
            let rn = r_b.cross(normal);
            let inv_i_rn = Vector3D::new(
                rn.x * body_b.inv_inertia.x,
                rn.y * body_b.inv_inertia.y,
                rn.z * body_b.inv_inertia.z,
            );
            inv_i_rn.cross(r_b).dot(normal)
        } else {
            0.0
        };

        let k_joint = linear_inv_mass + ang_a + ang_b;
        if k_joint <= 1e-7 {
            return;
        }

        // 1. Position-level constraint projection (XPBD)
        let correction_magnitude = (error / k_joint) * self.stiffness;
        let correction = normal * correction_magnitude;

        body_a.position -= correction * body_a.inverse_mass;
        body_b.position += correction * body_b.inverse_mass;

        // Apply angular correction to align limbs naturally
        if !body_a.is_static() && r_a.length_squared() > 1e-6 {
            let torque_a = r_a.cross(-correction);
            body_a.rotation += Vector3D::new(
                torque_a.x * body_a.inv_inertia.x,
                torque_a.y * body_a.inv_inertia.y,
                torque_a.z * body_a.inv_inertia.z,
            );
        }
        if !body_b.is_static() && r_b.length_squared() > 1e-6 {
            let torque_b = r_b.cross(correction);
            body_b.rotation += Vector3D::new(
                torque_b.x * body_b.inv_inertia.x,
                torque_b.y * body_b.inv_inertia.y,
                torque_b.z * body_b.inv_inertia.z,
            );
        }

        // 2. Velocity projection: eliminate separating/compressing velocity along constraint axis
        let v_a_anchor = body_a.velocity + body_a.angular_velocity.cross(r_a);
        let v_b_anchor = body_b.velocity + body_b.angular_velocity.cross(r_b);
        let rel_vel = v_a_anchor - v_b_anchor;
        let vel_along_axis = rel_vel.dot(normal);

        // Scale velocity correction per substep iteration to avoid over-damping
        let vel_correction = (vel_along_axis / k_joint) * (self.stiffness * 0.25);
        let vel_impulse = normal * vel_correction;

        body_a.velocity -= vel_impulse * body_a.inverse_mass;
        body_b.velocity += vel_impulse * body_b.inverse_mass;

        // 3. Muscle damping along constraint axis (stabilizes limbs and prevents rubber-banding)
        if self.damping > 0.0 {
            let damp_factor = (self.damping * 0.15).clamp(0.0, 0.4);
            let damp_impulse = normal * ((vel_along_axis / k_joint) * damp_factor);
            body_a.velocity -= damp_impulse * body_a.inverse_mass;
            body_b.velocity += damp_impulse * body_b.inverse_mass;
        }

        // Auto-wake connected limbs under tension or motion
        if error.abs() > 0.015 || vel_along_axis.abs() > 0.03 {
            body_a.wake();
            body_b.wake();
        }
    }
}
