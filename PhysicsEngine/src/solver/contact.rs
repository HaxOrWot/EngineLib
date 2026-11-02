use crate::collision::narrowphase::ContactManifold;
use crate::core::body::RigidBody3D;

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

        // Do not resolve impulse if bodies are already separating
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
