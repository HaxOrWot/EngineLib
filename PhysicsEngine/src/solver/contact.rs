use crate::collision::narrowphase::ContactManifold;
use crate::core::body::{RigidBody3D, ShapeType};

pub struct ContactSolver;

impl ContactSolver {
    const PENETRATION_SLOP: f32 = 0.005; // 5 mm allowance
    const BAUMGARTE_FACTOR: f32 = 0.50;  // Resolve 50% of penetration per frame
    const RESTING_VEL_THRESHOLD: f32 = 0.35; // Threshold to kill micro-bounce jitter

    pub fn resolve(
        body_a: &mut RigidBody3D,
        body_b: &mut RigidBody3D,
        manifold: &ContactManifold,
        dt: f32,
    ) {
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

        let mut impulse_scalar = 0.0;

        // 3. Elastic Restitution Impulse (only when closing in)
        if vel_along_normal < 0.0 {
            let e = if vel_along_normal.abs() < Self::RESTING_VEL_THRESHOLD {
                0.0 // Eliminate jitter for resting contact
            } else {
                body_a.restitution.min(body_b.restitution)
            };

            impulse_scalar = -(1.0 + e) * vel_along_normal / total_inv_mass;
            let impulse = normal * impulse_scalar;
            body_a.velocity += impulse * body_a.inverse_mass;
            body_b.velocity -= impulse * body_b.inverse_mass;
        }

        // 4. Coulomb Tangential Friction Dynamics & Resting Normal Pressure
        // Even when resting on a surface (vel_along_normal >= 0), normal contact pressure
        // holds the body against the surface.
        let sustained_normal_impulse = (effective_penetration * 30.0).max(9.81 * dt) / total_inv_mass;
        let normal_pressure = impulse_scalar.max(sustained_normal_impulse);

        let updated_rel_vel = body_a.velocity - body_b.velocity;
        let tangent_vel = updated_rel_vel - normal * updated_rel_vel.dot(normal);
        let tangent_speed = tangent_vel.length();

        if tangent_speed > 1e-4 {
            let tangent = tangent_vel * (1.0 / tangent_speed);
            let friction_coeff = (body_a.friction * body_b.friction).sqrt();

            // Total impulse needed to stop tangential sliding
            let friction_impulse_ideal = -tangent_speed / total_inv_mass;
            // Coulomb's friction cone limit: |j_t| <= mu * j_n
            let max_friction_impulse = friction_coeff * normal_pressure;

            let friction_impulse_clamped = friction_impulse_ideal.max(-max_friction_impulse);

            let friction_vec = tangent * friction_impulse_clamped;
            body_a.velocity += friction_vec * body_a.inverse_mass;
            body_b.velocity -= friction_vec * body_b.inverse_mass;

            // Rolling resistance damping for spheres in contact
            let is_sphere_a = body_a.shape_type == ShapeType::Sphere as u32;
            let is_sphere_b = body_b.shape_type == ShapeType::Sphere as u32;
            if is_sphere_a || is_sphere_b {
                let roll_drag = (1.0 - (friction_coeff * 0.12 * dt * 60.0)).clamp(0.0, 1.0);
                if is_sphere_a && !body_a.is_static() {
                    body_a.velocity *= roll_drag;
                }
                if is_sphere_b && !body_b.is_static() {
                    body_b.velocity *= roll_drag;
                }
            }
        }

        // Wake bodies if impact impulse or friction is significant
        if impulse_scalar.abs() > 0.1 || tangent_speed > 0.1 {
            body_a.wake();
            body_b.wake();
        }
    }
}
