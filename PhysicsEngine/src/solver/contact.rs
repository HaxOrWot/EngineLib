use crate::collision::narrowphase::ContactManifold;
use crate::core::body::{RigidBody3D, ShapeType};
use crate::math::vec3::Vector3D;

pub struct ContactSolver;

impl ContactSolver {
    const PENETRATION_SLOP: f32 = 0.001; // 1 mm allowance to eliminate visible floor sinking
    const BAUMGARTE_FACTOR: f32 = 0.35;  // Smooth 35% positional correction per step without bounce
    const MAX_POSITIONAL_CORRECTION: f32 = 0.08; // Clamp to avoid aggressive repulsion
    const RESTING_VEL_THRESHOLD: f32 = 0.25; // Velocity threshold to kill micro-bounce jitter
    const ROLLING_RESISTANCE_COEFF: f32 = 0.16; // Realistic rolling resistance (scales with surface friction)

    pub fn resolve(
        body_a: &mut RigidBody3D,
        body_b: &mut RigidBody3D,
        manifold: &ContactManifold,
        dt: f32,
    ) {
        let linear_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if linear_inv_mass <= 0.0 {
            return; // Immovable terrain vs immovable terrain
        }

        let normal = manifold.normal;
        let penetration = manifold.penetration;

        // 1. Baumgarte Positional Correction (Anti-Sinking)
        let effective_penetration = (penetration - Self::PENETRATION_SLOP).max(0.0);
        if effective_penetration > 0.0 {
            let correction_scalar = ((effective_penetration / linear_inv_mass) * Self::BAUMGARTE_FACTOR)
                .min(Self::MAX_POSITIONAL_CORRECTION);
            body_a.position += normal * (body_a.inverse_mass * correction_scalar);
            body_b.position -= normal * (body_b.inverse_mass * correction_scalar);
        }

        // Contact lever arms from center of mass to contact point
        let r_a = manifold.contact_point - body_a.position;
        let r_b = manifold.contact_point - body_b.position;

        // Relative contact velocity including rotational motion
        let v_a_contact = body_a.velocity + body_a.angular_velocity.cross(r_a);
        let v_b_contact = body_b.velocity + body_b.angular_velocity.cross(r_b);
        let rel_vel = v_a_contact - v_b_contact;
        let vel_along_normal = rel_vel.dot(normal);

        // Compute effective normal mass including rotational inertia:
        // K_n = m_A^-1 + m_B^-1 + ((I_A^-1 * (r_A x n)) x r_A) . n + ((I_B^-1 * (r_B x n)) x r_B) . n
        let ang_comp_a = if !body_a.is_static() {
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

        let ang_comp_b = if !body_b.is_static() {
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

        let k_normal = linear_inv_mass + ang_comp_a + ang_comp_b;
        if k_normal <= 1e-7 {
            return;
        }

        let mut impulse_scalar = 0.0;

        // 2. Elastic Restitution Impulse (only when closing in)
        if vel_along_normal < 0.0 {
            let e = if vel_along_normal.abs() < Self::RESTING_VEL_THRESHOLD {
                0.0 // Eliminate jitter for resting contact
            } else {
                body_a.restitution.min(body_b.restitution)
            };

            impulse_scalar = -(1.0 + e) * vel_along_normal / k_normal;
            let impulse = normal * impulse_scalar;
            body_a.velocity += impulse * body_a.inverse_mass;
            body_b.velocity -= impulse * body_b.inverse_mass;

            // Apply rotational torque from off-center impact
            if !body_a.is_static() {
                let torque_a = r_a.cross(impulse);
                body_a.angular_velocity += Vector3D::new(
                    torque_a.x * body_a.inv_inertia.x,
                    torque_a.y * body_a.inv_inertia.y,
                    torque_a.z * body_a.inv_inertia.z,
                );
            }
            if !body_b.is_static() {
                let torque_b = r_b.cross(-impulse);
                body_b.angular_velocity += Vector3D::new(
                    torque_b.x * body_b.inv_inertia.x,
                    torque_b.y * body_b.inv_inertia.y,
                    torque_b.z * body_b.inv_inertia.z,
                );
            }
        }

        // 3. Coulomb Tangential Friction Dynamics & Pure Rolling Transition
        let sustained_normal_impulse = (effective_penetration * 20.0).max(9.81 * dt) / linear_inv_mass;
        let normal_pressure = impulse_scalar.max(sustained_normal_impulse);

        let updated_v_a = body_a.velocity + body_a.angular_velocity.cross(r_a);
        let updated_v_b = body_b.velocity + body_b.angular_velocity.cross(r_b);
        let updated_rel_vel = updated_v_a - updated_v_b;
        let tangent_vel = updated_rel_vel - normal * updated_rel_vel.dot(normal);
        let tangent_speed = tangent_vel.length();

        if tangent_speed > 1e-4 {
            let tangent = tangent_vel * (1.0 / tangent_speed);

            // Compute effective tangential mass including rotational inertia:
            let ang_tangent_a = if !body_a.is_static() {
                let rt = r_a.cross(tangent);
                let inv_i_rt = Vector3D::new(
                    rt.x * body_a.inv_inertia.x,
                    rt.y * body_a.inv_inertia.y,
                    rt.z * body_a.inv_inertia.z,
                );
                inv_i_rt.cross(r_a).dot(tangent)
            } else {
                0.0
            };

            let ang_tangent_b = if !body_b.is_static() {
                let rt = r_b.cross(tangent);
                let inv_i_rt = Vector3D::new(
                    rt.x * body_b.inv_inertia.x,
                    rt.y * body_b.inv_inertia.y,
                    rt.z * body_b.inv_inertia.z,
                );
                inv_i_rt.cross(r_b).dot(tangent)
            } else {
                0.0
            };

            let k_tangent = linear_inv_mass + ang_tangent_a + ang_tangent_b;
            if k_tangent > 1e-7 {
                let friction_coeff = (body_a.friction * body_b.friction).sqrt();
                let friction_impulse_ideal = -tangent_speed / k_tangent;
                let max_friction_impulse = friction_coeff * normal_pressure;
                let friction_impulse_clamped = friction_impulse_ideal.max(-max_friction_impulse);

                let friction_vec = tangent * friction_impulse_clamped;
                body_a.velocity += friction_vec * body_a.inverse_mass;
                body_b.velocity -= friction_vec * body_b.inverse_mass;

                // Tangential torque inducing realistic roll and spin
                if !body_a.is_static() {
                    let f_torque_a = r_a.cross(friction_vec);
                    body_a.angular_velocity += Vector3D::new(
                        f_torque_a.x * body_a.inv_inertia.x,
                        f_torque_a.y * body_a.inv_inertia.y,
                        f_torque_a.z * body_a.inv_inertia.z,
                    );
                }
                if !body_b.is_static() {
                    let f_torque_b = r_b.cross(-friction_vec);
                    body_b.angular_velocity += Vector3D::new(
                        f_torque_b.x * body_b.inv_inertia.x,
                        f_torque_b.y * body_b.inv_inertia.y,
                        f_torque_b.z * body_b.inv_inertia.z,
                    );
                }
            }
        }

        // 4. Rolling Resistance Damping for Spheres (Smooth, physical deceleration)
        let is_sphere_a = body_a.shape_type == ShapeType::Sphere as u32;
        let is_sphere_b = body_b.shape_type == ShapeType::Sphere as u32;

        if is_sphere_a && !body_a.is_static() {
            let speed = body_a.velocity.length();
            let ang_speed = body_a.angular_velocity.length();
            let friction_coeff = (body_a.friction * body_b.friction).sqrt().max(0.1);
            let decel = Self::ROLLING_RESISTANCE_COEFF * friction_coeff * 9.81 * dt;

            if speed > 0.0 {
                let new_speed = (speed - decel).max(0.0);
                body_a.velocity *= new_speed / speed;
            }
            if ang_speed > 0.0 && body_a.radius > 0.0 {
                let ang_decel = decel / body_a.radius;
                let new_ang_speed = (ang_speed - ang_decel).max(0.0);
                body_a.angular_velocity *= new_ang_speed / ang_speed;
            }
        }

        if is_sphere_b && !body_b.is_static() {
            let speed = body_b.velocity.length();
            let ang_speed = body_b.angular_velocity.length();
            let friction_coeff = (body_a.friction * body_b.friction).sqrt().max(0.1);
            let decel = Self::ROLLING_RESISTANCE_COEFF * friction_coeff * 9.81 * dt;

            if speed > 0.0 {
                let new_speed = (speed - decel).max(0.0);
                body_b.velocity *= new_speed / speed;
            }
            if ang_speed > 0.0 && body_b.radius > 0.0 {
                let ang_decel = decel / body_b.radius;
                let new_ang_speed = (ang_speed - ang_decel).max(0.0);
                body_b.angular_velocity *= new_ang_speed / ang_speed;
            }
        }

        // Wake bodies if impact impulse or friction is significant
        if impulse_scalar.abs() > 0.05 || tangent_speed > 0.05 {
            body_a.wake();
            body_b.wake();
        }
    }
}
