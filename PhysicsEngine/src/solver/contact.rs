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

        // 2. Normal & Friction Resolution
        if let Some(cp2) = manifold.contact_point_2 {
            Self::resolve_dual_points(body_a, body_b, normal, manifold.contact_point, cp2, effective_penetration, dt);
        } else {
            Self::resolve_single_point(body_a, body_b, normal, manifold.contact_point, effective_penetration, dt);
        }

        // 3. Area Contact Rotational Friction Damping for face-face contacts
        if manifold.contact_point_2.is_some() {
            let rel_ang_z = body_a.angular_velocity.z - body_b.angular_velocity.z;
            if rel_ang_z.abs() > 1e-4 {
                let rot_inv_inertia = (if !body_a.is_static() { body_a.inv_inertia.z } else { 0.0 })
                    + (if !body_b.is_static() { body_b.inv_inertia.z } else { 0.0 });
                if rot_inv_inertia > 1e-6 {
                    let friction_coeff = (body_a.friction * body_b.friction).sqrt().max(0.6);
                    let damping_impulse = (rel_ang_z * 0.5 / rot_inv_inertia).clamp(-friction_coeff * 3.0, friction_coeff * 3.0);
                    if !body_a.is_static() {
                        body_a.angular_velocity.z -= damping_impulse * body_a.inv_inertia.z;
                    }
                    if !body_b.is_static() {
                        body_b.angular_velocity.z += damping_impulse * body_b.inv_inertia.z;
                    }
                }
            }
        }

        // 4. Rolling Resistance Damping for Spheres
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
    }

    #[inline]
    fn rot_normal_mass(body: &RigidBody3D, r: Vector3D, normal: Vector3D) -> f32 {
        if body.is_static() {
            0.0
        } else {
            let rn = r.cross(normal);
            let inv_i_rn = Vector3D::new(
                rn.x * body.inv_inertia.x,
                rn.y * body.inv_inertia.y,
                rn.z * body.inv_inertia.z,
            );
            inv_i_rn.cross(r).dot(normal)
        }
    }

    fn resolve_dual_points(
        body_a: &mut RigidBody3D,
        body_b: &mut RigidBody3D,
        normal: Vector3D,
        p1: Vector3D,
        p2: Vector3D,
        effective_penetration: f32,
        dt: f32,
    ) {
        let linear_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if linear_inv_mass <= 0.0 {
            return;
        }

        let r_a1 = p1 - body_a.position;
        let r_a2 = p2 - body_a.position;
        let r_b1 = p1 - body_b.position;
        let r_b2 = p2 - body_b.position;

        let v_rel1 = (body_a.velocity + body_a.angular_velocity.cross(r_a1))
            - (body_b.velocity + body_b.angular_velocity.cross(r_b1));
        let v_rel2 = (body_a.velocity + body_a.angular_velocity.cross(r_a2))
            - (body_b.velocity + body_b.angular_velocity.cross(r_b2));

        let vn1 = v_rel1.dot(normal);
        let vn2 = v_rel2.dot(normal);

        let k1 = linear_inv_mass + Self::rot_normal_mass(body_a, r_a1, normal) + Self::rot_normal_mass(body_b, r_b1, normal);
        let k2 = linear_inv_mass + Self::rot_normal_mass(body_a, r_a2, normal) + Self::rot_normal_mass(body_b, r_b2, normal);

        if k1 <= 1e-7 || k2 <= 1e-7 {
            return;
        }

        // Symmetric normal impulse resolution
        let vn_avg = (vn1 + vn2) * 0.5;
        let vn_diff = (vn2 - vn1) * 0.5;

        let mut total_impulse_scalar = 0.0;

        if vn_avg < 0.0 || vn_diff.abs() > 1e-4 {
            let e = if vn1.abs().max(vn2.abs()) < Self::RESTING_VEL_THRESHOLD {
                0.0
            } else {
                body_a.restitution.min(body_b.restitution)
            };

            let j_avg = if vn_avg < 0.0 {
                -(1.0 + e) * vn_avg / ((k1 + k2) * 0.5)
            } else {
                0.0
            };

            let j_diff = -vn_diff / ((k1 + k2) * 0.5);

            let j1 = (j_avg * 0.5 - j_diff).max(0.0);
            let j2 = (j_avg * 0.5 + j_diff).max(0.0);
            total_impulse_scalar = j1 + j2;

            let impulse1 = normal * j1;
            let impulse2 = normal * j2;
            let total_impulse = impulse1 + impulse2;

            body_a.velocity += total_impulse * body_a.inverse_mass;
            body_b.velocity -= total_impulse * body_b.inverse_mass;

            if !body_a.is_static() {
                let torque_a = r_a1.cross(impulse1) + r_a2.cross(impulse2);
                body_a.angular_velocity += Vector3D::new(
                    torque_a.x * body_a.inv_inertia.x,
                    torque_a.y * body_a.inv_inertia.y,
                    torque_a.z * body_a.inv_inertia.z,
                );
            }
            if !body_b.is_static() {
                let torque_b = r_b1.cross(-impulse1) + r_b2.cross(-impulse2);
                body_b.angular_velocity += Vector3D::new(
                    torque_b.x * body_b.inv_inertia.x,
                    torque_b.y * body_b.inv_inertia.y,
                    torque_b.z * body_b.inv_inertia.z,
                );
            }

            if vn1.abs() > Self::RESTING_VEL_THRESHOLD || vn2.abs() > Self::RESTING_VEL_THRESHOLD {
                body_a.wake();
                body_b.wake();
            }
        }

        // Mid-point friction resolution
        let mid_point = (p1 + p2) * 0.5;
        Self::resolve_friction(body_a, body_b, normal, mid_point, total_impulse_scalar, effective_penetration, dt);
    }

    fn resolve_single_point(
        body_a: &mut RigidBody3D,
        body_b: &mut RigidBody3D,
        normal: Vector3D,
        contact_point: Vector3D,
        effective_penetration: f32,
        dt: f32,
    ) {
        let linear_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if linear_inv_mass <= 0.0 {
            return;
        }

        let r_a = contact_point - body_a.position;
        let r_b = contact_point - body_b.position;

        let v_a_contact = body_a.velocity + body_a.angular_velocity.cross(r_a);
        let v_b_contact = body_b.velocity + body_b.angular_velocity.cross(r_b);
        let rel_vel = v_a_contact - v_b_contact;
        let vel_along_normal = rel_vel.dot(normal);

        let k_normal = linear_inv_mass
            + Self::rot_normal_mass(body_a, r_a, normal)
            + Self::rot_normal_mass(body_b, r_b, normal);

        if k_normal <= 1e-7 {
            return;
        }

        let mut impulse_scalar = 0.0;

        if vel_along_normal < 0.0 {
            let e = if vel_along_normal.abs() < Self::RESTING_VEL_THRESHOLD {
                0.0
            } else {
                body_a.restitution.min(body_b.restitution)
            };

            impulse_scalar = -(1.0 + e) * vel_along_normal / k_normal;
            let impulse = normal * impulse_scalar;
            body_a.velocity += impulse * body_a.inverse_mass;
            body_b.velocity -= impulse * body_b.inverse_mass;

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

            if vel_along_normal.abs() > Self::RESTING_VEL_THRESHOLD {
                body_a.wake();
                body_b.wake();
            }
        }

        Self::resolve_friction(body_a, body_b, normal, contact_point, impulse_scalar, effective_penetration, dt);
    }

    fn resolve_friction(
        body_a: &mut RigidBody3D,
        body_b: &mut RigidBody3D,
        normal: Vector3D,
        contact_point: Vector3D,
        normal_impulse_scalar: f32,
        effective_penetration: f32,
        dt: f32,
    ) {
        let linear_inv_mass = body_a.inverse_mass + body_b.inverse_mass;
        if linear_inv_mass <= 0.0 {
            return;
        }

        let r_a = contact_point - body_a.position;
        let r_b = contact_point - body_b.position;

        let sustained_normal_impulse = (effective_penetration * 20.0).max(9.81 * dt) / linear_inv_mass;
        let normal_pressure = normal_impulse_scalar.max(sustained_normal_impulse);

        let updated_v_a = body_a.velocity + body_a.angular_velocity.cross(r_a);
        let updated_v_b = body_b.velocity + body_b.angular_velocity.cross(r_b);
        let updated_rel_vel = updated_v_a - updated_v_b;
        let tangent_vel = updated_rel_vel - normal * updated_rel_vel.dot(normal);
        let tangent_speed = tangent_vel.length();

        if tangent_speed > 1e-4 {
            let tangent = tangent_vel * (1.0 / tangent_speed);

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
    }
}
