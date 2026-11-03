use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

pub struct Integrator;

impl Integrator {
    const SLEEP_VELOCITY_THRESHOLD_SQ: f32 = 0.08 * 0.08; // 0.08 m/s (~2.0 px/s threshold)
    const SLEEP_ANGULAR_THRESHOLD_SQ: f32 = 0.08 * 0.08;
    const SLEEP_TIME_REQUIRED: f32 = 0.35; // 0.35 seconds of near-rest to sleep

    /// Advances body kinematics by timestep dt using Symplectic (Semi-Implicit) Euler with linear & angular damping
    #[inline]
    pub fn integrate(body: &mut RigidBody3D, gravity: Vector3D, dt: f32) {
        if body.is_static() {
            return;
        }

        if body.is_sleeping == 1 {
            body.acceleration = Vector3D::ZERO;
            body.torque = Vector3D::ZERO;
            return;
        }

        // 1. Advance linear velocity by acceleration + global gravity
        body.velocity += (body.acceleration + gravity) * dt;

        // 2. Air resistance / linear drag damping
        if body.linear_damping > 0.0 {
            let damping_factor = 1.0 / (1.0 + body.linear_damping * dt);
            body.velocity *= damping_factor;
        }

        // 3. Angular acceleration & integration
        let ang_accel = Vector3D::new(
            body.torque.x * body.inv_inertia.x,
            body.torque.y * body.inv_inertia.y,
            body.torque.z * body.inv_inertia.z,
        );
        body.angular_velocity += ang_accel * dt;

        if body.angular_damping > 0.0 {
            let ang_drag = 1.0 / (1.0 + body.angular_damping * dt);
            body.angular_velocity *= ang_drag;
        }

        // 4. Sleeping check (both linear and angular velocity)
        let speed_sq = body.velocity.length_squared();
        let ang_speed_sq = body.angular_velocity.length_squared();
        if speed_sq < Self::SLEEP_VELOCITY_THRESHOLD_SQ && ang_speed_sq < Self::SLEEP_ANGULAR_THRESHOLD_SQ {
            body.sleep_timer += dt;
            if body.sleep_timer >= Self::SLEEP_TIME_REQUIRED {
                body.is_sleeping = 1;
                body.velocity = Vector3D::ZERO;
                body.angular_velocity = Vector3D::ZERO;
                body.acceleration = Vector3D::ZERO;
                body.torque = Vector3D::ZERO;
                return;
            }
        } else {
            body.sleep_timer = 0.0;
        }

        // 5. Advance position & orientation (Quaternion integration without gimbal lock)
        body.position += body.velocity * dt;
        body.orientation = body.orientation.integrate(body.angular_velocity, dt);
        body.rotation = body.orientation.to_euler();

        // 6. Reset accumulated forces for next frame
        body.acceleration = Vector3D::ZERO;
        body.torque = Vector3D::ZERO;
    }
}
