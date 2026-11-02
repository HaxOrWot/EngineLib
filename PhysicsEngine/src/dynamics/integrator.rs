use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

pub struct Integrator;

impl Integrator {
    const SLEEP_VELOCITY_THRESHOLD_SQ: f32 = 0.05 * 0.05; // 0.05 m/s threshold
    const SLEEP_TIME_REQUIRED: f32 = 0.5; // 0.5 seconds of near-rest to sleep

    /// Advances body kinematics by timestep dt using Symplectic (Semi-Implicit) Euler
    #[inline]
    pub fn integrate(body: &mut RigidBody3D, gravity: Vector3D, dt: f32) {
        if body.is_static() {
            return;
        }

        if body.is_sleeping == 1 {
            body.acceleration = Vector3D::ZERO;
            return;
        }

        // 1. Advance velocity by acceleration + global gravity
        body.velocity += (body.acceleration + gravity) * dt;

        // 2. Velocity damping / sleeping check
        let speed_sq = body.velocity.length_squared();
        if speed_sq < Self::SLEEP_VELOCITY_THRESHOLD_SQ {
            body.sleep_timer += dt;
            if body.sleep_timer >= Self::SLEEP_TIME_REQUIRED {
                body.is_sleeping = 1;
                body.velocity = Vector3D::ZERO;
                body.acceleration = Vector3D::ZERO;
                return;
            }
        } else {
            body.sleep_timer = 0.0;
        }

        // 3. Advance position by newly computed velocity
        body.position += body.velocity * dt;

        // 4. Reset accumulated forces for next frame
        body.acceleration = Vector3D::ZERO;
    }
}
