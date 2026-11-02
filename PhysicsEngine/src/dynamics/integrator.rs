use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

pub struct Integrator;

impl Integrator {
    /// Advances body kinematics by timestep dt using Symplectic (Semi-Implicit) Euler
    #[inline]
    pub fn integrate(body: &mut RigidBody3D, gravity: Vector3D, dt: f32) {
        if body.is_static() {
            return;
        }

        // 1. Advance velocity by acceleration + global gravity
        body.velocity += (body.acceleration + gravity) * dt;

        // 2. Advance position by newly computed velocity
        body.position += body.velocity * dt;

        // 3. Reset accumulated forces for next frame
        body.acceleration = Vector3D::ZERO;
    }
}
