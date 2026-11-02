use crate::math::vec3::Vector3D;

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeType {
    AABB = 0,
    Sphere = 1,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RigidBody3D {
    pub id: u32,
    pub position: Vector3D,
    pub velocity: Vector3D,
    pub acceleration: Vector3D,
    pub rotation: Vector3D,
    pub angular_velocity: Vector3D,
    pub torque: Vector3D,
    pub inv_inertia: Vector3D,
    pub size: Vector3D,
    pub mass: f32,
    pub inverse_mass: f32,
    pub restitution: f32,
    pub friction: f32,
    pub radius: f32,
    pub shape_type: u32,
    pub is_sleeping: u32, // 0 = active, 1 = sleeping
    pub sleep_timer: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

impl RigidBody3D {
    pub fn new(id: u32, position: Vector3D, size: Vector3D, mass: f32, restitution: f32) -> Self {
        let inv_mass = if mass > 0.0001 { 1.0 / mass } else { 0.0 };
        let clamped_restitution = restitution.clamp(0.0, 1.0);
        let radius = (size.x.max(size.y).max(size.z)) * 0.5;

        // Solid box inertia: I_x = 1/12 * m * (h^2 + d^2), etc.
        let inv_inertia = if inv_mass > 0.0 {
            let ix = (1.0 / 12.0) * mass * (size.y * size.y + size.z * size.z);
            let iy = (1.0 / 12.0) * mass * (size.x * size.x + size.z * size.z);
            let iz = (1.0 / 12.0) * mass * (size.x * size.x + size.y * size.y);
            Vector3D::new(
                if ix > 1e-5 { 1.0 / ix } else { 0.0 },
                if iy > 1e-5 { 1.0 / iy } else { 0.0 },
                if iz > 1e-5 { 1.0 / iz } else { 0.0 },
            )
        } else {
            Vector3D::ZERO
        };

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
            rotation: Vector3D::ZERO,
            angular_velocity: Vector3D::ZERO,
            torque: Vector3D::ZERO,
            inv_inertia,
            size,
            mass,
            inverse_mass: inv_mass,
            restitution: clamped_restitution,
            friction: 0.5,
            radius,
            shape_type: ShapeType::AABB as u32,
            is_sleeping: 0,
            sleep_timer: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.15,
        }
    }

    pub fn new_sphere(
        id: u32,
        position: Vector3D,
        radius: f32,
        mass: f32,
        restitution: f32,
        friction: f32,
    ) -> Self {
        let inv_mass = if mass > 0.0001 { 1.0 / mass } else { 0.0 };
        let clamped_restitution = restitution.clamp(0.0, 1.0);
        let clamped_friction = friction.max(0.0);
        let diam = radius * 2.0;

        // Solid sphere inertia: I = 2/5 * m * r^2
        let inv_inertia = if inv_mass > 0.0 {
            let i = (2.0 / 5.0) * mass * radius * radius;
            let inv_i = if i > 1e-5 { 1.0 / i } else { 0.0 };
            Vector3D::new(inv_i, inv_i, inv_i)
        } else {
            Vector3D::ZERO
        };

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
            rotation: Vector3D::ZERO,
            angular_velocity: Vector3D::ZERO,
            torque: Vector3D::ZERO,
            inv_inertia,
            size: Vector3D::new(diam, diam, diam),
            mass,
            inverse_mass: inv_mass,
            restitution: clamped_restitution,
            friction: clamped_friction,
            radius,
            shape_type: ShapeType::Sphere as u32,
            is_sleeping: 0,
            sleep_timer: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.15,
        }
    }

    #[inline(always)]
    pub fn is_static(&self) -> bool {
        self.inverse_mass == 0.0
    }

    #[inline(always)]
    pub fn wake(&mut self) {
        self.is_sleeping = 0;
        self.sleep_timer = 0.0;
    }

    #[inline(always)]
    pub fn apply_force(&mut self, force: Vector3D) {
        if !self.is_static() {
            self.wake();
            self.acceleration += force * self.inverse_mass;
        }
    }

    #[inline(always)]
    pub fn apply_torque(&mut self, torque: Vector3D) {
        if !self.is_static() {
            self.wake();
            self.torque += torque;
        }
    }

    #[inline(always)]
    pub fn apply_impulse_at_point(&mut self, impulse: Vector3D, point: Vector3D) {
        if !self.is_static() {
            self.wake();
            self.velocity += impulse * self.inverse_mass;
            let r = point - self.position;
            let torque_impulse = r.cross(impulse);
            self.angular_velocity += Vector3D::new(
                torque_impulse.x * self.inv_inertia.x,
                torque_impulse.y * self.inv_inertia.y,
                torque_impulse.z * self.inv_inertia.z,
            );
        }
    }

    #[inline(always)]
    pub fn aabb_bounds(&self) -> (Vector3D, Vector3D) {
        let half = self.size * 0.5;
        (self.position - half, self.position + half)
    }
}
