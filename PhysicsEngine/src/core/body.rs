use crate::math::quat::Quaternion;
use crate::math::vec3::Vector3D;

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeType {
    AABB = 0,
    Sphere = 1,
    Capsule = 2,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RigidBody3D {
    pub id: u32,
    pub position: Vector3D,
    pub velocity: Vector3D,
    pub acceleration: Vector3D,
    pub rotation: Vector3D, // Euler angles for legacy / C-ABI compatibility
    pub orientation: Quaternion, // Full 3D unit quaternion orientation
    pub angular_velocity: Vector3D,
    pub torque: Vector3D,
    pub inv_inertia: Vector3D,
    pub size: Vector3D,
    pub mass: f32,
    pub inverse_mass: f32,
    pub restitution: f32,
    pub friction: f32,
    pub radius: f32,
    pub height: f32,       // Cylinder segment height for Capsule
    pub shape_type: u32,
    pub is_sleeping: u32,  // 0 = active, 1 = sleeping
    pub is_sensor: u32,    // 0 = physical collider, 1 = trigger/sensor (no contact force)
    pub enable_ccd: u32,   // 0 = discrete, 1 = continuous collision detection
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
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector3D::ZERO,
            torque: Vector3D::ZERO,
            inv_inertia,
            size,
            mass,
            inverse_mass: inv_mass,
            restitution: clamped_restitution,
            friction: 0.5,
            radius,
            height: 0.0,
            shape_type: ShapeType::AABB as u32,
            is_sleeping: 0,
            is_sensor: 0,
            enable_ccd: 0,
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
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector3D::ZERO,
            torque: Vector3D::ZERO,
            inv_inertia,
            size: Vector3D::new(diam, diam, diam),
            mass,
            inverse_mass: inv_mass,
            restitution: clamped_restitution,
            friction: clamped_friction,
            radius,
            height: 0.0,
            shape_type: ShapeType::Sphere as u32,
            is_sleeping: 0,
            is_sensor: 0,
            enable_ccd: 0,
            sleep_timer: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.15,
        }
    }

    pub fn new_capsule(
        id: u32,
        position: Vector3D,
        radius: f32,
        height: f32,
        mass: f32,
        restitution: f32,
        friction: f32,
    ) -> Self {
        let inv_mass = if mass > 0.0001 { 1.0 / mass } else { 0.0 };
        let clamped_restitution = restitution.clamp(0.0, 1.0);
        let clamped_friction = friction.max(0.0);

        // Approximate cylinder inertia: I_y = 1/2 m r^2; I_x = I_z = m * (1/12 h^2 + 1/4 r^2)
        let inv_inertia = if inv_mass > 0.0 {
            let iy = 0.5 * mass * radius * radius;
            let ixz = mass * ((1.0 / 12.0) * height * height + 0.25 * radius * radius);
            Vector3D::new(
                if ixz > 1e-5 { 1.0 / ixz } else { 0.0 },
                if iy > 1e-5 { 1.0 / iy } else { 0.0 },
                if ixz > 1e-5 { 1.0 / ixz } else { 0.0 },
            )
        } else {
            Vector3D::ZERO
        };

        let total_h = height + radius * 2.0;
        let diam = radius * 2.0;

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
            rotation: Vector3D::ZERO,
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector3D::ZERO,
            torque: Vector3D::ZERO,
            inv_inertia,
            size: Vector3D::new(diam, total_h, diam),
            mass,
            inverse_mass: inv_mass,
            restitution: clamped_restitution,
            friction: clamped_friction,
            radius,
            height,
            shape_type: ShapeType::Capsule as u32,
            is_sleeping: 0,
            is_sensor: 0,
            enable_ccd: 0,
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

    /// Endpoints of the capsule line segment in world space
    #[inline]
    pub fn capsule_segment(&self) -> (Vector3D, Vector3D) {
        let up = self.orientation.rotate_vector(Vector3D::UP);
        let half_h = self.height * 0.5;
        (self.position - up * half_h, self.position + up * half_h)
    }

    #[inline(always)]
    pub fn aabb_bounds(&self) -> (Vector3D, Vector3D) {
        if self.shape_type == ShapeType::Capsule as u32 {
            let (p1, p2) = self.capsule_segment();
            let min_x = p1.x.min(p2.x) - self.radius;
            let min_y = p1.y.min(p2.y) - self.radius;
            let min_z = p1.z.min(p2.z) - self.radius;
            let max_x = p1.x.max(p2.x) + self.radius;
            let max_y = p1.y.max(p2.y) + self.radius;
            let max_z = p1.z.max(p2.z) + self.radius;
            (
                Vector3D::new(min_x, min_y, min_z),
                Vector3D::new(max_x, max_y, max_z),
            )
        } else {
            let half = self.size * 0.5;
            (self.position - half, self.position + half)
        }
    }
}
