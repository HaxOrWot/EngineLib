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
}

impl RigidBody3D {
    pub fn new(id: u32, position: Vector3D, size: Vector3D, mass: f32, restitution: f32) -> Self {
        let inv_mass = if mass > 0.0001 { 1.0 / mass } else { 0.0 };
        let clamped_restitution = restitution.clamp(0.0, 1.0);
        let radius = (size.x.max(size.y).max(size.z)) * 0.5;

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
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

        Self {
            id,
            position,
            velocity: Vector3D::ZERO,
            acceleration: Vector3D::ZERO,
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

    /// Computes world-space axis-aligned bounding box (min, max) for broadphase
    #[inline]
    pub fn aabb_bounds(&self) -> (Vector3D, Vector3D) {
        if self.shape_type == ShapeType::Sphere as u32 {
            let r = Vector3D::new(self.radius, self.radius, self.radius);
            (self.position - r, self.position + r)
        } else {
            let half = self.size * 0.5;
            (self.position - half, self.position + half)
        }
    }
}
