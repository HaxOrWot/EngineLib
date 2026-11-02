use std::collections::HashMap;
use crate::collision::broadphase::SpatialHash;
use crate::collision::narrowphase::Narrowphase;
use crate::core::body::{RigidBody3D, ShapeType};
use crate::dynamics::integrator::Integrator;
use crate::math::ray::{Ray, RaycastHit};
use crate::math::vec3::Vector3D;
use crate::solver::contact::ContactSolver;

pub struct PhysicsWorld {
    pub bodies: HashMap<u32, RigidBody3D>,
    pub next_id: u32,
    pub gravity: Vector3D,
    spatial_hash: SpatialHash,
}

impl PhysicsWorld {
    pub fn new(gx: f32, gy: f32, gz: f32) -> Self {
        Self {
            bodies: HashMap::new(),
            next_id: 1,
            gravity: Vector3D::new(gx, gy, gz),
            spatial_hash: SpatialHash::new(4.0),
        }
    }

    pub fn add_body(&mut self, pos: Vector3D, size: Vector3D, mass: f32, restitution: f32) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let body = RigidBody3D::new(id, pos, size, mass, restitution);
        self.bodies.insert(id, body);
        id
    }

    pub fn add_sphere(
        &mut self,
        pos: Vector3D,
        radius: f32,
        mass: f32,
        restitution: f32,
        friction: f32,
    ) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let body = RigidBody3D::new_sphere(id, pos, radius, mass, restitution, friction);
        self.bodies.insert(id, body);
        id
    }

    pub fn remove_body(&mut self, id: u32) -> bool {
        self.bodies.remove(&id).is_some()
    }

    pub fn get_body(&self, id: u32) -> Option<&RigidBody3D> {
        self.bodies.get(&id)
    }

    pub fn get_body_mut(&mut self, id: u32) -> Option<&mut RigidBody3D> {
        self.bodies.get_mut(&id)
    }

    /// Performs scene raycast against all bodies, returning the closest hit
    pub fn raycast(&self, ray: &Ray, max_dist: f32) -> Option<RaycastHit> {
        let mut closest_hit: Option<RaycastHit> = None;
        let mut closest_dist = max_dist;

        for (id, body) in self.bodies.iter() {
            let hit = if body.shape_type == ShapeType::Sphere as u32 {
                ray.intersect_sphere(body.position, body.radius, closest_dist)
            } else {
                ray.intersect_aabb(body.position, body.size, closest_dist)
            };

            if let Some((t, normal)) = hit {
                if t < closest_dist {
                    closest_dist = t;
                    closest_hit = Some(RaycastHit {
                        point: ray.origin + ray.direction * t,
                        normal,
                        distance: t,
                        body_id: *id,
                    });
                }
            }
        }

        closest_hit
    }

    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() {
            return;
        }

        // Phase 1: Numerical Integration
        for body in self.bodies.values_mut() {
            Integrator::integrate(body, self.gravity, dt);
        }

        // Phase 2: Spatial Hash Broadphase
        // (For small worlds <= 20 bodies, brute force has lower constant overhead; for > 20, spatial hash scales O(N))
        let candidate_pairs = if self.bodies.len() <= 20 {
            let keys: Vec<u32> = self.bodies.keys().copied().collect();
            let mut pairs = Vec::new();
            for i in 0..keys.len() {
                for j in (i + 1)..keys.len() {
                    pairs.push((keys[i], keys[j]));
                }
            }
            pairs
        } else {
            self.spatial_hash.generate_candidate_pairs(&self.bodies)
        };

        // Phase 3: Narrowphase & Contact Resolution
        for (id_a, id_b) in candidate_pairs {
            // Disjoint lookup validation
            if !self.bodies.contains_key(&id_a) || !self.bodies.contains_key(&id_b) {
                continue;
            }

            let ptr_a = self.bodies.get_mut(&id_a).unwrap() as *mut RigidBody3D;
            let ptr_b = self.bodies.get_mut(&id_b).unwrap() as *mut RigidBody3D;

            unsafe {
                let body_a = &mut *ptr_a;
                let body_b = &mut *ptr_b;

                // Skip if both bodies are asleep or static
                if (body_a.is_static() || body_a.is_sleeping == 1)
                    && (body_b.is_static() || body_b.is_sleeping == 1)
                {
                    continue;
                }

                if let Some(manifold) = Narrowphase::test_collision(body_a, body_b) {
                    ContactSolver::resolve(body_a, body_b, &manifold);
                }
            }
        }
    }
}
