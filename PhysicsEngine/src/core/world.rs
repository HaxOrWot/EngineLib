use std::collections::HashMap;
use crate::collision::narrowphase::Narrowphase;
use crate::core::body::RigidBody3D;
use crate::dynamics::integrator::Integrator;
use crate::math::vec3::Vector3D;
use crate::solver::contact::ContactSolver;

pub struct PhysicsWorld {
    pub bodies: HashMap<u32, RigidBody3D>,
    pub next_id: u32,
    pub gravity: Vector3D,
}

impl PhysicsWorld {
    pub fn new(gx: f32, gy: f32, gz: f32) -> Self {
        Self {
            bodies: HashMap::new(),
            next_id: 1,
            gravity: Vector3D::new(gx, gy, gz),
        }
    }

    pub fn add_body(&mut self, pos: Vector3D, size: Vector3D, mass: f32, restitution: f32) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let body = RigidBody3D::new(id, pos, size, mass, restitution);
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

    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() {
            return;
        }

        // Phase 1: Numerical Integration
        for body in self.bodies.values_mut() {
            Integrator::integrate(body, self.gravity, dt);
        }

        // Phase 2 & 3: Collision Detection and Contact Resolution
        let keys: Vec<u32> = self.bodies.keys().copied().collect();
        let len = keys.len();

        for i in 0..len {
            for j in (i + 1)..len {
                let id_a = keys[i];
                let id_b = keys[j];

                // Fetch references safely via raw pointers to allow disjoint mutable borrows
                let ptr_a = self.bodies.get_mut(&id_a).unwrap() as *mut RigidBody3D;
                let ptr_b = self.bodies.get_mut(&id_b).unwrap() as *mut RigidBody3D;

                unsafe {
                    let body_a = &mut *ptr_a;
                    let body_b = &mut *ptr_b;

                    if let Some(manifold) = Narrowphase::test_aabb(body_a, body_b) {
                        ContactSolver::resolve(body_a, body_b, &manifold);
                    }
                }
            }
        }
    }
}
