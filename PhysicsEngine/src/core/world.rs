use std::collections::{HashMap, HashSet};
use crate::collision::broadphase::SpatialHash;
use crate::collision::narrowphase::Narrowphase;
use crate::core::body::{RigidBody3D, ShapeType};
use crate::dynamics::integrator::Integrator;
use crate::math::ray::{Ray, RaycastHit};
use crate::math::vec3::Vector3D;
use crate::solver::contact::ContactSolver;
use crate::solver::joint::DistanceJoint;

pub struct PhysicsWorld {
    pub bodies: HashMap<u32, RigidBody3D>,
    pub joints: HashMap<u32, DistanceJoint>,
    pub disabled_collision_pairs: HashSet<(u32, u32)>,
    pub next_id: u32,
    pub next_joint_id: u32,
    pub gravity: Vector3D,
    pub substeps: u32,
    pub solver_iterations: u32,
    spatial_hash: SpatialHash,
}

impl PhysicsWorld {
    pub fn new(gx: f32, gy: f32, gz: f32) -> Self {
        Self {
            bodies: HashMap::new(),
            joints: HashMap::new(),
            disabled_collision_pairs: HashSet::new(),
            next_id: 1,
            next_joint_id: 1,
            gravity: Vector3D::new(gx, gy, gz),
            substeps: 2,
            solver_iterations: 4,
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

    pub fn add_distance_joint(
        &mut self,
        body_a: u32,
        body_b: u32,
        offset_a: Vector3D,
        offset_b: Vector3D,
        rest_length: f32,
        stiffness: f32,
        damping: f32,
    ) -> u32 {
        let id = self.next_joint_id;
        self.next_joint_id += 1;
        let joint = DistanceJoint::new(
            id,
            body_a,
            body_b,
            offset_a,
            offset_b,
            rest_length,
            stiffness,
            damping,
        );
        self.joints.insert(id, joint);
        // Automatically disable collision between connected limbs to eliminate internal constraint fighting
        self.disable_collision_pair(body_a, body_b);
        id
    }

    pub fn disable_collision_pair(&mut self, body_a: u32, body_b: u32) {
        let key = (body_a.min(body_b), body_a.max(body_b));
        self.disabled_collision_pairs.insert(key);
    }

    pub fn enable_collision_pair(&mut self, body_a: u32, body_b: u32) {
        let key = (body_a.min(body_b), body_a.max(body_b));
        self.disabled_collision_pairs.remove(&key);
    }

    pub fn remove_joint(&mut self, id: u32) -> bool {
        if let Some(joint) = self.joints.remove(&id) {
            self.enable_collision_pair(joint.body_a, joint.body_b);
            true
        } else {
            false
        }
    }

    pub fn remove_body(&mut self, id: u32) -> bool {
        // Remove connected joints and their collision filters
        let affected_joints: Vec<u32> = self
            .joints
            .iter()
            .filter(|(_, j)| j.body_a == id || j.body_b == id)
            .map(|(&j_id, _)| j_id)
            .collect();

        for j_id in affected_joints {
            self.remove_joint(j_id);
        }

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

        let num_substeps = self.substeps.max(1);
        let sub_dt = dt / (num_substeps as f32);

        for _ in 0..num_substeps {
            self.step_substep(sub_dt);
        }
    }

    fn step_substep(&mut self, sub_dt: f32) {
        // Phase 1: Numerical Integration with Air Damping
        for body in self.bodies.values_mut() {
            Integrator::integrate(body, self.gravity, sub_dt);
        }

        // Phase 2: Spatial Hash Broadphase
        let candidate_pairs = if self.bodies.len() <= 20 {
            let keys: Vec<u32> = self.bodies.keys().copied().collect();
            let mut pairs = Vec::new();
            for i in 0..keys.len() {
                for j in (i + 1)..keys.len() {
                    let key = (keys[i].min(keys[j]), keys[i].max(keys[j]));
                    if !self.disabled_collision_pairs.contains(&key) {
                        pairs.push((keys[i], keys[j]));
                    }
                }
            }
            pairs
        } else {
            let raw_pairs = self.spatial_hash.generate_candidate_pairs(&self.bodies);
            raw_pairs
                .into_iter()
                .filter(|&(a, b)| {
                    let key = (a.min(b), a.max(b));
                    !self.disabled_collision_pairs.contains(&key)
                })
                .collect()
        };

        // Phase 3: Iterative Constraint & Contact Solver
        let iters = self.solver_iterations.max(1);
        for _ in 0..iters {
            // Solve Collisions
            for (id_a, id_b) in &candidate_pairs {
                if !self.bodies.contains_key(id_a) || !self.bodies.contains_key(id_b) {
                    continue;
                }

                let ptr_a = self.bodies.get_mut(id_a).unwrap() as *mut RigidBody3D;
                let ptr_b = self.bodies.get_mut(id_b).unwrap() as *mut RigidBody3D;

                unsafe {
                    let body_a = &mut *ptr_a;
                    let body_b = &mut *ptr_b;

                    if (body_a.is_static() || body_a.is_sleeping == 1)
                        && (body_b.is_static() || body_b.is_sleeping == 1)
                    {
                        continue;
                    }

                    if let Some(manifold) = Narrowphase::test_collision(body_a, body_b) {
                        ContactSolver::resolve(body_a, body_b, &manifold, sub_dt);
                    }
                }
            }

            // Solve Articulated Joints
            let joint_ids: Vec<u32> = self.joints.keys().copied().collect();
            for j_id in joint_ids {
                if let Some(joint) = self.joints.get(&j_id).copied() {
                    let id_a = joint.body_a;
                    let id_b = joint.body_b;

                    if id_a == id_b || !self.bodies.contains_key(&id_a) || !self.bodies.contains_key(&id_b) {
                        continue;
                    }

                    let ptr_a = self.bodies.get_mut(&id_a).unwrap() as *mut RigidBody3D;
                    let ptr_b = self.bodies.get_mut(&id_b).unwrap() as *mut RigidBody3D;

                    unsafe {
                        let body_a = &mut *ptr_a;
                        let body_b = &mut *ptr_b;
                        joint.solve(body_a, body_b, sub_dt);
                    }
                }
            }
        }
    }
}
