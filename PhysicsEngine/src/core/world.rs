use std::collections::{HashMap, HashSet};
use crate::collision::broadphase::SpatialHash;
use crate::collision::narrowphase::Narrowphase;
use crate::core::body::{RigidBody3D, ShapeType};
use crate::dynamics::integrator::Integrator;
use crate::math::ray::{Ray, RaycastHit};
use crate::math::vec3::Vector3D;
use crate::solver::contact::ContactSolver;
use crate::solver::joint::DistanceJoint;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionEvent {
    pub body_a: u32,
    pub body_b: u32,
    pub contact_point: Vector3D,
    pub normal: Vector3D,
    pub impulse: f32,
    pub is_sensor: u32,
}

pub struct PhysicsWorld {
    pub bodies: HashMap<u32, RigidBody3D>,
    pub joints: HashMap<u32, DistanceJoint>,
    pub disabled_collision_pairs: HashSet<(u32, u32)>,
    pub collision_events: Vec<CollisionEvent>,
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
            collision_events: Vec::new(),
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

    pub fn add_capsule(
        &mut self,
        pos: Vector3D,
        radius: f32,
        height: f32,
        mass: f32,
        restitution: f32,
        friction: f32,
    ) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let body = RigidBody3D::new_capsule(id, pos, radius, height, mass, restitution, friction);
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
            } else if body.shape_type == ShapeType::Capsule as u32 {
                let (p1, p2) = body.capsule_segment();
                ray.intersect_capsule(p1, p2, body.radius, closest_dist)
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

        self.collision_events.clear();

        let num_substeps = self.substeps.max(1);
        let sub_dt = dt / (num_substeps as f32);

        let mut frame_events: HashMap<(u32, u32), CollisionEvent> = HashMap::new();

        for _ in 0..num_substeps {
            self.step_substep(sub_dt, &mut frame_events);
        }

        let mut events: Vec<CollisionEvent> = frame_events.into_values().collect();
        events.sort_by(|a, b| (a.body_a, a.body_b).cmp(&(b.body_a, b.body_b)));
        self.collision_events = events;
    }

    fn step_substep(&mut self, sub_dt: f32, frame_events: &mut HashMap<(u32, u32), CollisionEvent>) {
        // Phase 1: Numerical Integration with Air Damping
        for body in self.bodies.values_mut() {
            Integrator::integrate(body, self.gravity, sub_dt);
        }

        // Phase 1.5: Continuous Collision Detection (CCD) for fast-moving bodies
        let ccd_ids: Vec<u32> = self
            .bodies
            .iter()
            .filter(|(_, b)| b.enable_ccd == 1 && !b.is_static() && b.velocity.length_squared() > 1.0)
            .map(|(&id, _)| id)
            .collect();

        for id in ccd_ids {
            if let Some(body) = self.bodies.get(&id).copied() {
                let speed = body.velocity.length();
                let sweep_dist = speed * sub_dt;
                if sweep_dist > body.radius * 0.25 {
                    let ray_dir = body.velocity * (1.0 / speed);
                    let ray = Ray::new(body.position - ray_dir * sweep_dist, ray_dir);
                    if let Some(hit) = self.raycast(&ray, sweep_dist + body.radius) {
                        if hit.body_id != id {
                            if let Some(b_mut) = self.bodies.get_mut(&id) {
                                b_mut.position = hit.point + hit.normal * (b_mut.radius + 0.001);
                                let vn = b_mut.velocity.dot(hit.normal);
                                let impulse = if vn < 0.0 {
                                    let imp = -vn * (1.0 + b_mut.restitution) * b_mut.mass;
                                    b_mut.velocity -= hit.normal * (vn * (1.0 + b_mut.restitution));
                                    imp
                                } else {
                                    0.0
                                };

                                let key = (id.min(hit.body_id), id.max(hit.body_id));
                                frame_events
                                    .entry(key)
                                    .and_modify(|e| e.impulse += impulse)
                                    .or_insert(CollisionEvent {
                                        body_a: id,
                                        body_b: hit.body_id,
                                        contact_point: hit.point,
                                        normal: hit.normal,
                                        impulse,
                                        is_sensor: 0,
                                    });
                            }
                        }
                    }
                }
            }
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
                        let is_sensor = (body_a.is_sensor == 1 || body_b.is_sensor == 1) as u32;
                        let impulse = if is_sensor == 0 {
                            ContactSolver::resolve(body_a, body_b, &manifold, sub_dt)
                        } else {
                            0.0
                        };

                        let key = (*id_a.min(id_b), *id_a.max(id_b));
                        frame_events
                            .entry(key)
                            .and_modify(|e| e.impulse += impulse)
                            .or_insert(CollisionEvent {
                                body_a: *id_a,
                                body_b: *id_b,
                                contact_point: manifold.contact_point,
                                normal: manifold.normal,
                                impulse,
                                is_sensor,
                            });
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
