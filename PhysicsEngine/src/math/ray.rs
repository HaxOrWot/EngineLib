use crate::math::vec3::Vector3D;

#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Vector3D,
    pub direction: Vector3D, // Expected to be normalized
}

#[derive(Clone, Copy, Debug)]
pub struct RaycastHit {
    pub point: Vector3D,
    pub normal: Vector3D,
    pub distance: f32,
    pub body_id: u32,
}

impl Ray {
    pub fn new(origin: Vector3D, direction: Vector3D) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }

    /// Fast Slab Method for Ray-AABB intersection
    pub fn intersect_aabb(
        &self,
        box_center: Vector3D,
        box_size: Vector3D,
        max_dist: f32,
    ) -> Option<(f32, Vector3D)> {
        let half = box_size * 0.5;
        let min = box_center - half;
        let max = box_center + half;

        let mut tmin = 0.0f32;
        let mut tmax = max_dist;
        let mut hit_normal = Vector3D::ZERO;

        // Check X axis slab
        if self.direction.x.abs() > 1e-6 {
            let inv_d = 1.0 / self.direction.x;
            let mut t1 = (min.x - self.origin.x) * inv_d;
            let mut t2 = (max.x - self.origin.x) * inv_d;
            let mut n = Vector3D::new(-1.0, 0.0, 0.0);

            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
                n = Vector3D::new(1.0, 0.0, 0.0);
            }

            if t1 > tmin {
                tmin = t1;
                hit_normal = n;
            }
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        } else if self.origin.x < min.x || self.origin.x > max.x {
            return None;
        }

        // Check Y axis slab
        if self.direction.y.abs() > 1e-6 {
            let inv_d = 1.0 / self.direction.y;
            let mut t1 = (min.y - self.origin.y) * inv_d;
            let mut t2 = (max.y - self.origin.y) * inv_d;
            let mut n = Vector3D::new(0.0, -1.0, 0.0);

            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
                n = Vector3D::new(0.0, 1.0, 0.0);
            }

            if t1 > tmin {
                tmin = t1;
                hit_normal = n;
            }
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        } else if self.origin.y < min.y || self.origin.y > max.y {
            return None;
        }

        // Check Z axis slab
        if self.direction.z.abs() > 1e-6 {
            let inv_d = 1.0 / self.direction.z;
            let mut t1 = (min.z - self.origin.z) * inv_d;
            let mut t2 = (max.z - self.origin.z) * inv_d;
            let mut n = Vector3D::new(0.0, 0.0, -1.0);

            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
                n = Vector3D::new(0.0, 0.0, 1.0);
            }

            if t1 > tmin {
                tmin = t1;
                hit_normal = n;
            }
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        } else if self.origin.z < min.z || self.origin.z > max.z {
            return None;
        }

        if tmin <= max_dist && tmin >= 0.0 {
            Some((tmin, hit_normal))
        } else {
            None
        }
    }

    /// Quadratic Ray-Sphere intersection
    pub fn intersect_sphere(
        &self,
        center: Vector3D,
        radius: f32,
        max_dist: f32,
    ) -> Option<(f32, Vector3D)> {
        let oc = self.origin - center;
        let b = oc.dot(self.direction);
        let c = oc.dot(oc) - radius * radius;

        if c > 0.0 && b > 0.0 {
            return None; // Ray points away from sphere
        }

        let discriminant = b * b - c;
        if discriminant < 0.0 {
            return None; // No intersection
        }

        let mut t = -b - discriminant.sqrt();
        if t < 0.0 {
            t = -b + discriminant.sqrt(); // Origin was inside sphere
        }

        if t >= 0.0 && t <= max_dist {
            let hit_point = self.origin + self.direction * t;
            let normal = (hit_point - center).normalize();
            Some((t, normal))
        } else {
            None
        }
    }

    /// Ray-Capsule intersection (capped cylinder with two hemisphere caps)
    pub fn intersect_capsule(
        &self,
        p1: Vector3D,
        p2: Vector3D,
        radius: f32,
        max_dist: f32,
    ) -> Option<(f32, Vector3D)> {
        let mut best_t = max_dist;
        let mut best_normal = None;

        // 1. Check end spheres at p1 and p2
        if let Some((t1, n1)) = self.intersect_sphere(p1, radius, best_t) {
            best_t = t1;
            best_normal = Some(n1);
        }
        if let Some((t2, n2)) = self.intersect_sphere(p2, radius, best_t) {
            best_t = t2;
            best_normal = Some(n2);
        }

        // 2. Check cylindrical body
        let axis = p2 - p1;
        let axis_len_sq = axis.length_squared();
        if axis_len_sq > 1e-6 {
            let axis_dir = axis.normalize();
            let rc = self.origin - p1;
            let d_perp = self.direction - axis_dir * self.direction.dot(axis_dir);
            let rc_perp = rc - axis_dir * rc.dot(axis_dir);

            let a = d_perp.dot(d_perp);
            let b = d_perp.dot(rc_perp);
            let c = rc_perp.dot(rc_perp) - radius * radius;

            if a > 1e-6 {
                let disc = b * b - a * c;
                if disc >= 0.0 {
                    let mut t = (-b - disc.sqrt()) / a;
                    if t < 0.0 {
                        t = (-b + disc.sqrt()) / a;
                    }
                    if t >= 0.0 && t < best_t {
                        let hit_pos = self.origin + self.direction * t;
                        let proj = (hit_pos - p1).dot(axis_dir);
                        let axis_len = axis_len_sq.sqrt();
                        if proj >= 0.0 && proj <= axis_len {
                            let center_on_axis = p1 + axis_dir * proj;
                            let n = (hit_pos - center_on_axis).normalize();
                            best_t = t;
                            best_normal = Some(n);
                        }
                    }
                }
            }
        }

        best_normal.map(|n| (best_t, n))
    }
}
