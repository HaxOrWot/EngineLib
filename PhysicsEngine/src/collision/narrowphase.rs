use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

#[derive(Clone, Copy, Debug)]
pub struct ContactManifold {
    pub normal: Vector3D,
    pub penetration: f32,
    pub contact_point: Vector3D,
    pub contact_point_2: Option<Vector3D>,
}

impl ContactManifold {
    #[inline(always)]
    pub fn reversed(self) -> Self {
        Self {
            normal: -self.normal,
            penetration: self.penetration,
            contact_point: self.contact_point,
            contact_point_2: self.contact_point_2,
        }
    }
}

pub struct Narrowphase;

impl Narrowphase {
    const EPSILON: f32 = 1e-6;

    /// Dispatches collision check according to shape types (0 = AABB, 1 = Sphere, 2 = Capsule)
    pub fn test_collision(body_a: &RigidBody3D, body_b: &RigidBody3D) -> Option<ContactManifold> {
        match (body_a.shape_type, body_b.shape_type) {
            (0, 0) => Self::test_aabb(body_a, body_b),
            (1, 1) => Self::test_sphere_sphere(body_a, body_b),
            (1, 0) => Self::test_sphere_aabb(body_a, body_b),
            (0, 1) => Self::test_sphere_aabb(body_b, body_a).map(|m| m.reversed()),
            (1, 2) => Self::test_sphere_capsule(body_a, body_b),
            (2, 1) => Self::test_sphere_capsule(body_b, body_a).map(|m| m.reversed()),
            (2, 2) => Self::test_capsule_capsule(body_a, body_b),
            (2, 0) => Self::test_capsule_aabb(body_a, body_b),
            (0, 2) => Self::test_capsule_aabb(body_b, body_a).map(|m| m.reversed()),
            _ => None,
        }
    }

    /// Evaluates 3D AABB overlap between two rigid bodies
    pub fn test_aabb(body_a: &RigidBody3D, body_b: &RigidBody3D) -> Option<ContactManifold> {
        let delta = body_a.position - body_b.position;

        let overlap_x = (body_a.size.x + body_b.size.x) * 0.5 - delta.x.abs();
        if overlap_x <= 0.0 {
            return None;
        }

        let overlap_y = (body_a.size.y + body_b.size.y) * 0.5 - delta.y.abs();
        if overlap_y <= 0.0 {
            return None;
        }

        let overlap_z = (body_a.size.z + body_b.size.z) * 0.5 - delta.z.abs();
        if overlap_z <= 0.0 {
            return None;
        }

        let (normal, penetration) = if overlap_x < overlap_y && overlap_x < overlap_z {
            let sign = if delta.x.abs() > Self::EPSILON {
                delta.x.signum()
            } else {
                1.0
            };
            (Vector3D::new(sign, 0.0, 0.0), overlap_x)
        } else if overlap_y < overlap_x && overlap_y < overlap_z {
            let sign = if delta.y.abs() > Self::EPSILON {
                delta.y.signum()
            } else {
                1.0
            };
            (Vector3D::new(0.0, sign, 0.0), overlap_y)
        } else {
            let sign = if delta.z.abs() > Self::EPSILON {
                delta.z.signum()
            } else {
                1.0
            };
            (Vector3D::new(0.0, 0.0, sign), overlap_z)
        };

        let (min_a, max_a) = body_a.aabb_bounds();
        let (min_b, max_b) = body_b.aabb_bounds();
        let x_min = min_a.x.max(min_b.x);
        let x_max = max_a.x.min(max_b.x);
        let y_min = min_a.y.max(min_b.y);
        let y_max = max_a.y.min(max_b.y);
        let z_min = min_a.z.max(min_b.z);
        let z_max = max_a.z.min(max_b.z);

        let centroid = Vector3D::new(
            (x_min + x_max) * 0.5,
            (y_min + y_max) * 0.5,
            (z_min + z_max) * 0.5,
        );

        let mut point_1 = centroid;
        let mut contact_point_2 = None;

        // Generate dual contact points along the contact face for rotational stability
        if normal.y.abs() > 0.7 {
            let span_x = x_max - x_min;
            if span_x > 0.04 {
                let offset_x = span_x * 0.35;
                point_1.x = centroid.x - offset_x;
                let mut p2 = centroid;
                p2.x = centroid.x + offset_x;
                contact_point_2 = Some(p2);
            }
        } else if normal.x.abs() > 0.7 {
            let span_y = y_max - y_min;
            if span_y > 0.04 {
                let offset_y = span_y * 0.35;
                point_1.y = centroid.y - offset_y;
                let mut p2 = centroid;
                p2.y = centroid.y + offset_y;
                contact_point_2 = Some(p2);
            }
        }

        Some(ContactManifold {
            normal,
            penetration,
            contact_point: point_1,
            contact_point_2,
        })
    }

    /// Evaluates Sphere vs Sphere collision
    pub fn test_sphere_sphere(
        sphere_a: &RigidBody3D,
        sphere_b: &RigidBody3D,
    ) -> Option<ContactManifold> {
        let delta = sphere_a.position - sphere_b.position;
        let dist_sq = delta.length_squared();
        let radius_sum = sphere_a.radius + sphere_b.radius;

        if dist_sq >= radius_sum * radius_sum {
            return None;
        }

        let dist = dist_sq.sqrt();
        let penetration = radius_sum - dist;

        let normal = if dist > Self::EPSILON {
            delta * (1.0 / dist)
        } else {
            Vector3D::UP
        };

        let contact_point = sphere_a.position - normal * (sphere_a.radius - penetration * 0.5);

        Some(ContactManifold {
            normal,
            penetration,
            contact_point,
            contact_point_2: None,
        })
    }

    /// Evaluates Sphere vs AABB collision (normal points from AABB towards Sphere)
    pub fn test_sphere_aabb(
        sphere: &RigidBody3D,
        aabb: &RigidBody3D,
    ) -> Option<ContactManifold> {
        let half = aabb.size * 0.5;
        let box_min = aabb.position - half;
        let box_max = aabb.position + half;

        let closest = sphere.position.clamp(box_min, box_max);
        let delta = sphere.position - closest;
        let dist_sq = delta.length_squared();

        if dist_sq > sphere.radius * sphere.radius {
            return None;
        }

        let dist = dist_sq.sqrt();

        let (normal, penetration) = if dist > Self::EPSILON {
            (delta * (1.0 / dist), sphere.radius - dist)
        } else {
            let d = sphere.position - aabb.position;
            let ox = half.x - d.x.abs();
            let oy = half.y - d.y.abs();
            let oz = half.z - d.z.abs();

            if ox < oy && ox < oz {
                (Vector3D::new(if d.x >= 0.0 { 1.0 } else { -1.0 }, 0.0, 0.0), ox + sphere.radius)
            } else if oy < ox && oy < oz {
                (Vector3D::new(0.0, if d.y >= 0.0 { 1.0 } else { -1.0 }, 0.0), oy + sphere.radius)
            } else {
                (Vector3D::new(0.0, 0.0, if d.z >= 0.0 { 1.0 } else { -1.0 }), oz + sphere.radius)
            }
        };

        Some(ContactManifold {
            normal,
            penetration,
            contact_point: closest,
            contact_point_2: None,
        })
    }

    /// Finds closest point on a 3D line segment to a target point
    #[inline]
    pub fn closest_point_on_segment(point: Vector3D, a: Vector3D, b: Vector3D) -> Vector3D {
        let ab = b - a;
        let len_sq = ab.length_squared();
        if len_sq < 1e-7 {
            return a;
        }
        let t = ((point - a).dot(ab) / len_sq).clamp(0.0, 1.0);
        a + ab * t
    }

    /// Computes closest points between two arbitrary 3D line segments
    pub fn closest_points_between_segments(
        p1: Vector3D, q1: Vector3D,
        p2: Vector3D, q2: Vector3D,
    ) -> (Vector3D, Vector3D) {
        let d1 = q1 - p1;
        let d2 = q2 - p2;
        let r = p1 - p2;
        let a = d1.length_squared();
        let e = d2.length_squared();
        let f = d2.dot(r);

        let (s, t);

        if a <= 1e-7 && e <= 1e-7 {
            return (p1, p2);
        }
        if a <= 1e-7 {
            s = 0.0;
            t = (f / e).clamp(0.0, 1.0);
        } else {
            let c = d1.dot(r);
            if e <= 1e-7 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else {
                let b = d1.dot(d2);
                let denom = a * e - b * b;

                let s_val = if denom.abs() > 1e-7 {
                    ((b * f - c * e) / denom).clamp(0.0, 1.0)
                } else {
                    0.0
                };

                let t_val = (b * s_val + f) / e;

                if t_val < 0.0 {
                    t = 0.0;
                    s = (-c / a).clamp(0.0, 1.0);
                } else if t_val > 1.0 {
                    t = 1.0;
                    s = ((b - c) / a).clamp(0.0, 1.0);
                } else {
                    s = s_val;
                    t = t_val;
                }
            }
        }

        (p1 + d1 * s, p2 + d2 * t)
    }

    /// Evaluates Sphere vs Capsule collision (normal points from Capsule towards Sphere)
    pub fn test_sphere_capsule(
        sphere: &RigidBody3D,
        capsule: &RigidBody3D,
    ) -> Option<ContactManifold> {
        let (seg_a, seg_b) = capsule.capsule_segment();
        let closest_on_seg = Self::closest_point_on_segment(sphere.position, seg_a, seg_b);
        let delta = sphere.position - closest_on_seg;
        let dist_sq = delta.length_squared();
        let radius_sum = sphere.radius + capsule.radius;

        if dist_sq >= radius_sum * radius_sum {
            return None;
        }

        let dist = dist_sq.sqrt();
        let (normal, penetration) = if dist > Self::EPSILON {
            (delta * (1.0 / dist), radius_sum - dist)
        } else {
            (Vector3D::UP, radius_sum)
        };

        let contact_point = closest_on_seg + normal * capsule.radius;

        Some(ContactManifold {
            normal,
            penetration,
            contact_point,
            contact_point_2: None,
        })
    }

    /// Evaluates Capsule vs Capsule collision (normal points from cap_b towards cap_a)
    pub fn test_capsule_capsule(
        cap_a: &RigidBody3D,
        cap_b: &RigidBody3D,
    ) -> Option<ContactManifold> {
        let (a1, a2) = cap_a.capsule_segment();
        let (b1, b2) = cap_b.capsule_segment();
        let (cp_a, cp_b) = Self::closest_points_between_segments(a1, a2, b1, b2);

        let delta = cp_a - cp_b;
        let dist_sq = delta.length_squared();
        let radius_sum = cap_a.radius + cap_b.radius;

        if dist_sq >= radius_sum * radius_sum {
            return None;
        }

        let dist = dist_sq.sqrt();
        let (normal, penetration) = if dist > Self::EPSILON {
            (delta * (1.0 / dist), radius_sum - dist)
        } else {
            (Vector3D::UP, radius_sum)
        };

        let contact_point = (cp_a + cp_b) * 0.5;

        Some(ContactManifold {
            normal,
            penetration,
            contact_point,
            contact_point_2: None,
        })
    }

    /// Evaluates Capsule vs AABB collision (normal points from AABB towards Capsule)
    pub fn test_capsule_aabb(
        capsule: &RigidBody3D,
        aabb: &RigidBody3D,
    ) -> Option<ContactManifold> {
        let (seg_a, seg_b) = capsule.capsule_segment();
        let half = aabb.size * 0.5;
        let box_min = aabb.position - half;
        let box_max = aabb.position + half;

        // Iterative projection to find closest point pair between 3D segment and AABB
        let mut p_seg = (seg_a + seg_b) * 0.5;
        let mut p_box = p_seg.clamp(box_min, box_max);
        for _ in 0..3 {
            p_seg = Self::closest_point_on_segment(p_box, seg_a, seg_b);
            p_box = p_seg.clamp(box_min, box_max);
        }

        let delta = p_seg - p_box;
        let dist_sq = delta.length_squared();

        if dist_sq > capsule.radius * capsule.radius {
            return None;
        }

        let dist = dist_sq.sqrt();
        let (normal, penetration) = if dist > Self::EPSILON {
            (delta * (1.0 / dist), capsule.radius - dist)
        } else {
            let d = p_seg - aabb.position;
            let ox = half.x - d.x.abs();
            let oy = half.y - d.y.abs();
            let oz = half.z - d.z.abs();

            if ox < oy && ox < oz {
                (Vector3D::new(if d.x >= 0.0 { 1.0 } else { -1.0 }, 0.0, 0.0), ox + capsule.radius)
            } else if oy < ox && oy < oz {
                (Vector3D::new(0.0, if d.y >= 0.0 { 1.0 } else { -1.0 }, 0.0), oy + capsule.radius)
            } else {
                (Vector3D::new(0.0, 0.0, if d.z >= 0.0 { 1.0 } else { -1.0 }), oz + capsule.radius)
            }
        };

        Some(ContactManifold {
            normal,
            penetration,
            contact_point: p_box,
            contact_point_2: None,
        })
    }
}
