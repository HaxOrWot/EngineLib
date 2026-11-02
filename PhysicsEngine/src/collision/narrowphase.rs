use crate::core::body::{RigidBody3D, ShapeType};
use crate::math::vec3::Vector3D;

#[derive(Clone, Copy, Debug)]
pub struct ContactManifold {
    pub normal: Vector3D,
    pub penetration: f32,
}

pub struct Narrowphase;

impl Narrowphase {
    const EPSILON: f32 = 1e-6;

    /// Dispatches collision check according to shape types
    pub fn test_collision(body_a: &RigidBody3D, body_b: &RigidBody3D) -> Option<ContactManifold> {
        let is_sphere_a = body_a.shape_type == ShapeType::Sphere as u32;
        let is_sphere_b = body_b.shape_type == ShapeType::Sphere as u32;

        match (is_sphere_a, is_sphere_b) {
            (false, false) => Self::test_aabb(body_a, body_b),
            (true, true) => Self::test_sphere_sphere(body_a, body_b),
            (true, false) => Self::test_sphere_aabb(body_a, body_b),
            (false, true) => {
                // Sphere is body_b, AABB is body_a -> reverse normal so it points from b to a
                Self::test_sphere_aabb(body_b, body_a).map(|m| ContactManifold {
                    normal: -m.normal,
                    penetration: m.penetration,
                })
            }
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

        Some(ContactManifold {
            normal,
            penetration,
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
            Vector3D::UP // Default separation upward
        };

        Some(ContactManifold {
            normal,
            penetration,
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

        // Find closest point on AABB to sphere center
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
            // Sphere center is inside the AABB: push out along shallowest face
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
        })
    }
}
