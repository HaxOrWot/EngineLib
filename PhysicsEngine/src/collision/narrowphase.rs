use crate::core::body::RigidBody3D;
use crate::math::vec3::Vector3D;

#[derive(Clone, Copy, Debug)]
pub struct ContactManifold {
    pub normal: Vector3D,
    pub penetration: f32,
}

pub struct Narrowphase;

impl Narrowphase {
    const EPSILON: f32 = 1e-6;

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

        // Find normal along shallowest penetration axis (minimum overlap)
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
}
