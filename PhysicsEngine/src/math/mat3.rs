use crate::math::quat::Quaternion;
use crate::math::vec3::Vector3D;

/// 3x3 Matrix for rigid body inertia tensors and coordinate frame transformations
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3x3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3x3 {
    pub const IDENTITY: Self = Self {
        m: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    };

    pub const ZERO: Self = Self {
        m: [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
    };

    #[inline(always)]
    pub fn new(m: [[f32; 3]; 3]) -> Self {
        Self { m }
    }

    #[inline]
    pub fn from_quaternion(q: &Quaternion) -> Self {
        Self {
            m: q.to_rotation_matrix(),
        }
    }

    #[inline]
    pub fn from_diagonal(diag: Vector3D) -> Self {
        Self {
            m: [
                [diag.x, 0.0, 0.0],
                [0.0, diag.y, 0.0],
                [0.0, 0.0, diag.z],
            ],
        }
    }

    #[inline]
    pub fn transpose(&self) -> Self {
        Self {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2]],
            ],
        }
    }

    #[inline]
    pub fn mul_mat(&self, rhs: &Self) -> Self {
        let mut res = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                res[i][j] = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j];
            }
        }
        Self { m: res }
    }

    #[inline]
    pub fn mul_vec(&self, v: Vector3D) -> Vector3D {
        Vector3D::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    /// Transforms local inverse inertia tensor to world space: I_world^-1 = R * I_local^-1 * R^T
    #[inline]
    pub fn rotate_inertia_tensor(r: &Self, local_inv_inertia: Vector3D) -> Self {
        let inv_diag = Self::from_diagonal(local_inv_inertia);
        let r_inv_diag = r.mul_mat(&inv_diag);
        let r_t = r.transpose();
        r_inv_diag.mul_mat(&r_t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matrix_vector_multiplication() {
        let m = Mat3x3::new([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ]);
        let v = Vector3D::new(1.0, 1.0, 1.0);
        let res = m.mul_vec(v);
        assert_eq!(res, Vector3D::new(6.0, 15.0, 24.0));
    }

    #[test]
    fn test_matrix_transpose() {
        let m = Mat3x3::new([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ]);
        let t = m.transpose();
        assert_eq!(t.m[0][1], 4.0);
        assert_eq!(t.m[1][0], 2.0);
    }
}
