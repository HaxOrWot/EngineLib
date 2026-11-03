use crate::math::vec3::Vector3D;
use std::ops::Mul;

/// Unit Quaternion representing 3D rotation without gimbal lock
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    #[inline(always)]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    #[inline]
    pub fn from_axis_angle(axis: Vector3D, angle_rad: f32) -> Self {
        let half_angle = angle_rad * 0.5;
        let s = half_angle.sin();
        let norm_axis = axis.normalize();
        Self {
            x: norm_axis.x * s,
            y: norm_axis.y * s,
            z: norm_axis.z * s,
            w: half_angle.cos(),
        }
    }

    #[inline]
    pub fn from_euler(pitch_x: f32, yaw_y: f32, roll_z: f32) -> Self {
        let cx = (pitch_x * 0.5).cos();
        let sx = (pitch_x * 0.5).sin();
        let cy = (yaw_y * 0.5).cos();
        let sy = (yaw_y * 0.5).sin();
        let cz = (roll_z * 0.5).cos();
        let sz = (roll_z * 0.5).sin();

        Self {
            x: sx * cy * cz - cx * sy * sz,
            y: cx * sy * cz + sx * cy * sz,
            z: cx * cy * sz - sx * sy * cz,
            w: cx * cy * cz + sx * sy * sz,
        }
    }

    #[inline]
    pub fn to_euler(&self) -> Vector3D {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        Vector3D::new(roll, pitch, yaw)
    }

    #[inline(always)]
    pub fn conjugate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    #[inline(always)]
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    #[inline(always)]
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    #[inline]
    pub fn normalized(&self) -> Self {
        let len = self.length();
        if len > 1e-7 {
            let inv = 1.0 / len;
            Self {
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
                w: self.w * inv,
            }
        } else {
            Self::IDENTITY
        }
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        self.normalized()
    }

    /// Rotates a 3D vector by this quaternion using Rodrigues' formula
    #[inline]
    pub fn rotate_vector(&self, v: Vector3D) -> Vector3D {
        let qv = Vector3D::new(self.x, self.y, self.z);
        let uv = qv.cross(v);
        let uuv = qv.cross(uv);
        v + (uv * self.w + uuv) * 2.0
    }

    /// Advances quaternion orientation by angular velocity vector over timestep dt using exact exponential map
    #[inline]
    pub fn integrate(&self, omega: Vector3D, dt: f32) -> Self {
        let ang_speed = omega.length();
        let theta = ang_speed * dt;
        if theta > 1e-6 {
            let half_theta = theta * 0.5;
            let s = half_theta.sin() / ang_speed;
            let dq = Quaternion {
                x: omega.x * s,
                y: omega.y * s,
                z: omega.z * s,
                w: half_theta.cos(),
            };
            (dq * (*self)).normalized()
        } else {
            *self
        }
    }

    /// Converts quaternion to 3x3 orthonormal rotation matrix
    #[inline]
    pub fn to_rotation_matrix(&self) -> [[f32; 3]; 3] {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;

        let xx = self.x * x2;
        let xy = self.x * y2;
        let xz = self.x * z2;
        let yy = self.y * y2;
        let yz = self.y * z2;
        let zz = self.z * z2;
        let wx = self.w * x2;
        let wy = self.w * y2;
        let wz = self.w * z2;

        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    }
}

impl Mul for Quaternion {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::IDENTITY;
        let v = Vector3D::new(1.0, 2.0, 3.0);
        let rotated = q.rotate_vector(v);
        assert!((rotated.x - v.x).abs() < 1e-6);
        assert!((rotated.y - v.y).abs() < 1e-6);
        assert!((rotated.z - v.z).abs() < 1e-6);
    }

    #[test]
    fn test_quaternion_axis_angle_rotation() {
        // Rotate (1, 0, 0) by 90 degrees around Y axis -> should point to (0, 0, -1)
        let q = Quaternion::from_axis_angle(Vector3D::UP, PI * 0.5);
        let v = Vector3D::new(1.0, 0.0, 0.0);
        let rotated = q.rotate_vector(v);
        assert!(rotated.x.abs() < 1e-6, "expected x=0, got {}", rotated.x);
        assert!(rotated.y.abs() < 1e-6, "expected y=0, got {}", rotated.y);
        assert!((rotated.z - (-1.0)).abs() < 1e-6, "expected z=-1, got {}", rotated.z);
    }

    #[test]
    fn test_quaternion_integration() {
        let q = Quaternion::IDENTITY;
        let omega = Vector3D::new(0.0, PI, 0.0); // 180 deg/s around Y
        let dt = 0.5; // Rotate for 0.5s -> 90 deg rotation
        let q_new = q.integrate(omega, dt);

        let v = Vector3D::new(1.0, 0.0, 0.0);
        let rotated = q_new.rotate_vector(v);
        assert!(rotated.x.abs() < 1e-4, "expected x=0, got {}", rotated.x);
        assert!((rotated.z - (-1.0)).abs() < 1e-4, "expected z=-1, got {}", rotated.z);
    }
}
