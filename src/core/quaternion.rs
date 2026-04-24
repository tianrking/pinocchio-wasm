use crate::core::math::{Mat3, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quat {
    pub const fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn from_array(a: [f64; 4]) -> Self {
        Self {
            w: a[0],
            x: a[1],
            y: a[2],
            z: a[3],
        }
    }

    pub fn to_array(self) -> [f64; 4] {
        [self.w, self.x, self.y, self.z]
    }

    pub fn norm2(self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }

    pub fn normalize(self) -> Self {
        let n2 = self.norm2();
        if n2 <= 1e-16 {
            return Self::identity();
        }
        let inv = 1.0 / n2.sqrt();
        Self {
            w: self.w * inv,
            x: self.x * inv,
            y: self.y * inv,
            z: self.z * inv,
        }
    }

    pub fn is_normalized(self, tol: f64) -> bool {
        (self.norm2() - 1.0).abs() < tol
    }

    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }

    pub fn to_rotation_matrix(self) -> Mat3 {
        Mat3::from_quaternion(self.w, self.x, self.y, self.z)
    }

    pub fn from_rotation_matrix(m: &Mat3) -> Self {
        let r = m.m;
        let trace = r[0][0] + r[1][1] + r[2][2];
        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            Self {
                w: 0.25 / s,
                x: (r[2][1] - r[1][2]) * s,
                y: (r[0][2] - r[2][0]) * s,
                z: (r[1][0] - r[0][1]) * s,
            }
        } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
            let s = 2.0 * (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt();
            Self {
                w: (r[2][1] - r[1][2]) / s,
                x: 0.25 * s,
                y: (r[0][1] + r[1][0]) / s,
                z: (r[0][2] + r[2][0]) / s,
            }
        } else if r[1][1] > r[2][2] {
            let s = 2.0 * (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt();
            Self {
                w: (r[0][2] - r[2][0]) / s,
                x: (r[0][1] + r[1][0]) / s,
                y: 0.25 * s,
                z: (r[1][2] + r[2][1]) / s,
            }
        } else {
            let s = 2.0 * (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt();
            Self {
                w: (r[1][0] - r[0][1]) / s,
                x: (r[0][2] + r[2][0]) / s,
                y: (r[1][2] + r[2][1]) / s,
                z: 0.25 * s,
            }
        }
        .normalize()
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let n2 = axis.norm2();
        if n2 <= 1e-16 {
            return Self::identity();
        }
        let a = axis * (1.0 / n2.sqrt());
        let half = angle * 0.5;
        let s = half.sin();
        Self {
            w: half.cos(),
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
        }
    }

    pub fn delta(omega: Vec3, dt: f64) -> Self {
        let angle = omega.norm() * dt;
        if angle <= 1e-12 {
            return Self {
                w: 1.0,
                x: 0.5 * omega.x * dt,
                y: 0.5 * omega.y * dt,
                z: 0.5 * omega.z * dt,
            }
            .normalize();
        }
        let axis = omega * (1.0 / omega.norm());
        Self::from_axis_angle(axis, angle)
    }

    pub fn slerp(self, other: Self, t: f64) -> Self {
        let mut dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z;
        let mut b = other;
        if dot < 0.0 {
            b = Self {
                w: -other.w,
                x: -other.x,
                y: -other.y,
                z: -other.z,
            };
            dot = -dot;
        }
        if dot > 0.9995 {
            let s = Self {
                w: self.w + t * (b.w - self.w),
                x: self.x + t * (b.x - self.x),
                y: self.y + t * (b.y - self.y),
                z: self.z + t * (b.z - self.z),
            };
            return s.normalize();
        }
        let theta0 = dot.acos();
        let theta = theta0 * t;
        let sin_theta = theta.sin();
        let sin_theta0 = theta0.sin();
        let s0 = (theta0 - theta).sin() / sin_theta0;
        let s1 = sin_theta / sin_theta0;
        Self {
            w: s0 * self.w + s1 * b.w,
            x: s0 * self.x + s1 * b.x,
            y: s0 * self.y + s1 * b.y,
            z: s0 * self.z + s1 * b.z,
        }
        .normalize()
    }

    pub fn log(self) -> Vec3 {
        let q = self.normalize();
        let sin_half = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        let cos_half = q.w;
        if sin_half < 1e-10 {
            if cos_half > 0.0 {
                return Vec3::new(
                    2.0 * q.x,
                    2.0 * q.y,
                    2.0 * q.z,
                );
            }
            let v = Vec3::new(q.x, q.y, q.z);
            let n = v.norm();
            if n < 1e-10 {
                return Vec3::zero();
            }
            let angle = std::f64::consts::PI;
            return v * (angle / n);
        }
        let half_angle = sin_half.atan2(cos_half);
        let angle = 2.0 * half_angle;
        Vec3::new(q.x, q.y, q.z) * (angle / sin_half)
    }

    pub fn uniform_random(rng_val: f64) -> Self {
        let s0 = rng_val;
        let s1 = (2.0 * std::f64::consts::PI * s0).sin();
        let s2 = (2.0 * std::f64::consts::PI * s0).cos();
        let s3 = (1.0 - s0).sqrt();
        let s4 = s0.sqrt();
        Self {
            w: s1 * s3,
            x: s2 * s3,
            y: s1 * s4,
            z: s2 * s4,
        }
    }
}
