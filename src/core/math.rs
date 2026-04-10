use core::ops::{Add, AddAssign, Mul, Sub, SubAssign};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    pub fn norm2(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }

    pub fn scale(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }

    pub fn as_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        self.scale(rhs)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    pub m: [[f64; 3]; 3],
}

impl Default for Mat3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Mat3 {
    pub const fn new(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    pub const fn identity() -> Self {
        Self::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    }

    pub const fn zero() -> Self {
        Self::new([[0.0; 3]; 3])
    }

    pub fn transpose(self) -> Self {
        let m = self.m;
        Self::new([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }

    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    pub fn mul_mat(self, rhs: Self) -> Self {
        let mut out = [[0.0; 3]; 3];
        for (r, out_row) in out.iter_mut().enumerate() {
            for (c, out_cell) in out_row.iter_mut().enumerate() {
                *out_cell = self.m[r][0] * rhs.m[0][c]
                    + self.m[r][1] * rhs.m[1][c]
                    + self.m[r][2] * rhs.m[2][c];
            }
        }
        Self::new(out)
    }

    pub fn add_mat(self, rhs: Self) -> Self {
        let mut out = Self::zero();
        for (r, row) in out.m.iter_mut().enumerate() {
            for (c, cell) in row.iter_mut().enumerate() {
                *cell = self.m[r][c] + rhs.m[r][c];
            }
        }
        out
    }

    pub fn skew(v: Vec3) -> Self {
        Self::new([[0.0, -v.z, v.y], [v.z, 0.0, -v.x], [-v.y, v.x, 0.0]])
    }

    pub fn outer(a: Vec3, b: Vec3) -> Self {
        Self::new([
            [a.x * b.x, a.x * b.y, a.x * b.z],
            [a.y * b.x, a.y * b.y, a.y * b.z],
            [a.z * b.x, a.z * b.y, a.z * b.z],
        ])
    }

    pub fn trace(self) -> f64 {
        self.m[0][0] + self.m[1][1] + self.m[2][2]
    }

    /// SO(3) logarithm map: extract axis-angle from rotation matrix.
    /// Returns Vec3 where direction = axis, magnitude = angle (radians).
    pub fn log3(self) -> Vec3 {
        let cos_theta = (self.trace() - 1.0) / 2.0;
        let cos_theta = cos_theta.clamp(-1.0, 1.0);
        let theta = cos_theta.acos();

        if theta < 1e-10 {
            // Near identity: use first-order approximation
            // log(R) ≈ (R - Rᵀ)/2, extract [m21-m12, m02-m20, m10-m01] / 2
            let v = Vec3::new(
                self.m[2][1] - self.m[1][2],
                self.m[0][2] - self.m[2][0],
                self.m[1][0] - self.m[0][1],
            );
            let n = v.norm();
            if n < 1e-16 {
                return Vec3::zero();
            }
            // For small theta, v ≈ 2*sin(theta)*axis ≈ 2*theta*axis
            // so axis = v/(2*sin(theta)) and angle = theta
            return v * (theta / (n * 0.5));
        }

        if (theta - core::f64::consts::PI).abs() < 1e-6 {
            // Near π: R = I + 2*sin(θ)*[a]_x + 2*sin²(θ/2)*[a]_x²
            // Find the column of (R + I)/2 with the largest diagonal element
            let diag = Vec3::new(
                self.m[0][0] + 1.0,
                self.m[1][1] + 1.0,
                self.m[2][2] + 1.0,
            );
            let (k, _) = [(0usize, diag.x), (1, diag.y), (2, diag.z)]
                .into_iter()
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();

            let mut axis = Vec3::zero();
            if k == 0 {
                axis.x = (self.m[0][0] + 1.0).sqrt() * 0.5;
                axis.y = (self.m[1][0] + self.m[0][1]) / (4.0 * axis.x);
                axis.z = (self.m[2][0] + self.m[0][2]) / (4.0 * axis.x);
            } else if k == 1 {
                axis.y = (self.m[1][1] + 1.0).sqrt() * 0.5;
                axis.x = (self.m[1][0] + self.m[0][1]) / (4.0 * axis.y);
                axis.z = (self.m[2][1] + self.m[1][2]) / (4.0 * axis.y);
            } else {
                axis.z = (self.m[2][2] + 1.0).sqrt() * 0.5;
                axis.x = (self.m[2][0] + self.m[0][2]) / (4.0 * axis.z);
                axis.y = (self.m[2][1] + self.m[1][2]) / (4.0 * axis.z);
            }
            let n = axis.norm();
            if n > 1e-10 {
                axis = axis * (1.0 / n);
            }
            return axis * theta;
        }

        // General case: (R - Rᵀ) / (2*sin(θ)) gives the skew-symmetric part
        // axis = [R32-R23, R13-R31, R21-R12] / (2*sin(θ))
        let sin_theta = theta.sin();
        let axis = Vec3::new(
            self.m[2][1] - self.m[1][2],
            self.m[0][2] - self.m[2][0],
            self.m[1][0] - self.m[0][1],
        ) * (1.0 / (2.0 * sin_theta));

        axis * theta
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let n2 = axis.norm2();
        if n2 <= 1e-16 {
            return Self::identity();
        }
        let inv_n = 1.0 / n2.sqrt();
        let a = axis * inv_n;
        let s = angle.sin();
        let c = angle.cos();
        let i = Self::identity();
        let aa_t = Self::outer(a, a);
        let k = Self::skew(a);

        let mut out = Self::zero();
        for r in 0..3 {
            for col in 0..3 {
                out.m[r][col] = c * i.m[r][col] + (1.0 - c) * aa_t.m[r][col] + s * k.m[r][col];
            }
        }
        out
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub rotation: Mat3,
    pub translation: Vec3,
}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform {
    pub const fn new(rotation: Mat3, translation: Vec3) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub const fn identity() -> Self {
        Self::new(Mat3::identity(), Vec3::zero())
    }

    pub fn inverse(self) -> Self {
        let rt = self.rotation.transpose();
        let t = rt.mul_vec(self.translation) * -1.0;
        Self::new(rt, t)
    }

    pub fn multiply(self, rhs: Self) -> Self {
        let r = self.rotation.mul_mat(rhs.rotation);
        let t = self.translation + self.rotation.mul_vec(rhs.translation);
        Self::new(r, t)
    }

    pub fn transform_point(self, p: Vec3) -> Vec3 {
        self.translation + self.rotation.mul_vec(p)
    }
}

/// SE(3) logarithm map: compute 6D twist from a rigid body transform.
/// Returns `[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]`.
///
/// Given T = (R, t), computes the twist ξ such that exp(ξ) = T.
pub fn log6(tf: Transform) -> [f64; 6] {
    let omega = tf.rotation.log3();
    let theta = omega.norm();

    let v = if theta < 1e-10 {
        // Near identity: V⁻¹ ≈ I - ½[ω]_x
        tf.translation - omega.cross(tf.translation) * 0.5
    } else {
        // V⁻¹ = I/θ - ½[ω]_x + (1/θ² - (1+cos θ)/(2θ sin θ)) [ω]_x²
        let skew_w = Mat3::skew(omega);
        let half_theta = theta * 0.5;
        let cot_half = half_theta.cos() / half_theta.sin();
        let coeff = 1.0 / (theta * theta) - cot_half / (2.0 * theta);

        let neg_half_skew = Mat3::new([
            [-skew_w.m[0][0] * 0.5, -skew_w.m[0][1] * 0.5, -skew_w.m[0][2] * 0.5],
            [-skew_w.m[1][0] * 0.5, -skew_w.m[1][1] * 0.5, -skew_w.m[1][2] * 0.5],
            [-skew_w.m[2][0] * 0.5, -skew_w.m[2][1] * 0.5, -skew_w.m[2][2] * 0.5],
        ]);
        let skew_sq = skew_w.mul_mat(skew_w);
        let skew_sq_scaled = Mat3::new([
            [skew_sq.m[0][0] * coeff, skew_sq.m[0][1] * coeff, skew_sq.m[0][2] * coeff],
            [skew_sq.m[1][0] * coeff, skew_sq.m[1][1] * coeff, skew_sq.m[1][2] * coeff],
            [skew_sq.m[2][0] * coeff, skew_sq.m[2][1] * coeff, skew_sq.m[2][2] * coeff],
        ]);
        let v_inv = Mat3::identity().add_mat(neg_half_skew).add_mat(skew_sq_scaled);
        v_inv.mul_vec(tf.translation)
    };

    [omega.x, omega.y, omega.z, v.x, v.y, v.z]
}
