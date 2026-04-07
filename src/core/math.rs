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
