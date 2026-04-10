use crate::core::error::{PinocchioError, Result};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dual {
    pub value: f64,
    pub grad: f64,
}

impl Dual {
    pub const fn new(value: f64, grad: f64) -> Self {
        Self { value, grad }
    }

    pub fn sin(self) -> Self {
        Self::new(self.value.sin(), self.grad * self.value.cos())
    }

    pub fn cos(self) -> Self {
        Self::new(self.value.cos(), -self.grad * self.value.sin())
    }
}

impl core::ops::Add for Dual {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.value + rhs.value, self.grad + rhs.grad)
    }
}

impl core::ops::Sub for Dual {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.value - rhs.value, self.grad - rhs.grad)
    }
}

impl core::ops::Mul for Dual {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(
            self.value * rhs.value,
            self.value * rhs.grad + self.grad * rhs.value,
        )
    }
}

impl core::ops::Div for Dual {
    type Output = Self;
    fn div(self, rhs: Self) -> Self::Output {
        let den = rhs.value * rhs.value;
        Self::new(
            self.value / rhs.value,
            (self.grad * rhs.value - self.value * rhs.grad) / den,
        )
    }
}

pub fn jacobian_fd(
    x: &[f64],
    out_dim: usize,
    f: impl Fn(&[f64]) -> Result<Vec<f64>>,
) -> Result<Vec<f64>> {
    let y0 = f(x)?;
    if y0.len() != out_dim {
        return Err(PinocchioError::DimensionMismatch {
            expected: out_dim,
            got: y0.len(),
        });
    }
    let n = x.len();
    let eps = 1e-6;
    let mut j = vec![0.0; out_dim * n];
    for i in 0..n {
        let mut xp = x.to_vec();
        let mut xm = x.to_vec();
        xp[i] += eps;
        xm[i] -= eps;
        let yp = f(&xp)?;
        let ym = f(&xm)?;
        if yp.len() != out_dim || ym.len() != out_dim {
            return Err(PinocchioError::DimensionMismatch {
                expected: out_dim,
                got: yp.len().min(ym.len()),
            });
        }
        for r in 0..out_dim {
            j[r * n + i] = (yp[r] - ym[r]) / (2.0 * eps);
        }
    }
    Ok(j)
}
