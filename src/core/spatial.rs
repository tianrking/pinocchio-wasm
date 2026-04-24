use super::math::{Mat3, Vec3};

pub(crate) type SpatialMatrix = [[f64; 6]; 6];
pub(crate) type SpatialVector = [f64; 6];

pub(crate) fn spatial_zero() -> SpatialMatrix {
    [[0.0; 6]; 6]
}

pub(crate) fn spatial_vec_zero() -> SpatialVector {
    [0.0; 6]
}

pub(crate) fn spatial_vec(angular: Vec3, linear: Vec3) -> SpatialVector {
    [
        angular.x, angular.y, angular.z, linear.x, linear.y, linear.z,
    ]
}

pub(crate) fn spatial_dot(a: SpatialVector, b: SpatialVector) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

pub(crate) fn spatial_mat_vec(m: &SpatialMatrix, v: SpatialVector) -> SpatialVector {
    let mut out = [0.0; 6];
    for r in 0..6 {
        for (c, vc) in v.iter().copied().enumerate() {
            out[r] += m[r][c] * vc;
        }
    }
    out
}

pub(crate) fn spatial_add_assign(dst: &mut SpatialMatrix, src: &SpatialMatrix) {
    for r in 0..6 {
        for c in 0..6 {
            dst[r][c] += src[r][c];
        }
    }
}

pub(crate) fn spatial_vec_add_assign(dst: &mut SpatialVector, src: SpatialVector) {
    for i in 0..6 {
        dst[i] += src[i];
    }
}

pub(crate) fn spatial_inertia_from_blocks(theta: Mat3, h_mat: Mat3, m_mat: Mat3) -> SpatialMatrix {
    let mut out = spatial_zero();
    for r in 0..3 {
        for c in 0..3 {
            out[r][c] = theta.m[r][c];
            out[r][c + 3] = h_mat.m[r][c];
            out[r + 3][c] = h_mat.m[c][r];
            out[r + 3][c + 3] = m_mat.m[r][c];
        }
    }
    out
}

pub(crate) fn spatial_reduce(
    i_mat: &SpatialMatrix,
    u_vec: SpatialVector,
    d_inv: f64,
) -> SpatialMatrix {
    let mut out = *i_mat;
    for r in 0..6 {
        for c in 0..6 {
            out[r][c] -= u_vec[r] * u_vec[c] * d_inv;
        }
    }
    out
}

pub(crate) fn spatial_bias_reduce(
    p_vec: SpatialVector,
    u_vec: SpatialVector,
    ud: f64,
) -> SpatialVector {
    let mut out = p_vec;
    for i in 0..6 {
        out[i] += u_vec[i] * ud;
    }
    out
}

pub(crate) fn motion_shift_matrix(r_parent_to_acc_origin: Vec3) -> SpatialMatrix {
    let mut out = spatial_zero();
    for i in 0..3 {
        out[i][i] = 1.0;
        out[i + 3][i + 3] = 1.0;
    }
    let s = Mat3::skew(r_parent_to_acc_origin);
    for r in 0..3 {
        for c in 0..3 {
            out[r + 3][c] = -s.m[r][c];
        }
    }
    out
}

pub(crate) fn force_shift_matrix(r_parent_to_force_origin: Vec3) -> SpatialMatrix {
    let mut out = spatial_zero();
    for i in 0..3 {
        out[i][i] = 1.0;
        out[i + 3][i + 3] = 1.0;
    }
    let s = Mat3::skew(r_parent_to_force_origin);
    for r in 0..3 {
        for c in 0..3 {
            out[r][c + 3] = s.m[r][c];
        }
    }
    out
}

pub(crate) fn spatial_mul(a: &SpatialMatrix, b: &SpatialMatrix) -> SpatialMatrix {
    let mut out = spatial_zero();
    for r in 0..6 {
        for c in 0..6 {
            for k in 0..6 {
                out[r][c] += a[r][k] * b[k][c];
            }
        }
    }
    out
}

pub(crate) fn transform_spatial_inertia(
    i_mat: &SpatialMatrix,
    r_parent_to_acc_origin: Vec3,
    r_parent_to_force_origin: Vec3,
) -> SpatialMatrix {
    let x_acc = motion_shift_matrix(r_parent_to_acc_origin);
    let x_force = force_shift_matrix(r_parent_to_force_origin);
    spatial_mul(&x_force, &spatial_mul(i_mat, &x_acc))
}

pub(crate) fn transform_spatial_force(
    p_vec: SpatialVector,
    r_parent_to_force_origin: Vec3,
) -> SpatialVector {
    let x_force = force_shift_matrix(r_parent_to_force_origin);
    spatial_mat_vec(&x_force, p_vec)
}
