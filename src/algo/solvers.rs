use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::contact::{ContactPoint, FrictionContactPoint};
use super::dynamics::{aba, cholesky_solve, crba};
use super::jacobian::contact_jacobian_normal;

#[derive(Debug, Clone)]
pub struct ContactLinearProblem {
    pub delassus: Vec<Vec<f64>>,
    pub rhs: Vec<f64>,
}

pub(super) fn projected_gauss_seidel_nonnegative(
    a: &[Vec<f64>],
    rhs: &[f64],
    iters: usize,
) -> Vec<f64> {
    let k = rhs.len();
    let mut x = vec![0.0; k];
    for _ in 0..iters {
        for i in 0..k {
            let mut s = rhs[i];
            for (j, xj) in x.iter().copied().enumerate().take(k) {
                if i != j {
                    s -= a[i][j] * xj;
                }
            }
            let diag = a[i][i].max(1e-9);
            x[i] = (s / diag).max(0.0);
        }
    }
    x
}

pub(super) fn projected_gauss_seidel_friction(
    a: &[Vec<f64>],
    rhs: &[f64],
    contacts: &[FrictionContactPoint],
    iters: usize,
) -> Vec<f64> {
    let k = contacts.len();
    let d = 3 * k;
    let mut x = vec![0.0; d];
    for _ in 0..iters {
        for (c, cp) in contacts.iter().enumerate() {
            let in_idx = 3 * c;
            let it1_idx = in_idx + 1;
            let it2_idx = in_idx + 2;

            let mut rn = rhs[in_idx];
            let mut rt1 = rhs[it1_idx];
            let mut rt2 = rhs[it2_idx];
            for (j, xj) in x.iter().copied().enumerate().take(d) {
                if j != in_idx {
                    rn -= a[in_idx][j] * xj;
                }
                if j != it1_idx {
                    rt1 -= a[it1_idx][j] * xj;
                }
                if j != it2_idx {
                    rt2 -= a[it2_idx][j] * xj;
                }
            }

            let dn = a[in_idx][in_idx].max(1e-9);
            let dt1 = a[it1_idx][it1_idx].max(1e-9);
            let dt2 = a[it2_idx][it2_idx].max(1e-9);

            let lambda_n = (rn / dn).max(0.0);
            let mut lambda_t1 = rt1 / dt1;
            let mut lambda_t2 = rt2 / dt2;

            let mu = cp.friction_coeff.max(0.0);
            let tnorm = (lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2).sqrt();
            let max_t = mu * lambda_n;
            if tnorm > max_t && tnorm > 1e-12 {
                let s = max_t / tnorm;
                lambda_t1 *= s;
                lambda_t2 *= s;
            }

            x[in_idx] = lambda_n;
            x[it1_idx] = lambda_t1;
            x[it2_idx] = lambda_t2;
        }
    }
    x
}

pub fn build_delassus_matrix(
    model: &Model,
    q: &[f64],
    contacts: &[ContactPoint],
    ws: &mut Workspace,
) -> Result<Vec<Vec<f64>>> {
    let n = model.nv();
    let k = contacts.len();
    let j = contact_jacobian_normal(model, q, contacts, ws)?;
    let mass = crba(model, q, ws)?;
    let mut j_rows = Vec::with_capacity(k);
    for i in 0..k {
        let b = i * n;
        j_rows.push(j[b..b + n].to_vec());
    }
    let minv_jt = super::contact::solve_m_inv_jt_columns(&mass, &j_rows)?;
    let mut w = vec![vec![0.0; k]; k];
    for i in 0..k {
        for (j, col) in minv_jt.iter().enumerate().take(k) {
            w[i][j] = j_rows[i]
                .iter()
                .zip(col.iter())
                .map(|(a, b)| a * b)
                .sum::<f64>();
        }
        w[i][i] += 1e-8;
    }
    Ok(w)
}

pub fn build_contact_problem(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[ContactPoint],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<ContactLinearProblem> {
    let qdd_free = aba(model, q, qd, tau, gravity, ws)?;
    let n = model.nv();
    let k = contacts.len();
    let j = contact_jacobian_normal(model, q, contacts, ws)?;
    let mut rhs = vec![0.0; k];
    for i in 0..k {
        let b = i * n;
        let mut a = 0.0;
        for c in 0..n {
            a += j[b + c] * qdd_free[c];
        }
        rhs[i] = -(a + contacts[i].acceleration_bias);
    }
    let delassus = build_delassus_matrix(model, q, contacts, ws)?;
    Ok(ContactLinearProblem { delassus, rhs })
}

pub fn solve_contact_cholesky(problem: &ContactLinearProblem) -> Result<Vec<f64>> {
    cholesky_solve(&problem.delassus, &problem.rhs)
}

pub fn solve_contact_pgs(problem: &ContactLinearProblem, max_iters: usize) -> Vec<f64> {
    projected_gauss_seidel_nonnegative(&problem.delassus, &problem.rhs, max_iters.max(1))
}

pub fn solve_contact_admm(
    problem: &ContactLinearProblem,
    rho: f64,
    max_iters: usize,
) -> Result<Vec<f64>> {
    let k = problem.rhs.len();
    if rho <= 0.0 {
        return Err(PinocchioError::InvalidModel("rho must be > 0"));
    }
    let mut z = vec![0.0; k];
    let mut u = vec![0.0; k];
    let mut a_rho = problem.delassus.clone();
    for (i, row) in a_rho.iter_mut().enumerate().take(k) {
        row[i] += rho;
    }
    for _ in 0..max_iters.max(1) {
        let mut b = vec![0.0; k];
        for i in 0..k {
            b[i] = problem.rhs[i] + rho * (z[i] - u[i]);
        }
        let x = cholesky_solve(&a_rho, &b)?;
        for i in 0..k {
            z[i] = (x[i] + u[i]).max(0.0);
            u[i] += x[i] - z[i];
        }
    }
    Ok(z)
}
