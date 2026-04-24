use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::dynamics::{bias_forces, cholesky_solve, crba};

pub fn constrained_aba_locked_joints(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    locked: &[bool],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }
    if locked.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: locked.len(),
        });
    }

    let free: Vec<usize> = (0..n).filter(|i| !locked[*i]).collect();
    if free.is_empty() {
        return Ok(vec![0.0; n]);
    }

    let bias = bias_forces(model, q, qd, gravity, ws)?;
    let mass = crba(model, q, ws)?;

    let nf = free.len();
    let mut m_ff = vec![vec![0.0; nf]; nf];
    let mut rhs_f = vec![0.0; nf];
    for (r, &i) in free.iter().enumerate() {
        rhs_f[r] = tau[i] - bias[i];
        for (c, &j) in free.iter().enumerate() {
            m_ff[r][c] = mass[i][j];
        }
    }

    let qdd_f = cholesky_solve(&m_ff, &rhs_f)?;
    let mut qdd = vec![0.0; n];
    for (k, &idx) in free.iter().enumerate() {
        qdd[idx] = qdd_f[k];
    }
    Ok(qdd)
}

pub fn constrained_aba_locked_joints_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    tau_batch: &[f64],
    batch_size: usize,
    locked: &[bool],
    gravity: Vec3,
    ws: &mut Workspace,
    qdd_out: &mut [f64],
) -> Result<()> {
    let nq = model.nq();
    let n = model.nv();
    let expected_q = batch_size
        .checked_mul(nq)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    if q_batch.len() != expected_q {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_q,
            got: q_batch.len(),
        });
    }
    if qd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_batch.len(),
        });
    }
    if tau_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_batch.len(),
        });
    }
    if qdd_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qdd_out.len(),
        });
    }
    if locked.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: locked.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * nq;
        let b = step * n;
        let qdd = constrained_aba_locked_joints(
            model,
            &q_batch[qb..qb + nq],
            &qd_batch[b..b + n],
            &tau_batch[b..b + n],
            locked,
            gravity,
            ws,
        )?;
        qdd_out[b..b + n].copy_from_slice(&qdd);
    }
    Ok(())
}
