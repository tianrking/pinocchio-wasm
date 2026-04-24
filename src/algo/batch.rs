use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::dynamics::{aba, bias_forces, crba, gravity_torques, rnea};

pub fn rnea_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    qdd_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    tau_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if qd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_batch.len(),
        });
    }
    if qdd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qdd_batch.len(),
        });
    }
    if tau_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let tau = rnea(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &qdd_batch[b..b + n],
            gravity,
            ws,
        )?;
        tau_out[b..b + n].copy_from_slice(&tau);
    }
    Ok(())
}

pub fn aba_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    tau_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    qdd_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
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

    for step in 0..batch_size {
        let b = step * n;
        let qdd = aba(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &tau_batch[b..b + n],
            gravity,
            ws,
        )?;
        qdd_out[b..b + n].copy_from_slice(&qdd);
    }
    Ok(())
}

pub fn bias_forces_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    bias_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if qd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_batch.len(),
        });
    }
    if bias_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: bias_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let bias = bias_forces(model, &q_batch[b..b + n], &qd_batch[b..b + n], gravity, ws)?;
        bias_out[b..b + n].copy_from_slice(&bias);
    }
    Ok(())
}

pub fn gravity_torques_batch(
    model: &Model,
    q_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    g_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if g_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: g_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let g = gravity_torques(model, &q_batch[b..b + n], gravity, ws)?;
        g_out[b..b + n].copy_from_slice(&g);
    }
    Ok(())
}

pub fn crba_batch(
    model: &Model,
    q_batch: &[f64],
    batch_size: usize,
    ws: &mut Workspace,
    mass_out_row_major: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected_q = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_m = batch_size
        .checked_mul(n)
        .and_then(|x| x.checked_mul(n))
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected_q {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_q,
            got: q_batch.len(),
        });
    }
    if mass_out_row_major.len() != expected_m {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_m,
            got: mass_out_row_major.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let mb = step * n * n;
        let mass = crba(model, &q_batch[qb..qb + n], ws)?;
        for r in 0..n {
            for c in 0..n {
                mass_out_row_major[mb + r * n + c] = mass[r][c];
            }
        }
    }
    Ok(())
}
