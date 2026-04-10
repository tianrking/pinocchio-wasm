use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::dynamics::aba;

pub struct RolloutState<'a> {
    pub q0: &'a [f64],
    pub qd0: &'a [f64],
}

pub fn rollout_aba_euler(
    model: &Model,
    init: RolloutState<'_>,
    tau_batch: &[f64],
    batch_size: usize,
    dt: f64,
    gravity: Vec3,
    ws: &mut Workspace,
    q_out: &mut [f64],
    qd_out: &mut [f64],
) -> Result<()> {
    if dt <= 0.0 {
        return Err(PinocchioError::InvalidModel("dt must be > 0"));
    }

    let n = model.nv();
    model.check_state_dims(init.q0, init.qd0, None)?;
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if tau_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_batch.len(),
        });
    }
    if q_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_out.len(),
        });
    }
    if qd_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_out.len(),
        });
    }

    let mut q = init.q0.to_vec();
    let mut qd = init.qd0.to_vec();
    for step in 0..batch_size {
        let b = step * n;
        let tau = &tau_batch[b..b + n];
        let qdd = aba(model, &q, &qd, tau, gravity, ws)?;
        for i in 0..n {
            qd[i] += qdd[i] * dt;
            q[i] += qd[i] * dt;
        }
        q_out[b..b + n].copy_from_slice(&q);
        qd_out[b..b + n].copy_from_slice(&qd);
    }
    Ok(())
}
