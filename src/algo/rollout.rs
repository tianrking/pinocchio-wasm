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
        return Err(PinocchioError::invalid_model("dt must be > 0"));
    }

    let n = model.nv();
    model.check_state_dims(init.q0, init.qd0, None)?;
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
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

pub fn integrate_configuration(q: &[f64], v: &[f64], dt: f64) -> Result<Vec<f64>> {
    if q.len() != v.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q.len(),
            got: v.len(),
        });
    }
    if dt <= 0.0 {
        return Err(PinocchioError::invalid_model("dt must be > 0"));
    }
    let mut out = vec![0.0; q.len()];
    for i in 0..q.len() {
        out[i] = q[i] + v[i] * dt;
    }
    Ok(out)
}

pub fn difference_configuration(q0: &[f64], q1: &[f64]) -> Result<Vec<f64>> {
    if q0.len() != q1.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q0.len(),
            got: q1.len(),
        });
    }
    Ok(q1.iter().zip(q0.iter()).map(|(a, b)| a - b).collect())
}

pub fn interpolate_configuration(q0: &[f64], q1: &[f64], alpha: f64) -> Result<Vec<f64>> {
    if !(0.0..=1.0).contains(&alpha) {
        return Err(PinocchioError::invalid_model("alpha must be in [0,1]"));
    }
    if q0.len() != q1.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q0.len(),
            got: q1.len(),
        });
    }
    Ok(q0
        .iter()
        .zip(q1.iter())
        .map(|(a, b)| (1.0 - alpha) * a + alpha * b)
        .collect())
}

fn lcg_next_u64(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
    *state
}

pub fn random_configuration(lower: &[f64], upper: &[f64], seed: u64) -> Result<Vec<f64>> {
    if lower.len() != upper.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: lower.len(),
            got: upper.len(),
        });
    }
    let mut st = seed;
    let mut out = vec![0.0; lower.len()];
    for i in 0..lower.len() {
        if upper[i] < lower[i] {
            return Err(PinocchioError::invalid_model(
                "upper bound must be >= lower bound",
            ));
        }
        let r = (lcg_next_u64(&mut st) as f64) / (u64::MAX as f64);
        out[i] = lower[i] + r * (upper[i] - lower[i]);
    }
    Ok(out)
}
