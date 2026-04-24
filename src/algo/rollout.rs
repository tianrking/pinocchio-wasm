use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{JointType, Model, Workspace};

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

    let nq = model.nq();
    let n = model.nv();
    model.check_state_dims(init.q0, init.qd0, None)?;
    let expected_q = batch_size
        .checked_mul(nq)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    if tau_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_batch.len(),
        });
    }
    if q_out.len() != expected_q {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_q,
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
        let qb = step * nq;
        let b = step * n;
        let tau = &tau_batch[b..b + n];
        let qdd = aba(model, &q, &qd, tau, gravity, ws)?;
        for i in 0..n {
            qd[i] += qdd[i] * dt;
        }
        q = integrate_model_configuration(model, &q, &qd, dt)?;
        q_out[qb..qb + nq].copy_from_slice(&q);
        qd_out[b..b + n].copy_from_slice(&qd);
    }
    Ok(())
}

fn normalize_quat(mut q: [f64; 4]) -> [f64; 4] {
    let n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if n2 <= 1e-16 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let inv = 1.0 / n2.sqrt();
    for x in &mut q {
        *x *= inv;
    }
    q
}

fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn delta_quat(wx: f64, wy: f64, wz: f64, dt: f64) -> [f64; 4] {
    let w = Vec3::new(wx, wy, wz);
    let angle = w.norm() * dt;
    if angle <= 1e-12 {
        return normalize_quat([1.0, 0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt]);
    }
    let axis = w * (1.0 / w.norm());
    let half = 0.5 * angle;
    let s = half.sin();
    [half.cos(), axis.x * s, axis.y * s, axis.z * s]
}

pub fn integrate_model_configuration(
    model: &Model,
    q: &[f64],
    v: &[f64],
    dt: f64,
) -> Result<Vec<f64>> {
    if dt <= 0.0 {
        return Err(PinocchioError::invalid_model("dt must be > 0"));
    }
    model.check_state_dims(q, v, None)?;

    let mut out = q.to_vec();
    for j in 0..model.njoints() {
        let link_idx = model.joint_link(j).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");
        let qi = model.idx_q(j);
        let vi = model.idx_v(j);
        match joint.jtype {
            JointType::Revolute | JointType::Prismatic => {
                out[qi] = q[qi] + v[vi] * dt;
            }
            JointType::Fixed => {}
            JointType::Spherical => {
                let cur = normalize_quat([q[qi], q[qi + 1], q[qi + 2], q[qi + 3]]);
                let dq = delta_quat(v[vi], v[vi + 1], v[vi + 2], dt);
                let next = normalize_quat(quat_mul(cur, dq));
                out[qi..qi + 4].copy_from_slice(&next);
            }
            JointType::FreeFlyer => {
                out[qi] = q[qi] + v[vi] * dt;
                out[qi + 1] = q[qi + 1] + v[vi + 1] * dt;
                out[qi + 2] = q[qi + 2] + v[vi + 2] * dt;
                let cur = normalize_quat([q[qi + 3], q[qi + 4], q[qi + 5], q[qi + 6]]);
                let dq = delta_quat(v[vi + 3], v[vi + 4], v[vi + 5], dt);
                let next = normalize_quat(quat_mul(cur, dq));
                out[qi + 3..qi + 7].copy_from_slice(&next);
            }
        }
    }

    Ok(out)
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
