#![allow(clippy::too_many_arguments)]

use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::core::quaternion::Quat;
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
                let cur = Quat::from_array([q[qi], q[qi + 1], q[qi + 2], q[qi + 3]]).normalize();
                let omega = Vec3::new(v[vi], v[vi + 1], v[vi + 2]);
                let dq = Quat::delta(omega, dt);
                let next = cur.mul(dq).normalize();
                out[qi..qi + 4].copy_from_slice(&next.to_array());
            }
            JointType::FreeFlyer => {
                out[qi] = q[qi] + v[vi] * dt;
                out[qi + 1] = q[qi + 1] + v[vi + 1] * dt;
                out[qi + 2] = q[qi + 2] + v[vi + 2] * dt;
                let cur =
                    Quat::from_array([q[qi + 3], q[qi + 4], q[qi + 5], q[qi + 6]]).normalize();
                let omega = Vec3::new(v[vi + 3], v[vi + 4], v[vi + 5]);
                let dq = Quat::delta(omega, dt);
                let next = cur.mul(dq).normalize();
                out[qi + 3..qi + 7].copy_from_slice(&next.to_array());
            }
        }
    }

    Ok(out)
}

pub fn neutral_configuration(model: &Model) -> Vec<f64> {
    let mut out = vec![0.0; model.nq()];
    for j in 0..model.njoints() {
        let link_idx = model.joint_link(j).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");
        let qi = model.idx_q(j);
        match joint.jtype {
            JointType::Spherical => {
                out[qi..qi + 4].copy_from_slice(&[1.0, 0.0, 0.0, 0.0]);
            }
            JointType::FreeFlyer => {
                out[qi + 3..qi + 7].copy_from_slice(&[1.0, 0.0, 0.0, 0.0]);
            }
            _ => {}
        }
    }
    out
}

pub fn normalize_configuration(model: &Model, q: &[f64]) -> Result<Vec<f64>> {
    if q.len() != model.nq() {
        return Err(PinocchioError::DimensionMismatch {
            expected: model.nq(),
            got: q.len(),
        });
    }
    let mut out = q.to_vec();
    for j in 0..model.njoints() {
        let link_idx = model.joint_link(j).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");
        let qi = model.idx_q(j);
        match joint.jtype {
            JointType::Spherical => {
                let nq = Quat::from_array([q[qi], q[qi + 1], q[qi + 2], q[qi + 3]]).normalize();
                out[qi..qi + 4].copy_from_slice(&nq.to_array());
            }
            JointType::FreeFlyer => {
                let nq = Quat::from_array([
                    q[qi + 3],
                    q[qi + 4],
                    q[qi + 5],
                    q[qi + 6],
                ])
                .normalize();
                out[qi + 3..qi + 7].copy_from_slice(&nq.to_array());
            }
            _ => {}
        }
    }
    Ok(out)
}

pub fn is_normalized(model: &Model, q: &[f64], tol: f64) -> Result<bool> {
    if q.len() != model.nq() {
        return Err(PinocchioError::DimensionMismatch {
            expected: model.nq(),
            got: q.len(),
        });
    }
    for j in 0..model.njoints() {
        let link_idx = model.joint_link(j).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");
        let qi = model.idx_q(j);
        match joint.jtype {
            JointType::Spherical => {
                let q4 =
                    Quat::from_array([q[qi], q[qi + 1], q[qi + 2], q[qi + 3]]);
                if !q4.is_normalized(tol) {
                    return Ok(false);
                }
            }
            JointType::FreeFlyer => {
                let q4 = Quat::from_array([
                    q[qi + 3],
                    q[qi + 4],
                    q[qi + 5],
                    q[qi + 6],
                ]);
                if !q4.is_normalized(tol) {
                    return Ok(false);
                }
            }
            _ => {}
        }
    }
    Ok(true)
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

pub fn difference_configuration_plain(q0: &[f64], q1: &[f64]) -> Result<Vec<f64>> {
    if q0.len() != q1.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q0.len(),
            got: q1.len(),
        });
    }
    Ok(q1.iter().zip(q0.iter()).map(|(a, b)| a - b).collect())
}

pub fn interpolate_configuration_plain(q0: &[f64], q1: &[f64], alpha: f64) -> Result<Vec<f64>> {
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

pub fn random_configuration_plain(lower: &[f64], upper: &[f64], seed: u64) -> Result<Vec<f64>> {
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
        let r = lcg_next_f64(&mut st);
        out[i] = lower[i] + r * (upper[i] - lower[i]);
    }
    Ok(out)
}

pub fn difference_configuration(model: &Model, q0: &[f64], q1: &[f64]) -> Result<Vec<f64>> {
    if q0.len() != model.nq() || q1.len() != model.nq() {
        return Err(PinocchioError::DimensionMismatch {
            expected: model.nq(),
            got: q0.len(),
        });
    }
    let n = model.nv();
    let mut out = vec![0.0; n];
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
                out[vi] = q1[qi] - q0[qi];
            }
            JointType::Fixed => {}
            JointType::Spherical => {
                let a = Quat::from_array([q0[qi], q0[qi + 1], q0[qi + 2], q0[qi + 3]])
                    .normalize();
                let b = Quat::from_array([q1[qi], q1[qi + 1], q1[qi + 2], q1[qi + 3]])
                    .normalize();
                let diff = a.conjugate().mul(b).normalize();
                let omega = diff.log();
                out[vi] = omega.x;
                out[vi + 1] = omega.y;
                out[vi + 2] = omega.z;
            }
            JointType::FreeFlyer => {
                out[vi] = q1[qi] - q0[qi];
                out[vi + 1] = q1[qi + 1] - q0[qi + 1];
                out[vi + 2] = q1[qi + 2] - q0[qi + 2];
                let a = Quat::from_array([
                    q0[qi + 3],
                    q0[qi + 4],
                    q0[qi + 5],
                    q0[qi + 6],
                ])
                .normalize();
                let b = Quat::from_array([
                    q1[qi + 3],
                    q1[qi + 4],
                    q1[qi + 5],
                    q1[qi + 6],
                ])
                .normalize();
                let diff = a.conjugate().mul(b).normalize();
                let omega = diff.log();
                out[vi + 3] = omega.x;
                out[vi + 4] = omega.y;
                out[vi + 5] = omega.z;
            }
        }
    }
    Ok(out)
}

pub fn interpolate_configuration(
    model: &Model,
    q0: &[f64],
    q1: &[f64],
    alpha: f64,
) -> Result<Vec<f64>> {
    if !(0.0..=1.0).contains(&alpha) {
        return Err(PinocchioError::invalid_model("alpha must be in [0,1]"));
    }
    if q0.len() != model.nq() || q1.len() != model.nq() {
        return Err(PinocchioError::DimensionMismatch {
            expected: model.nq(),
            got: q0.len(),
        });
    }
    let nq = model.nq();
    let mut out = vec![0.0; nq];
    for j in 0..model.njoints() {
        let link_idx = model.joint_link(j).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");
        let qi = model.idx_q(j);
        match joint.jtype {
            JointType::Revolute | JointType::Prismatic => {
                out[qi] = (1.0 - alpha) * q0[qi] + alpha * q1[qi];
            }
            JointType::Fixed => {}
            JointType::Spherical => {
                let a = Quat::from_array([q0[qi], q0[qi + 1], q0[qi + 2], q0[qi + 3]])
                    .normalize();
                let b = Quat::from_array([q1[qi], q1[qi + 1], q1[qi + 2], q1[qi + 3]])
                    .normalize();
                let s = a.slerp(b, alpha);
                out[qi..qi + 4].copy_from_slice(&s.to_array());
            }
            JointType::FreeFlyer => {
                out[qi] = (1.0 - alpha) * q0[qi] + alpha * q1[qi];
                out[qi + 1] = (1.0 - alpha) * q0[qi + 1] + alpha * q1[qi + 1];
                out[qi + 2] = (1.0 - alpha) * q0[qi + 2] + alpha * q1[qi + 2];
                let a = Quat::from_array([
                    q0[qi + 3],
                    q0[qi + 4],
                    q0[qi + 5],
                    q0[qi + 6],
                ])
                .normalize();
                let b = Quat::from_array([
                    q1[qi + 3],
                    q1[qi + 4],
                    q1[qi + 5],
                    q1[qi + 6],
                ])
                .normalize();
                let s = a.slerp(b, alpha);
                out[qi + 3..qi + 7].copy_from_slice(&s.to_array());
            }
        }
    }
    Ok(out)
}

fn lcg_next_u64(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
    *state
}

fn lcg_next_f64(state: &mut u64) -> f64 {
    (lcg_next_u64(state) as f64) / (u64::MAX as f64)
}

pub fn random_configuration(model: &Model, lower: &[f64], upper: &[f64], seed: u64) -> Result<Vec<f64>> {
    if lower.len() != model.nq() || upper.len() != model.nq() {
        return Err(PinocchioError::DimensionMismatch {
            expected: model.nq(),
            got: lower.len(),
        });
    }
    let nq = model.nq();
    let mut st = seed;
    let mut out = vec![0.0; nq];
    for j in 0..model.njoints() {
        let link_idx = model.joint_link(j).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");
        let qi = model.idx_q(j);
        match joint.jtype {
            JointType::Revolute | JointType::Prismatic => {
                if upper[qi] < lower[qi] {
                    return Err(PinocchioError::invalid_model(
                        "upper bound must be >= lower bound",
                    ));
                }
                let r = lcg_next_f64(&mut st);
                out[qi] = lower[qi] + r * (upper[qi] - lower[qi]);
            }
            JointType::Fixed => {}
            JointType::Spherical => {
                let r1 = lcg_next_f64(&mut st);
                let _r2 = lcg_next_f64(&mut st);
                let q = Quat::uniform_random(r1);
                out[qi..qi + 4].copy_from_slice(&q.to_array());
            }
            JointType::FreeFlyer => {
                for k in 0..3 {
                    let idx = qi + k;
                    if upper[idx] < lower[idx] {
                        return Err(PinocchioError::invalid_model(
                            "upper bound must be >= lower bound",
                        ));
                    }
                    let r = lcg_next_f64(&mut st);
                    out[idx] = lower[idx] + r * (upper[idx] - lower[idx]);
                }
                let r1 = lcg_next_f64(&mut st);
                let _r2 = lcg_next_f64(&mut st);
                let q = Quat::uniform_random(r1);
                out[qi + 3..qi + 7].copy_from_slice(&q.to_array());
            }
        }
    }
    Ok(out)
}
