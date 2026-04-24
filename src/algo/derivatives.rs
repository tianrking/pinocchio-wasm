use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{JointType, Model, Workspace};

use super::com_energy::center_of_mass;
use super::constrained::constrained_aba_locked_joints;
use super::contact::apply_contact_impulses;
use super::dynamics::{aba, crba, is_ancestor, rnea};
use super::jacobian::frame_jacobian;
use super::kinematics::{forward_kinematics, forward_kinematics_poses};

#[derive(Debug, Clone)]
pub struct DerivativeResult {
    pub d_out_dq: Vec<Vec<f64>>,
    pub d_out_dv: Vec<Vec<f64>>,
    pub d_out_du: Vec<Vec<f64>>,
}

#[derive(Debug, Clone)]
pub struct KinematicsDerivativesResult {
    pub dpos_dq: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct FrameDerivativesResult {
    pub dframe_dq: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct SecondOrderDerivatives {
    pub d2_out_dq2: Vec<f64>,
    pub d2_out_dv2: Vec<f64>,
    pub d2_out_du2: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct ImpulseDerivativeResult {
    pub d_qd_plus_dq: Vec<Vec<f64>>,
    pub d_qd_plus_dqd_minus: Vec<Vec<f64>>,
    pub d_qd_plus_d_restitution: Vec<f64>,
}

pub fn rnea_derivatives_fd(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<DerivativeResult> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let eps = 1e-6;
    let mut d_tau_dq = vec![vec![0.0; n]; n];
    let mut d_tau_dqd = vec![vec![0.0; n]; n];
    let mut d_tau_dqdd = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let tp = rnea(model, &qp, qd, qdd, gravity, ws)?;
        let tm = rnea(model, &qm, qd, qdd, gravity, ws)?;
        for r in 0..n {
            d_tau_dq[r][i] = (tp[r] - tm[r]) / (2.0 * eps);
        }

        let mut v_p = qd.to_vec();
        let mut v_m = qd.to_vec();
        v_p[i] += eps;
        v_m[i] -= eps;
        let tp = rnea(model, q, &v_p, qdd, gravity, ws)?;
        let tm = rnea(model, q, &v_m, qdd, gravity, ws)?;
        for r in 0..n {
            d_tau_dqd[r][i] = (tp[r] - tm[r]) / (2.0 * eps);
        }

        let mut a_p = qdd.to_vec();
        let mut a_m = qdd.to_vec();
        a_p[i] += eps;
        a_m[i] -= eps;
        let tp = rnea(model, q, qd, &a_p, gravity, ws)?;
        let tm = rnea(model, q, qd, &a_m, gravity, ws)?;
        for r in 0..n {
            d_tau_dqdd[r][i] = (tp[r] - tm[r]) / (2.0 * eps);
        }
    }
    Ok(DerivativeResult {
        d_out_dq: d_tau_dq,
        d_out_dv: d_tau_dqd,
        d_out_du: d_tau_dqdd,
    })
}

pub fn rnea_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<DerivativeResult> {
    model.check_state_dims(q, qd, Some(qdd))?;

    // Batch 5 correctness gate: keep dq/dv on the already validated central
    // difference path, while preserving the analytical identity d(tau)/d(qdd)=M(q).
    //
    // The previous hand-written dq recursion missed ancestor perturbations in
    // downstream link force propagation. Until the full recursive sensitivity
    // pass is completed, this hybrid path is the safest production behavior:
    // q/v derivatives match RNEA exactly to finite-difference tolerance, and
    // acceleration derivatives remain exact through CRBA.
    let mut out = rnea_derivatives_fd(model, q, qd, qdd, gravity, ws)?;
    let d_tau_dqdd = crba(model, q, ws)?;
    out.d_out_du = d_tau_dqdd;
    Ok(out)
}

pub fn aba_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<DerivativeResult> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }
    let qdd = aba(model, q, qd, tau, gravity, ws)?;
    let rnea_deriv = rnea_derivatives(model, q, qd, &qdd, gravity, ws)?;
    let mass = &rnea_deriv.d_out_du;

    let mut dqdd_dtau = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut e = vec![0.0; n];
        e[i] = 1.0;
        let col = super::dynamics::cholesky_solve(mass, &e)?;
        for r in 0..n {
            dqdd_dtau[r][i] = col[r];
        }
    }

    let mut dqdd_dq = vec![vec![0.0; n]; n];
    let mut dqdd_dqd = vec![vec![0.0; n]; n];
    for c in 0..n {
        let rhs_q: Vec<f64> = (0..n).map(|r| -rnea_deriv.d_out_dq[r][c]).collect();
        let rhs_v: Vec<f64> = (0..n).map(|r| -rnea_deriv.d_out_dv[r][c]).collect();
        let sol_q = super::dynamics::cholesky_solve(mass, &rhs_q)?;
        let sol_v = super::dynamics::cholesky_solve(mass, &rhs_v)?;
        for r in 0..n {
            dqdd_dq[r][c] = sol_q[r];
            dqdd_dqd[r][c] = sol_v[r];
        }
    }

    Ok(DerivativeResult {
        d_out_dq: dqdd_dq,
        d_out_dv: dqdd_dqd,
        d_out_du: dqdd_dtau,
    })
}

pub fn kinematics_derivatives_fd(
    model: &Model,
    q: &[f64],
    target_link: usize,
    ws: &mut Workspace,
) -> Result<KinematicsDerivativesResult> {
    if target_link >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: target_link,
            len: model.nlinks(),
        });
    }
    let n = model.nv();
    let eps = 1e-6;
    let mut dpos_dq = vec![0.0; 3 * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let pp = forward_kinematics_poses(model, &qp, ws)?[target_link].translation;
        let pm = forward_kinematics_poses(model, &qm, ws)?[target_link].translation;
        dpos_dq[i] = (pp.x - pm.x) / (2.0 * eps);
        dpos_dq[n + i] = (pp.y - pm.y) / (2.0 * eps);
        dpos_dq[2 * n + i] = (pp.z - pm.z) / (2.0 * eps);
    }
    Ok(KinematicsDerivativesResult { dpos_dq })
}

pub fn kinematics_derivatives(
    model: &Model,
    q: &[f64],
    target_link: usize,
    ws: &mut Workspace,
) -> Result<KinematicsDerivativesResult> {
    if target_link >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: target_link,
            len: model.nlinks(),
        });
    }
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let p_target = ws.world_pose[target_link].translation;
    let mut dpos_dq = vec![0.0; 3 * n];

    for j in 0..model.njoints() {
        let link_of_joint = model.joint_link(j).expect("validated model");
        if !is_ancestor(model, link_of_joint, target_link) {
            continue;
        }
        let joint = model.links[link_of_joint]
            .joint
            .as_ref()
            .expect("validated model");
        if joint.nv() == 0 {
            continue;
        }

        let vi = model.idx_v(j);
        let axis = ws.world_joint_axis[j];
        let origin = ws.world_joint_origin[j];

        let dp_dq = match joint.jtype {
            JointType::Revolute => axis.cross(p_target - origin),
            JointType::Prismatic => axis,
            JointType::Fixed => continue,
        };
        dpos_dq[vi] = dp_dq.x;
        dpos_dq[n + vi] = dp_dq.y;
        dpos_dq[2 * n + vi] = dp_dq.z;
    }

    Ok(KinematicsDerivativesResult { dpos_dq })
}

pub fn frame_jacobian_derivatives(
    model: &Model,
    q: &[f64],
    target_link: usize,
    ws: &mut Workspace,
) -> Result<FrameDerivativesResult> {
    let n = model.nv();
    let eps = 1e-6;
    let mut dframe_dq = vec![0.0; 6 * n * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let jp = frame_jacobian(model, &qp, target_link, ws)?;
        let jm = frame_jacobian(model, &qm, target_link, ws)?;
        for r in 0..(6 * n) {
            dframe_dq[r * n + i] = (jp[r] - jm[r]) / (2.0 * eps);
        }
    }
    Ok(FrameDerivativesResult { dframe_dq })
}

pub fn center_of_mass_derivatives(
    model: &Model,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    let eps = 1e-6;
    let mut dcom_dq = vec![0.0; 3 * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let cp = center_of_mass(model, &qp, ws)?;
        let cm = center_of_mass(model, &qm, ws)?;
        dcom_dq[i] = (cp.x - cm.x) / (2.0 * eps);
        dcom_dq[n + i] = (cp.y - cm.y) / (2.0 * eps);
        dcom_dq[2 * n + i] = (cp.z - cm.z) / (2.0 * eps);
    }
    Ok(dcom_dq)
}

fn second_order_fd(
    x: &[f64],
    out_dim: usize,
    f: impl Fn(&[f64]) -> Result<Vec<f64>>,
) -> Result<Vec<f64>> {
    let n = x.len();
    let eps = 1e-5;
    let mut out = vec![0.0; out_dim * n * n];
    for i in 0..n {
        for j in 0..n {
            let mut xpp = x.to_vec();
            let mut xpm = x.to_vec();
            let mut xmp = x.to_vec();
            let mut xmm = x.to_vec();
            xpp[i] += eps;
            xpp[j] += eps;
            xpm[i] += eps;
            xpm[j] -= eps;
            xmp[i] -= eps;
            xmp[j] += eps;
            xmm[i] -= eps;
            xmm[j] -= eps;
            let ypp = f(&xpp)?;
            let ypm = f(&xpm)?;
            let ymp = f(&xmp)?;
            let ymm = f(&xmm)?;
            if ypp.len() != out_dim
                || ypm.len() != out_dim
                || ymp.len() != out_dim
                || ymm.len() != out_dim
            {
                return Err(PinocchioError::DimensionMismatch {
                    expected: out_dim,
                    got: ypp.len().min(ypm.len()).min(ymp.len()).min(ymm.len()),
                });
            }
            for r in 0..out_dim {
                out[(r * n + i) * n + j] = (ypp[r] - ypm[r] - ymp[r] + ymm[r]) / (4.0 * eps * eps);
            }
        }
    }
    Ok(out)
}

pub fn rnea_second_order_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    _ws: &mut Workspace,
) -> Result<SecondOrderDerivatives> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let d2_q = second_order_fd(q, n, |qv| {
        let mut ws = Workspace::new(model);
        rnea(model, qv, qd, qdd, gravity, &mut ws)
    })?;
    let d2_v = second_order_fd(qd, n, |vv| {
        let mut ws = Workspace::new(model);
        rnea(model, q, vv, qdd, gravity, &mut ws)
    })?;
    let d2_u = second_order_fd(qdd, n, |uv| {
        let mut ws = Workspace::new(model);
        rnea(model, q, qd, uv, gravity, &mut ws)
    })?;
    Ok(SecondOrderDerivatives {
        d2_out_dq2: d2_q,
        d2_out_dv2: d2_v,
        d2_out_du2: d2_u,
    })
}

pub fn constrained_dynamics_derivatives_locked_joints(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    locked: &[bool],
    gravity: Vec3,
    _ws: &mut Workspace,
) -> Result<DerivativeResult> {
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

    let eps = 1e-6;
    let mut dqdd_dq = vec![vec![0.0; n]; n];
    let mut dqdd_dqd = vec![vec![0.0; n]; n];
    let mut dqdd_dtau = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = constrained_aba_locked_joints(model, &qp, qd, tau, locked, gravity, &mut ws_p)?;
        let am = constrained_aba_locked_joints(model, &qm, qd, tau, locked, gravity, &mut ws_m)?;
        for r in 0..n {
            dqdd_dq[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut vp = qd.to_vec();
        let mut vm = qd.to_vec();
        vp[i] += eps;
        vm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = constrained_aba_locked_joints(model, q, &vp, tau, locked, gravity, &mut ws_p)?;
        let am = constrained_aba_locked_joints(model, q, &vm, tau, locked, gravity, &mut ws_m)?;
        for r in 0..n {
            dqdd_dqd[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut tp = tau.to_vec();
        let mut tm = tau.to_vec();
        tp[i] += eps;
        tm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = constrained_aba_locked_joints(model, q, qd, &tp, locked, gravity, &mut ws_p)?;
        let am = constrained_aba_locked_joints(model, q, qd, &tm, locked, gravity, &mut ws_m)?;
        for r in 0..n {
            dqdd_dtau[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }
    }
    Ok(DerivativeResult {
        d_out_dq: dqdd_dq,
        d_out_dv: dqdd_dqd,
        d_out_du: dqdd_dtau,
    })
}

pub fn impulse_dynamics_derivatives(
    model: &Model,
    q: &[f64],
    qd_minus: &[f64],
    contacts: &[super::contact::ContactPoint],
    restitution: f64,
    _ws: &mut Workspace,
) -> Result<ImpulseDerivativeResult> {
    let n = model.nv();
    model.check_state_dims(q, qd_minus, None)?;
    let eps = 1e-6;
    let mut d_dq = vec![vec![0.0; n]; n];
    let mut d_dv = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap =
            apply_contact_impulses(model, &qp, qd_minus, contacts, restitution, &mut ws_p)?.qd_plus;
        let am =
            apply_contact_impulses(model, &qm, qd_minus, contacts, restitution, &mut ws_m)?.qd_plus;
        for r in 0..n {
            d_dq[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut vp = qd_minus.to_vec();
        let mut vm = qd_minus.to_vec();
        vp[i] += eps;
        vm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = apply_contact_impulses(model, q, &vp, contacts, restitution, &mut ws_p)?.qd_plus;
        let am = apply_contact_impulses(model, q, &vm, contacts, restitution, &mut ws_m)?.qd_plus;
        for r in 0..n {
            d_dv[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }
    }

    let mut d_dr = vec![0.0; n];
    let mut ws_0 = Workspace::new(model);
    let y0 = apply_contact_impulses(model, q, qd_minus, contacts, restitution, &mut ws_0)?.qd_plus;
    let rp = restitution + eps;
    let mut ws_p = Workspace::new(model);
    let yp = apply_contact_impulses(model, q, qd_minus, contacts, rp, &mut ws_p)?.qd_plus;
    if restitution > eps {
        let rm = restitution - eps;
        let mut ws_m = Workspace::new(model);
        let ym = apply_contact_impulses(model, q, qd_minus, contacts, rm, &mut ws_m)?.qd_plus;
        for i in 0..n {
            d_dr[i] = (yp[i] - ym[i]) / (2.0 * eps);
        }
    } else {
        for i in 0..n {
            d_dr[i] = (yp[i] - y0[i]) / eps;
        }
    }

    Ok(ImpulseDerivativeResult {
        d_qd_plus_dq: d_dq,
        d_qd_plus_dqd_minus: d_dv,
        d_qd_plus_d_restitution: d_dr,
    })
}
