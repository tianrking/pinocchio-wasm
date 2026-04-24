#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null, parse_contacts,
    run_status,
};
use crate::algo;
use crate::core::math::Vec3;

#[unsafe(no_mangle)]
pub extern "C" fn pino_rnea_second_order_derivatives(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    gravity_xyz: *const f64,
    d2_tau_dq2_out: *mut f64,
    d2_tau_dqd2_out: *mut f64,
    d2_tau_dqdd2_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let cube = n
            .checked_mul(n)
            .and_then(|x| x.checked_mul(n))
            .ok_or(Status::InvalidInput)?;
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let d2_q = unsafe { as_mut_slice(d2_tau_dq2_out, cube)? };
        let d2_v = unsafe { as_mut_slice(d2_tau_dqd2_out, cube)? };
        let d2_u = unsafe { as_mut_slice(d2_tau_dqdd2_out, cube)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::rnea_second_order_derivatives(
            model_ref,
            q,
            qd,
            qdd,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        d2_q.copy_from_slice(&out.d2_out_dq2);
        d2_v.copy_from_slice(&out.d2_out_dv2);
        d2_u.copy_from_slice(&out.d2_out_du2);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_constrained_dynamics_derivatives_locked_joints(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    tau: *const f64,
    locked_mask_i32: *const i32,
    gravity_xyz: *const f64,
    dqdd_dq_out: *mut f64,
    dqdd_dqd_out: *mut f64,
    dqdd_dtau_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let mat = n.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let tau = unsafe { as_slice(tau, n)? };
        let locked_mask = unsafe { as_slice(locked_mask_i32, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let dq = unsafe { as_mut_slice(dqdd_dq_out, mat)? };
        let dv = unsafe { as_mut_slice(dqdd_dqd_out, mat)? };
        let du = unsafe { as_mut_slice(dqdd_dtau_out, mat)? };
        let locked: Vec<bool> = locked_mask.iter().map(|v| *v != 0).collect();
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::constrained_dynamics_derivatives_locked_joints(
            model_ref,
            q,
            qd,
            tau,
            &locked,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        for r in 0..n {
            for c in 0..n {
                dq[r * n + c] = out.d_out_dq[r][c];
                dv[r * n + c] = out.d_out_dv[r][c];
                du[r * n + c] = out.d_out_du[r][c];
            }
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_impulse_dynamics_derivatives(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd_minus: *const f64,
    restitution: f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    dqdplus_dq_out: *mut f64,
    dqdplus_dqdminus_out: *mut f64,
    dqdplus_drestitution_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let mat = n.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q = unsafe { as_slice(q, n)? };
        let qd_minus = unsafe { as_slice(qd_minus, n)? };
        let dq = unsafe { as_mut_slice(dqdplus_dq_out, mat)? };
        let dv = unsafe { as_mut_slice(dqdplus_dqdminus_out, mat)? };
        let dr = unsafe { as_mut_slice(dqdplus_drestitution_out, n)? };
        let zero_bias = vec![0.0; num_contacts];
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            zero_bias.as_ptr(),
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::impulse_dynamics_derivatives(
            model_ref,
            q,
            qd_minus,
            &contacts,
            restitution,
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        for r in 0..n {
            for c in 0..n {
                dq[r * n + c] = out.d_qd_plus_dq[r][c];
                dv[r * n + c] = out.d_qd_plus_dqd_minus[r][c];
            }
        }
        dr.copy_from_slice(&out.d_qd_plus_d_restitution);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_kinematics_derivatives(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    target_link: usize,
    dpos_dq_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let out_len = 3 * n;
        let q = unsafe { as_slice(q, n)? };
        let dpos_dq = unsafe { as_mut_slice(dpos_dq_out, out_len)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::kinematics_derivatives(model_ref, q, target_link, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        dpos_dq.copy_from_slice(&out.dpos_dq);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_rnea_derivatives(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    gravity_xyz: *const f64,
    dtau_dq_out: *mut f64,
    dtau_dqd_out: *mut f64,
    dtau_dqdd_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let mat = n.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let dq = unsafe { as_mut_slice(dtau_dq_out, mat)? };
        let dv = unsafe { as_mut_slice(dtau_dqd_out, mat)? };
        let da = unsafe { as_mut_slice(dtau_dqdd_out, mat)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let out =
            algo::rnea_derivatives(model_ref, q, qd, qdd, Vec3::new(g[0], g[1], g[2]), ws_ref)
                .map_err(|_| Status::AlgoFailed)?;
        for r in 0..n {
            for c in 0..n {
                dq[r * n + c] = out.d_out_dq[r][c];
                dv[r * n + c] = out.d_out_dv[r][c];
                da[r * n + c] = out.d_out_du[r][c];
            }
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_aba_derivatives(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    tau: *const f64,
    gravity_xyz: *const f64,
    dqdd_dq_out: *mut f64,
    dqdd_dqd_out: *mut f64,
    dqdd_dtau_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let mat = n.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let tau = unsafe { as_slice(tau, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let dq = unsafe { as_mut_slice(dqdd_dq_out, mat)? };
        let dv = unsafe { as_mut_slice(dqdd_dqd_out, mat)? };
        let du = unsafe { as_mut_slice(dqdd_dtau_out, mat)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::aba_derivatives(model_ref, q, qd, tau, Vec3::new(g[0], g[1], g[2]), ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        for r in 0..n {
            for c in 0..n {
                dq[r * n + c] = out.d_out_dq[r][c];
                dv[r * n + c] = out.d_out_dv[r][c];
                du[r * n + c] = out.d_out_du[r][c];
            }
        }
        Ok(())
    }) as i32
}
