#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null, run_status,
};
use crate::algo;
use crate::core::math::Vec3;

#[unsafe(no_mangle)]
pub extern "C" fn pino_rnea_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    qdd_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    tau_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;

        let q_batch = unsafe { as_slice(q_batch, total)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let qdd_batch = unsafe { as_slice(qdd_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let tau_out = unsafe { as_mut_slice(tau_out, total)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        algo::rnea_batch(
            model_ref,
            q_batch,
            qd_batch,
            qdd_batch,
            batch_size,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            tau_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_bias_forces_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    bias_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let bias_out = unsafe { as_mut_slice(bias_out, total)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::bias_forces_batch(
            model_ref,
            q_batch,
            qd_batch,
            batch_size,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            bias_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_gravity_torques_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    g_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let g_out = unsafe { as_mut_slice(g_out, total)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::gravity_torques_batch(
            model_ref,
            q_batch,
            batch_size,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            g_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_crba_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    batch_size: usize,
    mass_out_row_major: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let total_q = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_m = batch_size
            .checked_mul(n)
            .and_then(|x| x.checked_mul(n))
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let mass_out = unsafe { as_mut_slice(mass_out_row_major, total_m)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::crba_batch(model_ref, q_batch, batch_size, ws_ref, mass_out)
            .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_aba_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    tau_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    qdd_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;

        let q_batch = unsafe { as_slice(q_batch, total)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let tau_batch = unsafe { as_slice(tau_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, total)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        algo::aba_batch(
            model_ref,
            q_batch,
            qd_batch,
            tau_batch,
            batch_size,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            qdd_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_rollout_aba_euler(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q0: *const f64,
    qd0: *const f64,
    tau_batch: *const f64,
    batch_size: usize,
    dt: f64,
    gravity_xyz: *const f64,
    q_out: *mut f64,
    qd_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;

        let q0 = unsafe { as_slice(q0, n)? };
        let qd0 = unsafe { as_slice(qd0, n)? };
        let tau_batch = unsafe { as_slice(tau_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let q_out = unsafe { as_mut_slice(q_out, total)? };
        let qd_out = unsafe { as_mut_slice(qd_out, total)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        algo::rollout_aba_euler(
            model_ref,
            algo::RolloutState { q0, qd0 },
            tau_batch,
            batch_size,
            dt,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            q_out,
            qd_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}
