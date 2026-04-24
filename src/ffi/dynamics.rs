#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null, run_status,
};
use crate::algo;
use crate::core::math::Vec3;

#[unsafe(no_mangle)]
pub extern "C" fn pino_rnea(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    gravity_xyz: *const f64,
    tau_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();

        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let tau_out = unsafe { as_mut_slice(tau_out, n)? };

        let gravity = Vec3::new(g[0], g[1], g[2]);
        let ws_ref = unsafe { &mut (*ws).ws };
        let tau =
            algo::rnea(model_ref, q, qd, qdd, gravity, ws_ref).map_err(|_| Status::AlgoFailed)?;
        tau_out.copy_from_slice(&tau);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_aba(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    tau: *const f64,
    gravity_xyz: *const f64,
    qdd_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();

        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let tau = unsafe { as_slice(tau, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, n)? };

        let gravity = Vec3::new(g[0], g[1], g[2]);
        let ws_ref = unsafe { &mut (*ws).ws };
        let qdd =
            algo::aba(model_ref, q, qd, tau, gravity, ws_ref).map_err(|_| Status::AlgoFailed)?;
        qdd_out.copy_from_slice(&qdd);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_gravity_torques(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    gravity_xyz: *const f64,
    g_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let g_out = unsafe { as_mut_slice(g_out, n)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::gravity_torques(model_ref, q, Vec3::new(g[0], g[1], g[2]), ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        g_out.copy_from_slice(&out);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_coriolis_torques(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    c_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let c_out = unsafe { as_mut_slice(c_out, n)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let out =
            algo::coriolis_torques(model_ref, q, qd, ws_ref).map_err(|_| Status::AlgoFailed)?;
        c_out.copy_from_slice(&out);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_crba(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    mass_out_row_major: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let mass_out = unsafe { as_mut_slice(mass_out_row_major, n * n)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        let mass = algo::crba(model_ref, q, ws_ref).map_err(|_| Status::AlgoFailed)?;
        for r in 0..n {
            for c in 0..n {
                mass_out[r * n + c] = mass[r][c];
            }
        }
        Ok(())
    }) as i32
}
