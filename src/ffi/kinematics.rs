#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null, run_status,
};
use crate::algo;
use crate::core::math::Vec3;

#[unsafe(no_mangle)]
pub extern "C" fn pino_forward_kinematics_poses(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    translations_out: *mut f64,
    rotations_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let nl = model_ref.nlinks();
        let q = unsafe { as_slice(q, nq)? };
        let translations_out = unsafe { as_mut_slice(translations_out, 3 * nl)? };
        let rotations_out = unsafe { as_mut_slice(rotations_out, 9 * nl)? };
        let ws_ref = unsafe { &mut (*ws).ws };

        let poses =
            algo::forward_kinematics_poses(model_ref, q, ws_ref).map_err(|_| Status::AlgoFailed)?;
        for (l, pose) in poses.iter().enumerate() {
            let tb = l * 3;
            translations_out[tb] = pose.translation.x;
            translations_out[tb + 1] = pose.translation.y;
            translations_out[tb + 2] = pose.translation.z;

            let rb = l * 9;
            for r in 0..3 {
                for c in 0..3 {
                    rotations_out[rb + 3 * r + c] = pose.rotation.m[r][c];
                }
            }
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_forward_kinematics_poses_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    batch_size: usize,
    translations_out: *mut f64,
    rotations_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let nl = model_ref.nlinks();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total_t = batch_size
            .checked_mul(nl)
            .and_then(|x| x.checked_mul(3))
            .ok_or(Status::InvalidInput)?;
        let total_r = batch_size
            .checked_mul(nl)
            .and_then(|x| x.checked_mul(9))
            .ok_or(Status::InvalidInput)?;

        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let translations_out = unsafe { as_mut_slice(translations_out, total_t)? };
        let rotations_out = unsafe { as_mut_slice(rotations_out, total_r)? };
        let ws_ref = unsafe { &mut (*ws).ws };

        algo::forward_kinematics_poses_batch(
            model_ref,
            q_batch,
            batch_size,
            ws_ref,
            translations_out,
            rotations_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_compute_all_terms(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    gravity_xyz: *const f64,
    mass_out_row_major: *mut f64,
    bias_out: *mut f64,
    gravity_out: *mut f64,
    coriolis_out: *mut f64,
    com_out_xyz: *mut f64,
    kinetic_out: *mut f64,
    potential_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        if kinetic_out.is_null() || potential_out.is_null() {
            return Err(Status::NullPtr);
        }

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let mass_out = unsafe { as_mut_slice(mass_out_row_major, n * n)? };
        let bias_out = unsafe { as_mut_slice(bias_out, n)? };
        let gravity_out = unsafe { as_mut_slice(gravity_out, n)? };
        let coriolis_out = unsafe { as_mut_slice(coriolis_out, n)? };
        let com_out = unsafe { as_mut_slice(com_out_xyz, 3)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let terms = algo::compute_all_terms(model_ref, q, qd, Vec3::new(g[0], g[1], g[2]), ws_ref)
            .map_err(|_| Status::AlgoFailed)?;

        for r in 0..n {
            for c in 0..n {
                mass_out[r * n + c] = terms.mass[r][c];
            }
        }
        bias_out.copy_from_slice(&terms.bias);
        gravity_out.copy_from_slice(&terms.gravity_torques);
        coriolis_out.copy_from_slice(&terms.coriolis_torques);
        com_out[0] = terms.com.x;
        com_out[1] = terms.com.y;
        com_out[2] = terms.com.z;
        unsafe {
            *kinetic_out = terms.kinetic_energy;
            *potential_out = terms.potential_energy;
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_frame_jacobian(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    target_link: usize,
    jac_out_row_major_6xn: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let jac_out = unsafe { as_mut_slice(jac_out_row_major_6xn, 6 * n)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        let jac = algo::frame_jacobian(model_ref, q, target_link, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        jac_out.copy_from_slice(&jac);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_center_of_mass(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    com_out_xyz: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let q = unsafe { as_slice(q, nq)? };
        let com_out = unsafe { as_mut_slice(com_out_xyz, 3)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        let com = algo::center_of_mass(model_ref, q, ws_ref).map_err(|_| Status::AlgoFailed)?;
        com_out[0] = com.x;
        com_out[1] = com.y;
        com_out[2] = com.z;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_energy(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    gravity_xyz: *const f64,
    kinetic_out: *mut f64,
    potential_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        if kinetic_out.is_null() || potential_out.is_null() {
            return Err(Status::NullPtr);
        }

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        let ke = algo::kinetic_energy(model_ref, q, qd, ws_ref).map_err(|_| Status::AlgoFailed)?;
        let pe = algo::potential_energy(model_ref, q, Vec3::new(g[0], g[1], g[2]), ws_ref)
            .map_err(|_| Status::AlgoFailed)?;

        unsafe {
            *kinetic_out = ke;
            *potential_out = pe;
        }
        Ok(())
    }) as i32
}
