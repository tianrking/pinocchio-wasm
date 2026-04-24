#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{ModelHandle, Status, as_mut_slice, as_slice, check_non_null, run_status};
use crate::algo;
use crate::core::math::Vec3;

#[unsafe(no_mangle)]
pub extern "C" fn pino_inverse_dynamics_regressor(
    model: *const ModelHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    gravity_xyz: *const f64,
    regressor_out_row_major: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let p = 10 * model_ref.nlinks();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let out = unsafe { as_mut_slice(regressor_out_row_major, n * p)? };
        let y =
            algo::inverse_dynamics_regressor(model_ref, q, qd, qdd, Vec3::new(g[0], g[1], g[2]))
                .map_err(|_| Status::AlgoFailed)?;
        out.copy_from_slice(&y);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_inverse_dynamics_regressor_batch(
    model: *const ModelHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    qdd_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    regressor_out_row_major: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let p = 10 * model_ref.nlinks();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let qdd_batch = unsafe { as_slice(qdd_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let out = unsafe { as_mut_slice(regressor_out_row_major, batch_size * n * p)? };
        let y = algo::inverse_dynamics_regressor_batch(
            model_ref,
            q_batch,
            qd_batch,
            qdd_batch,
            batch_size,
            Vec3::new(g[0], g[1], g[2]),
        )
        .map_err(|_| Status::AlgoFailed)?;
        out.copy_from_slice(&y);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_kinetic_energy_regressor(
    model: *const ModelHandle,
    q: *const f64,
    qd: *const f64,
    regressor_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let p = 10 * model_ref.nlinks();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let out = unsafe { as_mut_slice(regressor_out, p)? };
        let y = algo::kinetic_energy_regressor(model_ref, q, qd).map_err(|_| Status::AlgoFailed)?;
        out.copy_from_slice(&y);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_potential_energy_regressor(
    model: *const ModelHandle,
    q: *const f64,
    gravity_xyz: *const f64,
    regressor_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let p = 10 * model_ref.nlinks();
        let q = unsafe { as_slice(q, nq)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let out = unsafe { as_mut_slice(regressor_out, p)? };
        let y = algo::potential_energy_regressor(model_ref, q, Vec3::new(g[0], g[1], g[2]))
            .map_err(|_| Status::AlgoFailed)?;
        out.copy_from_slice(&y);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_center_of_mass_regressor(
    model: *const ModelHandle,
    q: *const f64,
    regressor_out_3xp: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let p = 10 * model_ref.nlinks();
        let q = unsafe { as_slice(q, nq)? };
        let out = unsafe { as_mut_slice(regressor_out_3xp, 3 * p)? };
        let y = algo::center_of_mass_regressor(model_ref, q).map_err(|_| Status::AlgoFailed)?;
        out.copy_from_slice(&y);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_regressor_select_independent_columns(
    regressor_row_major: *const f64,
    rows: usize,
    cols: usize,
    tolerance: f64,
    out_selected_count: *mut usize,
    out_selected_indices: *mut usize,
    out_projected_row_major: *mut f64,
) -> i32 {
    run_status(|| {
        if out_selected_count.is_null() {
            return Err(Status::NullPtr);
        }
        let y = unsafe { as_slice(regressor_row_major, rows * cols)? };
        let idx_out = unsafe { as_mut_slice(out_selected_indices, cols)? };
        let proj_out = unsafe { as_mut_slice(out_projected_row_major, rows * cols)? };
        let basis = algo::select_independent_regressor_columns(y, rows, cols, tolerance)
            .map_err(|_| Status::AlgoFailed)?;
        let k = basis.selected_columns.len();
        unsafe {
            *out_selected_count = k;
        }
        idx_out[..k].copy_from_slice(&basis.selected_columns);
        for r in 0..rows {
            for c in 0..k {
                proj_out[r * cols + c] = basis.projected_row_major[r * k + c];
            }
        }
        Ok(())
    }) as i32
}
