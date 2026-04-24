#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null, parse_contacts,
    parse_friction_contacts, run_status,
};
use crate::algo;
use crate::core::math::Vec3;

#[unsafe(no_mangle)]
pub extern "C" fn pino_contact_constrained_dynamics(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    tau: *const f64,
    gravity_xyz: *const f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    contact_accel_bias: *const f64,
    qdd_out: *mut f64,
    lambda_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let tau = unsafe { as_slice(tau, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, n)? };
        let lambda_out = unsafe { as_mut_slice(lambda_out, num_contacts)? };
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            contact_accel_bias,
        )?;

        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::constrained_forward_dynamics_contacts(
            model_ref,
            q,
            qd,
            tau,
            &contacts,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        qdd_out.copy_from_slice(&out.qdd);
        lambda_out.copy_from_slice(&out.lambda_normal);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_contact_constrained_dynamics_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    tau_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    contact_accel_bias: *const f64,
    qdd_out: *mut f64,
    lambda_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_lambda = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let tau_batch = unsafe { as_slice(tau_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, total)? };
        let lambda_out = unsafe { as_mut_slice(lambda_out, total_lambda)? };
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            contact_accel_bias,
        )?;

        let ws_ref = unsafe { &mut (*ws).ws };
        algo::constrained_forward_dynamics_contacts_batch(
            model_ref,
            q_batch,
            qd_batch,
            tau_batch,
            batch_size,
            &contacts,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            qdd_out,
            lambda_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_apply_contact_impulse(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd_minus: *const f64,
    restitution: f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    qd_plus_out: *mut f64,
    impulse_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd_minus = unsafe { as_slice(qd_minus, n)? };
        let qd_plus_out = unsafe { as_mut_slice(qd_plus_out, n)? };
        let impulse_out = unsafe { as_mut_slice(impulse_out, num_contacts)? };
        let zero_bias = vec![0.0; num_contacts];
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            zero_bias.as_ptr(),
        )?;

        let ws_ref = unsafe { &mut (*ws).ws };
        let out =
            algo::apply_contact_impulses(model_ref, q, qd_minus, &contacts, restitution, ws_ref)
                .map_err(|_| Status::AlgoFailed)?;
        qd_plus_out.copy_from_slice(&out.qd_plus);
        impulse_out.copy_from_slice(&out.impulse_normal);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_apply_contact_impulse_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_minus_batch: *const f64,
    batch_size: usize,
    restitution: f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    qd_plus_out: *mut f64,
    impulse_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_impulse = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let qd_minus_batch = unsafe { as_slice(qd_minus_batch, total)? };
        let qd_plus_out = unsafe { as_mut_slice(qd_plus_out, total)? };
        let impulse_out = unsafe { as_mut_slice(impulse_out, total_impulse)? };
        let zero_bias = vec![0.0; num_contacts];
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            zero_bias.as_ptr(),
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::apply_contact_impulses_batch(
            model_ref,
            q_batch,
            qd_minus_batch,
            batch_size,
            &contacts,
            restitution,
            ws_ref,
            qd_plus_out,
            impulse_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_contact_jacobian_normal(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    jac_out_row_major_kxn: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let jac_out = unsafe { as_mut_slice(jac_out_row_major_kxn, num_contacts * n)? };
        let zero_bias = vec![0.0; num_contacts];
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            zero_bias.as_ptr(),
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        let jac = algo::contact_jacobian_normal(model_ref, q, &contacts, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        jac_out.copy_from_slice(&jac);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_contact_constrained_dynamics_friction(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    tau: *const f64,
    gravity_xyz: *const f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    contact_accel_bias: *const f64,
    contact_friction_coeff: *const f64,
    qdd_out: *mut f64,
    lambda_normal_out: *mut f64,
    lambda_tangent_out_2k: *mut f64,
    force_world_out_3k: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let tau = unsafe { as_slice(tau, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, n)? };
        let ln_out = unsafe { as_mut_slice(lambda_normal_out, num_contacts)? };
        let lt_out = unsafe { as_mut_slice(lambda_tangent_out_2k, num_contacts * 2)? };
        let f_out = unsafe { as_mut_slice(force_world_out_3k, num_contacts * 3)? };
        let contacts = parse_friction_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            contact_accel_bias,
            contact_friction_coeff,
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::constrained_forward_dynamics_contacts_friction(
            model_ref,
            q,
            qd,
            tau,
            &contacts,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        qdd_out.copy_from_slice(&out.qdd);
        ln_out.copy_from_slice(&out.lambda_normal);
        for i in 0..num_contacts {
            lt_out[2 * i] = out.lambda_tangent[i][0];
            lt_out[2 * i + 1] = out.lambda_tangent[i][1];
            f_out[3 * i] = out.contact_forces_world[i].x;
            f_out[3 * i + 1] = out.contact_forces_world[i].y;
            f_out[3 * i + 2] = out.contact_forces_world[i].z;
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_contact_constrained_dynamics_friction_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    tau_batch: *const f64,
    batch_size: usize,
    gravity_xyz: *const f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    contact_accel_bias: *const f64,
    contact_friction_coeff: *const f64,
    qdd_out: *mut f64,
    lambda_normal_out: *mut f64,
    lambda_tangent_out_2k: *mut f64,
    force_world_out_3k: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_k = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let tau_batch = unsafe { as_slice(tau_batch, total)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, total)? };
        let ln_out = unsafe { as_mut_slice(lambda_normal_out, total_k)? };
        let lt_out = unsafe { as_mut_slice(lambda_tangent_out_2k, total_k * 2)? };
        let f_out = unsafe { as_mut_slice(force_world_out_3k, total_k * 3)? };
        let contacts = parse_friction_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            contact_accel_bias,
            contact_friction_coeff,
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::constrained_forward_dynamics_contacts_friction_batch(
            model_ref,
            q_batch,
            qd_batch,
            tau_batch,
            batch_size,
            &contacts,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            qdd_out,
            ln_out,
            lt_out,
            f_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_apply_contact_impulse_friction(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd_minus: *const f64,
    restitution: f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    contact_friction_coeff: *const f64,
    qd_plus_out: *mut f64,
    impulse_normal_out: *mut f64,
    impulse_tangent_out_2k: *mut f64,
    impulse_world_out_3k: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd_minus = unsafe { as_slice(qd_minus, n)? };
        let qd_plus_out = unsafe { as_mut_slice(qd_plus_out, n)? };
        let in_out = unsafe { as_mut_slice(impulse_normal_out, num_contacts)? };
        let it_out = unsafe { as_mut_slice(impulse_tangent_out_2k, num_contacts * 2)? };
        let iw_out = unsafe { as_mut_slice(impulse_world_out_3k, num_contacts * 3)? };
        let zero_bias = vec![0.0; num_contacts];
        let contacts = parse_friction_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            zero_bias.as_ptr(),
            contact_friction_coeff,
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        let out = algo::apply_contact_impulses_friction(
            model_ref,
            q,
            qd_minus,
            &contacts,
            restitution,
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        qd_plus_out.copy_from_slice(&out.qd_plus);
        in_out.copy_from_slice(&out.impulse_normal);
        for i in 0..num_contacts {
            it_out[2 * i] = out.impulse_tangent[i][0];
            it_out[2 * i + 1] = out.impulse_tangent[i][1];
            iw_out[3 * i] = out.contact_impulses_world[i].x;
            iw_out[3 * i + 1] = out.contact_impulses_world[i].y;
            iw_out[3 * i + 2] = out.contact_impulses_world[i].z;
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_apply_contact_impulse_friction_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_minus_batch: *const f64,
    batch_size: usize,
    restitution: f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_normals_xyz: *const f64,
    contact_friction_coeff: *const f64,
    qd_plus_out: *mut f64,
    impulse_normal_out: *mut f64,
    impulse_tangent_out_2k: *mut f64,
    impulse_world_out_3k: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_k = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let qd_minus_batch = unsafe { as_slice(qd_minus_batch, total)? };
        let qd_plus_out = unsafe { as_mut_slice(qd_plus_out, total)? };
        let in_out = unsafe { as_mut_slice(impulse_normal_out, total_k)? };
        let it_out = unsafe { as_mut_slice(impulse_tangent_out_2k, total_k * 2)? };
        let iw_out = unsafe { as_mut_slice(impulse_world_out_3k, total_k * 3)? };
        let zero_bias = vec![0.0; num_contacts];
        let contacts = parse_friction_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            contact_normals_xyz,
            zero_bias.as_ptr(),
            contact_friction_coeff,
        )?;
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::apply_contact_impulses_friction_batch(
            model_ref,
            q_batch,
            qd_minus_batch,
            batch_size,
            &contacts,
            restitution,
            ws_ref,
            qd_plus_out,
            in_out,
            it_out,
            iw_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_constrained_aba_locked_joints(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    tau: *const f64,
    locked_mask_i32: *const i32,
    gravity_xyz: *const f64,
    qdd_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let tau = unsafe { as_slice(tau, n)? };
        let locked_mask = unsafe { as_slice(locked_mask_i32, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, n)? };

        let locked: Vec<bool> = locked_mask.iter().map(|v| *v != 0).collect();
        let ws_ref = unsafe { &mut (*ws).ws };
        let qdd = algo::constrained_aba_locked_joints(
            model_ref,
            q,
            qd,
            tau,
            &locked,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        qdd_out.copy_from_slice(&qdd);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_constrained_aba_locked_joints_batch(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    qd_batch: *const f64,
    tau_batch: *const f64,
    batch_size: usize,
    locked_mask_i32: *const i32,
    gravity_xyz: *const f64,
    qdd_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        check_non_null(gravity_xyz)?;

        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let total_q = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total_q)? };
        let qd_batch = unsafe { as_slice(qd_batch, total)? };
        let tau_batch = unsafe { as_slice(tau_batch, total)? };
        let locked_mask = unsafe { as_slice(locked_mask_i32, n)? };
        let g = unsafe { as_slice(gravity_xyz, 3)? };
        let qdd_out = unsafe { as_mut_slice(qdd_out, total)? };

        let locked: Vec<bool> = locked_mask.iter().map(|v| *v != 0).collect();
        let ws_ref = unsafe { &mut (*ws).ws };
        algo::constrained_aba_locked_joints_batch(
            model_ref,
            q_batch,
            qd_batch,
            tau_batch,
            batch_size,
            &locked,
            Vec3::new(g[0], g[1], g[2]),
            ws_ref,
            qdd_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}
