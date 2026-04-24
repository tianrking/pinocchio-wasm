#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null,
    parse_contact_forces_world, parse_contacts, run_status,
};
use crate::algo;

#[unsafe(no_mangle)]
pub extern "C" fn pino_centroidal_map(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    ag_out_row_major_6xn: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let ag_out = unsafe { as_mut_slice(ag_out_row_major_6xn, 6 * n)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let ag = algo::centroidal_map(model_ref, q, ws_ref).map_err(|_| Status::AlgoFailed)?;
        ag_out.copy_from_slice(&ag);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_centroidal_map_derivatives(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    dag_dq_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let dag_out = unsafe { as_mut_slice(dag_dq_out, 6 * n * n)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let dag = algo::centroidal_map_derivatives(model_ref, q, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        dag_out.copy_from_slice(&dag);
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_centroidal_momentum(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    momentum_out_6: *mut f64,
    com_out_xyz: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let m_out = unsafe { as_mut_slice(momentum_out_6, 6)? };
        let com_out = unsafe { as_mut_slice(com_out_xyz, 3)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let h =
            algo::centroidal_momentum(model_ref, q, qd, ws_ref).map_err(|_| Status::AlgoFailed)?;
        m_out[0] = h.linear.x;
        m_out[1] = h.linear.y;
        m_out[2] = h.linear.z;
        m_out[3] = h.angular.x;
        m_out[4] = h.angular.y;
        m_out[5] = h.angular.z;
        com_out[0] = h.com.x;
        com_out[1] = h.com.y;
        com_out[2] = h.com.z;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_centroidal_momentum_rate(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    hdot_out_6: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let out = unsafe { as_mut_slice(hdot_out_6, 6)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let hdot = algo::centroidal_momentum_rate(model_ref, q, qd, qdd, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        out[0] = hdot.linear.x;
        out[1] = hdot.linear.y;
        out[2] = hdot.linear.z;
        out[3] = hdot.angular.x;
        out[4] = hdot.angular.y;
        out[5] = hdot.angular.z;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_centroidal_full_terms(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    ag_out_6xn: *mut f64,
    dag_dq_out: *mut f64,
    momentum_out_6: *mut f64,
    hdot_out_6: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let ag_out = unsafe { as_mut_slice(ag_out_6xn, 6 * n)? };
        let dag_out = unsafe { as_mut_slice(dag_dq_out, 6 * n * n)? };
        let m_out = unsafe { as_mut_slice(momentum_out_6, 6)? };
        let hdot_out = unsafe { as_mut_slice(hdot_out_6, 6)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let full = algo::centroidal_full_terms(model_ref, q, qd, qdd, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        ag_out.copy_from_slice(&full.ag);
        dag_out.copy_from_slice(&full.dag_dq);
        m_out[0] = full.momentum.linear.x;
        m_out[1] = full.momentum.linear.y;
        m_out[2] = full.momentum.linear.z;
        m_out[3] = full.momentum.angular.x;
        m_out[4] = full.momentum.angular.y;
        m_out[5] = full.momentum.angular.z;
        hdot_out[0] = full.momentum_rate.linear.x;
        hdot_out[1] = full.momentum_rate.linear.y;
        hdot_out[2] = full.momentum_rate.linear.z;
        hdot_out[3] = full.momentum_rate.angular.x;
        hdot_out[4] = full.momentum_rate.angular.y;
        hdot_out[5] = full.momentum_rate.angular.z;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_centroidal_full_terms_with_contacts(
    model: *const ModelHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    num_contacts: usize,
    contact_link_indices_i32: *const i32,
    contact_points_xyz: *const f64,
    contact_forces_world_xyz: *const f64,
    ag_out_6xn: *mut f64,
    dag_dq_out: *mut f64,
    momentum_out_6: *mut f64,
    hdot_out_6: *mut f64,
    contact_wrench_out_6: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        let model_ref = unsafe { &(*model).model };
        let nq = model_ref.nq();
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, nq)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let ag_out = unsafe { as_mut_slice(ag_out_6xn, 6 * n)? };
        let dag_out = unsafe { as_mut_slice(dag_dq_out, 6 * n * n)? };
        let m_out = unsafe { as_mut_slice(momentum_out_6, 6)? };
        let hdot_out = unsafe { as_mut_slice(hdot_out_6, 6)? };
        let cw_out = unsafe { as_mut_slice(contact_wrench_out_6, 6)? };
        let zero_bias = vec![0.0; num_contacts];
        let unit_normals = vec![0.0_f64; num_contacts * 3];
        let contacts = parse_contacts(
            num_contacts,
            contact_link_indices_i32,
            contact_points_xyz,
            unit_normals.as_ptr(),
            zero_bias.as_ptr(),
        )?;
        let forces = parse_contact_forces_world(num_contacts, contact_forces_world_xyz)?;
        let ws_ref = unsafe { &mut (*ws).ws };
        let (full, wrench) = algo::centroidal_full_terms_with_contacts(
            model_ref, q, qd, qdd, &contacts, &forces, ws_ref,
        )
        .map_err(|_| Status::AlgoFailed)?;
        ag_out.copy_from_slice(&full.ag);
        dag_out.copy_from_slice(&full.dag_dq);
        m_out[0] = full.momentum.linear.x;
        m_out[1] = full.momentum.linear.y;
        m_out[2] = full.momentum.linear.z;
        m_out[3] = full.momentum.angular.x;
        m_out[4] = full.momentum.angular.y;
        m_out[5] = full.momentum.angular.z;
        hdot_out[0] = full.momentum_rate.linear.x;
        hdot_out[1] = full.momentum_rate.linear.y;
        hdot_out[2] = full.momentum_rate.linear.z;
        hdot_out[3] = full.momentum_rate.angular.x;
        hdot_out[4] = full.momentum_rate.angular.y;
        hdot_out[5] = full.momentum_rate.angular.z;
        cw_out[0] = wrench.linear.x;
        cw_out[1] = wrench.linear.y;
        cw_out[2] = wrench.linear.z;
        cw_out[3] = wrench.angular.x;
        cw_out[4] = wrench.angular.y;
        cw_out[5] = wrench.angular.z;
        Ok(())
    }) as i32
}
