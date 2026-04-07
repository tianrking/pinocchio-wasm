#![allow(clippy::not_unsafe_ptr_arg_deref)]

use crate::algo;
use crate::core::math::{Mat3, Vec3};
use crate::model::{Joint, Link, Model, Workspace};
use core::ptr;
use core::str;

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Ok = 0,
    NullPtr = -1,
    InvalidInput = -2,
    BuildModelFailed = -3,
    AlgoFailed = -4,
    Panic = -128,
}

pub struct ModelHandle {
    pub model: Model,
}

pub struct WorkspaceHandle {
    pub ws: Workspace,
}

fn check_non_null<T>(p: *const T) -> Result<(), Status> {
    if p.is_null() {
        Err(Status::NullPtr)
    } else {
        Ok(())
    }
}

unsafe fn as_slice<'a, T>(ptr: *const T, len: usize) -> Result<&'a [T], Status> {
    check_non_null(ptr)?;
    Ok(unsafe { core::slice::from_raw_parts(ptr, len) })
}

unsafe fn as_mut_slice<'a, T>(ptr: *mut T, len: usize) -> Result<&'a mut [T], Status> {
    if ptr.is_null() {
        return Err(Status::NullPtr);
    }
    Ok(unsafe { core::slice::from_raw_parts_mut(ptr, len) })
}

unsafe fn as_str<'a>(ptr: *const u8, len: usize) -> Result<&'a str, Status> {
    let bytes = unsafe { as_slice(ptr, len)? };
    str::from_utf8(bytes).map_err(|_| Status::InvalidInput)
}

fn run_status<F>(f: F) -> Status
where
    F: FnOnce() -> Result<(), Status>,
{
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(Ok(())) => Status::Ok,
        Ok(Err(s)) => s,
        Err(_) => Status::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_status_ok() -> i32 {
    Status::Ok as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_free(model: *mut ModelHandle) {
    if model.is_null() {
        return;
    }
    unsafe { drop(Box::from_raw(model)) };
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_workspace_free(ws: *mut WorkspaceHandle) {
    if ws.is_null() {
        return;
    }
    unsafe { drop(Box::from_raw(ws)) };
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_nq(model: *const ModelHandle) -> usize {
    if model.is_null() {
        return 0;
    }
    unsafe { (*model).model.nq() }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_nlinks(model: *const ModelHandle) -> usize {
    if model.is_null() {
        return 0;
    }
    unsafe { (*model).model.nlinks() }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_workspace_new(model: *const ModelHandle) -> *mut WorkspaceHandle {
    if model.is_null() {
        return ptr::null_mut();
    }
    let ws = unsafe { Workspace::new(&(*model).model) };
    Box::into_raw(Box::new(WorkspaceHandle { ws }))
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_create(
    nlinks: usize,
    parent_indices: *const i32,
    joint_axes_xyz: *const f64,
    joint_origins_xyz: *const f64,
    masses: *const f64,
    coms_xyz: *const f64,
    inertias_row_major: *const f64,
) -> *mut ModelHandle {
    if nlinks == 0 {
        return ptr::null_mut();
    }

    let build = || -> Result<ModelHandle, Status> {
        let parents = unsafe { as_slice(parent_indices, nlinks)? };
        let axes = unsafe { as_slice(joint_axes_xyz, 3 * nlinks)? };
        let origins = unsafe { as_slice(joint_origins_xyz, 3 * nlinks)? };
        let masses = unsafe { as_slice(masses, nlinks)? };
        let coms = unsafe { as_slice(coms_xyz, 3 * nlinks)? };
        let inertias = unsafe { as_slice(inertias_row_major, 9 * nlinks)? };

        let mut links = Vec::with_capacity(nlinks);
        for i in 0..nlinks {
            let mass = masses[i];
            let com = Vec3::new(coms[3 * i], coms[3 * i + 1], coms[3 * i + 2]);
            let inertia = Mat3::new([
                [inertias[9 * i], inertias[9 * i + 1], inertias[9 * i + 2]],
                [
                    inertias[9 * i + 3],
                    inertias[9 * i + 4],
                    inertias[9 * i + 5],
                ],
                [
                    inertias[9 * i + 6],
                    inertias[9 * i + 7],
                    inertias[9 * i + 8],
                ],
            ]);

            if i == 0 {
                links.push(Link::root("root", mass, com, inertia));
                continue;
            }

            let parent = usize::try_from(parents[i]).map_err(|_| Status::InvalidInput)?;
            let axis = Vec3::new(axes[3 * i], axes[3 * i + 1], axes[3 * i + 2]);
            let origin = Vec3::new(origins[3 * i], origins[3 * i + 1], origins[3 * i + 2]);
            let joint = Joint::revolute(axis, origin);
            links.push(Link::child(
                format!("link_{i}"),
                parent,
                joint,
                mass,
                com,
                inertia,
            ));
        }

        let model = Model::new(links).map_err(|_| Status::BuildModelFailed)?;
        Ok(ModelHandle { model })
    };

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(build)) {
        Ok(Ok(handle)) => Box::into_raw(Box::new(handle)),
        _ => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_create_from_json(
    json_ptr: *const u8,
    json_len: usize,
) -> *mut ModelHandle {
    let build = || -> Result<ModelHandle, Status> {
        let json = unsafe { as_str(json_ptr, json_len)? };
        let model = Model::from_json_str(json).map_err(|_| Status::BuildModelFailed)?;
        Ok(ModelHandle { model })
    };

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(build)) {
        Ok(Ok(handle)) => Box::into_raw(Box::new(handle)),
        _ => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_create_from_urdf(
    urdf_ptr: *const u8,
    urdf_len: usize,
) -> *mut ModelHandle {
    let build = || -> Result<ModelHandle, Status> {
        let urdf = unsafe { as_str(urdf_ptr, urdf_len)? };
        let model = Model::from_urdf_str(urdf).map_err(|_| Status::BuildModelFailed)?;
        Ok(ModelHandle { model })
    };

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(build)) {
        Ok(Ok(handle)) => Box::into_raw(Box::new(handle)),
        _ => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_create_from_sdf(
    sdf_ptr: *const u8,
    sdf_len: usize,
) -> *mut ModelHandle {
    let build = || -> Result<ModelHandle, Status> {
        let sdf = unsafe { as_str(sdf_ptr, sdf_len)? };
        let model = Model::from_sdf_str(sdf).map_err(|_| Status::BuildModelFailed)?;
        Ok(ModelHandle { model })
    };

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(build)) {
        Ok(Ok(handle)) => Box::into_raw(Box::new(handle)),
        _ => ptr::null_mut(),
    }
}

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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
