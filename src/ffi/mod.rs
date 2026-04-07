#![allow(clippy::not_unsafe_ptr_arg_deref)]

use crate::algo;
use crate::collision::{self, CollisionModel, Sphere};
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

pub struct CollisionHandle {
    pub collision: CollisionModel,
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
pub extern "C" fn pino_alloc(size: usize) -> *mut u8 {
    if size == 0 {
        return ptr::null_mut();
    }
    let mut buf = Vec::<u8>::with_capacity(size);
    let p = buf.as_mut_ptr();
    std::mem::forget(buf);
    p
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_dealloc(ptr_u8: *mut u8, size: usize) {
    if ptr_u8.is_null() || size == 0 {
        return;
    }
    unsafe {
        drop(Vec::from_raw_parts(ptr_u8, 0, size));
    }
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
pub extern "C" fn pino_collision_model_free(collision: *mut CollisionHandle) {
    if collision.is_null() {
        return;
    }
    unsafe { drop(Box::from_raw(collision)) };
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
pub extern "C" fn pino_collision_model_create(
    num_spheres: usize,
    link_indices: *const i32,
    centers_xyz: *const f64,
    radii: *const f64,
) -> *mut CollisionHandle {
    if num_spheres == 0 {
        return ptr::null_mut();
    }

    let build = || -> Result<CollisionHandle, Status> {
        let link_indices = unsafe { as_slice(link_indices, num_spheres)? };
        let centers_xyz = unsafe { as_slice(centers_xyz, 3 * num_spheres)? };
        let radii = unsafe { as_slice(radii, num_spheres)? };

        let mut spheres = Vec::with_capacity(num_spheres);
        for i in 0..num_spheres {
            let link_index = usize::try_from(link_indices[i]).map_err(|_| Status::InvalidInput)?;
            spheres.push(Sphere {
                link_index,
                center_local: Vec3::new(
                    centers_xyz[3 * i],
                    centers_xyz[3 * i + 1],
                    centers_xyz[3 * i + 2],
                ),
                radius: radii[i],
            });
        }
        let collision =
            CollisionModel::with_all_pairs(spheres).map_err(|_| Status::BuildModelFailed)?;
        Ok(CollisionHandle { collision })
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
pub extern "C" fn pino_model_create_from_mjcf(
    mjcf_ptr: *const u8,
    mjcf_len: usize,
) -> *mut ModelHandle {
    let build = || -> Result<ModelHandle, Status> {
        let mjcf = unsafe { as_str(mjcf_ptr, mjcf_len)? };
        let model = Model::from_mjcf_str(mjcf).map_err(|_| Status::BuildModelFailed)?;
        Ok(ModelHandle { model })
    };

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(build)) {
        Ok(Ok(handle)) => Box::into_raw(Box::new(handle)),
        _ => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_to_json(
    model: *const ModelHandle,
    out_json_ptr: *mut *mut u8,
    out_json_len: *mut usize,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        if out_json_ptr.is_null() || out_json_len.is_null() {
            return Err(Status::NullPtr);
        }
        let model_ref = unsafe { &(*model).model };
        let json = model_ref.to_json_string().map_err(|_| Status::AlgoFailed)?;
        let bytes = json.into_bytes();
        let len = bytes.len();
        let ptr_u8 = pino_alloc(len);
        if ptr_u8.is_null() {
            return Err(Status::InvalidInput);
        }
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), ptr_u8, len);
            *out_json_ptr = ptr_u8;
            *out_json_len = len;
        }
        Ok(())
    }) as i32
}

fn export_model_text(
    model: *const ModelHandle,
    out_ptr: *mut *mut u8,
    out_len: *mut usize,
    producer: impl FnOnce(&Model) -> Result<String, Status>,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        if out_ptr.is_null() || out_len.is_null() {
            return Err(Status::NullPtr);
        }
        let model_ref = unsafe { &(*model).model };
        let txt = producer(model_ref)?;
        let bytes = txt.into_bytes();
        let len = bytes.len();
        let ptr_u8 = pino_alloc(len);
        if ptr_u8.is_null() {
            return Err(Status::InvalidInput);
        }
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), ptr_u8, len);
            *out_ptr = ptr_u8;
            *out_len = len;
        }
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_to_urdf(
    model: *const ModelHandle,
    robot_name_ptr: *const u8,
    robot_name_len: usize,
    out_urdf_ptr: *mut *mut u8,
    out_urdf_len: *mut usize,
) -> i32 {
    let name = unsafe { as_str(robot_name_ptr, robot_name_len) };
    if name.is_err() {
        return Status::InvalidInput as i32;
    }
    let name = name.unwrap_or("pinocchio_wasm");
    export_model_text(model, out_urdf_ptr, out_urdf_len, |m| {
        m.to_urdf_string(name).map_err(|_| Status::AlgoFailed)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_to_sdf(
    model: *const ModelHandle,
    model_name_ptr: *const u8,
    model_name_len: usize,
    out_sdf_ptr: *mut *mut u8,
    out_sdf_len: *mut usize,
) -> i32 {
    let name = unsafe { as_str(model_name_ptr, model_name_len) };
    if name.is_err() {
        return Status::InvalidInput as i32;
    }
    let name = name.unwrap_or("pinocchio_wasm");
    export_model_text(model, out_sdf_ptr, out_sdf_len, |m| {
        m.to_sdf_string(name).map_err(|_| Status::AlgoFailed)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_model_to_mjcf(
    model: *const ModelHandle,
    model_name_ptr: *const u8,
    model_name_len: usize,
    out_mjcf_ptr: *mut *mut u8,
    out_mjcf_len: *mut usize,
) -> i32 {
    let name = unsafe { as_str(model_name_ptr, model_name_len) };
    if name.is_err() {
        return Status::InvalidInput as i32;
    }
    let name = name.unwrap_or("pinocchio_wasm");
    export_model_text(model, out_mjcf_ptr, out_mjcf_len, |m| {
        m.to_mjcf_string(name).map_err(|_| Status::AlgoFailed)
    })
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
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

#[unsafe(no_mangle)]
pub extern "C" fn pino_collision_min_distance(
    model: *const ModelHandle,
    collision: *const CollisionHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    distance_out: *mut f64,
    pair_out_i32x2: *mut i32,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(collision)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        if distance_out.is_null() || pair_out_i32x2.is_null() {
            return Err(Status::NullPtr);
        }

        let model_ref = unsafe { &(*model).model };
        let coll_ref = unsafe { &(*collision).collision };
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let pair_out = unsafe { as_mut_slice(pair_out_i32x2, 2)? };
        let ws_ref = unsafe { &mut (*ws).ws };

        let res = collision::minimum_distance(model_ref, coll_ref, q, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        unsafe {
            *distance_out = res.distance;
        }
        pair_out[0] = i32::try_from(res.pair.0).map_err(|_| Status::InvalidInput)?;
        pair_out[1] = i32::try_from(res.pair.1).map_err(|_| Status::InvalidInput)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_collision_min_distance_batch(
    model: *const ModelHandle,
    collision: *const CollisionHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    batch_size: usize,
    distances_out: *mut f64,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(collision)?;
        check_non_null(ws as *const WorkspaceHandle)?;

        let model_ref = unsafe { &(*model).model };
        let coll_ref = unsafe { &(*collision).collision };
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
        let distances_out = unsafe { as_mut_slice(distances_out, batch_size)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        collision::minimum_distance_batch(
            model_ref,
            coll_ref,
            q_batch,
            batch_size,
            ws_ref,
            distances_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

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
        let n = model_ref.nv();
        let nl = model_ref.nlinks();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let nl = model_ref.nlinks();
        let total_q = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
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
