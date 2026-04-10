#![allow(clippy::not_unsafe_ptr_arg_deref)]

use crate::algo;
use crate::collision::{self, CollisionModel, Geometry, Sphere};
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
        let collision = CollisionModel::from_spheres(spheres).map_err(|_| Status::BuildModelFailed)?;
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

fn parse_contacts(
    num_contacts: usize,
    link_indices_i32: *const i32,
    points_xyz: *const f64,
    normals_xyz: *const f64,
    accel_bias: *const f64,
) -> Result<Vec<algo::ContactPoint>, Status> {
    let links = unsafe { as_slice(link_indices_i32, num_contacts)? };
    let points = unsafe { as_slice(points_xyz, 3 * num_contacts)? };
    let normals = unsafe { as_slice(normals_xyz, 3 * num_contacts)? };
    let bias = unsafe { as_slice(accel_bias, num_contacts)? };
    let mut out = Vec::with_capacity(num_contacts);
    for i in 0..num_contacts {
        out.push(algo::ContactPoint {
            link_index: usize::try_from(links[i]).map_err(|_| Status::InvalidInput)?,
            point_local: Vec3::new(points[3 * i], points[3 * i + 1], points[3 * i + 2]),
            normal_world: Vec3::new(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]),
            acceleration_bias: bias[i],
        });
    }
    Ok(out)
}

fn parse_friction_contacts(
    num_contacts: usize,
    link_indices_i32: *const i32,
    points_xyz: *const f64,
    normals_xyz: *const f64,
    accel_bias: *const f64,
    friction_coeff: *const f64,
) -> Result<Vec<algo::FrictionContactPoint>, Status> {
    let links = unsafe { as_slice(link_indices_i32, num_contacts)? };
    let points = unsafe { as_slice(points_xyz, 3 * num_contacts)? };
    let normals = unsafe { as_slice(normals_xyz, 3 * num_contacts)? };
    let bias = unsafe { as_slice(accel_bias, num_contacts)? };
    let mu = unsafe { as_slice(friction_coeff, num_contacts)? };
    let mut out = Vec::with_capacity(num_contacts);
    for i in 0..num_contacts {
        out.push(algo::FrictionContactPoint {
            link_index: usize::try_from(links[i]).map_err(|_| Status::InvalidInput)?,
            point_local: Vec3::new(points[3 * i], points[3 * i + 1], points[3 * i + 2]),
            normal_world: Vec3::new(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]),
            acceleration_bias: bias[i],
            friction_coeff: mu[i],
        });
    }
    Ok(out)
}

fn parse_contact_forces_world(num_contacts: usize, forces_xyz: *const f64) -> Result<Vec<Vec3>, Status> {
    let f = unsafe { as_slice(forces_xyz, 3 * num_contacts)? };
    let mut out = Vec::with_capacity(num_contacts);
    for i in 0..num_contacts {
        out.push(Vec3::new(f[3 * i], f[3 * i + 1], f[3 * i + 2]));
    }
    Ok(out)
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_lambda = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let out = algo::apply_contact_impulses(
            model_ref,
            q,
            qd_minus,
            &contacts,
            restitution,
            ws_ref,
        )
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
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_impulse = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_k = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let total = batch_size.checked_mul(n).ok_or(Status::InvalidInput)?;
        let total_k = batch_size
            .checked_mul(num_contacts)
            .ok_or(Status::InvalidInput)?;
        let q_batch = unsafe { as_slice(q_batch, total)? };
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

const GEO_SPHERE: i32 = 0;
const GEO_BOX: i32 = 1;
const GEO_CAPSULE: i32 = 2;
const GEO_CYLINDER: i32 = 3;
const GEO_MESH_APPROX: i32 = 4;

#[unsafe(no_mangle)]
pub extern "C" fn pino_collision_model_create_geometries(
    num_geometries: usize,
    geom_types_i32: *const i32,
    link_indices_i32: *const i32,
    centers_xyz: *const f64,
    params_xyz: *const f64,
    pair_filter_flags_i32x2: *const i32,
) -> *mut CollisionHandle {
    if num_geometries == 0 {
        return ptr::null_mut();
    }

    let build = || -> Result<CollisionHandle, Status> {
        let geom_types = unsafe { as_slice(geom_types_i32, num_geometries)? };
        let link_indices = unsafe { as_slice(link_indices_i32, num_geometries)? };
        let centers_xyz = unsafe { as_slice(centers_xyz, 3 * num_geometries)? };
        let params_xyz = unsafe { as_slice(params_xyz, 3 * num_geometries)? };
        let flags = unsafe { as_slice(pair_filter_flags_i32x2, 2)? };

        let mut geometries = Vec::with_capacity(num_geometries);
        for i in 0..num_geometries {
            let link_index = usize::try_from(link_indices[i]).map_err(|_| Status::InvalidInput)?;
            let center = Vec3::new(
                centers_xyz[3 * i],
                centers_xyz[3 * i + 1],
                centers_xyz[3 * i + 2],
            );
            let p0 = params_xyz[3 * i];
            let p1 = params_xyz[3 * i + 1];
            let p2 = params_xyz[3 * i + 2];

            let geom = match geom_types[i] {
                GEO_SPHERE => Geometry::Sphere {
                    link_index,
                    center_local: center,
                    radius: p0,
                },
                GEO_BOX => Geometry::Box {
                    link_index,
                    center_local: center,
                    half_extents: Vec3::new(p0, p1, p2),
                },
                GEO_CAPSULE => Geometry::Capsule {
                    link_index,
                    center_local: center,
                    half_length: p0,
                    radius: p1,
                },
                GEO_CYLINDER => Geometry::Cylinder {
                    link_index,
                    center_local: center,
                    half_length: p0,
                    radius: p1,
                },
                GEO_MESH_APPROX => Geometry::MeshApprox {
                    link_index,
                    center_local: center,
                    half_extents: Vec3::new(p0, p1, p2),
                },
                _ => return Err(Status::InvalidInput),
            };
            geometries.push(geom);
        }

        let filter = collision::PairFilter {
            ignore_same_link: flags[0] != 0,
            ignore_parent_child: flags[1] != 0,
        };
        let mut pairs = Vec::new();
        for i in 0..num_geometries {
            for j in (i + 1)..num_geometries {
                pairs.push((i, j));
            }
        }
        let collision = CollisionModel::new_with_filter(geometries, pairs, filter)
            .map_err(|_| Status::BuildModelFailed)?;
        Ok(CollisionHandle { collision })
    };

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(build)) {
        Ok(Ok(handle)) => Box::into_raw(Box::new(handle)),
        _ => ptr::null_mut(),
    }
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
pub extern "C" fn pino_collision_min_distance_detailed(
    model: *const ModelHandle,
    collision: *const CollisionHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    distance_out: *mut f64,
    pair_out_i32x2: *mut i32,
    normal_out_xyz: *mut f64,
    point_a_out_xyz: *mut f64,
    point_b_out_xyz: *mut f64,
    penetration_out: *mut f64,
    is_colliding_out: *mut i32,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(collision)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        if distance_out.is_null()
            || pair_out_i32x2.is_null()
            || normal_out_xyz.is_null()
            || point_a_out_xyz.is_null()
            || point_b_out_xyz.is_null()
            || penetration_out.is_null()
            || is_colliding_out.is_null()
        {
            return Err(Status::NullPtr);
        }

        let model_ref = unsafe { &(*model).model };
        let coll_ref = unsafe { &(*collision).collision };
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let pair_out = unsafe { as_mut_slice(pair_out_i32x2, 2)? };
        let normal_out = unsafe { as_mut_slice(normal_out_xyz, 3)? };
        let point_a_out = unsafe { as_mut_slice(point_a_out_xyz, 3)? };
        let point_b_out = unsafe { as_mut_slice(point_b_out_xyz, 3)? };
        let ws_ref = unsafe { &mut (*ws).ws };

        let res = collision::minimum_distance_detailed(model_ref, coll_ref, q, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
        unsafe {
            *distance_out = res.distance;
            *penetration_out = res.penetration_depth;
            *is_colliding_out = if res.is_colliding { 1 } else { 0 };
        }
        pair_out[0] = i32::try_from(res.pair.0).map_err(|_| Status::InvalidInput)?;
        pair_out[1] = i32::try_from(res.pair.1).map_err(|_| Status::InvalidInput)?;
        normal_out[0] = res.normal_world.x;
        normal_out[1] = res.normal_world.y;
        normal_out[2] = res.normal_world.z;
        point_a_out[0] = res.point_a.x;
        point_a_out[1] = res.point_a.y;
        point_a_out[2] = res.point_a.z;
        point_b_out[0] = res.point_b.x;
        point_b_out[1] = res.point_b.y;
        point_b_out[2] = res.point_b.z;
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
pub extern "C" fn pino_collision_min_distance_detailed_batch(
    model: *const ModelHandle,
    collision: *const CollisionHandle,
    ws: *mut WorkspaceHandle,
    q_batch: *const f64,
    batch_size: usize,
    distances_out: *mut f64,
    penetration_out: *mut f64,
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
        let penetration_out = unsafe { as_mut_slice(penetration_out, batch_size)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        collision::minimum_distance_detailed_batch(
            model_ref,
            coll_ref,
            q_batch,
            batch_size,
            ws_ref,
            distances_out,
            penetration_out,
        )
        .map_err(|_| Status::AlgoFailed)?;
        Ok(())
    }) as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn pino_collision_query_details(
    model: *const ModelHandle,
    collision: *const CollisionHandle,
    ws: *mut WorkspaceHandle,
    q: *const f64,
    max_results: usize,
    out_result_count: *mut usize,
    pair_out_i32x2_flat: *mut i32,
    distance_out: *mut f64,
    normal_out_xyz_flat: *mut f64,
    point_a_out_xyz_flat: *mut f64,
    point_b_out_xyz_flat: *mut f64,
    penetration_out: *mut f64,
    is_colliding_out_i32: *mut i32,
) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        check_non_null(collision)?;
        check_non_null(ws as *const WorkspaceHandle)?;
        if out_result_count.is_null() {
            return Err(Status::NullPtr);
        }

        let model_ref = unsafe { &(*model).model };
        let coll_ref = unsafe { &(*collision).collision };
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let pair_out = unsafe { as_mut_slice(pair_out_i32x2_flat, max_results * 2)? };
        let distance_out = unsafe { as_mut_slice(distance_out, max_results)? };
        let normal_out = unsafe { as_mut_slice(normal_out_xyz_flat, max_results * 3)? };
        let pa_out = unsafe { as_mut_slice(point_a_out_xyz_flat, max_results * 3)? };
        let pb_out = unsafe { as_mut_slice(point_b_out_xyz_flat, max_results * 3)? };
        let penetration_out = unsafe { as_mut_slice(penetration_out, max_results)? };
        let colliding_out = unsafe { as_mut_slice(is_colliding_out_i32, max_results)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        let details =
            collision::collision_details(model_ref, coll_ref, q, ws_ref).map_err(|_| Status::AlgoFailed)?;
        let count = details.len().min(max_results);
        for i in 0..count {
            let d = details[i];
            pair_out[2 * i] = i32::try_from(d.pair.0).map_err(|_| Status::InvalidInput)?;
            pair_out[2 * i + 1] = i32::try_from(d.pair.1).map_err(|_| Status::InvalidInput)?;
            distance_out[i] = d.distance;
            normal_out[3 * i] = d.normal_world.x;
            normal_out[3 * i + 1] = d.normal_world.y;
            normal_out[3 * i + 2] = d.normal_world.z;
            pa_out[3 * i] = d.point_a.x;
            pa_out[3 * i + 1] = d.point_a.y;
            pa_out[3 * i + 2] = d.point_a.z;
            pb_out[3 * i] = d.point_b.x;
            pb_out[3 * i + 1] = d.point_b.y;
            pb_out[3 * i + 2] = d.point_b.z;
            penetration_out[i] = d.penetration_depth;
            colliding_out[i] = if d.is_colliding { 1 } else { 0 };
        }
        unsafe {
            *out_result_count = count;
        }
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let m_out = unsafe { as_mut_slice(momentum_out_6, 6)? };
        let com_out = unsafe { as_mut_slice(com_out_xyz, 3)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let h = algo::centroidal_momentum(model_ref, q, qd, ws_ref).map_err(|_| Status::AlgoFailed)?;
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
        let qd = unsafe { as_slice(qd, n)? };
        let qdd = unsafe { as_slice(qdd, n)? };
        let ag_out = unsafe { as_mut_slice(ag_out_6xn, 6 * n)? };
        let dag_out = unsafe { as_mut_slice(dag_dq_out, 6 * n * n)? };
        let m_out = unsafe { as_mut_slice(momentum_out_6, 6)? };
        let hdot_out = unsafe { as_mut_slice(hdot_out_6, 6)? };
        let ws_ref = unsafe { &mut (*ws).ws };
        let full =
            algo::centroidal_full_terms(model_ref, q, qd, qdd, ws_ref).map_err(|_| Status::AlgoFailed)?;
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
        let n = model_ref.nv();
        let q = unsafe { as_slice(q, n)? };
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
            model_ref,
            q,
            qd,
            qdd,
            &contacts,
            &forces,
            ws_ref,
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
