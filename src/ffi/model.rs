#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    CollisionHandle, ModelHandle, Status, WorkspaceHandle, as_slice, as_str, check_non_null,
    run_status,
};
use crate::collision::{self, CollisionModel, Geometry, Sphere};
use crate::core::math::{Mat3, Vec3};
use crate::model::{Joint, JointType, Link, Model, Workspace};
use core::ptr;

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
pub extern "C" fn pino_model_nv(model: *const ModelHandle) -> usize {
    if model.is_null() {
        return 0;
    }
    unsafe { (*model).model.nv() }
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
    joint_types_i32: *const i32,
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
        let jtypes = unsafe { as_slice(joint_types_i32, nlinks)? };

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
            let jtype = match jtypes[i] {
                0 => JointType::Revolute,
                1 => JointType::Prismatic,
                2 => JointType::Fixed,
                3 => JointType::Spherical,
                4 => JointType::FreeFlyer,
                _ => return Err(Status::InvalidInput),
            };
            let joint = match jtype {
                JointType::Revolute => Joint::revolute(axis, origin),
                JointType::Prismatic => Joint::prismatic(axis, origin),
                JointType::Fixed => Joint::fixed(origin),
                JointType::Spherical => Joint::spherical(origin),
                JointType::FreeFlyer => Joint::freeflyer(origin),
            };
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
            CollisionModel::from_spheres(spheres).map_err(|_| Status::BuildModelFailed)?;
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
        let ptr_u8 = super::pino_alloc(len);
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
        let ptr_u8 = super::pino_alloc(len);
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
