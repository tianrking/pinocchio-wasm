#![allow(clippy::not_unsafe_ptr_arg_deref)]

pub mod batch;
pub mod centroidal;
pub mod collision;
pub mod contact;
pub mod derivatives;
pub mod dynamics;
pub mod kinematics;
pub mod model;
pub mod regressors;

use crate::algo;
use crate::collision::CollisionModel;
use crate::core::math::Vec3;
use crate::model::Model;
use core::ptr;
use core::str;

// Re-export all pub extern "C" functions from sub-modules so they remain
// available as crate-level symbols (the wasm linker picks them up regardless
// because they are #[unsafe(no_mangle)], but this also keeps any
// crate-internal references working).
pub use batch::*;
pub use centroidal::*;
pub use collision::*;
pub use contact::*;
pub use derivatives::*;
pub use dynamics::*;
pub use kinematics::*;
pub use model::*;
pub use regressors::*;

// ---------------------------------------------------------------------------
// Shared types
// ---------------------------------------------------------------------------

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
    pub ws: crate::model::Workspace,
}

pub struct CollisionHandle {
    pub collision: CollisionModel,
}

// ---------------------------------------------------------------------------
// Shared helper functions
// ---------------------------------------------------------------------------

pub(super) fn check_non_null<T>(p: *const T) -> Result<(), Status> {
    if p.is_null() {
        Err(Status::NullPtr)
    } else {
        Ok(())
    }
}

pub(super) unsafe fn as_slice<'a, T>(ptr: *const T, len: usize) -> Result<&'a [T], Status> {
    check_non_null(ptr)?;
    Ok(unsafe { core::slice::from_raw_parts(ptr, len) })
}

pub(super) unsafe fn as_mut_slice<'a, T>(ptr: *mut T, len: usize) -> Result<&'a mut [T], Status> {
    if ptr.is_null() {
        return Err(Status::NullPtr);
    }
    Ok(unsafe { core::slice::from_raw_parts_mut(ptr, len) })
}

pub(super) unsafe fn as_str<'a>(ptr: *const u8, len: usize) -> Result<&'a str, Status> {
    let bytes = unsafe { as_slice(ptr, len)? };
    str::from_utf8(bytes).map_err(|_| Status::InvalidInput)
}

pub(super) fn run_status<F>(f: F) -> Status
where
    F: FnOnce() -> Result<(), Status>,
{
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(Ok(())) => Status::Ok,
        Ok(Err(s)) => s,
        Err(_) => Status::Panic,
    }
}

// ---------------------------------------------------------------------------
// Contact-parsing helpers (used by contact, centroidal, derivatives modules)
// ---------------------------------------------------------------------------

pub(super) fn parse_contacts(
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

pub(super) fn parse_friction_contacts(
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

pub(super) fn parse_contact_forces_world(
    num_contacts: usize,
    forces_xyz: *const f64,
) -> Result<Vec<Vec3>, Status> {
    let f = unsafe { as_slice(forces_xyz, 3 * num_contacts)? };
    let mut out = Vec::with_capacity(num_contacts);
    for i in 0..num_contacts {
        out.push(Vec3::new(f[3 * i], f[3 * i + 1], f[3 * i + 2]));
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Memory allocation helpers (used by model export functions)
// ---------------------------------------------------------------------------

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
pub extern "C" fn pino_status_ok() -> i32 {
    Status::Ok as i32
}
