#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::{
    CollisionHandle, ModelHandle, Status, WorkspaceHandle, as_mut_slice, as_slice, check_non_null,
    run_status,
};
use crate::collision;

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
        let nq = model_ref.nq();
        let q = unsafe { as_slice(q, nq)? };
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
        let nq = model_ref.nq();
        let q = unsafe { as_slice(q, nq)? };
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
        let nq = model_ref.nq();
        let total = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
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
        let nq = model_ref.nq();
        let total = batch_size.checked_mul(nq).ok_or(Status::InvalidInput)?;
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
        let nq = model_ref.nq();
        let q = unsafe { as_slice(q, nq)? };
        let pair_out = unsafe { as_mut_slice(pair_out_i32x2_flat, max_results * 2)? };
        let distance_out = unsafe { as_mut_slice(distance_out, max_results)? };
        let normal_out = unsafe { as_mut_slice(normal_out_xyz_flat, max_results * 3)? };
        let pa_out = unsafe { as_mut_slice(point_a_out_xyz_flat, max_results * 3)? };
        let pb_out = unsafe { as_mut_slice(point_b_out_xyz_flat, max_results * 3)? };
        let penetration_out = unsafe { as_mut_slice(penetration_out, max_results)? };
        let colliding_out = unsafe { as_mut_slice(is_colliding_out_i32, max_results)? };

        let ws_ref = unsafe { &mut (*ws).ws };
        let details = collision::collision_details(model_ref, coll_ref, q, ws_ref)
            .map_err(|_| Status::AlgoFailed)?;
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
