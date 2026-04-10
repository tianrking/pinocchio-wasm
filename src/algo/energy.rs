use crate::core::error::Result;
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::kinematics::forward_kinematics;

pub fn kinetic_energy(model: &Model, q: &[f64], qd: &[f64], ws: &mut Workspace) -> Result<f64> {
    let n = model.nv();
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, qd, &qdd, Vec3::zero(), ws)?;

    let mut e = 0.0;
    for link_idx in 0..model.nlinks() {
        let link = &model.links[link_idx];
        let pose = ws.world_pose[link_idx];
        let r_com = pose.rotation.mul_vec(link.com_local);
        let v_com = ws.vel_origin[link_idx] + ws.omega[link_idx].cross(r_com);
        let i_world = pose
            .rotation
            .mul_mat(link.inertia_local_com)
            .mul_mat(pose.rotation.transpose());
        let rotational = ws.omega[link_idx].dot(i_world.mul_vec(ws.omega[link_idx]));
        let translational = link.mass * v_com.norm2();
        e += 0.5 * (rotational + translational);
    }
    Ok(e)
}

pub fn potential_energy(
    model: &Model,
    q: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<f64> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let mut e = 0.0;
    for link_idx in 0..model.nlinks() {
        let link = &model.links[link_idx];
        let com_world = ws.world_pose[link_idx].transform_point(link.com_local);
        e += -link.mass * gravity.dot(com_world);
    }
    Ok(e)
}
