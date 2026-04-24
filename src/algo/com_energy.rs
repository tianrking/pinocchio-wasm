use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::dynamics::{bias_forces, coriolis_torques, crba, gravity_torques};
use super::kinematics::forward_kinematics;

#[derive(Debug, Clone)]
pub struct AllTermsResult {
    pub mass: Vec<Vec<f64>>,
    pub bias: Vec<f64>,
    pub gravity_torques: Vec<f64>,
    pub coriolis_torques: Vec<f64>,
    pub com: Vec3,
    pub kinetic_energy: f64,
    pub potential_energy: f64,
}

pub fn center_of_mass(model: &Model, q: &[f64], ws: &mut Workspace) -> Result<Vec3> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let mut weighted = Vec3::zero();
    let mut m_total = 0.0;
    for link_idx in 0..model.nlinks() {
        let link = &model.links[link_idx];
        let com_world = ws.world_pose[link_idx].transform_point(link.com_local);
        weighted += com_world * link.mass;
        m_total += link.mass;
    }

    if m_total <= 1e-12 {
        return Err(PinocchioError::InvalidModel("total mass must be > 0"));
    }
    Ok(weighted * (1.0 / m_total))
}

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

pub fn compute_all_terms(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<AllTermsResult> {
    let mass = crba(model, q, ws)?;
    let bias = bias_forces(model, q, qd, gravity, ws)?;
    let gravity_t = gravity_torques(model, q, gravity, ws)?;
    let coriolis_t = coriolis_torques(model, q, qd, ws)?;
    let com = center_of_mass(model, q, ws)?;
    let kinetic = kinetic_energy(model, q, qd, ws)?;
    let potential = potential_energy(model, q, gravity, ws)?;
    Ok(AllTermsResult {
        mass,
        bias,
        gravity_torques: gravity_t,
        coriolis_torques: coriolis_t,
        com,
        kinetic_energy: kinetic,
        potential_energy: potential,
    })
}
