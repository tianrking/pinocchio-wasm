use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::com_energy::center_of_mass;
use super::contact::ContactPoint;
use super::kinematics::forward_kinematics;

#[derive(Debug, Clone, Copy)]
pub struct CentroidalMomentum {
    pub linear: Vec3,
    pub angular: Vec3,
    pub com: Vec3,
    pub total_mass: f64,
}

#[derive(Debug, Clone)]
pub struct CentroidalDerivativesResult {
    pub d_h_dq: Vec<Vec<f64>>,
    pub d_h_dqd: Vec<Vec<f64>>,
}

#[derive(Debug, Clone, Copy)]
pub struct SpatialVec6 {
    pub linear: Vec3,
    pub angular: Vec3,
}

#[derive(Debug, Clone)]
pub struct CentroidalFullResult {
    pub ag: Vec<f64>,
    pub dag_dq: Vec<f64>,
    pub momentum: SpatialVec6,
    pub momentum_rate: SpatialVec6,
}

pub fn centroidal_momentum(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    ws: &mut Workspace,
) -> Result<CentroidalMomentum> {
    let n = model.nv();
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, qd, &qdd, Vec3::zero(), ws)?;
    let com = center_of_mass(model, q, ws)?;

    let mut linear = Vec3::zero();
    let mut angular = Vec3::zero();
    let mut total_mass = 0.0;
    for link_idx in 0..model.nlinks() {
        let link = &model.links[link_idx];
        let pose = ws.world_pose[link_idx];
        let r_com_local = pose.rotation.mul_vec(link.com_local);
        let p_com_world = pose.translation + r_com_local;
        let v_com = ws.vel_origin[link_idx] + ws.omega[link_idx].cross(r_com_local);
        let p = v_com * link.mass;
        linear += p;

        let i_world = pose
            .rotation
            .mul_mat(link.inertia_local_com)
            .mul_mat(pose.rotation.transpose());
        let h_rot = i_world.mul_vec(ws.omega[link_idx]);
        angular += h_rot + (p_com_world - com).cross(p);
        total_mass += link.mass;
    }

    Ok(CentroidalMomentum {
        linear,
        angular,
        com,
        total_mass,
    })
}

pub fn centroidal_map(model: &Model, q: &[f64], ws: &mut Workspace) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, &vec![0.0; n], None)?;
    let mut out = vec![0.0; 6 * n];
    for j in 0..n {
        let mut qd = vec![0.0; n];
        qd[j] = 1.0;
        let h = centroidal_momentum(model, q, &qd, ws)?;
        out[j] = h.linear.x;
        out[n + j] = h.linear.y;
        out[2 * n + j] = h.linear.z;
        out[3 * n + j] = h.angular.x;
        out[4 * n + j] = h.angular.y;
        out[5 * n + j] = h.angular.z;
    }
    Ok(out)
}

pub fn centroidal_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    ws: &mut Workspace,
) -> Result<CentroidalDerivativesResult> {
    let n = model.nv();
    let eps = 1e-6;
    let mut d_h_dq = vec![vec![0.0; n]; 6];
    let mut d_h_dqd = vec![vec![0.0; n]; 6];

    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let hp = centroidal_momentum(model, &qp, qd, ws)?;
        let hm = centroidal_momentum(model, &qm, qd, ws)?;
        d_h_dq[0][i] = (hp.linear.x - hm.linear.x) / (2.0 * eps);
        d_h_dq[1][i] = (hp.linear.y - hm.linear.y) / (2.0 * eps);
        d_h_dq[2][i] = (hp.linear.z - hm.linear.z) / (2.0 * eps);
        d_h_dq[3][i] = (hp.angular.x - hm.angular.x) / (2.0 * eps);
        d_h_dq[4][i] = (hp.angular.y - hm.angular.y) / (2.0 * eps);
        d_h_dq[5][i] = (hp.angular.z - hm.angular.z) / (2.0 * eps);

        let mut qdp = qd.to_vec();
        let mut qdm = qd.to_vec();
        qdp[i] += eps;
        qdm[i] -= eps;
        let hpv = centroidal_momentum(model, q, &qdp, ws)?;
        let hmv = centroidal_momentum(model, q, &qdm, ws)?;
        d_h_dqd[0][i] = (hpv.linear.x - hmv.linear.x) / (2.0 * eps);
        d_h_dqd[1][i] = (hpv.linear.y - hmv.linear.y) / (2.0 * eps);
        d_h_dqd[2][i] = (hpv.linear.z - hmv.linear.z) / (2.0 * eps);
        d_h_dqd[3][i] = (hpv.angular.x - hmv.angular.x) / (2.0 * eps);
        d_h_dqd[4][i] = (hpv.angular.y - hmv.angular.y) / (2.0 * eps);
        d_h_dqd[5][i] = (hpv.angular.z - hmv.angular.z) / (2.0 * eps);
    }

    Ok(CentroidalDerivativesResult { d_h_dq, d_h_dqd })
}

pub fn centroidal_map_derivatives(
    model: &Model,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, &vec![0.0; n], None)?;
    let eps = 1e-6;
    let mut out = vec![0.0; (6 * n) * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let ap = centroidal_map(model, &qp, ws)?;
        let am = centroidal_map(model, &qm, ws)?;
        for r in 0..(6 * n) {
            out[r * n + i] = (ap[r] - am[r]) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn centroidal_momentum_rate(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    ws: &mut Workspace,
) -> Result<SpatialVec6> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let dt = 1e-6;
    let h0 = centroidal_momentum(model, q, qd, ws)?;
    let mut q1 = q.to_vec();
    let mut qd1 = qd.to_vec();
    for i in 0..n {
        q1[i] += qd[i] * dt;
        qd1[i] += qdd[i] * dt;
    }
    let h1 = centroidal_momentum(model, &q1, &qd1, ws)?;
    Ok(SpatialVec6 {
        linear: (h1.linear - h0.linear) * (1.0 / dt),
        angular: (h1.angular - h0.angular) * (1.0 / dt),
    })
}

pub fn centroidal_contact_wrench(
    model: &Model,
    q: &[f64],
    contacts: &[ContactPoint],
    contact_forces_world: &[Vec3],
    ws: &mut Workspace,
) -> Result<SpatialVec6> {
    if contacts.len() != contact_forces_world.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: contacts.len(),
            got: contact_forces_world.len(),
        });
    }
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;
    let com = center_of_mass(model, q, ws)?;

    let mut f = Vec3::zero();
    let mut n_m = Vec3::zero();
    for (c, fw) in contacts.iter().zip(contact_forces_world.iter()) {
        if c.link_index >= model.nlinks() {
            return Err(PinocchioError::IndexOutOfBounds {
                index: c.link_index,
                len: model.nlinks(),
            });
        }
        let p = ws.world_pose[c.link_index].transform_point(c.point_local);
        f += *fw;
        n_m += (p - com).cross(*fw);
    }
    Ok(SpatialVec6 {
        linear: f,
        angular: n_m,
    })
}

pub fn centroidal_full_terms(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    ws: &mut Workspace,
) -> Result<CentroidalFullResult> {
    let ag = centroidal_map(model, q, ws)?;
    let dag_dq = centroidal_map_derivatives(model, q, ws)?;
    let h = centroidal_momentum(model, q, qd, ws)?;
    let hdot = centroidal_momentum_rate(model, q, qd, qdd, ws)?;
    Ok(CentroidalFullResult {
        ag,
        dag_dq,
        momentum: SpatialVec6 {
            linear: h.linear,
            angular: h.angular,
        },
        momentum_rate: hdot,
    })
}

pub fn centroidal_full_terms_with_contacts(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    contacts: &[ContactPoint],
    contact_forces_world: &[Vec3],
    ws: &mut Workspace,
) -> Result<(CentroidalFullResult, SpatialVec6)> {
    let full = centroidal_full_terms(model, q, qd, qdd, ws)?;
    let w = centroidal_contact_wrench(model, q, contacts, contact_forces_world, ws)?;
    Ok((full, w))
}
