use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};
use super::kinematics::forward_kinematics;
use super::dynamics::{aba, crba, cholesky_solve};
use super::jacobian::is_ancestor;

#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    pub link_index: usize,
    pub point_local: Vec3,
    pub normal_world: Vec3,
    pub acceleration_bias: f64,
}

#[derive(Debug, Clone)]
pub struct ContactSolveResult {
    pub qdd: Vec<f64>,
    pub lambda_normal: Vec<f64>,
    pub contact_forces_world: Vec<Vec3>,
}

#[derive(Debug, Clone)]
pub struct ContactImpulseResult {
    pub qd_plus: Vec<f64>,
    pub impulse_normal: Vec<f64>,
    pub contact_impulses_world: Vec<Vec3>,
}

pub(crate) fn contact_jacobian_row(
    model: &Model,
    ws: &Workspace,
    contact: ContactPoint,
) -> Result<Vec<f64>> {
    if contact.link_index >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: contact.link_index,
            len: model.nlinks(),
        });
    }
    let n = model.nv();
    let normal_norm = contact.normal_world.norm();
    if normal_norm <= 1e-12 {
        return Err(PinocchioError::InvalidModel(
            "contact normal must be non-zero",
        ));
    }
    let normal = contact.normal_world * (1.0 / normal_norm);
    let p_world = ws.world_pose[contact.link_index].transform_point(contact.point_local);
    let mut jrow = vec![0.0; n];

    for (j, cell) in jrow.iter_mut().enumerate().take(n) {
        let link_of_joint = model.joint_link(j).expect("validated model");
        if !is_ancestor(model, link_of_joint, contact.link_index) {
            continue;
        }
        let axis = ws.world_joint_axis[j];
        let origin = ws.world_joint_origin[j];
        let lin = axis.cross(p_world - origin);
        *cell = normal.dot(lin);
    }
    Ok(jrow)
}

pub(crate) fn solve_m_inv_jt_columns(mass: &[Vec<f64>], jrows: &[Vec<f64>]) -> Result<Vec<Vec<f64>>> {
    let n = mass.len();
    let k = jrows.len();
    let mut cols = vec![vec![0.0; n]; k];
    for c in 0..k {
        cols[c] = cholesky_solve(mass, &jrows[c])?;
    }
    Ok(cols)
}

pub(crate) fn projected_gauss_seidel_nonnegative(a: &[Vec<f64>], rhs: &[f64], iters: usize) -> Vec<f64> {
    let k = rhs.len();
    let mut x = vec![0.0; k];
    for _ in 0..iters {
        for i in 0..k {
            let mut s = rhs[i];
            for (j, xj) in x.iter().copied().enumerate().take(k) {
                if i != j {
                    s -= a[i][j] * xj;
                }
            }
            let diag = a[i][i].max(1e-9);
            x[i] = (s / diag).max(0.0);
        }
    }
    x
}

pub fn constrained_forward_dynamics_contacts(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[ContactPoint],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<ContactSolveResult> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }

    if contacts.is_empty() {
        let qdd = aba(model, q, qd, tau, gravity, ws)?;
        return Ok(ContactSolveResult {
            qdd,
            lambda_normal: Vec::new(),
            contact_forces_world: Vec::new(),
        });
    }

    let qdd_free = aba(model, q, qd, tau, gravity, ws)?;
    let mass = crba(model, q, ws)?;
    let qdd_zero = vec![0.0; n];
    forward_kinematics(model, q, qd, &qdd_zero, gravity, ws)?;

    let mut jrows = Vec::with_capacity(contacts.len());
    let mut normals = Vec::with_capacity(contacts.len());
    let mut rhs = vec![0.0; contacts.len()];

    for (ci, c) in contacts.iter().copied().enumerate() {
        let j = contact_jacobian_row(model, ws, c)?;
        let j_qdd_free = j
            .iter()
            .zip(qdd_free.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        rhs[ci] = -(j_qdd_free + c.acceleration_bias);
        let nrm = c.normal_world.norm();
        normals.push(c.normal_world * (1.0 / nrm.max(1e-12)));
        jrows.push(j);
    }

    let minv_jt = solve_m_inv_jt_columns(&mass, &jrows)?;
    let k = contacts.len();
    let mut a = vec![vec![0.0; k]; k];
    for i in 0..k {
        for (j, col) in minv_jt.iter().enumerate().take(k) {
            a[i][j] = jrows[i]
                .iter()
                .zip(col.iter())
                .map(|(x, y)| x * y)
                .sum::<f64>();
        }
        a[i][i] += 1e-8;
    }

    let lambda = projected_gauss_seidel_nonnegative(&a, &rhs, 40);

    let mut qdd = qdd_free.clone();
    for (c, col) in minv_jt.iter().enumerate().take(k) {
        let gain = lambda[c];
        for i in 0..n {
            qdd[i] += col[i] * gain;
        }
    }

    let mut f_world = Vec::with_capacity(k);
    for i in 0..k {
        f_world.push(normals[i] * lambda[i]);
    }

    Ok(ContactSolveResult {
        qdd,
        lambda_normal: lambda,
        contact_forces_world: f_world,
    })
}

pub fn apply_contact_impulses(
    model: &Model,
    q: &[f64],
    qd_minus: &[f64],
    contacts: &[ContactPoint],
    restitution: f64,
    ws: &mut Workspace,
) -> Result<ContactImpulseResult> {
    let n = model.nv();
    model.check_state_dims(q, qd_minus, None)?;
    if restitution < 0.0 {
        return Err(PinocchioError::InvalidModel("restitution must be >= 0"));
    }
    if contacts.is_empty() {
        return Ok(ContactImpulseResult {
            qd_plus: qd_minus.to_vec(),
            impulse_normal: Vec::new(),
            contact_impulses_world: Vec::new(),
        });
    }

    let qd0 = vec![0.0; n];
    let qdd0 = vec![0.0; n];
    forward_kinematics(model, q, &qd0, &qdd0, Vec3::zero(), ws)?;
    let mass = crba(model, q, ws)?;
    let mut jrows = Vec::with_capacity(contacts.len());
    let mut normals = Vec::with_capacity(contacts.len());
    let mut rhs = vec![0.0; contacts.len()];

    for (ci, c) in contacts.iter().copied().enumerate() {
        let j = contact_jacobian_row(model, ws, c)?;
        let vn = j
            .iter()
            .zip(qd_minus.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        rhs[ci] = -(1.0 + restitution) * vn;
        let nrm = c.normal_world.norm();
        normals.push(c.normal_world * (1.0 / nrm.max(1e-12)));
        jrows.push(j);
    }

    let minv_jt = solve_m_inv_jt_columns(&mass, &jrows)?;
    let k = contacts.len();
    let mut a = vec![vec![0.0; k]; k];
    for i in 0..k {
        for (j, col) in minv_jt.iter().enumerate().take(k) {
            a[i][j] = jrows[i]
                .iter()
                .zip(col.iter())
                .map(|(x, y)| x * y)
                .sum::<f64>();
        }
        a[i][i] += 1e-8;
    }
    let impulse = projected_gauss_seidel_nonnegative(&a, &rhs, 40);

    let mut qd_plus = qd_minus.to_vec();
    for (c, col) in minv_jt.iter().enumerate().take(k) {
        let gain = impulse[c];
        for i in 0..n {
            qd_plus[i] += col[i] * gain;
        }
    }

    let mut p_world = Vec::with_capacity(k);
    for i in 0..k {
        p_world.push(normals[i] * impulse[i]);
    }

    Ok(ContactImpulseResult {
        qd_plus,
        impulse_normal: impulse,
        contact_impulses_world: p_world,
    })
}
