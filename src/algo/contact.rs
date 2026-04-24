use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{JointType, Model, Workspace};

use super::dynamics::{cholesky_solve, crba, is_ancestor};
use super::kinematics::forward_kinematics;
use super::solvers::{projected_gauss_seidel_friction, projected_gauss_seidel_nonnegative};

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

#[derive(Debug, Clone, Copy)]
pub struct FrictionContactPoint {
    pub link_index: usize,
    pub point_local: Vec3,
    pub normal_world: Vec3,
    pub acceleration_bias: f64,
    pub friction_coeff: f64,
}

#[derive(Debug, Clone)]
pub struct FrictionContactSolveResult {
    pub qdd: Vec<f64>,
    pub lambda_normal: Vec<f64>,
    pub lambda_tangent: Vec<[f64; 2]>,
    pub contact_forces_world: Vec<Vec3>,
}

#[derive(Debug, Clone)]
pub struct FrictionContactImpulseResult {
    pub qd_plus: Vec<f64>,
    pub impulse_normal: Vec<f64>,
    pub impulse_tangent: Vec<[f64; 2]>,
    pub contact_impulses_world: Vec<Vec3>,
}

pub(super) fn contact_jacobian_row(
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
        return Err(PinocchioError::invalid_model(
            "contact normal must be non-zero",
        ));
    }
    let normal = contact.normal_world * (1.0 / normal_norm);
    let p_world = ws.world_pose[contact.link_index].transform_point(contact.point_local);
    let mut jrow = vec![0.0; n];

    for j in 0..model.njoints() {
        let link_of_joint = model.joint_link(j).expect("validated model");
        if !is_ancestor(model, link_of_joint, contact.link_index) {
            continue;
        }
        let joint = model.links[link_of_joint]
            .joint
            .as_ref()
            .expect("validated model");
        if joint.nv() == 0 {
            continue;
        }
        let vi = model.idx_v(j);
        let axis = ws.world_joint_axis[j];
        let origin = ws.world_joint_origin[j];
        let lin = match joint.jtype {
            JointType::Revolute => axis.cross(p_world - origin),
            JointType::Prismatic => axis,
            JointType::Fixed => continue,
        };
        jrow[vi] = normal.dot(lin);
    }
    Ok(jrow)
}

pub(super) fn solve_m_inv_jt_columns(
    mass: &[Vec<f64>],
    jrows: &[Vec<f64>],
) -> Result<Vec<Vec<f64>>> {
    let n = mass.len();
    let k = jrows.len();
    let mut cols = vec![vec![0.0; n]; k];
    for c in 0..k {
        cols[c] = cholesky_solve(mass, &jrows[c])?;
    }
    Ok(cols)
}

fn contact_tangent_basis(normal_world: Vec3) -> Result<(Vec3, Vec3, Vec3)> {
    let nrm = normal_world.norm();
    if nrm <= 1e-12 {
        return Err(PinocchioError::invalid_model(
            "contact normal must be non-zero",
        ));
    }
    let n = normal_world * (1.0 / nrm);
    let helper = if n.z.abs() < 0.9 {
        Vec3::new(0.0, 0.0, 1.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let t1_raw = n.cross(helper);
    let t1_norm = t1_raw.norm();
    if t1_norm <= 1e-12 {
        return Err(PinocchioError::invalid_model(
            "failed to build tangent basis",
        ));
    }
    let t1 = t1_raw * (1.0 / t1_norm);
    let t2 = n.cross(t1);
    Ok((n, t1, t2))
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
        let qdd = super::dynamics::aba(model, q, qd, tau, gravity, ws)?;
        return Ok(ContactSolveResult {
            qdd,
            lambda_normal: Vec::new(),
            contact_forces_world: Vec::new(),
        });
    }

    let qdd_free = super::dynamics::aba(model, q, qd, tau, gravity, ws)?;
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

pub fn constrained_forward_dynamics_contacts_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    tau_batch: &[f64],
    batch_size: usize,
    contacts: &[ContactPoint],
    gravity: Vec3,
    ws: &mut Workspace,
    qdd_out: &mut [f64],
    lambda_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let k = contacts.len();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_lambda = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if qd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_batch.len(),
        });
    }
    if tau_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_batch.len(),
        });
    }
    if qdd_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qdd_out.len(),
        });
    }
    if lambda_out.len() != expected_lambda {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_lambda,
            got: lambda_out.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let lb = step * k;
        let out = constrained_forward_dynamics_contacts(
            model,
            &q_batch[qb..qb + n],
            &qd_batch[qb..qb + n],
            &tau_batch[qb..qb + n],
            contacts,
            gravity,
            ws,
        )?;
        qdd_out[qb..qb + n].copy_from_slice(&out.qdd);
        lambda_out[lb..lb + k].copy_from_slice(&out.lambda_normal);
    }
    Ok(())
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
        return Err(PinocchioError::invalid_model("restitution must be >= 0"));
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

pub fn apply_contact_impulses_batch(
    model: &Model,
    q_batch: &[f64],
    qd_minus_batch: &[f64],
    batch_size: usize,
    contacts: &[ContactPoint],
    restitution: f64,
    ws: &mut Workspace,
    qd_plus_out: &mut [f64],
    impulse_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let k = contacts.len();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_impulse = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if qd_minus_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_minus_batch.len(),
        });
    }
    if qd_plus_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_plus_out.len(),
        });
    }
    if impulse_out.len() != expected_impulse {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_impulse,
            got: impulse_out.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let ib = step * k;
        let out = apply_contact_impulses(
            model,
            &q_batch[qb..qb + n],
            &qd_minus_batch[qb..qb + n],
            contacts,
            restitution,
            ws,
        )?;
        qd_plus_out[qb..qb + n].copy_from_slice(&out.qd_plus);
        impulse_out[ib..ib + k].copy_from_slice(&out.impulse_normal);
    }

    Ok(())
}

pub fn constrained_forward_dynamics_contacts_friction(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[FrictionContactPoint],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<FrictionContactSolveResult> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }
    if contacts.is_empty() {
        let qdd = super::dynamics::aba(model, q, qd, tau, gravity, ws)?;
        return Ok(FrictionContactSolveResult {
            qdd,
            lambda_normal: Vec::new(),
            lambda_tangent: Vec::new(),
            contact_forces_world: Vec::new(),
        });
    }

    let qdd_free = super::dynamics::aba(model, q, qd, tau, gravity, ws)?;
    let mass = crba(model, q, ws)?;
    let qdd_zero = vec![0.0; n];
    forward_kinematics(model, q, qd, &qdd_zero, gravity, ws)?;

    let k = contacts.len();
    let d = 3 * k;
    let mut dirs = Vec::with_capacity(k);
    let mut jrows = Vec::with_capacity(d);
    let mut rhs = vec![0.0; d];
    for (ci, c) in contacts.iter().copied().enumerate() {
        let (n_dir, t1, t2) = contact_tangent_basis(c.normal_world)?;
        dirs.push((n_dir, t1, t2));
        let jn = contact_jacobian_row(
            model,
            ws,
            ContactPoint {
                link_index: c.link_index,
                point_local: c.point_local,
                normal_world: n_dir,
                acceleration_bias: c.acceleration_bias,
            },
        )?;
        let jt1 = contact_jacobian_row(
            model,
            ws,
            ContactPoint {
                link_index: c.link_index,
                point_local: c.point_local,
                normal_world: t1,
                acceleration_bias: 0.0,
            },
        )?;
        let jt2 = contact_jacobian_row(
            model,
            ws,
            ContactPoint {
                link_index: c.link_index,
                point_local: c.point_local,
                normal_world: t2,
                acceleration_bias: 0.0,
            },
        )?;

        let a_n = jn
            .iter()
            .zip(qdd_free.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let a_t1 = jt1
            .iter()
            .zip(qdd_free.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let a_t2 = jt2
            .iter()
            .zip(qdd_free.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();

        let base = 3 * ci;
        rhs[base] = -(a_n + c.acceleration_bias);
        rhs[base + 1] = -a_t1;
        rhs[base + 2] = -a_t2;
        jrows.push(jn);
        jrows.push(jt1);
        jrows.push(jt2);
    }

    let minv_jt = solve_m_inv_jt_columns(&mass, &jrows)?;
    let mut a = vec![vec![0.0; d]; d];
    for i in 0..d {
        for (j, col) in minv_jt.iter().enumerate().take(d) {
            a[i][j] = jrows[i]
                .iter()
                .zip(col.iter())
                .map(|(x, y)| x * y)
                .sum::<f64>();
        }
        a[i][i] += 1e-8;
    }

    let lambda = projected_gauss_seidel_friction(&a, &rhs, contacts, 50);
    let mut qdd = qdd_free.clone();
    for (r, col) in minv_jt.iter().enumerate().take(d) {
        let gain = lambda[r];
        for i in 0..n {
            qdd[i] += col[i] * gain;
        }
    }

    let mut lambda_n = vec![0.0; k];
    let mut lambda_t = vec![[0.0; 2]; k];
    let mut forces = vec![Vec3::zero(); k];
    for c in 0..k {
        let base = 3 * c;
        let ln = lambda[base];
        let lt1 = lambda[base + 1];
        let lt2 = lambda[base + 2];
        lambda_n[c] = ln;
        lambda_t[c] = [lt1, lt2];
        let (n_dir, t1, t2) = dirs[c];
        forces[c] = n_dir * ln + t1 * lt1 + t2 * lt2;
    }

    Ok(FrictionContactSolveResult {
        qdd,
        lambda_normal: lambda_n,
        lambda_tangent: lambda_t,
        contact_forces_world: forces,
    })
}

pub fn constrained_forward_dynamics_contacts_friction_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    tau_batch: &[f64],
    batch_size: usize,
    contacts: &[FrictionContactPoint],
    gravity: Vec3,
    ws: &mut Workspace,
    qdd_out: &mut [f64],
    lambda_normal_out: &mut [f64],
    lambda_tangent_out: &mut [f64],
    force_world_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let k = contacts.len();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_k = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_t = expected_k
        .checked_mul(2)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_f = expected_k
        .checked_mul(3)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    if q_batch.len() != expected
        || qd_batch.len() != expected
        || tau_batch.len() != expected
        || qdd_out.len() != expected
    {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len().min(qd_batch.len()).min(tau_batch.len()),
        });
    }
    if lambda_normal_out.len() != expected_k {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_k,
            got: lambda_normal_out.len(),
        });
    }
    if lambda_tangent_out.len() != expected_t {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_t,
            got: lambda_tangent_out.len(),
        });
    }
    if force_world_out.len() != expected_f {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_f,
            got: force_world_out.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let kb = step * k;
        let tb = step * k * 2;
        let fb = step * k * 3;
        let out = constrained_forward_dynamics_contacts_friction(
            model,
            &q_batch[qb..qb + n],
            &qd_batch[qb..qb + n],
            &tau_batch[qb..qb + n],
            contacts,
            gravity,
            ws,
        )?;
        qdd_out[qb..qb + n].copy_from_slice(&out.qdd);
        lambda_normal_out[kb..kb + k].copy_from_slice(&out.lambda_normal);
        for c in 0..k {
            lambda_tangent_out[tb + 2 * c] = out.lambda_tangent[c][0];
            lambda_tangent_out[tb + 2 * c + 1] = out.lambda_tangent[c][1];
            force_world_out[fb + 3 * c] = out.contact_forces_world[c].x;
            force_world_out[fb + 3 * c + 1] = out.contact_forces_world[c].y;
            force_world_out[fb + 3 * c + 2] = out.contact_forces_world[c].z;
        }
    }
    Ok(())
}

pub fn apply_contact_impulses_friction(
    model: &Model,
    q: &[f64],
    qd_minus: &[f64],
    contacts: &[FrictionContactPoint],
    restitution: f64,
    ws: &mut Workspace,
) -> Result<FrictionContactImpulseResult> {
    let n = model.nv();
    model.check_state_dims(q, qd_minus, None)?;
    if restitution < 0.0 {
        return Err(PinocchioError::invalid_model("restitution must be >= 0"));
    }
    if contacts.is_empty() {
        return Ok(FrictionContactImpulseResult {
            qd_plus: qd_minus.to_vec(),
            impulse_normal: Vec::new(),
            impulse_tangent: Vec::new(),
            contact_impulses_world: Vec::new(),
        });
    }

    let qd0 = vec![0.0; n];
    let qdd0 = vec![0.0; n];
    forward_kinematics(model, q, &qd0, &qdd0, Vec3::zero(), ws)?;
    let mass = crba(model, q, ws)?;

    let k = contacts.len();
    let d = 3 * k;
    let mut dirs = Vec::with_capacity(k);
    let mut jrows = Vec::with_capacity(d);
    let mut rhs = vec![0.0; d];
    for (ci, c) in contacts.iter().copied().enumerate() {
        let (n_dir, t1, t2) = contact_tangent_basis(c.normal_world)?;
        dirs.push((n_dir, t1, t2));
        let jn = contact_jacobian_row(
            model,
            ws,
            ContactPoint {
                link_index: c.link_index,
                point_local: c.point_local,
                normal_world: n_dir,
                acceleration_bias: 0.0,
            },
        )?;
        let jt1 = contact_jacobian_row(
            model,
            ws,
            ContactPoint {
                link_index: c.link_index,
                point_local: c.point_local,
                normal_world: t1,
                acceleration_bias: 0.0,
            },
        )?;
        let jt2 = contact_jacobian_row(
            model,
            ws,
            ContactPoint {
                link_index: c.link_index,
                point_local: c.point_local,
                normal_world: t2,
                acceleration_bias: 0.0,
            },
        )?;
        let vn = jn
            .iter()
            .zip(qd_minus.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let vt1 = jt1
            .iter()
            .zip(qd_minus.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let vt2 = jt2
            .iter()
            .zip(qd_minus.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let base = 3 * ci;
        rhs[base] = -(1.0 + restitution) * vn;
        rhs[base + 1] = -vt1;
        rhs[base + 2] = -vt2;
        jrows.push(jn);
        jrows.push(jt1);
        jrows.push(jt2);
    }
    let minv_jt = solve_m_inv_jt_columns(&mass, &jrows)?;
    let mut a = vec![vec![0.0; d]; d];
    for i in 0..d {
        for (j, col) in minv_jt.iter().enumerate().take(d) {
            a[i][j] = jrows[i]
                .iter()
                .zip(col.iter())
                .map(|(x, y)| x * y)
                .sum::<f64>();
        }
        a[i][i] += 1e-8;
    }
    let impulse = projected_gauss_seidel_friction(&a, &rhs, contacts, 50);

    let mut qd_plus = qd_minus.to_vec();
    for (r, col) in minv_jt.iter().enumerate().take(d) {
        let gain = impulse[r];
        for i in 0..n {
            qd_plus[i] += col[i] * gain;
        }
    }

    let mut impulse_n = vec![0.0; k];
    let mut impulse_t = vec![[0.0; 2]; k];
    let mut impulses_world = vec![Vec3::zero(); k];
    for c in 0..k {
        let base = 3 * c;
        let in_ = impulse[base];
        let it1 = impulse[base + 1];
        let it2 = impulse[base + 2];
        impulse_n[c] = in_;
        impulse_t[c] = [it1, it2];
        let (n_dir, t1, t2) = dirs[c];
        impulses_world[c] = n_dir * in_ + t1 * it1 + t2 * it2;
    }

    Ok(FrictionContactImpulseResult {
        qd_plus,
        impulse_normal: impulse_n,
        impulse_tangent: impulse_t,
        contact_impulses_world: impulses_world,
    })
}

pub fn apply_contact_impulses_friction_batch(
    model: &Model,
    q_batch: &[f64],
    qd_minus_batch: &[f64],
    batch_size: usize,
    contacts: &[FrictionContactPoint],
    restitution: f64,
    ws: &mut Workspace,
    qd_plus_out: &mut [f64],
    impulse_normal_out: &mut [f64],
    impulse_tangent_out: &mut [f64],
    impulse_world_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let k = contacts.len();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_k = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_t = expected_k
        .checked_mul(2)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_w = expected_k
        .checked_mul(3)
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;

    if q_batch.len() != expected
        || qd_minus_batch.len() != expected
        || qd_plus_out.len() != expected
    {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len().min(qd_minus_batch.len()),
        });
    }
    if impulse_normal_out.len() != expected_k {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_k,
            got: impulse_normal_out.len(),
        });
    }
    if impulse_tangent_out.len() != expected_t {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_t,
            got: impulse_tangent_out.len(),
        });
    }
    if impulse_world_out.len() != expected_w {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_w,
            got: impulse_world_out.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let kb = step * k;
        let tb = step * k * 2;
        let wb = step * k * 3;
        let out = apply_contact_impulses_friction(
            model,
            &q_batch[qb..qb + n],
            &qd_minus_batch[qb..qb + n],
            contacts,
            restitution,
            ws,
        )?;
        qd_plus_out[qb..qb + n].copy_from_slice(&out.qd_plus);
        impulse_normal_out[kb..kb + k].copy_from_slice(&out.impulse_normal);
        for c in 0..k {
            impulse_tangent_out[tb + 2 * c] = out.impulse_tangent[c][0];
            impulse_tangent_out[tb + 2 * c + 1] = out.impulse_tangent[c][1];
            impulse_world_out[wb + 3 * c] = out.contact_impulses_world[c].x;
            impulse_world_out[wb + 3 * c + 1] = out.contact_impulses_world[c].y;
            impulse_world_out[wb + 3 * c + 2] = out.contact_impulses_world[c].z;
        }
    }
    Ok(())
}
