use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Transform, Vec3};
use crate::model::{Model, Workspace};

fn ensure_state(model: &Model, q: &[f64], qd: &[f64], qdd: &[f64]) -> Result<()> {
    model.check_state_dims(q, qd, Some(qdd))
}

pub fn forward_kinematics(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<()> {
    ensure_state(model, q, qd, qdd)?;
    if model.nlinks() != ws.world_pose.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: model.nlinks(),
            got: ws.world_pose.len(),
        });
    }

    ws.world_pose[0] = Transform::identity();
    ws.omega[0] = Vec3::zero();
    ws.vel_origin[0] = Vec3::zero();
    ws.alpha[0] = Vec3::zero();
    ws.acc_origin[0] = gravity * -1.0;

    for link_idx in 1..model.nlinks() {
        let link = &model.links[link_idx];
        let parent = link.parent.expect("validated model");
        let parent_pose = ws.world_pose[parent];
        let parent_w = ws.omega[parent];
        let parent_v = ws.vel_origin[parent];
        let parent_alpha = ws.alpha[parent];
        let parent_acc = ws.acc_origin[parent];

        let joint = link.joint.as_ref().expect("validated model");
        let jidx = model.link_joint(link_idx).expect("validated model");

        let q_j = q[jidx];
        let qd_j = qd[jidx];
        let qdd_j = qdd[jidx];

        let joint_frame_rotation = parent_pose.rotation.mul_mat(joint.origin.rotation);
        let joint_origin_world = parent_pose.transform_point(joint.origin.translation);
        let axis_world = joint_frame_rotation.mul_vec(joint.axis);
        let rot_delta = Mat3::from_axis_angle(joint.axis, q_j);
        let link_rotation = joint_frame_rotation.mul_mat(rot_delta);

        ws.world_pose[link_idx] = Transform::new(link_rotation, joint_origin_world);
        ws.world_joint_axis[jidx] = axis_world;
        ws.world_joint_origin[jidx] = joint_origin_world;

        let r_parent_to_joint = joint_origin_world - parent_pose.translation;

        ws.omega[link_idx] = parent_w + axis_world * qd_j;
        ws.vel_origin[link_idx] = parent_v + parent_w.cross(r_parent_to_joint);
        ws.alpha[link_idx] = parent_alpha + axis_world * qdd_j + parent_w.cross(axis_world * qd_j);
        ws.acc_origin[link_idx] = parent_acc
            + parent_alpha.cross(r_parent_to_joint)
            + parent_w.cross(parent_w.cross(r_parent_to_joint));
    }

    Ok(())
}

pub fn rnea(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    forward_kinematics(model, q, qd, qdd, gravity, ws)?;

    for link_idx in 0..model.nlinks() {
        let link = &model.links[link_idx];
        let pose = ws.world_pose[link_idx];
        let r_com = pose.rotation.mul_vec(link.com_local);
        let com_acc = ws.acc_origin[link_idx]
            + ws.alpha[link_idx].cross(r_com)
            + ws.omega[link_idx].cross(ws.omega[link_idx].cross(r_com));

        let f = com_acc * link.mass;
        let i_world = pose
            .rotation
            .mul_mat(link.inertia_local_com)
            .mul_mat(pose.rotation.transpose());
        let iw = i_world.mul_vec(ws.omega[link_idx]);
        let n_com = i_world.mul_vec(ws.alpha[link_idx]) + ws.omega[link_idx].cross(iw);
        let n_origin = n_com + r_com.cross(f);

        ws.force[link_idx] = f;
        ws.torque[link_idx] = n_origin;
    }

    let mut tau = vec![0.0; model.nv()];
    for link_idx in (1..model.nlinks()).rev() {
        let jidx = model.link_joint(link_idx).expect("validated model");
        let axis = ws.world_joint_axis[jidx];
        tau[jidx] = ws.torque[link_idx].dot(axis);

        let parent = model.links[link_idx].parent.expect("validated model");
        let r_parent_to_joint =
            ws.world_pose[link_idx].translation - ws.world_pose[parent].translation;
        let child_force = ws.force[link_idx];
        let child_torque = ws.torque[link_idx];
        ws.force[parent] += child_force;
        ws.torque[parent] += child_torque + r_parent_to_joint.cross(child_force);
    }

    Ok(tau)
}

pub fn bias_forces(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let qdd = vec![0.0; model.nv()];
    rnea(model, q, qd, &qdd, gravity, ws)
}

pub fn gravity_torques(
    model: &Model,
    q: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    rnea(model, q, &qd, &qdd, gravity, ws)
}

pub fn coriolis_torques(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let qdd = vec![0.0; model.nv()];
    rnea(model, q, qd, &qdd, Vec3::zero(), ws)
}

pub fn crba(model: &Model, q: &[f64], ws: &mut Workspace) -> Result<Vec<Vec<f64>>> {
    let n = model.nv();
    model.check_state_dims(q, &vec![0.0; n], None)?;

    let gravity = Vec3::zero();
    let mut mass = vec![vec![0.0; n]; n];
    let qd_zero = vec![0.0; n];

    for col in 0..n {
        let mut qdd = vec![0.0; n];
        qdd[col] = 1.0;
        let tau = rnea(model, q, &qd_zero, &qdd, gravity, ws)?;
        for row in 0..n {
            mass[row][col] = tau[row];
        }
    }

    for r in 0..n {
        for c in (r + 1)..n {
            let v = 0.5 * (mass[r][c] + mass[c][r]);
            mass[r][c] = v;
            mass[c][r] = v;
        }
    }

    Ok(mass)
}

fn cholesky_solve(a: &[Vec<f64>], b: &[f64]) -> Result<Vec<f64>> {
    let n = a.len();
    if n == 0 || b.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: b.len(),
        });
    }
    for row in a {
        if row.len() != n {
            return Err(PinocchioError::InvalidModel("matrix must be square"));
        }
    }

    let mut l = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i][j];
            for k in 0..j {
                sum -= l[i][k] * l[j][k];
            }
            if i == j {
                if sum <= 1e-12 {
                    return Err(PinocchioError::SingularMatrix);
                }
                l[i][j] = sum.sqrt();
            } else {
                l[i][j] = sum / l[j][j];
            }
        }
    }

    let mut y = vec![0.0; n];
    for i in 0..n {
        let mut sum = b[i];
        for (k, yk) in y.iter().take(i).copied().enumerate() {
            sum -= l[i][k] * yk;
        }
        y[i] = sum / l[i][i];
    }

    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = y[i];
        for k in (i + 1)..n {
            sum -= l[k][i] * x[k];
        }
        x[i] = sum / l[i][i];
    }

    Ok(x)
}

pub fn aba(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }

    let bias = bias_forces(model, q, qd, gravity, ws)?;
    let mass = crba(model, q, ws)?;
    let rhs: Vec<f64> = tau
        .iter()
        .zip(bias.iter())
        .map(|(tau_i, b_i)| tau_i - b_i)
        .collect();

    cholesky_solve(&mass, &rhs)
}

fn is_ancestor(model: &Model, maybe_ancestor: usize, mut node: usize) -> bool {
    loop {
        if node == maybe_ancestor {
            return true;
        }
        match model.links[node].parent {
            Some(parent) => node = parent,
            None => return false,
        }
    }
}

pub fn frame_jacobian(
    model: &Model,
    q: &[f64],
    target_link: usize,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    if target_link >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: target_link,
            len: model.nlinks(),
        });
    }

    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let mut jac = vec![0.0; 6 * n];
    let p_target = ws.world_pose[target_link].translation;

    for j in 0..n {
        let link_of_joint = model.joint_link(j).expect("validated model");
        if !is_ancestor(model, link_of_joint, target_link) {
            continue;
        }

        let axis = ws.world_joint_axis[j];
        let origin = ws.world_joint_origin[j];
        let lin = axis.cross(p_target - origin);
        let col_base = j;

        jac[col_base] = lin.x;
        jac[n + col_base] = lin.y;
        jac[2 * n + col_base] = lin.z;
        jac[3 * n + col_base] = axis.x;
        jac[4 * n + col_base] = axis.y;
        jac[5 * n + col_base] = axis.z;
    }

    Ok(jac)
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

pub fn rnea_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    qdd_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    tau_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
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
    if qdd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qdd_batch.len(),
        });
    }
    if tau_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let tau = rnea(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &qdd_batch[b..b + n],
            gravity,
            ws,
        )?;
        tau_out[b..b + n].copy_from_slice(&tau);
    }
    Ok(())
}

pub fn aba_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    tau_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    qdd_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
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

    for step in 0..batch_size {
        let b = step * n;
        let qdd = aba(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &tau_batch[b..b + n],
            gravity,
            ws,
        )?;
        qdd_out[b..b + n].copy_from_slice(&qdd);
    }
    Ok(())
}

pub fn bias_forces_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    bias_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
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
    if bias_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: bias_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let bias = bias_forces(model, &q_batch[b..b + n], &qd_batch[b..b + n], gravity, ws)?;
        bias_out[b..b + n].copy_from_slice(&bias);
    }
    Ok(())
}

pub fn gravity_torques_batch(
    model: &Model,
    q_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
    ws: &mut Workspace,
    g_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if g_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: g_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let g = gravity_torques(model, &q_batch[b..b + n], gravity, ws)?;
        g_out[b..b + n].copy_from_slice(&g);
    }
    Ok(())
}

pub fn crba_batch(
    model: &Model,
    q_batch: &[f64],
    batch_size: usize,
    ws: &mut Workspace,
    mass_out_row_major: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected_q = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_m = batch_size
        .checked_mul(n)
        .and_then(|x| x.checked_mul(n))
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected_q {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_q,
            got: q_batch.len(),
        });
    }
    if mass_out_row_major.len() != expected_m {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_m,
            got: mass_out_row_major.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let mb = step * n * n;
        let mass = crba(model, &q_batch[qb..qb + n], ws)?;
        for r in 0..n {
            for c in 0..n {
                mass_out_row_major[mb + r * n + c] = mass[r][c];
            }
        }
    }
    Ok(())
}

pub fn constrained_aba_locked_joints(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    locked: &[bool],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }
    if locked.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: locked.len(),
        });
    }

    let free: Vec<usize> = (0..n).filter(|i| !locked[*i]).collect();
    if free.is_empty() {
        return Ok(vec![0.0; n]);
    }

    let bias = bias_forces(model, q, qd, gravity, ws)?;
    let mass = crba(model, q, ws)?;

    let nf = free.len();
    let mut m_ff = vec![vec![0.0; nf]; nf];
    let mut rhs_f = vec![0.0; nf];
    for (r, &i) in free.iter().enumerate() {
        rhs_f[r] = tau[i] - bias[i];
        for (c, &j) in free.iter().enumerate() {
            m_ff[r][c] = mass[i][j];
        }
    }

    let qdd_f = cholesky_solve(&m_ff, &rhs_f)?;
    let mut qdd = vec![0.0; n];
    for (k, &idx) in free.iter().enumerate() {
        qdd[idx] = qdd_f[k];
    }
    Ok(qdd)
}

pub fn constrained_aba_locked_joints_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    tau_batch: &[f64],
    batch_size: usize,
    locked: &[bool],
    gravity: Vec3,
    ws: &mut Workspace,
    qdd_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
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
    if locked.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: locked.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let qdd = constrained_aba_locked_joints(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &tau_batch[b..b + n],
            locked,
            gravity,
            ws,
        )?;
        qdd_out[b..b + n].copy_from_slice(&qdd);
    }
    Ok(())
}

pub struct RolloutState<'a> {
    pub q0: &'a [f64],
    pub qd0: &'a [f64],
}

pub fn rollout_aba_euler(
    model: &Model,
    init: RolloutState<'_>,
    tau_batch: &[f64],
    batch_size: usize,
    dt: f64,
    gravity: Vec3,
    ws: &mut Workspace,
    q_out: &mut [f64],
    qd_out: &mut [f64],
) -> Result<()> {
    if dt <= 0.0 {
        return Err(PinocchioError::InvalidModel("dt must be > 0"));
    }

    let n = model.nv();
    model.check_state_dims(init.q0, init.qd0, None)?;
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if tau_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: tau_batch.len(),
        });
    }
    if q_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_out.len(),
        });
    }
    if qd_out.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: qd_out.len(),
        });
    }

    let mut q = init.q0.to_vec();
    let mut qd = init.qd0.to_vec();
    for step in 0..batch_size {
        let b = step * n;
        let tau = &tau_batch[b..b + n];
        let qdd = aba(model, &q, &qd, tau, gravity, ws)?;
        for i in 0..n {
            qd[i] += qdd[i] * dt;
            q[i] += qd[i] * dt;
        }
        q_out[b..b + n].copy_from_slice(&q);
        qd_out[b..b + n].copy_from_slice(&qd);
    }
    Ok(())
}

pub fn forward_kinematics_poses(
    model: &Model,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<Transform>> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;
    Ok(ws.world_pose.clone())
}

pub fn forward_kinematics_poses_batch(
    model: &Model,
    q_batch: &[f64],
    batch_size: usize,
    ws: &mut Workspace,
    translations_out: &mut [f64],
    rotations_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let nl = model.nlinks();
    let expected_q = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected_q {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_q,
            got: q_batch.len(),
        });
    }
    let expected_t = batch_size
        .checked_mul(nl)
        .and_then(|x| x.checked_mul(3))
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_r = batch_size
        .checked_mul(nl)
        .and_then(|x| x.checked_mul(9))
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if translations_out.len() != expected_t {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_t,
            got: translations_out.len(),
        });
    }
    if rotations_out.len() != expected_r {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_r,
            got: rotations_out.len(),
        });
    }

    for step in 0..batch_size {
        let qb = step * n;
        let poses = forward_kinematics_poses(model, &q_batch[qb..qb + n], ws)?;
        for (l, pose) in poses.iter().enumerate() {
            let tb = (step * nl + l) * 3;
            translations_out[tb] = pose.translation.x;
            translations_out[tb + 1] = pose.translation.y;
            translations_out[tb + 2] = pose.translation.z;

            let rb = (step * nl + l) * 9;
            for r in 0..3 {
                for c in 0..3 {
                    rotations_out[rb + 3 * r + c] = pose.rotation.m[r][c];
                }
            }
        }
    }
    Ok(())
}
