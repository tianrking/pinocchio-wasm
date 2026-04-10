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

fn contact_jacobian_row(
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

pub fn contact_jacobian_normal(
    model: &Model,
    q: &[f64],
    contacts: &[ContactPoint],
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let k = contacts.len();
    let mut out = vec![0.0; k * n];
    for (i, c) in contacts.iter().copied().enumerate() {
        let row = contact_jacobian_row(model, ws, c)?;
        let base = i * n;
        out[base..base + n].copy_from_slice(&row);
    }
    Ok(out)
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

fn solve_m_inv_jt_columns(mass: &[Vec<f64>], jrows: &[Vec<f64>]) -> Result<Vec<Vec<f64>>> {
    let n = mass.len();
    let k = jrows.len();
    let mut cols = vec![vec![0.0; n]; k];
    for c in 0..k {
        cols[c] = cholesky_solve(mass, &jrows[c])?;
    }
    Ok(cols)
}

fn projected_gauss_seidel_nonnegative(a: &[Vec<f64>], rhs: &[f64], iters: usize) -> Vec<f64> {
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

fn contact_tangent_basis(normal_world: Vec3) -> Result<(Vec3, Vec3, Vec3)> {
    let nrm = normal_world.norm();
    if nrm <= 1e-12 {
        return Err(PinocchioError::InvalidModel(
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
        return Err(PinocchioError::InvalidModel(
            "failed to build tangent basis",
        ));
    }
    let t1 = t1_raw * (1.0 / t1_norm);
    let t2 = n.cross(t1);
    Ok((n, t1, t2))
}

fn projected_gauss_seidel_friction(
    a: &[Vec<f64>],
    rhs: &[f64],
    contacts: &[FrictionContactPoint],
    iters: usize,
) -> Vec<f64> {
    let k = contacts.len();
    let d = 3 * k;
    let mut x = vec![0.0; d];
    for _ in 0..iters {
        for (c, cp) in contacts.iter().enumerate() {
            let in_idx = 3 * c;
            let it1_idx = in_idx + 1;
            let it2_idx = in_idx + 2;

            let mut rn = rhs[in_idx];
            let mut rt1 = rhs[it1_idx];
            let mut rt2 = rhs[it2_idx];
            for (j, xj) in x.iter().copied().enumerate().take(d) {
                if j != in_idx {
                    rn -= a[in_idx][j] * xj;
                }
                if j != it1_idx {
                    rt1 -= a[it1_idx][j] * xj;
                }
                if j != it2_idx {
                    rt2 -= a[it2_idx][j] * xj;
                }
            }

            let dn = a[in_idx][in_idx].max(1e-9);
            let dt1 = a[it1_idx][it1_idx].max(1e-9);
            let dt2 = a[it2_idx][it2_idx].max(1e-9);

            let lambda_n = (rn / dn).max(0.0);
            let mut lambda_t1 = rt1 / dt1;
            let mut lambda_t2 = rt2 / dt2;

            let mu = cp.friction_coeff.max(0.0);
            let tnorm = (lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2).sqrt();
            let max_t = mu * lambda_n;
            if tnorm > max_t && tnorm > 1e-12 {
                let s = max_t / tnorm;
                lambda_t1 *= s;
                lambda_t2 *= s;
            }

            x[in_idx] = lambda_n;
            x[it1_idx] = lambda_t1;
            x[it2_idx] = lambda_t2;
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
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_lambda = batch_size
        .checked_mul(k)
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
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_impulse = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
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
        let qdd = aba(model, q, qd, tau, gravity, ws)?;
        return Ok(FrictionContactSolveResult {
            qdd,
            lambda_normal: Vec::new(),
            lambda_tangent: Vec::new(),
            contact_forces_world: Vec::new(),
        });
    }

    let qdd_free = aba(model, q, qd, tau, gravity, ws)?;
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
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_k = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_t = expected_k
        .checked_mul(2)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_f = expected_k
        .checked_mul(3)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
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
        return Err(PinocchioError::InvalidModel("restitution must be >= 0"));
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
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_k = batch_size
        .checked_mul(k)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_t = expected_k
        .checked_mul(2)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    let expected_w = expected_k
        .checked_mul(3)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;

    if q_batch.len() != expected || qd_minus_batch.len() != expected || qd_plus_out.len() != expected
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

#[derive(Debug, Clone)]
pub struct DerivativeResult {
    pub d_out_dq: Vec<Vec<f64>>,
    pub d_out_dv: Vec<Vec<f64>>,
    pub d_out_du: Vec<Vec<f64>>,
}

#[derive(Debug, Clone)]
pub struct KinematicsDerivativesResult {
    pub dpos_dq: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct FrameDerivativesResult {
    pub dframe_dq: Vec<f64>,
}

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

#[derive(Debug, Clone)]
pub struct ContactLinearProblem {
    pub delassus: Vec<Vec<f64>>,
    pub rhs: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct SecondOrderDerivatives {
    pub d2_out_dq2: Vec<f64>,
    pub d2_out_dv2: Vec<f64>,
    pub d2_out_du2: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct ImpulseDerivativeResult {
    pub d_qd_plus_dq: Vec<Vec<f64>>,
    pub d_qd_plus_dqd_minus: Vec<Vec<f64>>,
    pub d_qd_plus_d_restitution: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct RegressorBasisResult {
    pub selected_columns: Vec<usize>,
    pub projected_row_major: Vec<f64>,
    pub rows: usize,
    pub cols: usize,
}

pub fn integrate_configuration(q: &[f64], v: &[f64], dt: f64) -> Result<Vec<f64>> {
    if q.len() != v.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q.len(),
            got: v.len(),
        });
    }
    if dt <= 0.0 {
        return Err(PinocchioError::InvalidModel("dt must be > 0"));
    }
    let mut out = vec![0.0; q.len()];
    for i in 0..q.len() {
        out[i] = q[i] + v[i] * dt;
    }
    Ok(out)
}

pub fn difference_configuration(q0: &[f64], q1: &[f64]) -> Result<Vec<f64>> {
    if q0.len() != q1.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q0.len(),
            got: q1.len(),
        });
    }
    Ok(q1.iter().zip(q0.iter()).map(|(a, b)| a - b).collect())
}

pub fn interpolate_configuration(q0: &[f64], q1: &[f64], alpha: f64) -> Result<Vec<f64>> {
    if !(0.0..=1.0).contains(&alpha) {
        return Err(PinocchioError::InvalidModel("alpha must be in [0,1]"));
    }
    if q0.len() != q1.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: q0.len(),
            got: q1.len(),
        });
    }
    Ok(q0
        .iter()
        .zip(q1.iter())
        .map(|(a, b)| (1.0 - alpha) * a + alpha * b)
        .collect())
}

fn lcg_next_u64(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
    *state
}

pub fn random_configuration(
    lower: &[f64],
    upper: &[f64],
    seed: u64,
) -> Result<Vec<f64>> {
    if lower.len() != upper.len() {
        return Err(PinocchioError::DimensionMismatch {
            expected: lower.len(),
            got: upper.len(),
        });
    }
    let mut st = seed;
    let mut out = vec![0.0; lower.len()];
    for i in 0..lower.len() {
        if upper[i] < lower[i] {
            return Err(PinocchioError::InvalidModel("upper bound must be >= lower bound"));
        }
        let r = (lcg_next_u64(&mut st) as f64) / (u64::MAX as f64);
        out[i] = lower[i] + r * (upper[i] - lower[i]);
    }
    Ok(out)
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

pub fn centroidal_map(
    model: &Model,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
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

pub fn rnea_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<DerivativeResult> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let eps = 1e-6;
    let mut d_tau_dq = vec![vec![0.0; n]; n];
    let mut d_tau_dqd = vec![vec![0.0; n]; n];
    let mut d_tau_dqdd = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let tp = rnea(model, &qp, qd, qdd, gravity, ws)?;
        let tm = rnea(model, &qm, qd, qdd, gravity, ws)?;
        for r in 0..n {
            d_tau_dq[r][i] = (tp[r] - tm[r]) / (2.0 * eps);
        }

        let mut v_p = qd.to_vec();
        let mut v_m = qd.to_vec();
        v_p[i] += eps;
        v_m[i] -= eps;
        let tp = rnea(model, q, &v_p, qdd, gravity, ws)?;
        let tm = rnea(model, q, &v_m, qdd, gravity, ws)?;
        for r in 0..n {
            d_tau_dqd[r][i] = (tp[r] - tm[r]) / (2.0 * eps);
        }

        let mut a_p = qdd.to_vec();
        let mut a_m = qdd.to_vec();
        a_p[i] += eps;
        a_m[i] -= eps;
        let tp = rnea(model, q, qd, &a_p, gravity, ws)?;
        let tm = rnea(model, q, qd, &a_m, gravity, ws)?;
        for r in 0..n {
            d_tau_dqdd[r][i] = (tp[r] - tm[r]) / (2.0 * eps);
        }
    }
    Ok(DerivativeResult {
        d_out_dq: d_tau_dq,
        d_out_dv: d_tau_dqd,
        d_out_du: d_tau_dqdd,
    })
}

pub fn aba_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<DerivativeResult> {
    let n = model.nv();
    model.check_state_dims(q, qd, None)?;
    if tau.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: tau.len(),
        });
    }
    let eps = 1e-6;
    let mut dqdd_dq = vec![vec![0.0; n]; n];
    let mut dqdd_dqd = vec![vec![0.0; n]; n];
    let mut dqdd_dtau = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let ap = aba(model, &qp, qd, tau, gravity, ws)?;
        let am = aba(model, &qm, qd, tau, gravity, ws)?;
        for r in 0..n {
            dqdd_dq[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut vp = qd.to_vec();
        let mut vm = qd.to_vec();
        vp[i] += eps;
        vm[i] -= eps;
        let ap = aba(model, q, &vp, tau, gravity, ws)?;
        let am = aba(model, q, &vm, tau, gravity, ws)?;
        for r in 0..n {
            dqdd_dqd[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut tp = tau.to_vec();
        let mut tm = tau.to_vec();
        tp[i] += eps;
        tm[i] -= eps;
        let ap = aba(model, q, qd, &tp, gravity, ws)?;
        let am = aba(model, q, qd, &tm, gravity, ws)?;
        for r in 0..n {
            dqdd_dtau[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }
    }
    Ok(DerivativeResult {
        d_out_dq: dqdd_dq,
        d_out_dv: dqdd_dqd,
        d_out_du: dqdd_dtau,
    })
}

pub fn kinematics_derivatives(
    model: &Model,
    q: &[f64],
    target_link: usize,
    ws: &mut Workspace,
) -> Result<KinematicsDerivativesResult> {
    if target_link >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: target_link,
            len: model.nlinks(),
        });
    }
    let n = model.nv();
    let eps = 1e-6;
    let mut dpos_dq = vec![0.0; 3 * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let pp = forward_kinematics_poses(model, &qp, ws)?[target_link].translation;
        let pm = forward_kinematics_poses(model, &qm, ws)?[target_link].translation;
        dpos_dq[i] = (pp.x - pm.x) / (2.0 * eps);
        dpos_dq[n + i] = (pp.y - pm.y) / (2.0 * eps);
        dpos_dq[2 * n + i] = (pp.z - pm.z) / (2.0 * eps);
    }
    Ok(KinematicsDerivativesResult { dpos_dq })
}

pub fn frame_jacobian_derivatives(
    model: &Model,
    q: &[f64],
    target_link: usize,
    ws: &mut Workspace,
) -> Result<FrameDerivativesResult> {
    let n = model.nv();
    let eps = 1e-6;
    let mut dframe_dq = vec![0.0; 6 * n * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let jp = frame_jacobian(model, &qp, target_link, ws)?;
        let jm = frame_jacobian(model, &qm, target_link, ws)?;
        for r in 0..(6 * n) {
            dframe_dq[r * n + i] = (jp[r] - jm[r]) / (2.0 * eps);
        }
    }
    Ok(FrameDerivativesResult { dframe_dq })
}

pub fn center_of_mass_derivatives(
    model: &Model,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<f64>> {
    let n = model.nv();
    let eps = 1e-6;
    let mut dcom_dq = vec![0.0; 3 * n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let cp = center_of_mass(model, &qp, ws)?;
        let cm = center_of_mass(model, &qm, ws)?;
        dcom_dq[i] = (cp.x - cm.x) / (2.0 * eps);
        dcom_dq[n + i] = (cp.y - cm.y) / (2.0 * eps);
        dcom_dq[2 * n + i] = (cp.z - cm.z) / (2.0 * eps);
    }
    Ok(dcom_dq)
}

fn inertial_params_of_link(model: &Model, link_idx: usize) -> [f64; 10] {
    let l = &model.links[link_idx];
    [
        l.mass,
        l.mass * l.com_local.x,
        l.mass * l.com_local.y,
        l.mass * l.com_local.z,
        l.inertia_local_com.m[0][0],
        l.inertia_local_com.m[1][1],
        l.inertia_local_com.m[2][2],
        l.inertia_local_com.m[0][1],
        l.inertia_local_com.m[0][2],
        l.inertia_local_com.m[1][2],
    ]
}

fn set_inertial_params_of_link(model: &mut Model, link_idx: usize, p: [f64; 10]) {
    let mass = p[0].max(1e-6);
    model.links[link_idx].mass = mass;
    model.links[link_idx].com_local = Vec3::new(p[1] / mass, p[2] / mass, p[3] / mass);
    model.links[link_idx].inertia_local_com = Mat3::new([
        [p[4], p[7], p[8]],
        [p[7], p[5], p[9]],
        [p[8], p[9], p[6]],
    ]);
}

pub fn inverse_dynamics_regressor(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let p = 10 * model.nlinks();
    let mut y = vec![0.0; n * p];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base_param = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base_param;
            let mut p_minus = base_param;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let tau_plus = rnea(&plus, q, qd, qdd, gravity, &mut ws_plus)?;
            let tau_minus = rnea(&minus, q, qd, qdd, gravity, &mut ws_minus)?;
            let col = link_idx * 10 + k;
            for row in 0..n {
                y[row * p + col] = (tau_plus[row] - tau_minus[row]) / (2.0 * eps);
            }
        }
    }
    Ok(y)
}

pub fn inverse_dynamics_regressor_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    qdd_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
) -> Result<Vec<f64>> {
    let n = model.nv();
    let p = 10 * model.nlinks();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected || qd_batch.len() != expected || qdd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len().min(qd_batch.len()).min(qdd_batch.len()),
        });
    }
    let mut out = vec![0.0; batch_size * n * p];
    for step in 0..batch_size {
        let b = step * n;
        let y = inverse_dynamics_regressor(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &qdd_batch[b..b + n],
            gravity,
        )?;
        let ob = step * n * p;
        out[ob..ob + n * p].copy_from_slice(&y);
    }
    Ok(out)
}

pub fn kinetic_energy_regressor(
    model: &Model,
    q: &[f64],
    qd: &[f64],
) -> Result<Vec<f64>> {
    let nparam = 10 * model.nlinks();
    let mut out = vec![0.0; nparam];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base;
            let mut p_minus = base;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let e_plus = kinetic_energy(&plus, q, qd, &mut ws_plus)?;
            let e_minus = kinetic_energy(&minus, q, qd, &mut ws_minus)?;
            out[link_idx * 10 + k] = (e_plus - e_minus) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn potential_energy_regressor(
    model: &Model,
    q: &[f64],
    gravity: Vec3,
) -> Result<Vec<f64>> {
    let nparam = 10 * model.nlinks();
    let mut out = vec![0.0; nparam];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base;
            let mut p_minus = base;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let e_plus = potential_energy(&plus, q, gravity, &mut ws_plus)?;
            let e_minus = potential_energy(&minus, q, gravity, &mut ws_minus)?;
            out[link_idx * 10 + k] = (e_plus - e_minus) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn center_of_mass_regressor(
    model: &Model,
    q: &[f64],
) -> Result<Vec<f64>> {
    let p = 10 * model.nlinks();
    let mut out = vec![0.0; 3 * p];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base;
            let mut p_minus = base;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let c_plus = center_of_mass(&plus, q, &mut ws_plus)?;
            let c_minus = center_of_mass(&minus, q, &mut ws_minus)?;
            let col = link_idx * 10 + k;
            out[col] = (c_plus.x - c_minus.x) / (2.0 * eps);
            out[p + col] = (c_plus.y - c_minus.y) / (2.0 * eps);
            out[2 * p + col] = (c_plus.z - c_minus.z) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn select_independent_regressor_columns(
    regressor_row_major: &[f64],
    rows: usize,
    cols: usize,
    tolerance: f64,
) -> Result<RegressorBasisResult> {
    if regressor_row_major.len() != rows * cols {
        return Err(PinocchioError::DimensionMismatch {
            expected: rows * cols,
            got: regressor_row_major.len(),
        });
    }
    if tolerance <= 0.0 {
        return Err(PinocchioError::InvalidModel("tolerance must be > 0"));
    }
    let mut selected = Vec::<usize>::new();
    let mut q_cols: Vec<Vec<f64>> = Vec::new();

    for c in 0..cols {
        let mut v = vec![0.0; rows];
        for r in 0..rows {
            v[r] = regressor_row_major[r * cols + c];
        }
        for qv in &q_cols {
            let mut alpha = 0.0;
            let mut qnorm2 = 0.0;
            for r in 0..rows {
                alpha += v[r] * qv[r];
                qnorm2 += qv[r] * qv[r];
            }
            if qnorm2 > 1e-18 {
                let s = alpha / qnorm2;
                for r in 0..rows {
                    v[r] -= s * qv[r];
                }
            }
        }
        let norm2 = v.iter().map(|x| x * x).sum::<f64>();
        if norm2.sqrt() > tolerance {
            selected.push(c);
            q_cols.push(v);
        }
    }

    let k = selected.len();
    let mut projected = vec![0.0; rows * k];
    for (j, &col) in selected.iter().enumerate() {
        for r in 0..rows {
            projected[r * k + j] = regressor_row_major[r * cols + col];
        }
    }
    Ok(RegressorBasisResult {
        selected_columns: selected,
        projected_row_major: projected,
        rows,
        cols: k,
    })
}

pub fn build_delassus_matrix(
    model: &Model,
    q: &[f64],
    contacts: &[ContactPoint],
    ws: &mut Workspace,
) -> Result<Vec<Vec<f64>>> {
    let n = model.nv();
    let k = contacts.len();
    let j = contact_jacobian_normal(model, q, contacts, ws)?;
    let mass = crba(model, q, ws)?;
    let mut j_rows = Vec::with_capacity(k);
    for i in 0..k {
        let b = i * n;
        j_rows.push(j[b..b + n].to_vec());
    }
    let minv_jt = solve_m_inv_jt_columns(&mass, &j_rows)?;
    let mut w = vec![vec![0.0; k]; k];
    for i in 0..k {
        for (j, col) in minv_jt.iter().enumerate().take(k) {
            w[i][j] = j_rows[i]
                .iter()
                .zip(col.iter())
                .map(|(a, b)| a * b)
                .sum::<f64>();
        }
        w[i][i] += 1e-8;
    }
    Ok(w)
}

pub fn build_contact_problem(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[ContactPoint],
    gravity: Vec3,
    ws: &mut Workspace,
) -> Result<ContactLinearProblem> {
    let qdd_free = aba(model, q, qd, tau, gravity, ws)?;
    let n = model.nv();
    let k = contacts.len();
    let j = contact_jacobian_normal(model, q, contacts, ws)?;
    let mut rhs = vec![0.0; k];
    for i in 0..k {
        let b = i * n;
        let mut a = 0.0;
        for c in 0..n {
            a += j[b + c] * qdd_free[c];
        }
        rhs[i] = -(a + contacts[i].acceleration_bias);
    }
    let delassus = build_delassus_matrix(model, q, contacts, ws)?;
    Ok(ContactLinearProblem { delassus, rhs })
}

pub fn solve_contact_cholesky(problem: &ContactLinearProblem) -> Result<Vec<f64>> {
    cholesky_solve(&problem.delassus, &problem.rhs)
}

pub fn solve_contact_pgs(problem: &ContactLinearProblem, max_iters: usize) -> Vec<f64> {
    projected_gauss_seidel_nonnegative(&problem.delassus, &problem.rhs, max_iters.max(1))
}

pub fn solve_contact_admm(
    problem: &ContactLinearProblem,
    rho: f64,
    max_iters: usize,
) -> Result<Vec<f64>> {
    let k = problem.rhs.len();
    if rho <= 0.0 {
        return Err(PinocchioError::InvalidModel("rho must be > 0"));
    }
    let mut z = vec![0.0; k];
    let mut u = vec![0.0; k];
    let mut a_rho = problem.delassus.clone();
    for (i, row) in a_rho.iter_mut().enumerate().take(k) {
        row[i] += rho;
    }
    for _ in 0..max_iters.max(1) {
        let mut b = vec![0.0; k];
        for i in 0..k {
            b[i] = problem.rhs[i] + rho * (z[i] - u[i]);
        }
        let x = cholesky_solve(&a_rho, &b)?;
        for i in 0..k {
            z[i] = (x[i] + u[i]).max(0.0);
            u[i] += x[i] - z[i];
        }
    }
    Ok(z)
}

fn second_order_fd(
    x: &[f64],
    out_dim: usize,
    f: impl Fn(&[f64]) -> Result<Vec<f64>>,
) -> Result<Vec<f64>> {
    let n = x.len();
    let eps = 1e-5;
    let mut out = vec![0.0; out_dim * n * n];
    for i in 0..n {
        for j in 0..n {
            let mut xpp = x.to_vec();
            let mut xpm = x.to_vec();
            let mut xmp = x.to_vec();
            let mut xmm = x.to_vec();
            xpp[i] += eps;
            xpp[j] += eps;
            xpm[i] += eps;
            xpm[j] -= eps;
            xmp[i] -= eps;
            xmp[j] += eps;
            xmm[i] -= eps;
            xmm[j] -= eps;
            let ypp = f(&xpp)?;
            let ypm = f(&xpm)?;
            let ymp = f(&xmp)?;
            let ymm = f(&xmm)?;
            if ypp.len() != out_dim || ypm.len() != out_dim || ymp.len() != out_dim || ymm.len() != out_dim {
                return Err(PinocchioError::DimensionMismatch {
                    expected: out_dim,
                    got: ypp.len().min(ypm.len()).min(ymp.len()).min(ymm.len()),
                });
            }
            for r in 0..out_dim {
                out[(r * n + i) * n + j] = (ypp[r] - ypm[r] - ymp[r] + ymm[r]) / (4.0 * eps * eps);
            }
        }
    }
    Ok(out)
}

pub fn rnea_second_order_derivatives(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
    _ws: &mut Workspace,
) -> Result<SecondOrderDerivatives> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let d2_q = second_order_fd(q, n, |qv| {
        let mut ws = Workspace::new(model);
        rnea(model, qv, qd, qdd, gravity, &mut ws)
    })?;
    let d2_v = second_order_fd(qd, n, |vv| {
        let mut ws = Workspace::new(model);
        rnea(model, q, vv, qdd, gravity, &mut ws)
    })?;
    let d2_u = second_order_fd(qdd, n, |uv| {
        let mut ws = Workspace::new(model);
        rnea(model, q, qd, uv, gravity, &mut ws)
    })?;
    Ok(SecondOrderDerivatives {
        d2_out_dq2: d2_q,
        d2_out_dv2: d2_v,
        d2_out_du2: d2_u,
    })
}

pub fn constrained_dynamics_derivatives_locked_joints(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    locked: &[bool],
    gravity: Vec3,
    _ws: &mut Workspace,
) -> Result<DerivativeResult> {
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

    let eps = 1e-6;
    let mut dqdd_dq = vec![vec![0.0; n]; n];
    let mut dqdd_dqd = vec![vec![0.0; n]; n];
    let mut dqdd_dtau = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = constrained_aba_locked_joints(model, &qp, qd, tau, locked, gravity, &mut ws_p)?;
        let am = constrained_aba_locked_joints(model, &qm, qd, tau, locked, gravity, &mut ws_m)?;
        for r in 0..n {
            dqdd_dq[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut vp = qd.to_vec();
        let mut vm = qd.to_vec();
        vp[i] += eps;
        vm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = constrained_aba_locked_joints(model, q, &vp, tau, locked, gravity, &mut ws_p)?;
        let am = constrained_aba_locked_joints(model, q, &vm, tau, locked, gravity, &mut ws_m)?;
        for r in 0..n {
            dqdd_dqd[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut tp = tau.to_vec();
        let mut tm = tau.to_vec();
        tp[i] += eps;
        tm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = constrained_aba_locked_joints(model, q, qd, &tp, locked, gravity, &mut ws_p)?;
        let am = constrained_aba_locked_joints(model, q, qd, &tm, locked, gravity, &mut ws_m)?;
        for r in 0..n {
            dqdd_dtau[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }
    }
    Ok(DerivativeResult {
        d_out_dq: dqdd_dq,
        d_out_dv: dqdd_dqd,
        d_out_du: dqdd_dtau,
    })
}

pub fn impulse_dynamics_derivatives(
    model: &Model,
    q: &[f64],
    qd_minus: &[f64],
    contacts: &[ContactPoint],
    restitution: f64,
    _ws: &mut Workspace,
) -> Result<ImpulseDerivativeResult> {
    let n = model.nv();
    model.check_state_dims(q, qd_minus, None)?;
    let eps = 1e-6;
    let mut d_dq = vec![vec![0.0; n]; n];
    let mut d_dv = vec![vec![0.0; n]; n];
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += eps;
        qm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = apply_contact_impulses(model, &qp, qd_minus, contacts, restitution, &mut ws_p)?.qd_plus;
        let am = apply_contact_impulses(model, &qm, qd_minus, contacts, restitution, &mut ws_m)?.qd_plus;
        for r in 0..n {
            d_dq[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }

        let mut vp = qd_minus.to_vec();
        let mut vm = qd_minus.to_vec();
        vp[i] += eps;
        vm[i] -= eps;
        let mut ws_p = Workspace::new(model);
        let mut ws_m = Workspace::new(model);
        let ap = apply_contact_impulses(model, q, &vp, contacts, restitution, &mut ws_p)?.qd_plus;
        let am = apply_contact_impulses(model, q, &vm, contacts, restitution, &mut ws_m)?.qd_plus;
        for r in 0..n {
            d_dv[r][i] = (ap[r] - am[r]) / (2.0 * eps);
        }
    }

    let mut d_dr = vec![0.0; n];
    let mut ws_0 = Workspace::new(model);
    let y0 = apply_contact_impulses(model, q, qd_minus, contacts, restitution, &mut ws_0)?.qd_plus;
    let rp = restitution + eps;
    let mut ws_p = Workspace::new(model);
    let yp = apply_contact_impulses(model, q, qd_minus, contacts, rp, &mut ws_p)?.qd_plus;
    if restitution > eps {
        let rm = restitution - eps;
        let mut ws_m = Workspace::new(model);
        let ym = apply_contact_impulses(model, q, qd_minus, contacts, rm, &mut ws_m)?.qd_plus;
        for i in 0..n {
            d_dr[i] = (yp[i] - ym[i]) / (2.0 * eps);
        }
    } else {
        for i in 0..n {
            d_dr[i] = (yp[i] - y0[i]) / eps;
        }
    }

    Ok(ImpulseDerivativeResult {
        d_qd_plus_dq: d_dq,
        d_qd_plus_dqd_minus: d_dv,
        d_qd_plus_d_restitution: d_dr,
    })
}
