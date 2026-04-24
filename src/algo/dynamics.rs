use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{JointType, Model, Workspace};

use super::kinematics::forward_kinematics;

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

    let n = model.nv();
    let mut tau = vec![0.0; n];
    for link_idx in (1..model.nlinks()).rev() {
        let jidx = model.link_joint(link_idx).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");

        // Only compute tau for joints with velocity DOFs
        if joint.nv() > 0 {
            let vi = model.idx_v(jidx);
            let axis = ws.world_joint_axis[jidx];
            match joint.jtype {
                JointType::Revolute => {
                    tau[vi] = ws.torque[link_idx].dot(axis);
                }
                JointType::Prismatic => {
                    tau[vi] = ws.force[link_idx].dot(axis);
                }
                JointType::Fixed => {}
            }
        }

        // Always propagate forces/torques to parent
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

pub fn cholesky_solve(a: &[Vec<f64>], b: &[f64]) -> Result<Vec<f64>> {
    let n = a.len();
    if n == 0 || b.len() != n {
        return Err(PinocchioError::DimensionMismatch {
            expected: n,
            got: b.len(),
        });
    }
    for row in a {
        if row.len() != n {
            return Err(PinocchioError::invalid_model("matrix must be square"));
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

pub fn is_ancestor(model: &Model, maybe_ancestor: usize, mut node: usize) -> bool {
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
