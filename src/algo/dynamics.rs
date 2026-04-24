use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Vec3};
use crate::core::spatial::*;
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

        if joint.nv() > 0 {
            let vi = model.idx_v(jidx);
            for k in 0..joint.nv() {
                tau[vi + k] = ws.force[link_idx].dot(ws.world_motion_linear[vi + k])
                    + ws.torque[link_idx].dot(ws.world_motion_angular[vi + k]);
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

/// O(n³) ABA: compute joint accelerations via CRBA + Cholesky decomposition.
/// This is kept for cross-validation against the O(n) ABA.
pub fn aba_crba(
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

/// O(n) Articulated Body Algorithm (Featherstone 2008).
///
/// Computes `qdd = M^{-1} * (tau - b)` in O(n) using three passes:
///   1. Forward pass: FK to compute positions, velocities, and bias terms (qdd=0)
///   2. Backward pass: compute articulated-body inertias and bias forces
///   3. Forward pass: compute joint accelerations
///
/// Uses 6D spatial algebra decomposed into 3x3 blocks:
///   Spatial inertia Ia = | Theta   H   |   (Theta = rotational, H = coupling, M = translational)
///                        |  H^T    M   |
///   Spatial bias   pa = | pa_rot  |   (angular and linear bias forces)
///                       | pa_lin  |
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

    if model.links.iter().skip(1).any(|link| {
        link.joint.as_ref().is_some_and(|joint| {
            !matches!(
                joint.jtype,
                JointType::Revolute | JointType::Prismatic | JointType::Fixed
            )
        })
    }) {
        return aba_crba(model, q, qd, tau, gravity, ws);
    }

    let nlinks = model.nlinks();
    let njoints = model.njoints();

    let mut articulated_inertia = vec![spatial_zero(); nlinks];
    let mut articulated_bias = vec![spatial_vec_zero(); nlinks];
    let mut d = vec![0.0; njoints];
    let mut u = vec![0.0; njoints];
    let mut u_vec = vec![spatial_vec_zero(); njoints];

    // ========================================================================
    // Pass 1: Forward kinematics (with qdd=0 to get velocities and bias terms)
    // ========================================================================
    let qdd_zero = vec![0.0; n];
    forward_kinematics(model, q, qd, &qdd_zero, gravity, ws)?;

    // Initialize articulated body inertias from rigid body properties.
    // Spatial inertia of link i about its origin:
    //   Theta = I_c + m*(|r|^2 * I3 - r*r^T)  (rotational, parallel axis theorem)
    //   H     = m * skew(r)                      (coupling)
    //   M     = m * I3                            (translational)
    // where r = r_com (CoM offset from link origin, in world frame).
    //
    // Bias force (the force needed to produce the FK-computed acceleration when qdd=0):
    //   pa_rot = torque about origin
    //   pa_lin = force on link
    for i in 0..nlinks {
        let link = &model.links[i];
        let pose = ws.world_pose[i];
        let r_com = pose.rotation.mul_vec(link.com_local); // CoM in world, relative to link origin
        let i_world = pose
            .rotation
            .mul_mat(link.inertia_local_com)
            .mul_mat(pose.rotation.transpose());

        // Apply parallel axis theorem to get inertia about link origin.
        let r_sq = r_com.dot(r_com);
        let theta = i_world.add_mat(
            Mat3::identity()
                .scale(link.mass * r_sq)
                .add_mat(Mat3::outer(r_com, r_com).scale(-link.mass)),
        );
        let h_mat = Mat3::skew(r_com).scale(link.mass);
        let m_mat = Mat3::identity().scale(link.mass);
        articulated_inertia[i] = spatial_inertia_from_blocks(theta, h_mat, m_mat);

        // Compute the bias force: the net force/torque on the link from the
        // FK-computed motion (which includes gravity and Coriolis effects
        // because qdd=0 but gravity was passed to FK).
        let iw = i_world.mul_vec(ws.omega[i]);
        let com_acc = ws.acc_origin[i]
            + ws.alpha[i].cross(r_com)
            + ws.omega[i].cross(ws.omega[i].cross(r_com));
        let f = com_acc * link.mass;
        let n_origin = i_world.mul_vec(ws.alpha[i]) + ws.omega[i].cross(iw) + r_com.cross(f);

        articulated_bias[i] = spatial_vec(n_origin, f);
    }

    // ========================================================================
    // Pass 2: Backward pass (leaves to root) — compute articulated body inertias
    // ========================================================================
    for link_idx in (1..nlinks).rev() {
        let jidx = model.link_joint(link_idx).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");

        let parent = model.links[link_idx].parent.expect("validated model");
        let r_force = ws.world_pose[link_idx].translation - ws.world_pose[parent].translation;
        let r_acc = match joint.jtype {
            JointType::Prismatic => ws.world_joint_origin[jidx] - ws.world_pose[parent].translation,
            JointType::Revolute | JointType::Fixed => r_force,
            JointType::Spherical | JointType::FreeFlyer => unreachable!(),
        };

        let (reduced_inertia, reduced_bias) = if joint.nv() == 0 {
            (articulated_inertia[link_idx], articulated_bias[link_idx])
        } else {
            let vi = model.idx_v(jidx);
            let axis = ws.world_joint_axis[jidx];
            let s_vec = match joint.jtype {
                JointType::Revolute => spatial_vec(axis, Vec3::zero()),
                JointType::Prismatic => spatial_vec(Vec3::zero(), axis),
                JointType::Fixed | JointType::Spherical | JointType::FreeFlyer => unreachable!(),
            };
            let ia_s = spatial_mat_vec(&articulated_inertia[link_idx], s_vec);
            d[jidx] = spatial_dot(s_vec, ia_s);
            u[jidx] = tau[vi] - spatial_dot(s_vec, articulated_bias[link_idx]);
            u_vec[jidx] = ia_s;

            if d[jidx].abs() < 1e-30 {
                return Err(PinocchioError::SingularMatrix);
            }
            let d_inv = 1.0 / d[jidx];
            (
                spatial_reduce(&articulated_inertia[link_idx], ia_s, d_inv),
                spatial_bias_reduce(articulated_bias[link_idx], ia_s, u[jidx] * d_inv),
            )
        };

        let shifted_inertia = transform_spatial_inertia(&reduced_inertia, r_acc, r_force);
        let shifted_bias = transform_spatial_force(reduced_bias, r_force);
        spatial_add_assign(&mut articulated_inertia[parent], &shifted_inertia);
        spatial_vec_add_assign(&mut articulated_bias[parent], shifted_bias);
    }

    // ========================================================================
    // Pass 3: Forward pass (root to leaves) — compute joint accelerations
    // ========================================================================
    let mut qdd = vec![0.0; n];

    // Track the change in spatial acceleration due to non-zero qdd.
    // The root is fixed, so its delta acceleration is zero.
    // (Gravity and Coriolis are already accounted for in the bias forces.)
    let mut delta_alpha = vec![Vec3::zero(); nlinks]; // delta angular acceleration
    let mut delta_a = vec![Vec3::zero(); nlinks]; // delta linear acceleration
    // delta_alpha[0] and delta_a[0] are already zero

    for link_idx in 1..nlinks {
        let parent = model.links[link_idx].parent.expect("validated model");
        let jidx = model.link_joint(link_idx).expect("validated model");
        let joint = model.links[link_idx]
            .joint
            .as_ref()
            .expect("validated model");

        if joint.nv() == 0 {
            let r_parent_to_child =
                ws.world_pose[link_idx].translation - ws.world_pose[parent].translation;
            delta_alpha[link_idx] = delta_alpha[parent];
            delta_a[link_idx] = delta_a[parent] + delta_alpha[parent].cross(r_parent_to_child);
            continue;
        }

        let vi = model.idx_v(jidx);
        let axis = ws.world_joint_axis[jidx];

        // The joint acceleration uses the parent delta acceleration shifted
        // to this link origin, because all articulated quantities are stored
        // at the child link origin.
        let r_parent_to_child = match joint.jtype {
            JointType::Prismatic => ws.world_joint_origin[jidx] - ws.world_pose[parent].translation,
            JointType::Revolute => {
                ws.world_pose[link_idx].translation - ws.world_pose[parent].translation
            }
            JointType::Fixed | JointType::Spherical | JointType::FreeFlyer => unreachable!(),
        };
        let base_alpha = delta_alpha[parent];
        let base_acc = delta_a[parent] + base_alpha.cross(r_parent_to_child);

        let base_spatial_acc = spatial_vec(base_alpha, base_acc);
        let qdd_j = match joint.jtype {
            JointType::Revolute => (u[jidx] - spatial_dot(u_vec[jidx], base_spatial_acc)) / d[jidx],
            JointType::Prismatic => {
                (u[jidx] - spatial_dot(u_vec[jidx], base_spatial_acc)) / d[jidx]
            }
            JointType::Fixed => unreachable!(),
            JointType::Spherical | JointType::FreeFlyer => unreachable!(),
        };

        qdd[vi] = qdd_j;

        // Propagate the delta acceleration from parent to this link.
        // Only terms that depend on qdd change; Coriolis and centripetal are constant.
        match joint.jtype {
            JointType::Revolute => {
                delta_alpha[link_idx] = base_alpha + axis * qdd_j;
                delta_a[link_idx] = base_acc;
            }
            JointType::Prismatic => {
                delta_alpha[link_idx] = base_alpha;
                delta_a[link_idx] = base_acc + axis * qdd_j;
            }
            JointType::Fixed => unreachable!(),
            JointType::Spherical | JointType::FreeFlyer => unreachable!(),
        }
    }

    Ok(qdd)
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
