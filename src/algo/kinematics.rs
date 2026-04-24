use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Transform, Vec3};
use crate::model::{JointType, Model, Workspace};

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

        let joint_frame_rotation = parent_pose.rotation.mul_mat(joint.origin.rotation);
        let joint_origin_world = parent_pose.transform_point(joint.origin.translation);
        let axis_world = joint_frame_rotation.mul_vec(joint.axis);

        let (link_rotation, link_translation) = match joint.jtype {
            JointType::Revolute => {
                let vi = model.idx_v(jidx);
                let q_j = q[vi];
                let rot_delta = Mat3::from_axis_angle(joint.axis, q_j);
                (joint_frame_rotation.mul_mat(rot_delta), joint_origin_world)
            }
            JointType::Prismatic => {
                let vi = model.idx_v(jidx);
                let q_j = q[vi];
                (joint_frame_rotation, joint_origin_world + axis_world * q_j)
            }
            JointType::Fixed => (joint_frame_rotation, joint_origin_world),
        };

        ws.world_pose[link_idx] = Transform::new(link_rotation, link_translation);
        ws.world_joint_axis[jidx] = axis_world;
        ws.world_joint_origin[jidx] = joint_origin_world;

        let r_parent_to_joint = joint_origin_world - parent_pose.translation;

        match joint.jtype {
            JointType::Revolute => {
                let vi = model.idx_v(jidx);
                let qd_j = qd[vi];
                let qdd_j = qdd[vi];
                ws.omega[link_idx] = parent_w + axis_world * qd_j;
                ws.vel_origin[link_idx] = parent_v + parent_w.cross(r_parent_to_joint);
                ws.alpha[link_idx] =
                    parent_alpha + axis_world * qdd_j + parent_w.cross(axis_world * qd_j);
                ws.acc_origin[link_idx] = parent_acc
                    + parent_alpha.cross(r_parent_to_joint)
                    + parent_w.cross(parent_w.cross(r_parent_to_joint));
            }
            JointType::Prismatic => {
                let vi = model.idx_v(jidx);
                let qd_j = qd[vi];
                let qdd_j = qdd[vi];
                ws.omega[link_idx] = parent_w;
                ws.vel_origin[link_idx] =
                    parent_v + parent_w.cross(r_parent_to_joint) + axis_world * qd_j;
                ws.alpha[link_idx] = parent_alpha;
                ws.acc_origin[link_idx] = parent_acc
                    + parent_alpha.cross(r_parent_to_joint)
                    + parent_w.cross(parent_w.cross(r_parent_to_joint))
                    + parent_w.cross(axis_world * qd_j) * 2.0
                    + axis_world * qdd_j;
            }
            JointType::Fixed => {
                ws.omega[link_idx] = parent_w;
                ws.vel_origin[link_idx] = parent_v + parent_w.cross(r_parent_to_joint);
                ws.alpha[link_idx] = parent_alpha;
                ws.acc_origin[link_idx] = parent_acc
                    + parent_alpha.cross(r_parent_to_joint)
                    + parent_w.cross(parent_w.cross(r_parent_to_joint));
            }
        }
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
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    if q_batch.len() != expected_q {
        return Err(PinocchioError::DimensionMismatch {
            expected: expected_q,
            got: q_batch.len(),
        });
    }
    let expected_t = batch_size
        .checked_mul(nl)
        .and_then(|x| x.checked_mul(3))
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
    let expected_r = batch_size
        .checked_mul(nl)
        .and_then(|x| x.checked_mul(9))
        .ok_or(PinocchioError::invalid_model("batch size overflow"))?;
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
