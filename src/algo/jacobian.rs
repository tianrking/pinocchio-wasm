use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::Model;
use crate::model::Workspace;

use super::contact::ContactPoint;
use super::dynamics::is_ancestor;
use super::kinematics::forward_kinematics;

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

    for j in 0..model.njoints() {
        let link_of_joint = model.joint_link(j).expect("validated model");
        if !is_ancestor(model, link_of_joint, target_link) {
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
        let origin = ws.world_joint_origin[j];
        for k in 0..joint.nv() {
            let col = vi + k;
            let lin =
                ws.world_motion_linear[col] + ws.world_motion_angular[col].cross(p_target - origin);
            let ang = ws.world_motion_angular[col];
            jac[col] = lin.x;
            jac[n + col] = lin.y;
            jac[2 * n + col] = lin.z;
            jac[3 * n + col] = ang.x;
            jac[4 * n + col] = ang.y;
            jac[5 * n + col] = ang.z;
        }
    }

    Ok(jac)
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
        let row = super::contact::contact_jacobian_row(model, ws, c)?;
        let base = i * n;
        out[base..base + n].copy_from_slice(&row);
    }
    Ok(out)
}
