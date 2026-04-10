use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

use super::kinematics::forward_kinematics;

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
