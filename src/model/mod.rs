use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Transform, Vec3};

#[derive(Debug, Clone)]
pub struct Joint {
    pub axis: Vec3,
    pub origin: Transform,
}

impl Joint {
    pub fn revolute(axis: Vec3, origin_translation: Vec3) -> Self {
        Self {
            axis,
            origin: Transform::new(Mat3::identity(), origin_translation),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Link {
    pub name: String,
    pub parent: Option<usize>,
    pub mass: f64,
    pub com_local: Vec3,
    pub inertia_local_com: Mat3,
    pub joint: Option<Joint>,
}

impl Link {
    pub fn root(
        name: impl Into<String>,
        mass: f64,
        com_local: Vec3,
        inertia_local_com: Mat3,
    ) -> Self {
        Self {
            name: name.into(),
            parent: None,
            mass,
            com_local,
            inertia_local_com,
            joint: None,
        }
    }

    pub fn child(
        name: impl Into<String>,
        parent: usize,
        joint: Joint,
        mass: f64,
        com_local: Vec3,
        inertia_local_com: Mat3,
    ) -> Self {
        Self {
            name: name.into(),
            parent: Some(parent),
            mass,
            com_local,
            inertia_local_com,
            joint: Some(joint),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Model {
    pub links: Vec<Link>,
    parent_to_children: Vec<Vec<usize>>,
    joint_to_link: Vec<usize>,
    link_to_joint: Vec<Option<usize>>,
}

impl Model {
    pub fn new(links: Vec<Link>) -> Result<Self> {
        if links.is_empty() {
            return Err(PinocchioError::InvalidModel(
                "model must contain at least one root link",
            ));
        }
        if links[0].parent.is_some() || links[0].joint.is_some() {
            return Err(PinocchioError::InvalidModel(
                "link[0] must be a fixed root link",
            ));
        }

        let mut parent_to_children = vec![Vec::new(); links.len()];
        let mut joint_to_link = Vec::new();
        let mut link_to_joint = vec![None; links.len()];

        for (idx, link) in links.iter().enumerate().skip(1) {
            let parent = link.parent.ok_or(PinocchioError::InvalidModel(
                "non-root link must have a parent",
            ))?;
            if parent >= idx {
                return Err(PinocchioError::InvalidModel(
                    "parents must appear before children (topological order)",
                ));
            }
            if link.joint.is_none() {
                return Err(PinocchioError::InvalidModel(
                    "non-root link must have a joint",
                ));
            }
            parent_to_children[parent].push(idx);
            let jidx = joint_to_link.len();
            joint_to_link.push(idx);
            link_to_joint[idx] = Some(jidx);
        }

        Ok(Self {
            links,
            parent_to_children,
            joint_to_link,
            link_to_joint,
        })
    }

    pub fn nq(&self) -> usize {
        self.joint_to_link.len()
    }

    pub fn nv(&self) -> usize {
        self.nq()
    }

    pub fn nlinks(&self) -> usize {
        self.links.len()
    }

    pub fn children_of(&self, parent: usize) -> &[usize] {
        &self.parent_to_children[parent]
    }

    pub fn joint_link(&self, joint_index: usize) -> Option<usize> {
        self.joint_to_link.get(joint_index).copied()
    }

    pub fn link_joint(&self, link_index: usize) -> Option<usize> {
        self.link_to_joint.get(link_index).and_then(|x| *x)
    }

    pub fn check_state_dims(&self, q: &[f64], qd: &[f64], qdd: Option<&[f64]>) -> Result<()> {
        let n = self.nv();
        if q.len() != n {
            return Err(PinocchioError::DimensionMismatch {
                expected: n,
                got: q.len(),
            });
        }
        if qd.len() != n {
            return Err(PinocchioError::DimensionMismatch {
                expected: n,
                got: qd.len(),
            });
        }
        if let Some(qdd) = qdd
            && qdd.len() != n
        {
            return Err(PinocchioError::DimensionMismatch {
                expected: n,
                got: qdd.len(),
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct Workspace {
    pub world_pose: Vec<Transform>,
    pub world_joint_axis: Vec<Vec3>,
    pub world_joint_origin: Vec<Vec3>,
    pub omega: Vec<Vec3>,
    pub vel_origin: Vec<Vec3>,
    pub alpha: Vec<Vec3>,
    pub acc_origin: Vec<Vec3>,
    pub force: Vec<Vec3>,
    pub torque: Vec<Vec3>,
}

impl Workspace {
    pub fn new(model: &Model) -> Self {
        let nlinks = model.nlinks();
        Self {
            world_pose: vec![Transform::identity(); nlinks],
            world_joint_axis: vec![Vec3::zero(); model.nv()],
            world_joint_origin: vec![Vec3::zero(); model.nv()],
            omega: vec![Vec3::zero(); nlinks],
            vel_origin: vec![Vec3::zero(); nlinks],
            alpha: vec![Vec3::zero(); nlinks],
            acc_origin: vec![Vec3::zero(); nlinks],
            force: vec![Vec3::zero(); nlinks],
            torque: vec![Vec3::zero(); nlinks],
        }
    }
}
