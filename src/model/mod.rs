pub mod json;
pub mod mjcf;
pub mod sdf;
pub mod urdf;

use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Transform, Vec3};
use crate::core::spatial::{SpatialMatrix, SpatialVector, spatial_vec_zero, spatial_zero};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,  // nq=1, nv=1: rotation around axis
    Prismatic, // nq=1, nv=1: translation along axis
    Fixed,     // nq=0, nv=0: no degrees of freedom
    Spherical, // nq=4 quaternion, nv=3 angular velocity
    FreeFlyer, // nq=7 xyz+quaternion, nv=6 linear+angular velocity
}

#[derive(Debug, Clone)]
pub struct Joint {
    pub jtype: JointType,
    pub axis: Vec3,
    pub origin: Transform,
}

impl Joint {
    pub fn revolute(axis: Vec3, origin_translation: Vec3) -> Self {
        Self {
            jtype: JointType::Revolute,
            axis,
            origin: Transform::new(Mat3::identity(), origin_translation),
        }
    }

    pub fn prismatic(axis: Vec3, origin_translation: Vec3) -> Self {
        Self {
            jtype: JointType::Prismatic,
            axis,
            origin: Transform::new(Mat3::identity(), origin_translation),
        }
    }

    pub fn fixed(origin_translation: Vec3) -> Self {
        Self {
            jtype: JointType::Fixed,
            axis: Vec3::new(0.0, 0.0, 1.0), // axis unused for fixed
            origin: Transform::new(Mat3::identity(), origin_translation),
        }
    }

    pub fn spherical(origin_translation: Vec3) -> Self {
        Self {
            jtype: JointType::Spherical,
            axis: Vec3::new(0.0, 0.0, 1.0),
            origin: Transform::new(Mat3::identity(), origin_translation),
        }
    }

    pub fn freeflyer(origin_translation: Vec3) -> Self {
        Self {
            jtype: JointType::FreeFlyer,
            axis: Vec3::new(0.0, 0.0, 1.0),
            origin: Transform::new(Mat3::identity(), origin_translation),
        }
    }

    pub fn nq(&self) -> usize {
        match self.jtype {
            JointType::Revolute | JointType::Prismatic => 1,
            JointType::Fixed => 0,
            JointType::Spherical => 4,
            JointType::FreeFlyer => 7,
        }
    }

    pub fn nv(&self) -> usize {
        match self.jtype {
            JointType::Revolute | JointType::Prismatic => 1,
            JointType::Fixed => 0,
            JointType::Spherical => 3,
            JointType::FreeFlyer => 6,
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
    joint_to_link: Vec<usize>,         // ALL joints (including fixed)
    link_to_joint: Vec<Option<usize>>, // link -> joint index
    joint_nq: Vec<usize>,              // nq for each joint
    joint_nv: Vec<usize>,              // nv for each joint
    joint_idx_v: Vec<usize>,           // velocity-space index offset for each joint
    joint_idx_q: Vec<usize>,           // config-space index offset for each joint
}

impl Model {
    pub fn new(links: Vec<Link>) -> Result<Self> {
        if links.is_empty() {
            return Err(PinocchioError::invalid_model(
                "model must contain at least one root link",
            ));
        }
        if links[0].parent.is_some() || links[0].joint.is_some() {
            return Err(PinocchioError::invalid_model(
                "link[0] must be a fixed root link",
            ));
        }

        let mut parent_to_children = vec![Vec::new(); links.len()];
        let mut joint_to_link = Vec::new();
        let mut link_to_joint = vec![None; links.len()];

        for (idx, link) in links.iter().enumerate().skip(1) {
            let parent = link.parent.ok_or(PinocchioError::invalid_model(
                "non-root link must have a parent",
            ))?;
            if parent >= idx {
                return Err(PinocchioError::invalid_model(
                    "parents must appear before children (topological order)",
                ));
            }
            if link.joint.is_none() {
                return Err(PinocchioError::invalid_model(
                    "non-root link must have a joint",
                ));
            }
            parent_to_children[parent].push(idx);
            let jidx = joint_to_link.len();
            joint_to_link.push(idx);
            link_to_joint[idx] = Some(jidx);
        }

        // Build index offsets for each joint
        let mut joint_nq = Vec::with_capacity(joint_to_link.len());
        let mut joint_nv = Vec::with_capacity(joint_to_link.len());
        let mut joint_idx_q = Vec::with_capacity(joint_to_link.len());
        let mut joint_idx_v = Vec::with_capacity(joint_to_link.len());

        let mut offset_q: usize = 0;
        let mut offset_v: usize = 0;
        for &link_idx in &joint_to_link {
            let joint = links[link_idx].joint.as_ref().expect("validated model");
            let nq = joint.nq();
            let nv = joint.nv();
            joint_nq.push(nq);
            joint_nv.push(nv);
            joint_idx_q.push(offset_q);
            joint_idx_v.push(offset_v);
            offset_q += nq;
            offset_v += nv;
        }

        Ok(Self {
            links,
            parent_to_children,
            joint_to_link,
            link_to_joint,
            joint_nq,
            joint_nv,
            joint_idx_v,
            joint_idx_q,
        })
    }

    pub fn nq(&self) -> usize {
        self.joint_nq.iter().sum()
    }

    pub fn nv(&self) -> usize {
        self.joint_nv.iter().sum()
    }

    pub fn njoints(&self) -> usize {
        self.joint_to_link.len()
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

    pub fn idx_q(&self, joint_index: usize) -> usize {
        self.joint_idx_q[joint_index]
    }

    pub fn idx_v(&self, joint_index: usize) -> usize {
        self.joint_idx_v[joint_index]
    }

    pub fn joint_type(&self, joint_index: usize) -> JointType {
        self.links[self.joint_to_link[joint_index]]
            .joint
            .as_ref()
            .unwrap()
            .jtype
    }

    pub fn check_state_dims(&self, q: &[f64], qd: &[f64], qdd: Option<&[f64]>) -> Result<()> {
        let nq = self.nq();
        let nv = self.nv();
        if q.len() != nq {
            return Err(PinocchioError::DimensionMismatch {
                expected: nq,
                got: q.len(),
            });
        }
        if qd.len() != nv {
            return Err(PinocchioError::DimensionMismatch {
                expected: nv,
                got: qd.len(),
            });
        }
        if let Some(qdd) = qdd
            && qdd.len() != nv
        {
            return Err(PinocchioError::DimensionMismatch {
                expected: nv,
                got: qdd.len(),
            });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Shared helper types used by urdf, sdf, and mjcf loaders
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct UrdfLinkSpec {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [[f64; 3]; 3],
}

#[derive(Debug, Clone)]
pub(crate) struct UrdfJointSpec {
    pub parent_link: String,
    pub child_link: String,
    pub axis: [f64; 3],
    pub origin: [f64; 3],
    pub jtype: JointType,
}

// ---------------------------------------------------------------------------
// Shared helper functions used by urdf, sdf, and/or mjcf loaders
// ---------------------------------------------------------------------------

pub(crate) fn required_attr(node: roxmltree::Node<'_, '_>, name: &str) -> Result<String> {
    node.attribute(name)
        .map(str::to_string)
        .ok_or(PinocchioError::invalid_model(
            "missing required urdf attribute",
        ))
}

pub(crate) fn parse_xyz_node(node: roxmltree::Node<'_, '_>) -> Result<[f64; 3]> {
    let raw = node.attribute("xyz").ok_or(PinocchioError::invalid_model(
        "missing xyz attribute in urdf node",
    ))?;
    parse_vec3_text(raw)
}

pub(crate) fn parse_vec3_text(raw: &str) -> Result<[f64; 3]> {
    let vals: Vec<f64> = raw
        .split_whitespace()
        .map(|v| v.parse::<f64>())
        .collect::<std::result::Result<Vec<_>, _>>()
        .map_err(|_| PinocchioError::invalid_model("invalid numeric value in urdf"))?;
    if vals.len() != 3 {
        return Err(PinocchioError::invalid_model(
            "expected xyz with exactly 3 values",
        ));
    }
    Ok([vals[0], vals[1], vals[2]])
}

pub(crate) fn parse_urdf_inertia_node(node: roxmltree::Node<'_, '_>) -> Result<[[f64; 3]; 3]> {
    let get = |name: &str| -> Result<f64> {
        node.attribute(name)
            .ok_or(PinocchioError::invalid_model("missing inertia attribute"))?
            .parse::<f64>()
            .map_err(|_| PinocchioError::invalid_model("invalid inertia numeric value"))
    };
    let ixx = get("ixx")?;
    let iyy = get("iyy")?;
    let izz = get("izz")?;
    let ixy = get("ixy").unwrap_or(0.0);
    let ixz = get("ixz").unwrap_or(0.0);
    let iyz = get("iyz").unwrap_or(0.0);
    Ok([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
}

pub(crate) fn parse_sdf_inertia_node(node: roxmltree::Node<'_, '_>) -> Result<[[f64; 3]; 3]> {
    let get = |name: &str| -> Result<f64> {
        text_of_child(node, name)
            .ok_or(PinocchioError::invalid_model("missing sdf inertia element"))?
            .parse::<f64>()
            .map_err(|_| PinocchioError::invalid_model("invalid sdf inertia numeric value"))
    };
    let ixx = get("ixx")?;
    let iyy = get("iyy")?;
    let izz = get("izz")?;
    let ixy = get("ixy").unwrap_or(0.0);
    let ixz = get("ixz").unwrap_or(0.0);
    let iyz = get("iyz").unwrap_or(0.0);
    Ok([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
}

pub(crate) fn text_of_child(node: roxmltree::Node<'_, '_>, child_name: &str) -> Option<String> {
    node.children()
        .find(|n| n.has_tag_name(child_name))
        .and_then(|n| n.text())
        .map(|s| s.trim().to_string())
}

pub(crate) fn parse_pose_xyz_text(raw: String) -> Result<[f64; 3]> {
    let vals: Vec<f64> = raw
        .split_whitespace()
        .map(|v| v.parse::<f64>())
        .collect::<std::result::Result<Vec<_>, _>>()
        .map_err(|_| PinocchioError::invalid_model("invalid numeric value in pose"))?;
    if vals.len() < 3 {
        return Err(PinocchioError::invalid_model(
            "pose must contain at least xyz values",
        ));
    }
    Ok([vals[0], vals[1], vals[2]])
}

pub(crate) fn fmt_vec3(v: [f64; 3]) -> String {
    format!("{} {} {}", v[0], v[1], v[2])
}

pub(crate) fn fmt_pose_xyz(v: [f64; 3]) -> String {
    format!("{} {} {} 0 0 0", v[0], v[1], v[2])
}

pub(crate) fn build_tree_model_from_specs(
    link_specs: BTreeMap<String, UrdfLinkSpec>,
    joints: Vec<UrdfJointSpec>,
    source_name: &str,
) -> Result<Model> {
    let mut children_set: BTreeSet<String> = BTreeSet::new();
    for j in &joints {
        children_set.insert(j.child_link.clone());
    }
    let roots: Vec<String> = link_specs
        .keys()
        .filter(|k| !children_set.contains(*k))
        .cloned()
        .collect();
    if roots.len() != 1 {
        return Err(PinocchioError::invalid_model(
            "loader requires exactly one root link",
        ));
    }
    let root_name = roots[0].clone();

    let mut adjacency: BTreeMap<String, Vec<UrdfJointSpec>> = BTreeMap::new();
    for j in joints {
        adjacency.entry(j.parent_link.clone()).or_default().push(j);
    }
    for children in adjacency.values_mut() {
        children.sort_by(|a, b| a.child_link.cmp(&b.child_link));
    }

    let root_spec = link_specs
        .get(&root_name)
        .ok_or(PinocchioError::invalid_model("root link missing"))?;
    let mut links = vec![Link::root(
        root_name.clone(),
        root_spec.mass,
        Vec3::new(root_spec.com[0], root_spec.com[1], root_spec.com[2]),
        Mat3::new(root_spec.inertia),
    )];
    let mut index_of: BTreeMap<String, usize> = BTreeMap::new();
    index_of.insert(root_name.clone(), 0);

    let mut queue = VecDeque::new();
    queue.push_back(root_name);

    while let Some(parent_name) = queue.pop_front() {
        let parent_idx = *index_of
            .get(&parent_name)
            .ok_or(PinocchioError::invalid_model("parent index missing"))?;
        let Some(children) = adjacency.get(&parent_name) else {
            continue;
        };
        for joint_spec in children {
            if index_of.contains_key(&joint_spec.child_link) {
                return Err(PinocchioError::invalid_model(
                    "graph has duplicate child links or cycles",
                ));
            }
            let child_spec =
                link_specs
                    .get(&joint_spec.child_link)
                    .ok_or(PinocchioError::invalid_model(
                        "joint references missing child link",
                    ))?;
            let joint = match joint_spec.jtype {
                JointType::Revolute => Joint::revolute(
                    Vec3::new(joint_spec.axis[0], joint_spec.axis[1], joint_spec.axis[2]),
                    Vec3::new(
                        joint_spec.origin[0],
                        joint_spec.origin[1],
                        joint_spec.origin[2],
                    ),
                ),
                JointType::Prismatic => Joint::prismatic(
                    Vec3::new(joint_spec.axis[0], joint_spec.axis[1], joint_spec.axis[2]),
                    Vec3::new(
                        joint_spec.origin[0],
                        joint_spec.origin[1],
                        joint_spec.origin[2],
                    ),
                ),
                JointType::Fixed => Joint::fixed(Vec3::new(
                    joint_spec.origin[0],
                    joint_spec.origin[1],
                    joint_spec.origin[2],
                )),
                JointType::Spherical => Joint::spherical(Vec3::new(
                    joint_spec.origin[0],
                    joint_spec.origin[1],
                    joint_spec.origin[2],
                )),
                JointType::FreeFlyer => Joint::freeflyer(Vec3::new(
                    joint_spec.origin[0],
                    joint_spec.origin[1],
                    joint_spec.origin[2],
                )),
            };
            links.push(Link::child(
                joint_spec.child_link.clone(),
                parent_idx,
                joint,
                child_spec.mass,
                Vec3::new(child_spec.com[0], child_spec.com[1], child_spec.com[2]),
                Mat3::new(child_spec.inertia),
            ));
            let new_idx = links.len() - 1;
            index_of.insert(joint_spec.child_link.clone(), new_idx);
            queue.push_back(joint_spec.child_link.clone());
        }
    }

    if index_of.len() != link_specs.len() {
        return Err(PinocchioError::invalid_model(match source_name {
            "urdf" => "urdf graph is disconnected from root",
            "sdf" => "sdf graph is disconnected from root",
            "mjcf" => "mjcf graph is disconnected from root",
            _ => "graph is disconnected from root",
        }));
    }

    Model::new(links)
}

// ---------------------------------------------------------------------------
// Workspace
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Workspace {
    pub world_pose: Vec<Transform>,
    pub world_joint_axis: Vec<Vec3>,
    pub world_joint_origin: Vec<Vec3>,
    pub world_motion_linear: Vec<Vec3>,
    pub world_motion_angular: Vec<Vec3>,
    pub omega: Vec<Vec3>,
    pub vel_origin: Vec<Vec3>,
    pub alpha: Vec<Vec3>,
    pub acc_origin: Vec<Vec3>,
    pub force: Vec<Vec3>,
    pub torque: Vec<Vec3>,
    pub(crate) aba_inertia: Vec<SpatialMatrix>,
    pub(crate) aba_bias: Vec<SpatialVector>,
    pub(crate) aba_d: Vec<SpatialMatrix>,
    pub(crate) aba_u: Vec<SpatialVector>,
    pub(crate) aba_u_cols: Vec<SpatialMatrix>,
}

impl Workspace {
    pub fn new(model: &Model) -> Self {
        let nlinks = model.nlinks();
        let njoints = model.njoints();
        Self {
            world_pose: vec![Transform::identity(); nlinks],
            world_joint_axis: vec![Vec3::zero(); njoints],
            world_joint_origin: vec![Vec3::zero(); njoints],
            world_motion_linear: vec![Vec3::zero(); model.nv()],
            world_motion_angular: vec![Vec3::zero(); model.nv()],
            omega: vec![Vec3::zero(); nlinks],
            vel_origin: vec![Vec3::zero(); nlinks],
            alpha: vec![Vec3::zero(); nlinks],
            acc_origin: vec![Vec3::zero(); nlinks],
            force: vec![Vec3::zero(); nlinks],
            torque: vec![Vec3::zero(); nlinks],
            aba_inertia: vec![spatial_zero(); nlinks],
            aba_bias: vec![spatial_vec_zero(); nlinks],
            aba_d: vec![spatial_zero(); njoints],
            aba_u: vec![spatial_vec_zero(); njoints],
            aba_u_cols: vec![spatial_zero(); njoints],
        }
    }
}
