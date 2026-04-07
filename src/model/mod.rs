use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Transform, Vec3};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

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

    pub fn from_json_str(input: &str) -> Result<Self> {
        let spec: JsonModelSpec = serde_json::from_str(input)
            .map_err(|_| PinocchioError::InvalidModel("failed to parse model json"))?;
        if spec.links.is_empty() {
            return Err(PinocchioError::InvalidModel(
                "json model must contain at least one link",
            ));
        }

        let mut links = Vec::with_capacity(spec.links.len());
        for (idx, l) in spec.links.into_iter().enumerate() {
            let com = Vec3::new(l.com[0], l.com[1], l.com[2]);
            let inertia = Mat3::new(l.inertia);
            if idx == 0 {
                if l.parent.is_some() || l.joint.is_some() {
                    return Err(PinocchioError::InvalidModel(
                        "root json link must not have parent/joint",
                    ));
                }
                links.push(Link::root(l.name, l.mass, com, inertia));
                continue;
            }

            let parent = l.parent.ok_or(PinocchioError::InvalidModel(
                "non-root json link must have parent",
            ))?;
            let joint = l.joint.ok_or(PinocchioError::InvalidModel(
                "non-root json link must have revolute joint",
            ))?;
            let joint = Joint::revolute(
                Vec3::new(joint.axis[0], joint.axis[1], joint.axis[2]),
                Vec3::new(joint.origin[0], joint.origin[1], joint.origin[2]),
            );
            links.push(Link::child(l.name, parent, joint, l.mass, com, inertia));
        }

        Model::new(links)
    }

    pub fn from_urdf_str(input: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(input)
            .map_err(|_| PinocchioError::InvalidModel("failed to parse urdf xml"))?;
        let robot = doc.descendants().find(|n| n.has_tag_name("robot")).ok_or(
            PinocchioError::InvalidModel("urdf must contain <robot> root"),
        )?;

        let mut link_specs: BTreeMap<String, UrdfLinkSpec> = BTreeMap::new();
        for link_node in robot.children().filter(|n| n.has_tag_name("link")) {
            let name = required_attr(link_node, "name")?;
            let inertial = link_node.children().find(|n| n.has_tag_name("inertial"));
            let (mass, com, inertia) = if let Some(inertial_node) = inertial {
                let mass_node = inertial_node.children().find(|n| n.has_tag_name("mass"));
                let mass = mass_node
                    .and_then(|n| n.attribute("value"))
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(1.0);
                let com = inertial_node
                    .children()
                    .find(|n| n.has_tag_name("origin"))
                    .map(parse_xyz_node)
                    .transpose()?
                    .unwrap_or([0.0, 0.0, 0.0]);
                let inertia = inertial_node
                    .children()
                    .find(|n| n.has_tag_name("inertia"))
                    .map(parse_urdf_inertia_node)
                    .transpose()?
                    .unwrap_or(Mat3::identity().m);
                (mass, com, inertia)
            } else {
                (1.0, [0.0, 0.0, 0.0], Mat3::identity().m)
            };

            link_specs.insert(name, UrdfLinkSpec { mass, com, inertia });
        }
        if link_specs.is_empty() {
            return Err(PinocchioError::InvalidModel(
                "urdf must contain at least one <link>",
            ));
        }

        let mut joints: Vec<UrdfJointSpec> = Vec::new();
        for joint_node in robot.children().filter(|n| n.has_tag_name("joint")) {
            let joint_type = required_attr(joint_node, "type")?;
            if joint_type != "revolute" && joint_type != "continuous" {
                return Err(PinocchioError::InvalidModel(
                    "only revolute/continuous joints are supported in urdf loader",
                ));
            }
            let parent_link = joint_node
                .children()
                .find(|n| n.has_tag_name("parent"))
                .and_then(|n| n.attribute("link"))
                .ok_or(PinocchioError::InvalidModel(
                    "joint must contain <parent link=\"...\">",
                ))?
                .to_string();
            let child_link = joint_node
                .children()
                .find(|n| n.has_tag_name("child"))
                .and_then(|n| n.attribute("link"))
                .ok_or(PinocchioError::InvalidModel(
                    "joint must contain <child link=\"...\">",
                ))?
                .to_string();
            let origin = joint_node
                .children()
                .find(|n| n.has_tag_name("origin"))
                .map(parse_xyz_node)
                .transpose()?
                .unwrap_or([0.0, 0.0, 0.0]);
            let axis = joint_node
                .children()
                .find(|n| n.has_tag_name("axis"))
                .map(parse_xyz_node)
                .transpose()?
                .unwrap_or([0.0, 0.0, 1.0]);
            joints.push(UrdfJointSpec {
                parent_link,
                child_link,
                origin,
                axis,
            });
        }

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
            return Err(PinocchioError::InvalidModel(
                "urdf loader requires exactly one root link",
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
            .ok_or(PinocchioError::InvalidModel("root link missing"))?;
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
                .ok_or(PinocchioError::InvalidModel("parent index missing"))?;
            let Some(children) = adjacency.get(&parent_name) else {
                continue;
            };
            for joint_spec in children {
                if index_of.contains_key(&joint_spec.child_link) {
                    return Err(PinocchioError::InvalidModel(
                        "urdf has duplicate child links or cycles",
                    ));
                }
                let child_spec =
                    link_specs
                        .get(&joint_spec.child_link)
                        .ok_or(PinocchioError::InvalidModel(
                            "joint references missing child link",
                        ))?;
                let joint = Joint::revolute(
                    Vec3::new(joint_spec.axis[0], joint_spec.axis[1], joint_spec.axis[2]),
                    Vec3::new(
                        joint_spec.origin[0],
                        joint_spec.origin[1],
                        joint_spec.origin[2],
                    ),
                );
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
            return Err(PinocchioError::InvalidModel(
                "urdf graph is disconnected from root",
            ));
        }

        Model::new(links)
    }
}

#[derive(Debug, Deserialize)]
struct JsonModelSpec {
    links: Vec<JsonLinkSpec>,
}

#[derive(Debug, Deserialize)]
struct JsonLinkSpec {
    name: String,
    parent: Option<usize>,
    mass: f64,
    com: [f64; 3],
    inertia: [[f64; 3]; 3],
    joint: Option<JsonJointSpec>,
}

#[derive(Debug, Deserialize)]
struct JsonJointSpec {
    axis: [f64; 3],
    origin: [f64; 3],
}

#[derive(Debug, Clone)]
struct UrdfLinkSpec {
    mass: f64,
    com: [f64; 3],
    inertia: [[f64; 3]; 3],
}

#[derive(Debug, Clone)]
struct UrdfJointSpec {
    parent_link: String,
    child_link: String,
    axis: [f64; 3],
    origin: [f64; 3],
}

fn required_attr(node: roxmltree::Node<'_, '_>, name: &str) -> Result<String> {
    node.attribute(name)
        .map(str::to_string)
        .ok_or(PinocchioError::InvalidModel(
            "missing required urdf attribute",
        ))
}

fn parse_xyz_node(node: roxmltree::Node<'_, '_>) -> Result<[f64; 3]> {
    let raw = node.attribute("xyz").ok_or(PinocchioError::InvalidModel(
        "missing xyz attribute in urdf node",
    ))?;
    parse_vec3_text(raw)
}

fn parse_vec3_text(raw: &str) -> Result<[f64; 3]> {
    let vals: Vec<f64> = raw
        .split_whitespace()
        .map(|v| v.parse::<f64>())
        .collect::<std::result::Result<Vec<_>, _>>()
        .map_err(|_| PinocchioError::InvalidModel("invalid numeric value in urdf"))?;
    if vals.len() != 3 {
        return Err(PinocchioError::InvalidModel(
            "expected xyz with exactly 3 values",
        ));
    }
    Ok([vals[0], vals[1], vals[2]])
}

fn parse_urdf_inertia_node(node: roxmltree::Node<'_, '_>) -> Result<[[f64; 3]; 3]> {
    let get = |name: &str| -> Result<f64> {
        node.attribute(name)
            .ok_or(PinocchioError::InvalidModel("missing inertia attribute"))?
            .parse::<f64>()
            .map_err(|_| PinocchioError::InvalidModel("invalid inertia numeric value"))
    };
    let ixx = get("ixx")?;
    let iyy = get("iyy")?;
    let izz = get("izz")?;
    let ixy = get("ixy").unwrap_or(0.0);
    let ixz = get("ixz").unwrap_or(0.0);
    let iyz = get("iyz").unwrap_or(0.0);
    Ok([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
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
