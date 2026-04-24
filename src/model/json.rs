use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Vec3};
use crate::model::{Joint, JointType, Link, Model};
use serde::{Deserialize, Serialize};

impl Model {
    pub fn from_json_str(input: &str) -> Result<Self> {
        let spec: JsonModelSpec = serde_json::from_str(input)
            .map_err(|_| PinocchioError::invalid_model("failed to parse model json"))?;
        if spec.links.is_empty() {
            return Err(PinocchioError::invalid_model(
                "json model must contain at least one link",
            ));
        }

        let mut links = Vec::with_capacity(spec.links.len());
        for (idx, l) in spec.links.into_iter().enumerate() {
            let com = Vec3::new(l.com[0], l.com[1], l.com[2]);
            let inertia = Mat3::new(l.inertia);
            if idx == 0 {
                if l.parent.is_some() || l.joint.is_some() {
                    return Err(PinocchioError::invalid_model(
                        "root json link must not have parent/joint",
                    ));
                }
                links.push(Link::root(l.name, l.mass, com, inertia));
                continue;
            }

            let parent = l.parent.ok_or(PinocchioError::invalid_model(
                "non-root json link must have parent",
            ))?;
            let joint_spec = l.joint.ok_or(PinocchioError::invalid_model(
                "non-root json link must have joint",
            ))?;
            let jtype = match joint_spec.jtype.as_str() {
                "prismatic" => JointType::Prismatic,
                "fixed" => JointType::Fixed,
                _ => JointType::Revolute, // default for backward compat
            };
            let joint = match jtype {
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
            };
            links.push(Link::child(l.name, parent, joint, l.mass, com, inertia));
        }

        Model::new(links)
    }

    pub fn to_json_string(&self) -> Result<String> {
        let mut links = Vec::with_capacity(self.links.len());
        for (idx, l) in self.links.iter().enumerate() {
            let joint = if idx == 0 {
                None
            } else {
                l.joint.as_ref().map(|j| {
                    let jtype_str = match j.jtype {
                        JointType::Revolute => "revolute",
                        JointType::Prismatic => "prismatic",
                        JointType::Fixed => "fixed",
                    };
                    JsonJointSpec {
                        axis: [j.axis.x, j.axis.y, j.axis.z],
                        origin: [
                            j.origin.translation.x,
                            j.origin.translation.y,
                            j.origin.translation.z,
                        ],
                        jtype: jtype_str.to_string(),
                    }
                })
            };
            links.push(JsonLinkSpec {
                name: l.name.clone(),
                parent: l.parent,
                mass: l.mass,
                com: [l.com_local.x, l.com_local.y, l.com_local.z],
                inertia: l.inertia_local_com.m,
                joint,
            });
        }
        serde_json::to_string_pretty(&JsonModelSpec { links })
            .map_err(|_| PinocchioError::invalid_model("failed to serialize model json"))
    }
}

#[derive(Debug, Deserialize, Serialize)]
struct JsonModelSpec {
    links: Vec<JsonLinkSpec>,
}

#[derive(Debug, Deserialize, Serialize)]
struct JsonLinkSpec {
    name: String,
    parent: Option<usize>,
    mass: f64,
    com: [f64; 3],
    inertia: [[f64; 3]; 3],
    joint: Option<JsonJointSpec>,
}

fn default_joint_type() -> String {
    "revolute".to_string()
}

#[derive(Debug, Deserialize, Serialize)]
struct JsonJointSpec {
    axis: [f64; 3],
    origin: [f64; 3],
    #[serde(default = "default_joint_type")]
    jtype: String,
}
