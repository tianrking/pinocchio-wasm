use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Vec3};
use crate::model::{Joint, Link, Model};
use serde::{Deserialize, Serialize};

impl Model {
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

    pub fn to_json_string(&self) -> Result<String> {
        let mut links = Vec::with_capacity(self.links.len());
        for (idx, l) in self.links.iter().enumerate() {
            let joint = if idx == 0 {
                None
            } else {
                l.joint.as_ref().map(|j| JsonJointSpec {
                    axis: [j.axis.x, j.axis.y, j.axis.z],
                    origin: [
                        j.origin.translation.x,
                        j.origin.translation.y,
                        j.origin.translation.z,
                    ],
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
            .map_err(|_| PinocchioError::InvalidModel("failed to serialize model json"))
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

#[derive(Debug, Deserialize, Serialize)]
struct JsonJointSpec {
    axis: [f64; 3],
    origin: [f64; 3],
}
