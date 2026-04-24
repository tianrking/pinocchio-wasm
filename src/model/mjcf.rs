use crate::core::error::{PinocchioError, Result};
use crate::model::{
    Model, UrdfJointSpec, UrdfLinkSpec, build_tree_model_from_specs, fmt_vec3, parse_vec3_text,
    required_attr,
};
use std::collections::BTreeMap;

impl Model {
    pub fn to_mjcf_string(&self, model_name: &str) -> Result<String> {
        if self.links.is_empty() {
            return Err(PinocchioError::InvalidModel("model must not be empty"));
        }
        let mut out = String::new();
        out.push_str(&format!("<mujoco model=\"{model_name}\">\n"));
        out.push_str("  <worldbody>\n");
        self.emit_mjcf_body(0, 2, &mut out)?;
        out.push_str("  </worldbody>\n");
        out.push_str("</mujoco>\n");
        Ok(out)
    }

    fn emit_mjcf_body(&self, link_idx: usize, indent: usize, out: &mut String) -> Result<()> {
        let l = &self.links[link_idx];
        let pad = " ".repeat(indent);
        out.push_str(&format!("{pad}<body name=\"{}\" pos=\"0 0 0\">\n", l.name));
        out.push_str(&format!(
            "{pad}  <inertial pos=\"{}\" mass=\"{}\" diaginertia=\"{}\"/>\n",
            fmt_vec3([l.com_local.x, l.com_local.y, l.com_local.z]),
            l.mass,
            fmt_vec3([
                l.inertia_local_com.m[0][0],
                l.inertia_local_com.m[1][1],
                l.inertia_local_com.m[2][2]
            ])
        ));
        if link_idx != 0 {
            let joint = l
                .joint
                .as_ref()
                .ok_or(PinocchioError::InvalidModel("non-root link missing joint"))?;
            out.push_str(&format!(
                "{pad}  <joint name=\"j{}\" type=\"hinge\" axis=\"{}\" pos=\"{}\"/>\n",
                link_idx,
                fmt_vec3([joint.axis.x, joint.axis.y, joint.axis.z]),
                fmt_vec3([
                    joint.origin.translation.x,
                    joint.origin.translation.y,
                    joint.origin.translation.z
                ])
            ));
        }
        for &child in self.children_of(link_idx) {
            self.emit_mjcf_body(child, indent + 2, out)?;
        }
        out.push_str(&format!("{pad}</body>\n"));
        Ok(())
    }

    pub fn from_mjcf_str(input: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(input)
            .map_err(|_| PinocchioError::InvalidModel("failed to parse mjcf xml"))?;
        let mujoco = doc.descendants().find(|n| n.has_tag_name("mujoco")).ok_or(
            PinocchioError::InvalidModel("mjcf must contain a <mujoco> root"),
        )?;
        let worldbody = mujoco
            .children()
            .find(|n| n.has_tag_name("worldbody"))
            .ok_or(PinocchioError::InvalidModel(
                "mjcf must contain a <worldbody> element",
            ))?;

        let top_bodies: Vec<_> = worldbody
            .children()
            .filter(|n| n.has_tag_name("body"))
            .collect();
        if top_bodies.len() != 1 {
            return Err(PinocchioError::InvalidModel(
                "mjcf loader currently requires exactly one top-level body in <worldbody>",
            ));
        }

        let mut link_specs: BTreeMap<String, UrdfLinkSpec> = BTreeMap::new();
        let mut joints: Vec<UrdfJointSpec> = Vec::new();
        parse_mjcf_body_tree(top_bodies[0], None, &mut link_specs, &mut joints)?;

        build_tree_model_from_specs(link_specs, joints, "mjcf")
    }
}

fn parse_mjcf_body_tree(
    body_node: roxmltree::Node<'_, '_>,
    parent_name: Option<String>,
    link_specs: &mut BTreeMap<String, UrdfLinkSpec>,
    joints: &mut Vec<UrdfJointSpec>,
) -> Result<()> {
    let body_name = required_attr(body_node, "name")?;
    if link_specs.contains_key(&body_name) {
        return Err(PinocchioError::InvalidModel("duplicate body name in mjcf"));
    }

    let inertial = body_node.children().find(|n| n.has_tag_name("inertial"));
    let (mass, com, inertia) = if let Some(inertial_node) = inertial {
        let mass = inertial_node
            .attribute("mass")
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap_or(1.0);
        let com = inertial_node
            .attribute("pos")
            .map(parse_vec3_text)
            .transpose()?
            .unwrap_or([0.0, 0.0, 0.0]);
        let inertia = if let Some(diag) = inertial_node.attribute("diaginertia") {
            let d = parse_vec3_text(diag)?;
            [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]]
        } else {
            crate::core::math::Mat3::identity().m
        };
        (mass, com, inertia)
    } else {
        (1.0, [0.0, 0.0, 0.0], crate::core::math::Mat3::identity().m)
    };
    link_specs.insert(body_name.clone(), UrdfLinkSpec { mass, com, inertia });

    if let Some(parent) = parent_name {
        let joint_node = body_node
            .children()
            .find(|n| n.has_tag_name("joint"))
            .ok_or(PinocchioError::InvalidModel(
                "non-root mjcf body must have a hinge joint",
            ))?;
        let joint_type = joint_node.attribute("type").unwrap_or("hinge");
        if joint_type != "hinge" {
            return Err(PinocchioError::InvalidModel(
                "only hinge joints are supported in mjcf loader",
            ));
        }
        let axis = joint_node
            .attribute("axis")
            .map(parse_vec3_text)
            .transpose()?
            .unwrap_or([0.0, 0.0, 1.0]);
        let origin = if let Some(pos) = joint_node.attribute("pos") {
            parse_vec3_text(pos)?
        } else if let Some(pos) = body_node.attribute("pos") {
            parse_vec3_text(pos)?
        } else {
            [0.0, 0.0, 0.0]
        };

        joints.push(UrdfJointSpec {
            parent_link: parent,
            child_link: body_name.clone(),
            axis,
            origin,
        });
    }

    for child_body in body_node.children().filter(|n| n.has_tag_name("body")) {
        parse_mjcf_body_tree(child_body, Some(body_name.clone()), link_specs, joints)?;
    }
    Ok(())
}
