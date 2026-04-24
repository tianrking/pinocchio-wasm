use crate::core::error::{PinocchioError, Result};
use crate::core::math::Mat3;
use crate::model::{
    JointType, Model, UrdfJointSpec, UrdfLinkSpec, build_tree_model_from_specs, fmt_vec3,
    parse_urdf_inertia_node, parse_xyz_node, required_attr,
};
use std::collections::BTreeMap;

impl Model {
    pub fn to_urdf_string(&self, robot_name: &str) -> Result<String> {
        if self.links.is_empty() {
            return Err(PinocchioError::invalid_model("model must not be empty"));
        }
        let mut out = String::new();
        out.push_str(&format!("<robot name=\"{robot_name}\">\n"));
        for l in &self.links {
            out.push_str(&format!("  <link name=\"{}\">\n", l.name));
            out.push_str("    <inertial>\n");
            out.push_str(&format!(
                "      <origin xyz=\"{}\"/>\n",
                fmt_vec3([l.com_local.x, l.com_local.y, l.com_local.z])
            ));
            out.push_str(&format!("      <mass value=\"{}\"/>\n", l.mass));
            out.push_str(&format!(
                "      <inertia ixx=\"{}\" ixy=\"{}\" ixz=\"{}\" iyy=\"{}\" iyz=\"{}\" izz=\"{}\"/>\n",
                l.inertia_local_com.m[0][0],
                l.inertia_local_com.m[0][1],
                l.inertia_local_com.m[0][2],
                l.inertia_local_com.m[1][1],
                l.inertia_local_com.m[1][2],
                l.inertia_local_com.m[2][2]
            ));
            out.push_str("    </inertial>\n");
            out.push_str("  </link>\n");
        }
        for (idx, l) in self.links.iter().enumerate().skip(1) {
            let parent = l.parent.ok_or(PinocchioError::invalid_model(
                "non-root link missing parent",
            ))?;
            let joint = l
                .joint
                .as_ref()
                .ok_or(PinocchioError::invalid_model("non-root link missing joint"))?;
            let type_str = match joint.jtype {
                JointType::Revolute => "revolute",
                JointType::Prismatic => "prismatic",
                JointType::Fixed => "fixed",
                JointType::Spherical => "ball",
                JointType::FreeFlyer => "floating",
            };
            out.push_str(&format!("  <joint name=\"j{idx}\" type=\"{type_str}\">\n"));
            out.push_str(&format!(
                "    <parent link=\"{}\"/>\n",
                self.links[parent].name
            ));
            out.push_str(&format!("    <child link=\"{}\"/>\n", l.name));
            out.push_str(&format!(
                "    <origin xyz=\"{}\"/>\n",
                fmt_vec3([
                    joint.origin.translation.x,
                    joint.origin.translation.y,
                    joint.origin.translation.z
                ])
            ));
            if matches!(joint.jtype, JointType::Revolute | JointType::Prismatic) {
                out.push_str(&format!(
                    "    <axis xyz=\"{}\"/>\n",
                    fmt_vec3([joint.axis.x, joint.axis.y, joint.axis.z])
                ));
            }
            out.push_str("  </joint>\n");
        }
        out.push_str("</robot>\n");
        Ok(out)
    }

    pub fn from_urdf_str(input: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(input)
            .map_err(|_| PinocchioError::invalid_model("failed to parse urdf xml"))?;
        let robot = doc.descendants().find(|n| n.has_tag_name("robot")).ok_or(
            PinocchioError::invalid_model("urdf must contain <robot> root"),
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
            return Err(PinocchioError::invalid_model(
                "urdf must contain at least one <link>",
            ));
        }

        let mut joints: Vec<UrdfJointSpec> = Vec::new();
        for joint_node in robot.children().filter(|n| n.has_tag_name("joint")) {
            let joint_type = required_attr(joint_node, "type")?;
            let jtype = match joint_type.as_str() {
                "revolute" | "continuous" => JointType::Revolute,
                "prismatic" => JointType::Prismatic,
                "fixed" => JointType::Fixed,
                "ball" | "spherical" => JointType::Spherical,
                "floating" | "freeflyer" | "free_flyer" => JointType::FreeFlyer,
                other => {
                    return Err(PinocchioError::invalid_model(format!(
                        "unsupported joint type '{other}' in urdf loader"
                    )));
                }
            };
            let parent_link = joint_node
                .children()
                .find(|n| n.has_tag_name("parent"))
                .and_then(|n| n.attribute("link"))
                .ok_or(PinocchioError::invalid_model(
                    "joint must contain <parent link=\"...\">",
                ))?
                .to_string();
            let child_link = joint_node
                .children()
                .find(|n| n.has_tag_name("child"))
                .and_then(|n| n.attribute("link"))
                .ok_or(PinocchioError::invalid_model(
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
                jtype,
            });
        }

        build_tree_model_from_specs(link_specs, joints, "urdf")
    }
}
