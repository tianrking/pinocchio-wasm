use crate::core::error::{PinocchioError, Result};
use crate::core::math::Mat3;
use crate::model::{
    JointType, Model, UrdfJointSpec, UrdfLinkSpec, build_tree_model_from_specs, fmt_pose_xyz,
    fmt_vec3, parse_pose_xyz_text, parse_sdf_inertia_node, parse_vec3_text, required_attr,
    text_of_child,
};
use std::collections::BTreeMap;

impl Model {
    pub fn to_sdf_string(&self, model_name: &str) -> Result<String> {
        if self.links.is_empty() {
            return Err(PinocchioError::invalid_model("model must not be empty"));
        }
        let mut out = String::new();
        out.push_str("<sdf version=\"1.7\">\n");
        out.push_str(&format!("  <model name=\"{model_name}\">\n"));
        for l in &self.links {
            out.push_str(&format!("    <link name=\"{}\">\n", l.name));
            out.push_str("      <inertial>\n");
            out.push_str(&format!(
                "        <pose>{}</pose>\n",
                fmt_pose_xyz([l.com_local.x, l.com_local.y, l.com_local.z])
            ));
            out.push_str(&format!("        <mass>{}</mass>\n", l.mass));
            out.push_str("        <inertia>\n");
            out.push_str(&format!(
                "          <ixx>{}</ixx>\n",
                l.inertia_local_com.m[0][0]
            ));
            out.push_str(&format!(
                "          <ixy>{}</ixy>\n",
                l.inertia_local_com.m[0][1]
            ));
            out.push_str(&format!(
                "          <ixz>{}</ixz>\n",
                l.inertia_local_com.m[0][2]
            ));
            out.push_str(&format!(
                "          <iyy>{}</iyy>\n",
                l.inertia_local_com.m[1][1]
            ));
            out.push_str(&format!(
                "          <iyz>{}</iyz>\n",
                l.inertia_local_com.m[1][2]
            ));
            out.push_str(&format!(
                "          <izz>{}</izz>\n",
                l.inertia_local_com.m[2][2]
            ));
            out.push_str("        </inertia>\n");
            out.push_str("      </inertial>\n");
            out.push_str("    </link>\n");
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
            out.push_str(&format!(
                "    <joint name=\"j{idx}\" type=\"{type_str}\">\n"
            ));
            out.push_str(&format!(
                "      <parent>{}</parent>\n",
                self.links[parent].name
            ));
            out.push_str(&format!("      <child>{}</child>\n", l.name));
            out.push_str(&format!(
                "      <pose>{}</pose>\n",
                fmt_pose_xyz([
                    joint.origin.translation.x,
                    joint.origin.translation.y,
                    joint.origin.translation.z
                ])
            ));
            if matches!(joint.jtype, JointType::Revolute | JointType::Prismatic) {
                out.push_str("      <axis>\n");
                out.push_str(&format!(
                    "        <xyz>{}</xyz>\n",
                    fmt_vec3([joint.axis.x, joint.axis.y, joint.axis.z])
                ));
                out.push_str("      </axis>\n");
            }
            out.push_str("    </joint>\n");
        }
        out.push_str("  </model>\n");
        out.push_str("</sdf>\n");
        Ok(out)
    }

    pub fn from_sdf_str(input: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(input)
            .map_err(|_| PinocchioError::invalid_model("failed to parse sdf xml"))?;
        let model_node = doc.descendants().find(|n| n.has_tag_name("model")).ok_or(
            PinocchioError::invalid_model("sdf must contain a <model> element"),
        )?;

        let mut link_specs: BTreeMap<String, UrdfLinkSpec> = BTreeMap::new();
        for link_node in model_node.children().filter(|n| n.has_tag_name("link")) {
            let name = required_attr(link_node, "name")?;
            let inertial = link_node.children().find(|n| n.has_tag_name("inertial"));
            let (mass, com, inertia) = if let Some(inertial_node) = inertial {
                let mass = text_of_child(inertial_node, "mass")
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(1.0);
                let com = text_of_child(inertial_node, "pose")
                    .map(parse_pose_xyz_text)
                    .transpose()?
                    .unwrap_or([0.0, 0.0, 0.0]);
                let inertia = inertial_node
                    .children()
                    .find(|n| n.has_tag_name("inertia"))
                    .map(parse_sdf_inertia_node)
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
                "sdf model must contain at least one <link>",
            ));
        }

        let mut joints: Vec<UrdfJointSpec> = Vec::new();
        for joint_node in model_node.children().filter(|n| n.has_tag_name("joint")) {
            let joint_type = required_attr(joint_node, "type")?;
            let jtype = match joint_type.as_str() {
                "revolute" | "continuous" => JointType::Revolute,
                "prismatic" => JointType::Prismatic,
                "fixed" => JointType::Fixed,
                "ball" | "spherical" => JointType::Spherical,
                "floating" | "freeflyer" | "free_flyer" => JointType::FreeFlyer,
                other => {
                    return Err(PinocchioError::invalid_model(format!(
                        "unsupported joint type '{other}' in sdf loader"
                    )));
                }
            };
            let parent_link = text_of_child(joint_node, "parent")
                .ok_or(PinocchioError::invalid_model("joint must contain <parent>"))?;
            let child_link = text_of_child(joint_node, "child")
                .ok_or(PinocchioError::invalid_model("joint must contain <child>"))?;
            let origin = text_of_child(joint_node, "pose")
                .map(parse_pose_xyz_text)
                .transpose()?
                .unwrap_or([0.0, 0.0, 0.0]);
            let axis = joint_node
                .children()
                .find(|n| n.has_tag_name("axis"))
                .and_then(|axis_node| text_of_child(axis_node, "xyz"))
                .map(|s| parse_vec3_text(&s))
                .transpose()?
                .unwrap_or([0.0, 0.0, 1.0]);
            joints.push(UrdfJointSpec {
                parent_link,
                child_link,
                axis,
                origin,
                jtype,
            });
        }

        build_tree_model_from_specs(link_specs, joints, "sdf")
    }
}
