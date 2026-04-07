use pinocchio_wasm::algo::rnea;
use pinocchio_wasm::core::math::Vec3;
use pinocchio_wasm::{Model, Workspace};

#[test]
fn build_model_from_urdf() {
    let urdf = r#"
    <robot name="two_link">
      <link name="base"/>
      <link name="link1">
        <inertial>
          <origin xyz="0.5 0.0 0.0"/>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
      <link name="link2">
        <inertial>
          <origin xyz="0.5 0.0 0.0"/>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>

      <joint name="j1" type="revolute">
        <parent link="base"/>
        <child link="link1"/>
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="1 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
    "#;

    let model = Model::from_urdf_str(urdf).expect("urdf model");
    assert_eq!(model.nlinks(), 3);
    assert_eq!(model.nq(), 2);

    let mut ws = Workspace::new(&model);
    let tau = rnea(
        &model,
        &[0.2, -0.5],
        &[0.3, 0.1],
        &[0.4, -0.2],
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
    )
    .expect("rnea");
    assert_eq!(tau.len(), 2);
}

#[test]
fn reject_non_revolute_urdf_joint() {
    let urdf = r#"
    <robot name="bad">
      <link name="base"/>
      <link name="l1"/>
      <joint name="j1" type="fixed">
        <parent link="base"/>
        <child link="l1"/>
      </joint>
    </robot>
    "#;

    let err = Model::from_urdf_str(urdf).expect_err("must reject fixed joint");
    assert!(format!("{err}").contains("supported"));
}
