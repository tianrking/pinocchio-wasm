use pinocchio_wasm::algo::rnea;
use pinocchio_wasm::core::math::Vec3;
use pinocchio_wasm::{Model, Workspace};

#[test]
fn build_model_from_sdf() {
    let sdf = r#"
    <sdf version="1.7">
      <model name="two_link">
        <link name="base"/>
        <link name="link1">
          <inertial>
            <pose>0.5 0 0 0 0 0</pose>
            <mass>1.0</mass>
            <inertia>
              <ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
              <iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz>
            </inertia>
          </inertial>
        </link>
        <joint name="j1" type="revolute">
          <parent>base</parent>
          <child>link1</child>
          <pose>0 0 0 0 0 0</pose>
          <axis><xyz>0 0 1</xyz></axis>
        </joint>
      </model>
    </sdf>
    "#;

    let model = Model::from_sdf_str(sdf).expect("sdf model");
    assert_eq!(model.nlinks(), 2);
    assert_eq!(model.nq(), 1);

    let mut ws = Workspace::new(&model);
    let tau = rnea(
        &model,
        &[0.2],
        &[0.3],
        &[0.4],
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
    )
    .expect("rnea");
    assert_eq!(tau.len(), 1);
}
