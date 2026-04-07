use pinocchio_wasm::algo::rnea;
use pinocchio_wasm::core::math::Vec3;
use pinocchio_wasm::{Model, Workspace};

#[test]
fn build_model_from_mjcf() {
    let mjcf = r#"
    <mujoco model="two_link">
      <worldbody>
        <body name="base" pos="0 0 0">
          <inertial pos="0 0 0" mass="0.1" diaginertia="1 1 1"/>
          <body name="link1" pos="0 0 0">
            <joint name="j1" type="hinge" axis="0 0 1" pos="0 0 0"/>
            <inertial pos="0.5 0 0" mass="1.0" diaginertia="1 1 1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    "#;

    let model = Model::from_mjcf_str(mjcf).expect("mjcf model");
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
