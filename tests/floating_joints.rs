use pinocchio_wasm::algo::{aba, bias_forces, crba, frame_jacobian, rnea};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, JointType, Link, Model, Workspace};

fn spherical_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 0.1, Vec3::zero(), i),
        Link::child(
            "ball",
            0,
            Joint::spherical(Vec3::zero()),
            1.5,
            Vec3::new(0.2, 0.1, 0.0),
            i,
        ),
    ])
    .expect("model")
}

fn freeflyer_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("world", 0.1, Vec3::zero(), i),
        Link::child(
            "body",
            0,
            Joint::freeflyer(Vec3::zero()),
            2.0,
            Vec3::new(0.1, 0.2, 0.3),
            i,
        ),
    ])
    .expect("model")
}

fn assert_rnea_closure(model: &Model, q: &[f64], qd: &[f64], qdd: &[f64]) {
    let g = Vec3::new(0.0, 0.0, -9.81);
    let mut ws = Workspace::new(model);
    let tau = rnea(model, q, qd, qdd, g, &mut ws).expect("rnea");
    let bias = bias_forces(model, q, qd, g, &mut ws).expect("bias");
    let mass = crba(model, q, &mut ws).expect("crba");

    let n = model.nv();
    for r in 0..n {
        let mut expected = bias[r];
        for c in 0..n {
            expected += mass[r][c] * qdd[c];
        }
        assert!(
            (tau[r] - expected).abs() < 1e-8,
            "rnea closure row {r}: tau={} expected={}",
            tau[r],
            expected
        );
    }
}

#[test]
fn spherical_joint_dimensions_and_dynamics() {
    let model = spherical_model();
    assert_eq!(model.nq(), 4);
    assert_eq!(model.nv(), 3);
    assert_eq!(model.joint_type(0), JointType::Spherical);

    let q = [0.9238795325, 0.0, 0.0, 0.3826834324];
    let qd = [0.2, -0.1, 0.3];
    let qdd = [0.4, 0.1, -0.2];
    assert_rnea_closure(&model, &q, &qd, &qdd);

    let mut ws = Workspace::new(&model);
    let jac = frame_jacobian(&model, &q, 1, &mut ws).expect("jacobian");
    assert_eq!(jac.len(), 6 * model.nv());
    let n = model.nv();
    assert!(jac[3 * n].abs() > 0.0 || jac[4 * n].abs() > 0.0 || jac[5 * n].abs() > 0.0);
}

#[test]
fn freeflyer_joint_dimensions_dynamics_and_aba() {
    let model = freeflyer_model();
    assert_eq!(model.nq(), 7);
    assert_eq!(model.nv(), 6);
    assert_eq!(model.joint_type(0), JointType::FreeFlyer);

    let q = [0.3, -0.2, 0.5, 0.9659258263, 0.0, 0.2588190451, 0.0];
    let qd = [0.1, -0.2, 0.3, 0.2, 0.1, -0.1];
    let qdd = [0.3, -0.1, 0.2, -0.2, 0.4, 0.1];
    assert_rnea_closure(&model, &q, &qd, &qdd);

    let tau = [1.0, -0.5, 0.25, 0.2, -0.1, 0.3];
    let g = Vec3::new(0.0, 0.0, -9.81);
    let mut ws = Workspace::new(&model);
    let qdd_aba = aba(&model, &q, &qd, &tau, g, &mut ws).expect("aba");
    let tau_recon = rnea(&model, &q, &qd, &qdd_aba, g, &mut ws).expect("rnea");
    for i in 0..model.nv() {
        assert!((tau[i] - tau_recon[i]).abs() < 1e-8);
    }
}

#[test]
fn floating_joints_json_roundtrip() {
    let model = Model::from_json_str(
        r#"
        {
          "links": [
            {"name":"world","parent":null,"mass":0.1,"com":[0,0,0],"inertia":[[1,0,0],[0,1,0],[0,0,1]],"joint":null},
            {"name":"body","parent":0,"mass":1.0,"com":[0,0,0],"inertia":[[1,0,0],[0,1,0],[0,0,1]],"joint":{"jtype":"freeflyer","axis":[0,0,1],"origin":[0,0,0]}}
          ]
        }
        "#,
    )
    .expect("json");
    assert_eq!(model.nq(), 7);
    assert_eq!(model.nv(), 6);
    let json = model.to_json_string().expect("to json");
    assert!(json.contains("freeflyer"));
    let rt = Model::from_json_str(&json).expect("roundtrip");
    assert_eq!(rt.joint_type(0), JointType::FreeFlyer);
}

#[test]
fn floating_joints_xml_loaders() {
    let urdf = r#"
    <robot name="floating">
      <link name="world"/>
      <link name="ball"/>
      <link name="floating_body"/>
      <joint name="j_ball" type="ball">
        <parent link="world"/>
        <child link="ball"/>
        <origin xyz="0 0 0"/>
      </joint>
      <joint name="j_float" type="floating">
        <parent link="ball"/>
        <child link="floating_body"/>
        <origin xyz="0 0 0"/>
      </joint>
    </robot>
    "#;
    let model = Model::from_urdf_str(urdf).expect("urdf ball/floating");
    assert_eq!(model.nq(), 11);
    assert_eq!(model.nv(), 9);
    assert_eq!(model.joint_type(0), JointType::Spherical);
    assert_eq!(model.joint_type(1), JointType::FreeFlyer);

    let sdf = r#"
    <sdf version="1.7">
      <model name="m">
        <link name="world"/>
        <link name="ball"/>
        <joint name="j" type="ball">
          <parent>world</parent>
          <child>ball</child>
        </joint>
      </model>
    </sdf>
    "#;
    let model = Model::from_sdf_str(sdf).expect("sdf ball");
    assert_eq!(model.nq(), 4);
    assert_eq!(model.nv(), 3);
    assert_eq!(model.joint_type(0), JointType::Spherical);

    let mjcf = r#"
    <mujoco model="m">
      <worldbody>
        <body name="world">
          <body name="body">
            <joint name="j" type="free"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let model = Model::from_mjcf_str(mjcf).expect("mjcf free");
    assert_eq!(model.nq(), 7);
    assert_eq!(model.nv(), 6);
    assert_eq!(model.joint_type(0), JointType::FreeFlyer);
}
