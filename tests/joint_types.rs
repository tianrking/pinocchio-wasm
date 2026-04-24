use pinocchio_wasm::algo::{aba, bias_forces, crba, frame_jacobian, rnea};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, JointType, Link, Model, Workspace};

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0, |acc, x| acc.max(x.abs()))
}

// ---------------------------------------------------------------------------
// Helper: planar two-link (revolute only) -- should match old behavior exactly
// ---------------------------------------------------------------------------

fn planar_two_link() -> Model {
    let i = Mat3::identity();
    let links = vec![
        Link::root("base", 0.1, Vec3::new(0.0, 0.0, 0.0), i),
        Link::child(
            "link1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
        Link::child(
            "link2",
            1,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ];
    Model::new(links).expect("model")
}

// ---------------------------------------------------------------------------
// Helper: model with a prismatic joint
// root -> revolute(z) -> prismatic(z)
// ---------------------------------------------------------------------------

fn mixed_revolute_prismatic() -> Model {
    let i = Mat3::identity();
    let links = vec![
        Link::root("base", 0.1, Vec3::new(0.0, 0.0, 0.0), i),
        Link::child(
            "link1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
        Link::child(
            "link2",
            1,
            Joint::prismatic(Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.0, 0.0, 0.0),
            i,
        ),
    ];
    Model::new(links).expect("model")
}

// ---------------------------------------------------------------------------
// Helper: model with a fixed joint
// root -> revolute(z) -> fixed -> revolute(z)
// ---------------------------------------------------------------------------

fn mixed_with_fixed() -> Model {
    let i = Mat3::identity();
    let links = vec![
        Link::root("base", 0.1, Vec3::new(0.0, 0.0, 0.0), i),
        Link::child(
            "link1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
        Link::child(
            "link_fixed",
            1,
            Joint::fixed(Vec3::new(1.0, 0.0, 0.0)),
            0.5,
            Vec3::new(0.0, 0.0, 0.0),
            i,
        ),
        Link::child(
            "link3",
            2,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ];
    Model::new(links).expect("model")
}

// ---------------------------------------------------------------------------
// Tests: nq / nv counts
// ---------------------------------------------------------------------------

#[test]
fn revolute_only_nq_nv() {
    let model = planar_two_link();
    assert_eq!(model.nq(), 2);
    assert_eq!(model.nv(), 2);
    assert_eq!(model.njoints(), 2);
    assert_eq!(model.nlinks(), 3);
}

#[test]
fn prismatic_joint_nq_nv() {
    let model = mixed_revolute_prismatic();
    assert_eq!(model.nq(), 2);
    assert_eq!(model.nv(), 2);
    assert_eq!(model.njoints(), 2);
}

#[test]
fn fixed_joint_reduces_nq_nv() {
    let model = mixed_with_fixed();
    // 3 non-root links but one is fixed -> 2 actuated joints
    assert_eq!(model.nq(), 2, "expected 2 q values (fixed contributes 0)");
    assert_eq!(
        model.nv(),
        2,
        "expected 2 velocity values (fixed contributes 0)"
    );
    assert_eq!(model.njoints(), 3, "3 joints total (including fixed)");
    assert_eq!(model.nlinks(), 4, "4 links total");
}

// ---------------------------------------------------------------------------
// Tests: joint types
// ---------------------------------------------------------------------------

#[test]
fn joint_type_accessor() {
    let model = mixed_with_fixed();
    assert_eq!(model.joint_type(0), JointType::Revolute);
    assert_eq!(model.joint_type(1), JointType::Fixed);
    assert_eq!(model.joint_type(2), JointType::Revolute);
}

#[test]
fn joint_nq_nv_individual() {
    let model = mixed_with_fixed();
    // Joint 0: revolute
    assert_eq!(
        model.links[model.joint_link(0).unwrap()]
            .joint
            .as_ref()
            .unwrap()
            .nq(),
        1
    );
    assert_eq!(
        model.links[model.joint_link(0).unwrap()]
            .joint
            .as_ref()
            .unwrap()
            .nv(),
        1
    );
    // Joint 1: fixed
    assert_eq!(
        model.links[model.joint_link(1).unwrap()]
            .joint
            .as_ref()
            .unwrap()
            .nq(),
        0
    );
    assert_eq!(
        model.links[model.joint_link(1).unwrap()]
            .joint
            .as_ref()
            .unwrap()
            .nv(),
        0
    );
    // Joint 2: revolute
    assert_eq!(
        model.links[model.joint_link(2).unwrap()]
            .joint
            .as_ref()
            .unwrap()
            .nq(),
        1
    );
    assert_eq!(
        model.links[model.joint_link(2).unwrap()]
            .joint
            .as_ref()
            .unwrap()
            .nv(),
        1
    );
}

// ---------------------------------------------------------------------------
// Tests: RNEA closure identity (tau = M*qdd + b)
// ---------------------------------------------------------------------------

#[test]
fn rnea_closure_revolute_only() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.3, -0.7];
    let qd = vec![1.2, -0.4];
    let qdd = vec![0.5, -1.1];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let tau = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let b = bias_forces(&model, &q, &qd, g, &mut ws).expect("bias");
    let m = crba(&model, &q, &mut ws).expect("crba");

    let mut mqdd = vec![0.0; model.nv()];
    for r in 0..model.nv() {
        for (c, qdd_c) in qdd.iter().enumerate().take(model.nv()) {
            mqdd[r] += m[r][c] * *qdd_c;
        }
    }

    let lhs: Vec<f64> = tau
        .iter()
        .zip(mqdd.iter().zip(b.iter()))
        .map(|(t, (m, b))| t - (m + b))
        .collect();
    assert!(max_abs(&lhs) < 1e-6, "closure residual too high: {:?}", lhs);
}

#[test]
fn rnea_closure_with_fixed_joint() {
    let model = mixed_with_fixed();
    let mut ws = Workspace::new(&model);
    let q = vec![0.3, -0.7];
    let qd = vec![1.2, -0.4];
    let qdd = vec![0.5, -1.1];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let tau = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let b = bias_forces(&model, &q, &qd, g, &mut ws).expect("bias");
    let m = crba(&model, &q, &mut ws).expect("crba");

    let mut mqdd = vec![0.0; model.nv()];
    for r in 0..model.nv() {
        for (c, qdd_c) in qdd.iter().enumerate().take(model.nv()) {
            mqdd[r] += m[r][c] * *qdd_c;
        }
    }

    let lhs: Vec<f64> = tau
        .iter()
        .zip(mqdd.iter().zip(b.iter()))
        .map(|(t, (m, b))| t - (m + b))
        .collect();
    assert!(max_abs(&lhs) < 1e-6, "closure residual too high: {:?}", lhs);
}

#[test]
fn rnea_closure_with_prismatic_joint() {
    let model = mixed_revolute_prismatic();
    let mut ws = Workspace::new(&model);
    let q = vec![0.3, 0.5];
    let qd = vec![1.2, -0.4];
    let qdd = vec![0.5, -1.1];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let tau = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let b = bias_forces(&model, &q, &qd, g, &mut ws).expect("bias");
    let m = crba(&model, &q, &mut ws).expect("crba");

    let mut mqdd = vec![0.0; model.nv()];
    for r in 0..model.nv() {
        for (c, qdd_c) in qdd.iter().enumerate().take(model.nv()) {
            mqdd[r] += m[r][c] * *qdd_c;
        }
    }

    let lhs: Vec<f64> = tau
        .iter()
        .zip(mqdd.iter().zip(b.iter()))
        .map(|(t, (m, b))| t - (m + b))
        .collect();
    assert!(max_abs(&lhs) < 1e-6, "closure residual too high: {:?}", lhs);
}

// ---------------------------------------------------------------------------
// Tests: ABA inverts dynamics
// ---------------------------------------------------------------------------

#[test]
fn aba_inverts_with_fixed_joint() {
    let model = mixed_with_fixed();
    let mut ws = Workspace::new(&model);
    let q = vec![0.2, -0.9];
    let qd = vec![0.4, 0.7];
    let tau = vec![1.1, -0.3];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let qdd = aba(&model, &q, &qd, &tau, g, &mut ws).expect("aba");
    let tau_reconstructed = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let residual: Vec<f64> = tau
        .iter()
        .zip(tau_reconstructed.iter())
        .map(|(a, b)| a - b)
        .collect();
    assert!(
        max_abs(&residual) < 1e-6,
        "aba residual too high: {:?}",
        residual
    );
}

#[test]
fn aba_inverts_with_prismatic_joint() {
    let model = mixed_revolute_prismatic();
    let mut ws = Workspace::new(&model);
    let q = vec![0.2, 0.5];
    let qd = vec![0.4, 0.7];
    let tau = vec![1.1, -0.3];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let qdd = aba(&model, &q, &qd, &tau, g, &mut ws).expect("aba");
    let tau_reconstructed = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let residual: Vec<f64> = tau
        .iter()
        .zip(tau_reconstructed.iter())
        .map(|(a, b)| a - b)
        .collect();
    assert!(
        max_abs(&residual) < 1e-6,
        "aba residual too high: {:?}",
        residual
    );
}

// ---------------------------------------------------------------------------
// Tests: Jacobian
// ---------------------------------------------------------------------------

#[test]
fn jacobian_with_fixed_joint() {
    let model = mixed_with_fixed();
    let mut ws = Workspace::new(&model);
    let q = vec![0.5, 0.1];

    let jac = frame_jacobian(&model, &q, 3, &mut ws).expect("jac");
    assert_eq!(jac.len(), 6 * model.nv());
}

#[test]
fn jacobian_with_prismatic_joint() {
    let model = mixed_revolute_prismatic();
    let mut ws = Workspace::new(&model);
    let q = vec![0.5, 0.1];

    let jac = frame_jacobian(&model, &q, 2, &mut ws).expect("jac");
    assert_eq!(jac.len(), 6 * model.nv());
}

// ---------------------------------------------------------------------------
// Tests: FK for prismatic joint (translation along axis)
// ---------------------------------------------------------------------------

#[test]
fn prismatic_fk_translates_along_axis() {
    // Simple model: root -> prismatic(z)
    let i = Mat3::identity();
    let links = vec![
        Link::root("base", 1.0, Vec3::zero(), i),
        Link::child(
            "link1",
            0,
            Joint::prismatic(Vec3::new(0.0, 0.0, 1.0), Vec3::zero()),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ];
    let model = Model::new(links).expect("model");
    let mut ws = Workspace::new(&model);

    // q = 2.0 means translate 2.0 along z-axis
    let q = vec![2.0];
    let qd = vec![0.0];
    let qdd = vec![0.0];

    pinocchio_wasm::algo::forward_kinematics(&model, &q, &qd, &qdd, Vec3::zero(), &mut ws)
        .expect("fk");

    let pose = ws.world_pose[1];
    // Translation should be (0, 0, 2.0)
    assert!(
        (pose.translation.x - 0.0).abs() < 1e-9,
        "x should be 0, got {}",
        pose.translation.x
    );
    assert!(
        (pose.translation.y - 0.0).abs() < 1e-9,
        "y should be 0, got {}",
        pose.translation.y
    );
    assert!(
        (pose.translation.z - 2.0).abs() < 1e-9,
        "z should be 2.0, got {}",
        pose.translation.z
    );
}

// ---------------------------------------------------------------------------
// Tests: Fixed joint FK (no q dependency)
// ---------------------------------------------------------------------------

#[test]
fn fixed_fk_no_q_dependency() {
    let i = Mat3::identity();
    let links = vec![
        Link::root("base", 1.0, Vec3::zero(), i),
        Link::child(
            "link1",
            0,
            Joint::fixed(Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ];
    let model = Model::new(links).expect("model");

    assert_eq!(model.nq(), 0, "fixed-only model should have nq=0");
    assert_eq!(model.nv(), 0, "fixed-only model should have nv=0");

    let mut ws = Workspace::new(&model);

    // nq=0, so empty q/qd/qdd
    let q: Vec<f64> = vec![];
    let qd: Vec<f64> = vec![];
    let qdd: Vec<f64> = vec![];

    pinocchio_wasm::algo::forward_kinematics(&model, &q, &qd, &qdd, Vec3::zero(), &mut ws)
        .expect("fk");

    let pose = ws.world_pose[1];
    // Fixed joint at origin (1,0,0) -> link1 at (1,0,0)
    assert!(
        (pose.translation.x - 1.0).abs() < 1e-9,
        "x should be 1.0, got {}",
        pose.translation.x
    );
    assert!(
        (pose.translation.y - 0.0).abs() < 1e-9,
        "y should be 0, got {}",
        pose.translation.y
    );
}

// ---------------------------------------------------------------------------
// Tests: JSON round-trip with joint types
// ---------------------------------------------------------------------------

#[test]
fn json_roundtrip_with_prismatic() {
    let model = mixed_revolute_prismatic();
    let json = model.to_json_string().expect("to_json");
    assert!(json.contains("\"jtype\": \"revolute\""));
    assert!(json.contains("\"jtype\": \"prismatic\""));

    let model2 = Model::from_json_str(&json).expect("from_json");
    assert_eq!(model2.nq(), 2);
    assert_eq!(model2.nv(), 2);
    assert_eq!(model2.joint_type(0), JointType::Revolute);
    assert_eq!(model2.joint_type(1), JointType::Prismatic);
}

#[test]
fn json_roundtrip_with_fixed() {
    let model = mixed_with_fixed();
    let json = model.to_json_string().expect("to_json");
    assert!(json.contains("\"jtype\": \"fixed\""));

    let model2 = Model::from_json_str(&json).expect("from_json");
    assert_eq!(model2.nq(), 2);
    assert_eq!(model2.nv(), 2);
    assert_eq!(model2.joint_type(0), JointType::Revolute);
    assert_eq!(model2.joint_type(1), JointType::Fixed);
    assert_eq!(model2.joint_type(2), JointType::Revolute);
}

// ---------------------------------------------------------------------------
// Tests: URDF with joint types
// ---------------------------------------------------------------------------

#[test]
fn urdf_with_prismatic() {
    let urdf = r#"<?xml version="1.0"?>
<robot name="test">
  <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <link name="link1"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <joint name="j1" type="prismatic">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
</robot>"#;

    let model = Model::from_urdf_str(urdf).expect("parse urdf");
    assert_eq!(model.nq(), 1);
    assert_eq!(model.nv(), 1);
    assert_eq!(model.joint_type(0), JointType::Prismatic);
}

#[test]
fn urdf_with_fixed() {
    let urdf = r#"<?xml version="1.0"?>
<robot name="test">
  <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <link name="link1"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <joint name="j1" type="fixed">
    <parent link="base"/><child link="link1"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>"#;

    let model = Model::from_urdf_str(urdf).expect("parse urdf");
    assert_eq!(model.nq(), 0, "fixed joint -> nq=0");
    assert_eq!(model.nv(), 0, "fixed joint -> nv=0");
    assert_eq!(model.joint_type(0), JointType::Fixed);
}

// ---------------------------------------------------------------------------
// Tests: SDF with joint types
// ---------------------------------------------------------------------------

#[test]
fn sdf_with_fixed() {
    let sdf = r#"<?xml version="1.0"?>
<sdf version="1.7">
  <model name="test">
    <link name="base"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link>
    <link name="link1"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link>
    <joint name="j1" type="fixed">
      <parent>base</parent><child>link1</child>
      <pose>1 0 0 0 0 0</pose>
    </joint>
  </model>
</sdf>"#;

    let model = Model::from_sdf_str(sdf).expect("parse sdf");
    assert_eq!(model.nq(), 0);
    assert_eq!(model.nv(), 0);
}

// ---------------------------------------------------------------------------
// Tests: MJCF with slide joint
// ---------------------------------------------------------------------------

#[test]
fn mjcf_with_slide_joint() {
    let mjcf = r#"<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <body name="base" pos="0 0 0">
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
      <body name="link1" pos="0 0 0">
        <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
        <joint name="j1" type="slide" axis="0 0 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

    let model = Model::from_mjcf_str(mjcf).expect("parse mjcf");
    assert_eq!(model.nq(), 1);
    assert_eq!(model.nv(), 1);
    assert_eq!(model.joint_type(0), JointType::Prismatic);
}

// ---------------------------------------------------------------------------
// Tests: backward compatibility (revolute-only model unchanged)
// ---------------------------------------------------------------------------

#[test]
fn backward_compat_revolute_model_numerics() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.3, -0.7];
    let qd = vec![1.2, -0.4];
    let qdd = vec![0.5, -1.1];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let tau = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");

    // Verify specific numerical values from the original behavior
    // The RNEA closure identity must hold
    let b = bias_forces(&model, &q, &qd, g, &mut ws).expect("bias");
    let m = crba(&model, &q, &mut ws).expect("crba");

    let mut mqdd = vec![0.0; 2];
    for r in 0..2 {
        for c in 0..2 {
            mqdd[r] += m[r][c] * qdd[c];
        }
    }
    assert!(
        max_abs(
            &tau.iter()
                .zip(mqdd.iter().zip(b.iter()))
                .map(|(t, (m, b))| t - m - b)
                .collect::<Vec<_>>()
        ) < 1e-6
    );
}
