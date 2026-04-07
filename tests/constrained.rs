use pinocchio_wasm::algo::{
    aba, constrained_aba_locked_joints, constrained_aba_locked_joints_batch,
};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn two_joint() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 0.1, Vec3::new(0.0, 0.0, 0.0), i),
        Link::child(
            "l1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
        Link::child(
            "l2",
            1,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model")
}

#[test]
fn constrained_locked_joint_enforced() {
    let model = two_joint();
    let mut ws = Workspace::new(&model);
    let q = [0.1, -0.2];
    let qd = [0.3, 0.4];
    let tau = [1.0, 0.2];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let qdd = constrained_aba_locked_joints(&model, &q, &qd, &tau, &[false, true], g, &mut ws)
        .expect("constrained");
    assert_eq!(qdd.len(), 2);
    assert!(qdd[0].is_finite());
    assert_eq!(qdd[1], 0.0);

    let full = aba(&model, &q, &qd, &tau, g, &mut ws).expect("aba");
    assert!(full[1].abs() > 1e-8 || full[0].abs() > 1e-8);
}

#[test]
fn constrained_batch_smoke() {
    let model = two_joint();
    let mut ws = Workspace::new(&model);
    let q = [0.1, -0.2, 0.2, -0.1];
    let qd = [0.3, 0.4, 0.1, -0.2];
    let tau = [1.0, 0.2, 0.5, -0.1];
    let mut out = [0.0; 4];

    constrained_aba_locked_joints_batch(
        &model,
        &q,
        &qd,
        &tau,
        2,
        &[false, true],
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
        &mut out,
    )
    .expect("batch");

    assert!(out.iter().all(|x| x.is_finite()));
    assert_eq!(out[1], 0.0);
    assert_eq!(out[3], 0.0);
}
