use pinocchio_wasm::algo::{kinematics_derivatives, kinematics_derivatives_fd};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn planar_two_link() -> Model {
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

fn planar_three_link() -> Model {
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
        Link::child(
            "l3",
            2,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model")
}

fn mixed_joint_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 1.0, Vec3::zero(), i),
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
            Joint::prismatic(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model")
}

fn fixed_joint_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 1.0, Vec3::zero(), i),
        Link::child(
            "l1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
        Link::child(
            "l2_fixed",
            1,
            Joint::fixed(Vec3::new(1.0, 0.0, 0.0)),
            0.5,
            Vec3::new(0.25, 0.0, 0.0),
            i,
        ),
        Link::child(
            "l3",
            2,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.25, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model")
}

const TOL: f64 = 1e-8;

fn assert_match(model: &Model, q: &[f64], target: usize, label: &str) {
    let mut ws1 = Workspace::new(model);
    let mut ws2 = Workspace::new(model);
    let an = kinematics_derivatives(model, q, target, &mut ws1).expect("analytical");
    let fd = kinematics_derivatives_fd(model, q, target, &mut ws2).expect("fd");
    assert_eq!(
        an.dpos_dq.len(),
        fd.dpos_dq.len(),
        "{label}: length mismatch"
    );
    let n = model.nv();
    for k in 0..an.dpos_dq.len() {
        let row = k / n;
        let col = k % n;
        let diff = (an.dpos_dq[k] - fd.dpos_dq[k]).abs();
        assert!(
            diff < TOL,
            "{}: dpos_dq[row={},col={}] mismatch: analytical={} fd={} diff={}",
            label,
            row,
            col,
            an.dpos_dq[k],
            fd.dpos_dq[k],
            diff
        );
    }
}

#[test]
fn kinematics_derivatives_planar_two_link() {
    let model = planar_two_link();
    assert_match(&model, &[0.0, 0.0], 2, "two_link_zero");
    assert_match(&model, &[0.5, -0.3], 2, "two_link_general");
    assert_match(&model, &[1.0, 0.5], 1, "two_link_link1");
}

#[test]
fn kinematics_derivatives_planar_three_link() {
    let model = planar_three_link();
    assert_match(&model, &[0.0, 0.0, 0.0], 3, "three_link_zero");
    assert_match(&model, &[0.3, -0.2, 0.5], 3, "three_link_general");
    assert_match(&model, &[0.3, -0.2, 0.5], 2, "three_link_mid");
    assert_match(&model, &[-1.0, 0.5, 0.8], 1, "three_link_link1");
}

#[test]
fn kinematics_derivatives_mixed_joints() {
    let model = mixed_joint_model();
    assert_match(&model, &[0.0, 0.0], 2, "mixed_zero");
    assert_match(&model, &[0.5, 0.3], 2, "mixed_general");
}

#[test]
fn kinematics_derivatives_fixed_joint() {
    let model = fixed_joint_model();
    assert_match(&model, &[0.0, 0.0], 3, "fixed_zero");
    assert_match(&model, &[0.5, -0.3], 3, "fixed_general");
    assert_match(&model, &[0.5, -0.3], 2, "fixed_link2");
}
