use pinocchio_wasm::algo::{forward_kinematics_poses, forward_kinematics_poses_batch};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn model2() -> Model {
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
    ])
    .expect("model")
}

#[test]
fn fk_poses_smoke() {
    let model = model2();
    let mut ws = Workspace::new(&model);
    let poses = forward_kinematics_poses(&model, &[0.2], &mut ws).expect("fk");
    assert_eq!(poses.len(), model.nlinks());
    assert!(poses[1].translation.x.is_finite());
}

#[test]
fn fk_poses_batch_smoke() {
    let model = model2();
    let mut ws = Workspace::new(&model);
    let mut t = vec![0.0; 2 * model.nlinks() * 3];
    let mut r = vec![0.0; 2 * model.nlinks() * 9];
    forward_kinematics_poses_batch(&model, &[0.2, -0.4], 2, &mut ws, &mut t, &mut r)
        .expect("fk batch");
    assert!(t.iter().all(|x| x.is_finite()));
    assert!(r.iter().all(|x| x.is_finite()));
}
