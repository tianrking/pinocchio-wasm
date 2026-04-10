use pinocchio_wasm::algo::{inverse_kinematics, inverse_kinematics_position, forward_kinematics_poses};
use pinocchio_wasm::core::math::Vec3;
use pinocchio_wasm::{Model, Workspace};

fn two_link_planar() -> (Model, Workspace) {
    // 2-link planar arm: both joints rotate about Z, link1 length=1.0, link2 length=1.0
    let json = r#"{
        "links": [
            {"name": "base", "parent": null, "mass": 0.1, "com": [0,0,0],
             "inertia": [[1,0,0],[0,1,0],[0,0,1]], "joint": null},
            {"name": "link1", "parent": 0, "mass": 1.0, "com": [0.5,0,0],
             "inertia": [[1,0,0],[0,1,0],[0,0,1]],
             "joint": {"axis": [0,0,1], "origin": [0,0,0]}},
            {"name": "link2", "parent": 1, "mass": 1.0, "com": [0.5,0,0],
             "inertia": [[1,0,0],[0,1,0],[0,0,1]],
             "joint": {"axis": [0,0,1], "origin": [1,0,0]}}
        ]
    }"#;
    let model = Model::from_json_str(json).unwrap();
    let ws = Workspace::new(&model);
    (model, ws)
}

#[test]
fn ik_position_converges_for_reachable_target() {
    let (model, mut ws) = two_link_planar();

    let q_true = [std::f64::consts::FRAC_PI_6, std::f64::consts::FRAC_PI_3];
    let poses = forward_kinematics_poses(&model, &q_true, &mut ws).unwrap();
    let tip_pos = poses[2].translation;

    let q_init = [0.5, 0.2];
    let result =
        inverse_kinematics_position(&model, &q_init, 2, tip_pos, &mut ws, 200, 1e-6, 1e-4)
            .unwrap();

    assert!(result.converged, "IK did not converge, err = {}", result.err);
    assert!(result.err < 1e-4, "Final error too large: {}", result.err);

    let poses_check = forward_kinematics_poses(&model, &result.q, &mut ws).unwrap();
    let tip_check = poses_check[2].translation;
    let pos_err = (tip_check - tip_pos).norm();
    assert!(pos_err < 1e-3, "FK verification failed, pos error = {}", pos_err);
}

#[test]
fn ik_se3_converges_for_reachable_target() {
    let (model, mut ws) = two_link_planar();

    let q_true = [0.3, -0.8];
    let poses = forward_kinematics_poses(&model, &q_true, &mut ws).unwrap();
    let target_tf = poses[2];

    let q_init = [0.0, 0.0];
    let result = inverse_kinematics(
        &model,
        &q_init,
        2,
        target_tf.translation,
        target_tf.rotation,
        &mut ws,
        300,
        1e-6,
        1e-4,
    )
    .unwrap();

    assert!(result.converged, "SE3 IK did not converge, err = {}", result.err);
    assert!(result.err < 1e-3, "SE3 final error too large: {}", result.err);

    let poses_check = forward_kinematics_poses(&model, &result.q, &mut ws).unwrap();
    let pos_err = (poses_check[2].translation - target_tf.translation).norm();
    assert!(pos_err < 1e-2, "SE3 FK verification failed, pos error = {}", pos_err);
}

#[test]
fn ik_position_handles_zero_init() {
    let (model, mut ws) = two_link_planar();

    // q = [π/4, 0] → tip at (cos(π/4), sin(π/4), 0) ≈ (0.707, 0.707, 0)
    let target = Vec3::new(std::f64::consts::FRAC_PI_4.cos(), std::f64::consts::FRAC_PI_4.sin(), 0.0);
    let q_init = [0.0, 0.0];
    let result =
        inverse_kinematics_position(&model, &q_init, 2, target, &mut ws, 200, 1e-6, 1e-4)
            .unwrap();

    assert!(result.converged, "IK zero-init did not converge, err = {}", result.err);

    let poses_check = forward_kinematics_poses(&model, &result.q, &mut ws).unwrap();
    let tip_check = poses_check[2].translation;
    let pos_err = (tip_check - target).norm();
    assert!(pos_err < 1e-3, "FK verification failed, pos error = {}", pos_err);
}
