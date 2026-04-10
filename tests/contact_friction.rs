use pinocchio_wasm::algo::{
    FrictionContactPoint, apply_contact_impulses_friction,
    constrained_forward_dynamics_contacts_friction,
};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn planar_two_link() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
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
    ])
    .expect("model")
}

#[test]
fn friction_contact_solve_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let contacts = vec![FrictionContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, -1.0, 0.0),
        acceleration_bias: 0.0,
        friction_coeff: 0.5,
    }];

    let out = constrained_forward_dynamics_contacts_friction(
        &model,
        &[0.0, 0.0],
        &[1.0, 0.2],
        &[0.0, 0.0],
        &contacts,
        Vec3::zero(),
        &mut ws,
    )
    .expect("solve");

    assert_eq!(out.qdd.len(), model.nv());
    assert_eq!(out.lambda_normal.len(), 1);
    assert_eq!(out.lambda_tangent.len(), 1);
    assert_eq!(out.contact_forces_world.len(), 1);
    assert!(out.qdd.iter().all(|v| v.is_finite()));
    assert!(out.lambda_normal[0] >= -1e-9);
}

#[test]
fn friction_impulse_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let contacts = vec![FrictionContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, -1.0, 0.0),
        acceleration_bias: 0.0,
        friction_coeff: 0.5,
    }];

    let out = apply_contact_impulses_friction(
        &model,
        &[0.0, 0.0],
        &[1.2, -0.3],
        &contacts,
        0.0,
        &mut ws,
    )
    .expect("impulse");

    assert_eq!(out.qd_plus.len(), model.nv());
    assert_eq!(out.impulse_normal.len(), 1);
    assert_eq!(out.impulse_tangent.len(), 1);
    assert_eq!(out.contact_impulses_world.len(), 1);
    assert!(out.qd_plus.iter().all(|v| v.is_finite()));
    assert!(out.impulse_normal[0] >= -1e-9);
}
