use pinocchio_wasm::algo::{
    ContactPoint, apply_contact_impulses, apply_contact_impulses_batch,
    constrained_forward_dynamics_contacts, constrained_forward_dynamics_contacts_batch,
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

fn contact_set() -> Vec<ContactPoint> {
    vec![ContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, -1.0, 0.0),
        acceleration_bias: 0.0,
    }]
}

#[test]
fn contact_constrained_dynamics_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = [0.0, 0.0];
    let qd = [0.8, -0.2];
    let tau = [0.0, 0.0];
    let out = constrained_forward_dynamics_contacts(
        &model,
        &q,
        &qd,
        &tau,
        &contact_set(),
        Vec3::zero(),
        &mut ws,
    )
    .expect("contact dynamics");

    assert_eq!(out.qdd.len(), model.nv());
    assert_eq!(out.lambda_normal.len(), 1);
    assert!(out.qdd.iter().all(|v| v.is_finite()));
    assert!(out.lambda_normal[0].is_finite());
    assert!(out.lambda_normal[0] >= -1e-9);
}

#[test]
fn contact_impulse_and_batch_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = [0.0, 0.0];
    let qd_minus = [1.0, 1.0];
    let out = apply_contact_impulses(&model, &q, &qd_minus, &contact_set(), 0.0, &mut ws)
        .expect("impulse");
    assert_eq!(out.qd_plus.len(), model.nv());
    assert_eq!(out.impulse_normal.len(), 1);
    assert!(out.impulse_normal[0].is_finite());
    assert!(out.impulse_normal[0] >= -1e-9);

    let q_batch = [0.0, 0.0, 0.1, -0.2];
    let qd_batch = [1.0, 1.0, 0.3, -0.6];
    let tau_batch = [0.0, 0.0, 0.0, 0.0];
    let mut qdd_out = [0.0; 4];
    let mut lambda_out = [0.0; 2];
    constrained_forward_dynamics_contacts_batch(
        &model,
        &q_batch,
        &qd_batch,
        &tau_batch,
        2,
        &contact_set(),
        Vec3::zero(),
        &mut ws,
        &mut qdd_out,
        &mut lambda_out,
    )
    .expect("contact batch");
    assert!(qdd_out.iter().all(|v| v.is_finite()));
    assert!(lambda_out.iter().all(|v| v.is_finite()));

    let mut qd_plus_out = [0.0; 4];
    let mut impulse_out = [0.0; 2];
    apply_contact_impulses_batch(
        &model,
        &q_batch,
        &qd_batch,
        2,
        &contact_set(),
        0.0,
        &mut ws,
        &mut qd_plus_out,
        &mut impulse_out,
    )
    .expect("impulse batch");
    assert!(qd_plus_out.iter().all(|v| v.is_finite()));
    assert!(impulse_out.iter().all(|v| v.is_finite()));
}
