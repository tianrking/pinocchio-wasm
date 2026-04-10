use pinocchio_wasm::algo::{ContactPoint, compute_all_terms, contact_jacobian_normal};
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
fn compute_all_terms_consistency() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = [0.2, -0.4];
    let qd = [0.7, 0.1];
    let g = Vec3::new(0.0, 0.0, -9.81);
    let t = compute_all_terms(&model, &q, &qd, g, &mut ws).expect("all terms");

    assert_eq!(t.mass.len(), model.nv());
    assert_eq!(t.mass[0].len(), model.nv());
    assert_eq!(t.bias.len(), model.nv());
    assert_eq!(t.gravity_torques.len(), model.nv());
    assert_eq!(t.coriolis_torques.len(), model.nv());
    assert!(t.kinetic_energy.is_finite());
    assert!(t.potential_energy.is_finite());

    for i in 0..model.nv() {
        let recon = t.gravity_torques[i] + t.coriolis_torques[i];
        assert!((recon - t.bias[i]).abs() < 1e-6);
    }
}

#[test]
fn contact_jacobian_normal_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = [0.0, 0.0];
    let contacts = vec![ContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, 1.0, 0.0),
        acceleration_bias: 0.0,
    }];

    let jac = contact_jacobian_normal(&model, &q, &contacts, &mut ws).expect("contact jac");
    assert_eq!(jac.len(), model.nv());
    assert!(jac.iter().all(|v| v.is_finite()));
}
