use pinocchio_wasm::algo::{
    ContactPoint, centroidal_full_terms, centroidal_full_terms_with_contacts, centroidal_map,
    centroidal_map_derivatives, centroidal_momentum_rate,
};
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

#[test]
fn centroidal_full_stack_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let qdd = [0.3, -0.2];

    let ag = centroidal_map(&model, &q, &mut ws).expect("ag");
    assert_eq!(ag.len(), 6 * model.nv());

    let dag = centroidal_map_derivatives(&model, &q, &mut ws).expect("dag");
    assert_eq!(dag.len(), 6 * model.nv() * model.nv());

    let hdot = centroidal_momentum_rate(&model, &q, &qd, &qdd, &mut ws).expect("hdot");
    assert!(hdot.linear.norm().is_finite());

    let full = centroidal_full_terms(&model, &q, &qd, &qdd, &mut ws).expect("full");
    assert_eq!(full.ag.len(), 6 * model.nv());
    assert_eq!(full.dag_dq.len(), 6 * model.nv() * model.nv());

    let contacts = vec![ContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, 1.0, 0.0),
        acceleration_bias: 0.0,
    }];
    let forces = vec![Vec3::new(0.0, 10.0, 0.0)];
    let (full2, wrench) =
        centroidal_full_terms_with_contacts(&model, &q, &qd, &qdd, &contacts, &forces, &mut ws)
            .expect("full contacts");
    assert_eq!(full2.ag.len(), 6 * model.nv());
    assert!(wrench.linear.norm().is_finite());
}
