use pinocchio_wasm::collision::{CollisionModel, Sphere, minimum_distance, minimum_distance_batch};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn two_link() -> Model {
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
fn minimum_distance_smoke() {
    let model = two_link();
    let mut ws = Workspace::new(&model);
    let coll = CollisionModel::with_all_pairs(vec![
        Sphere {
            link_index: 1,
            center_local: Vec3::new(0.5, 0.0, 0.0),
            radius: 0.2,
        },
        Sphere {
            link_index: 2,
            center_local: Vec3::new(0.5, 0.0, 0.0),
            radius: 0.2,
        },
    ])
    .expect("collision");

    let res = minimum_distance(&model, &coll, &[0.0, 0.0], &mut ws).expect("distance");
    assert!(res.distance.is_finite());
}

#[test]
fn minimum_distance_batch_smoke() {
    let model = two_link();
    let mut ws = Workspace::new(&model);
    let coll = CollisionModel::with_all_pairs(vec![
        Sphere {
            link_index: 1,
            center_local: Vec3::new(0.5, 0.0, 0.0),
            radius: 0.2,
        },
        Sphere {
            link_index: 2,
            center_local: Vec3::new(0.5, 0.0, 0.0),
            radius: 0.2,
        },
    ])
    .expect("collision");

    let q_batch = vec![0.0, 0.0, 0.3, -0.2, -0.4, 0.5];
    let mut out = vec![0.0; 3];
    minimum_distance_batch(&model, &coll, &q_batch, 3, &mut ws, &mut out).expect("batch");
    assert!(out.iter().all(|x| x.is_finite()));
}
