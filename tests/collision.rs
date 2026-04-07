use pinocchio_wasm::collision::{
    CollisionModel, Geometry, PairFilter, Sphere, collision_details, minimum_distance,
    minimum_distance_batch, minimum_distance_detailed,
};
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
    let coll = CollisionModel::from_spheres(vec![
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
    let coll = CollisionModel::from_spheres(vec![
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

#[test]
fn geometry_detailed_and_filter_smoke() {
    let model = two_link();
    let mut ws = Workspace::new(&model);
    let geometries = vec![
        Geometry::Box {
            link_index: 1,
            center_local: Vec3::new(0.5, 0.0, 0.0),
            half_extents: Vec3::new(0.2, 0.1, 0.1),
        },
        Geometry::Capsule {
            link_index: 2,
            center_local: Vec3::new(0.5, 0.0, 0.0),
            half_length: 0.3,
            radius: 0.1,
        },
        Geometry::Cylinder {
            link_index: 2,
            center_local: Vec3::new(0.2, 0.0, 0.0),
            half_length: 0.2,
            radius: 0.1,
        },
        Geometry::MeshApprox {
            link_index: 1,
            center_local: Vec3::new(0.3, 0.0, 0.0),
            half_extents: Vec3::new(0.1, 0.1, 0.1),
        },
    ];
    let mut pairs = Vec::new();
    for i in 0..geometries.len() {
        for j in (i + 1)..geometries.len() {
            pairs.push((i, j));
        }
    }
    let coll = CollisionModel::new_with_filter(
        geometries,
        pairs,
        PairFilter {
            ignore_same_link: true,
            ignore_parent_child: false,
        },
    )
    .expect("collision");

    let details = collision_details(&model, &coll, &[0.0, 0.0], &mut ws).expect("details");
    assert!(!details.is_empty());
    assert!(details.iter().all(|d| d.distance.is_finite()));

    let best = minimum_distance_detailed(&model, &coll, &[0.0, 0.0], &mut ws).expect("best");
    assert!(best.distance.is_finite());
    assert!(best.penetration_depth >= 0.0);
}
