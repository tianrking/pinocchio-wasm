use pinocchio_wasm::algo::{aba_batch, rnea_batch};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn planar_two_link() -> Model {
    let i = Mat3::identity();
    let links = vec![
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
    ];
    Model::new(links).expect("model")
}

#[test]
fn batch_rnea_and_aba_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let n = model.nq();
    let steps = 3;

    let q = vec![0.1, -0.3, 0.2, -0.1, 0.4, 0.2];
    let qd = vec![0.0, 0.1, 0.2, 0.1, -0.2, 0.3];
    let qdd = vec![0.4, -0.2, 0.1, -0.3, 0.2, 0.5];
    let tau = vec![1.0, -0.2, 0.8, 0.1, 0.3, -0.4];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let mut tau_out = vec![0.0; steps * n];
    rnea_batch(&model, &q, &qd, &qdd, steps, g, &mut ws, &mut tau_out).expect("rnea batch");

    let mut qdd_out = vec![0.0; steps * n];
    aba_batch(&model, &q, &qd, &tau, steps, g, &mut ws, &mut qdd_out).expect("aba batch");

    assert_eq!(tau_out.len(), steps * n);
    assert_eq!(qdd_out.len(), steps * n);
    assert!(tau_out.iter().all(|x| x.is_finite()));
    assert!(qdd_out.iter().all(|x| x.is_finite()));
}
