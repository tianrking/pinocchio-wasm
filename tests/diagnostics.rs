use pinocchio_wasm::algo::{
    bias_forces, bias_forces_batch, coriolis_torques, crba_batch, gravity_torques,
    gravity_torques_batch,
};
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
fn bias_equals_coriolis_plus_gravity() {
    let model = model2();
    let mut ws = Workspace::new(&model);
    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let gvec = Vec3::new(0.0, 0.0, -9.81);

    let b = bias_forces(&model, &q, &qd, gvec, &mut ws).expect("b");
    let c = coriolis_torques(&model, &q, &qd, &mut ws).expect("c");
    let g = gravity_torques(&model, &q, gvec, &mut ws).expect("g");

    for i in 0..model.nq() {
        assert!((b[i] - (c[i] + g[i])).abs() < 1e-6);
    }
}

#[test]
fn diagnostics_batch_smoke() {
    let model = model2();
    let mut ws = Workspace::new(&model);
    let q_batch = [0.1, -0.2, 0.3, -0.1];
    let qd_batch = [0.2, 0.1, 0.0, -0.3];

    let mut b_out = [0.0; 4];
    bias_forces_batch(
        &model,
        &q_batch,
        &qd_batch,
        2,
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
        &mut b_out,
    )
    .expect("bias batch");

    let mut g_out = [0.0; 4];
    gravity_torques_batch(
        &model,
        &q_batch,
        2,
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
        &mut g_out,
    )
    .expect("g batch");

    let mut m_out = [0.0; 8];
    crba_batch(&model, &q_batch, 2, &mut ws, &mut m_out).expect("crba batch");

    assert!(b_out.iter().all(|x| x.is_finite()));
    assert!(g_out.iter().all(|x| x.is_finite()));
    assert!(m_out.iter().all(|x| x.is_finite()));
}
