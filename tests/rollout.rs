use pinocchio_wasm::algo::{RolloutState, rollout_aba_euler};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn single_joint() -> Model {
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
    ];
    Model::new(links).expect("model")
}

#[test]
fn rollout_aba_euler_smoke() {
    let model = single_joint();
    let mut ws = Workspace::new(&model);
    let steps = 4usize;
    let tau = vec![0.0; steps * model.nq()];
    let mut q_out = vec![0.0; steps * model.nq()];
    let mut qd_out = vec![0.0; steps * model.nq()];

    rollout_aba_euler(
        &model,
        RolloutState {
            q0: &[0.0],
            qd0: &[0.0],
        },
        &tau,
        steps,
        0.01,
        Vec3::zero(),
        &mut ws,
        &mut q_out,
        &mut qd_out,
    )
    .expect("rollout");

    assert!(q_out.iter().all(|x| x.is_finite()));
    assert!(qd_out.iter().all(|x| x.is_finite()));
}
