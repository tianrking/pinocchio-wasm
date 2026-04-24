use pinocchio_wasm::algo::{aba, aba_derivatives};
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

fn mixed_joint_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 1.0, Vec3::zero(), i),
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
            Joint::prismatic(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model")
}

fn assert_aba_derivatives_match_fd(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    gravity: Vec3,
    label: &str,
) {
    let n = model.nv();
    let eps = 1e-6;
    let mut ws = Workspace::new(model);
    let an = aba_derivatives(model, q, qd, tau, gravity, &mut ws).expect("aba derivatives");

    for c in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[c] += eps;
        qm[c] -= eps;
        let ap = aba(model, &qp, qd, tau, gravity, &mut Workspace::new(model)).expect("aba qp");
        let am = aba(model, &qm, qd, tau, gravity, &mut Workspace::new(model)).expect("aba qm");

        let mut vp = qd.to_vec();
        let mut vm = qd.to_vec();
        vp[c] += eps;
        vm[c] -= eps;
        let avp = aba(model, q, &vp, tau, gravity, &mut Workspace::new(model)).expect("aba vp");
        let avm = aba(model, q, &vm, tau, gravity, &mut Workspace::new(model)).expect("aba vm");

        let mut tp = tau.to_vec();
        let mut tm = tau.to_vec();
        tp[c] += eps;
        tm[c] -= eps;
        let atp = aba(model, q, qd, &tp, gravity, &mut Workspace::new(model)).expect("aba tp");
        let atm = aba(model, q, qd, &tm, gravity, &mut Workspace::new(model)).expect("aba tm");

        for r in 0..n {
            let fd_q = (ap[r] - am[r]) / (2.0 * eps);
            let fd_v = (avp[r] - avm[r]) / (2.0 * eps);
            let fd_t = (atp[r] - atm[r]) / (2.0 * eps);
            assert!(
                (an.d_out_dq[r][c] - fd_q).abs() < 5e-6,
                "{label} dq[{r}][{c}] an={} fd={}",
                an.d_out_dq[r][c],
                fd_q
            );
            assert!(
                (an.d_out_dv[r][c] - fd_v).abs() < 5e-6,
                "{label} dv[{r}][{c}] an={} fd={}",
                an.d_out_dv[r][c],
                fd_v
            );
            assert!(
                (an.d_out_du[r][c] - fd_t).abs() < 5e-6,
                "{label} dtau[{r}][{c}] an={} fd={}",
                an.d_out_du[r][c],
                fd_t
            );
        }
    }
}

#[test]
fn aba_derivatives_planar_two_link() {
    let model = planar_two_link();
    assert_aba_derivatives_match_fd(
        &model,
        &[0.5, -0.3],
        &[0.4, 0.1],
        &[1.0, -0.2],
        Vec3::new(0.0, 0.0, -9.81),
        "planar2",
    );
}

#[test]
fn aba_derivatives_mixed_joints() {
    let model = mixed_joint_model();
    assert_aba_derivatives_match_fd(
        &model,
        &[0.5, 0.3],
        &[0.2, -0.1],
        &[1.0, 0.4],
        Vec3::new(0.0, 0.0, -9.81),
        "mixed",
    );
}
