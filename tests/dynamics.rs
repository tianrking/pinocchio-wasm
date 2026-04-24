use pinocchio_wasm::algo::{
    aba, aba_crba, bias_forces, center_of_mass, crba, frame_jacobian, kinetic_energy,
    potential_energy, rnea,
};
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

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0, |acc, x| acc.max(x.abs()))
}

#[test]
fn rnea_matches_mass_times_acc_plus_bias() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.3, -0.7];
    let qd = vec![1.2, -0.4];
    let qdd = vec![0.5, -1.1];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let tau = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let b = bias_forces(&model, &q, &qd, g, &mut ws).expect("bias");
    let m = crba(&model, &q, &mut ws).expect("crba");

    let mut mqdd = vec![0.0; model.nv()];
    for r in 0..model.nv() {
        for (c, qdd_c) in qdd.iter().enumerate().take(model.nv()) {
            mqdd[r] += m[r][c] * *qdd_c;
        }
    }

    let lhs: Vec<f64> = tau
        .iter()
        .zip(mqdd.iter().zip(b.iter()))
        .map(|(t, (m, b))| t - (m + b))
        .collect();
    assert!(max_abs(&lhs) < 1e-6, "closure residual too high: {:?}", lhs);
}

#[test]
fn aba_inverts_dynamics() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.2, -0.9];
    let qd = vec![0.4, 0.7];
    let tau = vec![1.1, -0.3];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let qdd = aba(&model, &q, &qd, &tau, g, &mut ws).expect("aba");
    let tau_reconstructed = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let residual: Vec<f64> = tau
        .iter()
        .zip(tau_reconstructed.iter())
        .map(|(a, b)| a - b)
        .collect();
    assert!(
        max_abs(&residual) < 1e-6,
        "aba residual too high: {:?}",
        residual
    );
}

#[test]
fn jacobian_com_energy_api_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.5, 0.1];
    let qd = vec![0.2, -0.2];

    let jac = frame_jacobian(&model, &q, 2, &mut ws).expect("jac");
    assert_eq!(jac.len(), 6 * model.nv());

    let com = center_of_mass(&model, &q, &mut ws).expect("com");
    assert!(com.norm2() > 0.0);

    let ke = kinetic_energy(&model, &q, &qd, &mut ws).expect("ke");
    let pe = potential_energy(&model, &q, Vec3::new(0.0, 0.0, -9.81), &mut ws).expect("pe");
    assert!(ke >= 0.0);
    assert!(pe.is_finite());
}

// ---------------------------------------------------------------------------
// Cross-validation: O(n) ABA vs O(n³) CRBA+Cholesky ABA
// ---------------------------------------------------------------------------

/// 3-DOF planar arm with three revolute joints.
fn planar_three_link() -> Model {
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
        Link::child(
            "link3",
            2,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.8, 0.0, 0.0)),
            0.5,
            Vec3::new(0.3, 0.0, 0.0),
            i,
        ),
    ];
    Model::new(links).expect("model")
}

/// Model with mixed revolute and prismatic joints.
fn mixed_joint_model() -> Model {
    let i = Mat3::identity();
    let links = vec![
        Link::root("base", 0.1, Vec3::new(0.0, 0.0, 0.0), i),
        Link::child(
            "rev1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            2.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
        Link::child(
            "pris1",
            1,
            Joint::prismatic(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.0, 0.0, 0.0),
            i,
        ),
    ];
    Model::new(links).expect("model")
}

/// Model with a fixed joint in the chain.
fn fixed_joint_model() -> Model {
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
            "fixed_link",
            1,
            Joint::fixed(Vec3::new(0.3, 0.0, 0.0)),
            0.5,
            Vec3::new(0.15, 0.0, 0.0),
            i,
        ),
        Link::child(
            "link3",
            2,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.2, 0.0, 0.0)),
            0.8,
            Vec3::new(0.4, 0.0, 0.0),
            i,
        ),
    ];
    Model::new(links).expect("model")
}

fn assert_aba_matches_crba(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    gravity: Vec3,
    label: &str,
) {
    let mut ws1 = Workspace::new(model);
    let mut ws2 = Workspace::new(model);

    let qdd_aba = aba(model, q, qd, tau, gravity, &mut ws1).expect("aba");
    let qdd_crba = aba_crba(model, q, qd, tau, gravity, &mut ws2).expect("aba_crba");

    let diff: Vec<f64> = qdd_aba
        .iter()
        .zip(qdd_crba.iter())
        .map(|(a, b)| (a - b).abs())
        .collect();
    let max_err = diff.iter().fold(0.0_f64, |a, &b| a.max(b));
    assert!(
        max_err < 1e-8,
        "{label}: O(n) ABA vs CRBA mismatch (max_err={max_err:.2e}): aba={qdd_aba:?} crba={qdd_crba:?}",
    );

    // Also verify the O(n) ABA result is self-consistent with RNEA.
    let tau_recon = rnea(model, q, qd, &qdd_aba, gravity, &mut ws1).expect("rnea");
    let residual: Vec<f64> = tau
        .iter()
        .zip(tau_recon.iter())
        .map(|(a, b)| a - b)
        .collect();
    let max_res = residual.iter().fold(0.0_f64, |a, &b| a.max(b));
    assert!(
        max_res < 1e-8,
        "{label}: O(n) ABA not self-consistent with RNEA (max_res={max_res:.2e})",
    );
}

#[test]
fn aba_matches_crba_planar_two_link() {
    let model = planar_two_link();
    assert_aba_matches_crba(
        &model,
        &[0.2, -0.9],
        &[0.4, 0.7],
        &[1.1, -0.3],
        Vec3::new(0.0, 0.0, -9.81),
        "planar2",
    );
}

#[test]
fn aba_matches_crba_planar_two_link_zero_velocity() {
    let model = planar_two_link();
    assert_aba_matches_crba(
        &model,
        &[0.5, 0.3],
        &[0.0, 0.0],
        &[2.0, -1.0],
        Vec3::new(0.0, 0.0, -9.81),
        "planar2_zerov",
    );
}

#[test]
fn aba_matches_crba_planar_three_link() {
    let model = planar_three_link();
    assert_aba_matches_crba(
        &model,
        &[0.3, -0.5, 0.8],
        &[1.0, -0.5, 0.3],
        &[5.0, -2.0, 1.0],
        Vec3::new(0.0, 0.0, -9.81),
        "planar3",
    );
}

#[test]
fn aba_matches_crba_planar_three_link_various_configs() {
    let model = planar_three_link();
    // Test several configurations to exercise different parts of the workspace.
    let configs: Vec<(&[f64], &[f64], &[f64])> = vec![
        (&[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0]),
        (&[1.0, 0.5, -0.3], &[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0]),
        (&[0.0, 0.0, 0.0], &[1.0, 0.5, -0.3], &[0.0, 0.0, 0.0]),
        (&[-0.8, 1.2, -0.4], &[0.5, -0.3, 0.7], &[3.0, -1.0, 0.5]),
    ];
    for (qi, qdi, taui) in configs {
        assert_aba_matches_crba(
            &model,
            qi,
            qdi,
            taui,
            Vec3::new(0.0, 0.0, -9.81),
            "planar3_multi",
        );
    }
}

#[test]
fn aba_matches_crba_mixed_joints() {
    let model = mixed_joint_model();
    assert_aba_matches_crba(
        &model,
        &[0.5, 0.3],
        &[1.0, -0.5],
        &[2.0, -1.0],
        Vec3::new(0.0, 0.0, -9.81),
        "mixed",
    );
}

#[test]
fn aba_matches_crba_mixed_joints_zero_gravity() {
    let model = mixed_joint_model();
    assert_aba_matches_crba(
        &model,
        &[-0.3, 0.8],
        &[0.5, 1.0],
        &[1.0, 0.5],
        Vec3::new(0.0, 0.0, 0.0),
        "mixed_zerog",
    );
}

#[test]
fn aba_matches_crba_fixed_joint() {
    let model = fixed_joint_model();
    assert_aba_matches_crba(
        &model,
        &[0.3, -0.5],
        &[0.5, -0.3],
        &[2.0, -0.5],
        Vec3::new(0.0, 0.0, -9.81),
        "fixed",
    );
}

#[test]
fn aba_crba_still_works() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.2, -0.9];
    let qd = vec![0.4, 0.7];
    let tau = vec![1.1, -0.3];
    let g = Vec3::new(0.0, 0.0, -9.81);

    let qdd = aba_crba(&model, &q, &qd, &tau, g, &mut ws).expect("aba_crba");
    let tau_reconstructed = rnea(&model, &q, &qd, &qdd, g, &mut ws).expect("rnea");
    let residual: Vec<f64> = tau
        .iter()
        .zip(tau_reconstructed.iter())
        .map(|(a, b)| a - b)
        .collect();
    assert!(
        max_abs(&residual) < 1e-6,
        "aba_crba residual too high: {:?}",
        residual
    );
}
