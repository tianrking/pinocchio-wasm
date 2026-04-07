use pinocchio_wasm::algo::{aba, bias_forces, center_of_mass, crba, frame_jacobian, kinetic_energy, potential_energy, rnea};
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

    let lhs: Vec<f64> = tau.iter().zip(mqdd.iter().zip(b.iter())).map(|(t, (m, b))| t - (m + b)).collect();
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
    let residual: Vec<f64> = tau.iter().zip(tau_reconstructed.iter()).map(|(a, b)| a - b).collect();
    assert!(max_abs(&residual) < 1e-6, "aba residual too high: {:?}", residual);
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
