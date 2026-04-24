use pinocchio_wasm::algo::{
    difference_configuration, integrate_model_configuration, interpolate_configuration,
    is_normalized, neutral_configuration, normalize_configuration, random_configuration,
};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::core::quaternion::Quat;
use pinocchio_wasm::{Joint, Link, Model, Workspace};

fn planar_two_link() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 0.1, Vec3::zero(), i),
        Link::child(
            "l1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::zero()),
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

fn spherical_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 0.1, Vec3::zero(), i),
        Link::child(
            "ball",
            0,
            Joint::spherical(Vec3::zero()),
            1.5,
            Vec3::new(0.2, 0.1, 0.0),
            i,
        ),
    ])
    .expect("model")
}

fn freeflyer_model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("world", 0.1, Vec3::zero(), i),
        Link::child(
            "body",
            0,
            Joint::freeflyer(Vec3::zero()),
            2.0,
            Vec3::new(0.1, 0.2, 0.3),
            i,
        ),
    ])
    .expect("model")
}

// --- Quaternion unit tests ---

#[test]
fn quat_identity_normalize() {
    let q = Quat::identity();
    assert!(q.is_normalized(1e-10));
    assert_eq!(q.to_array(), [1.0, 0.0, 0.0, 0.0]);
}

#[test]
fn quat_mul_identity() {
    let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5);
    let result = q.mul(Quat::identity());
    assert!((result.w - q.w).abs() < 1e-10);
    assert!((result.x - q.x).abs() < 1e-10);
}

#[test]
fn quat_roundtrip_axis_angle() {
    let axis = Vec3::new(1.0, 2.0, 3.0);
    let angle = 1.234;
    let q = Quat::from_axis_angle(axis, angle);
    assert!(q.is_normalized(1e-10));
    let r = q.to_rotation_matrix();
    let q2 = Quat::from_rotation_matrix(&r);
    assert!((q.w - q2.w).abs() < 1e-8 || (q.w + q2.w).abs() < 1e-8);
    assert!((q.x - q2.x).abs() < 1e-8 || (q.w + q2.w).abs() < 1e-8);
}

#[test]
fn quat_rotation_matrix_roundtrip() {
    let q = Quat::new(0.7071, 0.0, 0.7071, 0.0).normalize();
    let m = q.to_rotation_matrix();
    let q2 = Quat::from_rotation_matrix(&m);
    let dot = q.w * q2.w + q.x * q2.x + q.y * q2.y + q.z * q2.z;
    assert!((dot.abs() - 1.0).abs() < 1e-8);
}

#[test]
fn quat_slerp_endpoints() {
    let a = Quat::identity();
    let b = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 1.0);
    let s0 = a.slerp(b, 0.0);
    let s1 = a.slerp(b, 1.0);
    assert!((s0.w - a.w).abs() < 1e-8);
    let dot = s1.w * b.w + s1.x * b.x + s1.y * b.y + s1.z * b.z;
    assert!((dot.abs() - 1.0).abs() < 1e-8);
}

#[test]
fn quat_log_exp_roundtrip() {
    let omega = Vec3::new(0.3, -0.5, 0.7);
    let q = Quat::delta(omega, 1.0).normalize();
    let log = q.log();
    let expected_norm = omega.norm();
    let actual_norm = log.norm();
    assert!((actual_norm - expected_norm).abs() < 1e-6);
}

// --- Configuration-space tests ---

#[test]
fn neutral_planar() {
    let model = planar_two_link();
    let q = neutral_configuration(&model);
    assert_eq!(q.len(), model.nq());
    assert_eq!(q, vec![0.0, 0.0]);
}

#[test]
fn neutral_spherical() {
    let model = spherical_model();
    let q = neutral_configuration(&model);
    assert_eq!(q.len(), 4);
    assert!((q[0] - 1.0).abs() < 1e-10);
    assert!((q[1]).abs() < 1e-10);
    assert!((q[2]).abs() < 1e-10);
    assert!((q[3]).abs() < 1e-10);
}

#[test]
fn neutral_freeflyer() {
    let model = freeflyer_model();
    let q = neutral_configuration(&model);
    assert_eq!(q.len(), 7);
    assert_eq!(q[0], 0.0);
    assert_eq!(q[1], 0.0);
    assert_eq!(q[2], 0.0);
    assert!((q[3] - 1.0).abs() < 1e-10);
    assert!((q[4]).abs() < 1e-10);
    assert!((q[5]).abs() < 1e-10);
    assert!((q[6]).abs() < 1e-10);
}

#[test]
fn is_normalized_neutral() {
    let model = spherical_model();
    let q = neutral_configuration(&model);
    assert!(is_normalized(&model, &q, 1e-10).expect("check"));
}

#[test]
fn is_normalized_unnormalized() {
    let model = spherical_model();
    let q = vec![2.0, 0.0, 0.0, 0.0];
    assert!(!is_normalized(&model, &q, 1e-6).expect("check"));
}

#[test]
fn normalize_fixes_unnormalized() {
    let model = spherical_model();
    let q = vec![2.0, 0.0, 0.0, 0.0];
    let nq = normalize_configuration(&model, &q).expect("normalize");
    assert!((nq[0] - 1.0).abs() < 1e-10);
    assert!(is_normalized(&model, &nq, 1e-10).expect("check"));
}

#[test]
fn integrate_spherical_roundtrip() {
    let model = spherical_model();
    let q0 = neutral_configuration(&model);
    let v = vec![0.1, 0.2, 0.3];
    let dt = 0.01;
    let q1 = integrate_model_configuration(&model, &q0, &v, dt).expect("integrate");
    assert_eq!(q1.len(), 4);
    let q1_norm = (q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3]).sqrt();
    assert!((q1_norm - 1.0).abs() < 1e-8, "quaternion not normalized: norm={q1_norm}");
}

#[test]
fn difference_spherical_identity() {
    let model = spherical_model();
    let q = neutral_configuration(&model);
    let dv = difference_configuration(&model, &q, &q).expect("diff");
    assert_eq!(dv.len(), 3);
    for v in &dv {
        assert!(v.abs() < 1e-8, "expected zero diff");
    }
}

#[test]
fn difference_spherical_roundtrip() {
    let model = spherical_model();
    let q0 = neutral_configuration(&model);
    let v = vec![0.1, 0.2, 0.3];
    let dt = 0.01;
    let q1 = integrate_model_configuration(&model, &q0, &v, dt).expect("integrate");
    let dv = difference_configuration(&model, &q0, &q1).expect("diff");
    assert_eq!(dv.len(), 3);
    for i in 0..3 {
        assert!(
            (dv[i] - v[i] * dt).abs() < 1e-6,
            "diff mismatch at {i}: got {} expected {}",
            dv[i],
            v[i] * dt
        );
    }
}

#[test]
fn interpolate_spherical_endpoints() {
    let model = spherical_model();
    let q0 = neutral_configuration(&model);
    let q1 = vec![0.9238795325, 0.0, 0.0, 0.3826834324];
    let mid = interpolate_configuration(&model, &q0, &q1, 0.0).expect("interp");
    for i in 0..4 {
        assert!((mid[i] - q0[i]).abs() < 1e-8);
    }
    let end = interpolate_configuration(&model, &q0, &q1, 1.0).expect("interp");
    let dot = end[0] * q1[0] + end[1] * q1[1] + end[2] * q1[2] + end[3] * q1[3];
    assert!((dot.abs() - 1.0).abs() < 1e-8);
}

#[test]
fn interpolate_freeflyer() {
    let model = freeflyer_model();
    let q0 = neutral_configuration(&model);
    let q1 = vec![1.0, 2.0, 3.0, 0.9238795325, 0.0, 0.0, 0.3826834324];
    let mid = interpolate_configuration(&model, &q0, &q1, 0.5).expect("interp");
    assert_eq!(mid.len(), 7);
    assert!((mid[0] - 0.5).abs() < 1e-8);
    assert!((mid[1] - 1.0).abs() < 1e-8);
    assert!((mid[2] - 1.5).abs() < 1e-8);
    let quat_norm = (mid[3] * mid[3] + mid[4] * mid[4] + mid[5] * mid[5] + mid[6] * mid[6]).sqrt();
    assert!((quat_norm - 1.0).abs() < 1e-8);
}

#[test]
fn random_spherical_is_normalized() {
    let model = spherical_model();
    let lower = vec![0.0; 4];
    let upper = vec![1.0; 4];
    for seed in 0..10 {
        let q = random_configuration(&model, &lower, &upper, seed).expect("rand");
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((norm - 1.0).abs() < 1e-8, "seed={seed} norm={norm}");
    }
}

#[test]
fn random_freeflyer_is_normalized() {
    let model = freeflyer_model();
    let lower = vec![-1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0];
    let upper = vec![1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
    let q = random_configuration(&model, &lower, &upper, 42).expect("rand");
    assert_eq!(q.len(), 7);
    let quat_norm = (q[3] * q[3] + q[4] * q[4] + q[5] * q[5] + q[6] * q[6]).sqrt();
    assert!((quat_norm - 1.0).abs() < 1e-8);
}

#[test]
fn difference_freeflyer_roundtrip() {
    let model = freeflyer_model();
    let q0 = neutral_configuration(&model);
    let v = vec![0.1, -0.2, 0.3, 0.4, 0.1, -0.1];
    let dt = 0.01;
    let q1 = integrate_model_configuration(&model, &q0, &v, dt).expect("integrate");
    let dv = difference_configuration(&model, &q0, &q1).expect("diff");
    assert_eq!(dv.len(), 6);
    for i in 0..3 {
        assert!(
            (dv[i] - v[i] * dt).abs() < 1e-6,
            "linear diff mismatch at {i}"
        );
    }
    for i in 3..6 {
        assert!(
            (dv[i] - v[i] * dt).abs() < 1e-4,
            "angular diff mismatch at {i}: got {} expected {}",
            dv[i],
            v[i] * dt
        );
    }
}
