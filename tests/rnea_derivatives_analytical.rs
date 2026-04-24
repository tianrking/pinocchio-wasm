use pinocchio_wasm::algo::{crba, rnea_derivatives, rnea_derivatives_fd};
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

fn planar_three_link() -> Model {
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
        Link::child(
            "l3",
            2,
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

const TOL: f64 = 1e-6;

fn assert_match(model: &Model, q: &[f64], qd: &[f64], qdd: &[f64], gravity: Vec3, label: &str) {
    let mut ws1 = Workspace::new(model);
    let mut ws2 = Workspace::new(model);
    let an = rnea_derivatives(model, q, qd, qdd, gravity, &mut ws1).expect("analytical");
    let fd = rnea_derivatives_fd(model, q, qd, qdd, gravity, &mut ws2).expect("fd");
    let n = model.nv();

    let pairs: [(&str, &Vec<Vec<f64>>, &Vec<Vec<f64>>); 3] = [
        ("dtau_dq", &an.d_out_dq, &fd.d_out_dq),
        ("dtau_dqd", &an.d_out_dv, &fd.d_out_dv),
        ("dtau_dqdd", &an.d_out_du, &fd.d_out_du),
    ];
    for (name, mat_an, mat_fd) in &pairs {
        for r in 0..n {
            for c in 0..n {
                let diff = (mat_an[r][c] - mat_fd[r][c]).abs();
                assert!(
                    diff < TOL,
                    "{} {}: [{}][{}] an={} fd={} diff={}",
                    label,
                    name,
                    r,
                    c,
                    mat_an[r][c],
                    mat_fd[r][c],
                    diff
                );
            }
        }
    }
}

#[test]
fn rnea_derivatives_planar_two_link() {
    let model = planar_two_link();
    let g = Vec3::new(0.0, 0.0, -9.81);
    assert_match(
        &model,
        &[0.0, 0.0],
        &[0.0, 0.0],
        &[0.0, 0.0],
        g,
        "two_link_zero",
    );
    assert_match(
        &model,
        &[0.5, -0.3],
        &[0.4, 0.1],
        &[0.3, -0.2],
        g,
        "two_link_general",
    );
}

#[test]
fn rnea_derivatives_planar_three_link() {
    let model = planar_three_link();
    let g = Vec3::new(0.0, 0.0, -9.81);
    assert_match(
        &model,
        &[0.0, 0.0, 0.0],
        &[0.0, 0.0, 0.0],
        &[0.0, 0.0, 0.0],
        g,
        "three_link_zero",
    );
    assert_match(
        &model,
        &[0.3, -0.2, 0.5],
        &[0.1, 0.2, -0.1],
        &[0.2, 0.0, -0.3],
        g,
        "three_link_general",
    );
}

#[test]
fn rnea_derivatives_mixed_joints() {
    let model = mixed_joint_model();
    let g = Vec3::new(0.0, 0.0, -9.81);
    assert_match(
        &model,
        &[0.0, 0.0],
        &[0.0, 0.0],
        &[0.0, 0.0],
        g,
        "mixed_zero",
    );
    assert_match(
        &model,
        &[0.5, 0.3],
        &[0.2, -0.1],
        &[0.1, 0.4],
        g,
        "mixed_general",
    );
}

#[test]
fn rnea_derivatives_zero_velocity() {
    let model = planar_two_link();
    let g = Vec3::new(0.0, 0.0, -9.81);
    assert_match(
        &model,
        &[0.5, -0.3],
        &[0.0, 0.0],
        &[0.0, 0.0],
        g,
        "zero_vel",
    );
}

#[test]
fn rnea_derivatives_zero_gravity() {
    let model = planar_two_link();
    let g = Vec3::zero();
    assert_match(
        &model,
        &[0.5, -0.3],
        &[0.4, 0.1],
        &[0.3, -0.2],
        g,
        "zero_grav",
    );
}

#[test]
fn rnea_derivatives_dqdd_equals_crba() {
    let model = planar_two_link();
    let g = Vec3::new(0.0, 0.0, -9.81);
    let q = [0.5, -0.3];
    let qd = [0.4, 0.1];
    let qdd = [0.3, -0.2];
    let mut ws1 = Workspace::new(&model);
    let mut ws2 = Workspace::new(&model);
    let an = rnea_derivatives(&model, &q, &qd, &qdd, g, &mut ws1).expect("analytical");
    let mass = crba(&model, &q, &mut ws2).expect("crba");
    let n = model.nv();
    for r in 0..n {
        for c in 0..n {
            let diff = (an.d_out_du[r][c] - mass[r][c]).abs();
            assert!(
                diff < 1e-10,
                "dtau_dqdd vs CRBA [{}][{}]: deriv={} crba={} diff={}",
                r,
                c,
                an.d_out_du[r][c],
                mass[r][c],
                diff
            );
        }
    }
}
