use pinocchio_wasm::algo::{
    ContactPoint, aba_derivatives, build_contact_problem, center_of_mass_derivatives,
    centroidal_derivatives, centroidal_map, centroidal_momentum, difference_configuration,
    frame_jacobian_derivatives, integrate_configuration, interpolate_configuration,
    inverse_dynamics_regressor, kinematics_derivatives, random_configuration, rnea_derivatives,
    solve_contact_admm, solve_contact_cholesky, solve_contact_pgs,
};
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

#[test]
fn derivatives_centroidal_regressor_and_solvers_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = vec![0.2, -0.3];
    let qd = vec![0.4, 0.1];
    let qdd = vec![0.3, -0.2];
    let tau = vec![0.1, -0.1];

    let dr = rnea_derivatives(&model, &q, &qd, &qdd, Vec3::new(0.0, 0.0, -9.81), &mut ws)
        .expect("rnea deriv");
    assert_eq!(dr.d_out_dq.len(), model.nv());

    let da = aba_derivatives(&model, &q, &qd, &tau, Vec3::new(0.0, 0.0, -9.81), &mut ws)
        .expect("aba deriv");
    assert_eq!(da.d_out_du.len(), model.nv());

    let kd = kinematics_derivatives(&model, &q, 2, &mut ws).expect("kin deriv");
    assert_eq!(kd.dpos_dq.len(), 3 * model.nv());

    let fd = frame_jacobian_derivatives(&model, &q, 2, &mut ws).expect("frame deriv");
    assert_eq!(fd.dframe_dq.len(), 6 * model.nv() * model.nv());

    let dcom = center_of_mass_derivatives(&model, &q, &mut ws).expect("com deriv");
    assert_eq!(dcom.len(), 3 * model.nv());

    let h = centroidal_momentum(&model, &q, &qd, &mut ws).expect("centroidal");
    assert!(h.total_mass > 0.0);

    let cmap = centroidal_map(&model, &q, &mut ws).expect("cmap");
    assert_eq!(cmap.len(), 6 * model.nv());

    let cd = centroidal_derivatives(&model, &q, &qd, &mut ws).expect("centroidal deriv");
    assert_eq!(cd.d_h_dq.len(), 6);

    let y = inverse_dynamics_regressor(&model, &q, &qd, &qdd, Vec3::new(0.0, 0.0, -9.81))
        .expect("regressor");
    assert_eq!(y.len(), model.nv() * 10 * model.nlinks());

    let contacts = vec![ContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, 1.0, 0.0),
        acceleration_bias: 0.0,
    }];
    let prob = build_contact_problem(
        &model,
        &q,
        &qd,
        &tau,
        &contacts,
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
    )
    .expect("contact problem");
    let x_chol = solve_contact_cholesky(&prob).expect("chol");
    let x_pgs = solve_contact_pgs(&prob, 30);
    let x_admm = solve_contact_admm(&prob, 1.0, 20).expect("admm");
    assert_eq!(x_chol.len(), contacts.len());
    assert_eq!(x_pgs.len(), contacts.len());
    assert_eq!(x_admm.len(), contacts.len());
}

#[test]
fn config_space_tools_smoke() {
    let q0 = vec![0.0, 1.0, 2.0];
    let v = vec![1.0, -1.0, 0.5];
    let q1 = integrate_configuration(&q0, &v, 0.1).expect("integrate");
    let dq = difference_configuration(&q0, &q1).expect("diff");
    let qh = interpolate_configuration(&q0, &q1, 0.5).expect("interp");
    let qr = random_configuration(&[-1.0, -2.0], &[1.0, 2.0], 42).expect("rand");
    assert_eq!(q1.len(), q0.len());
    assert_eq!(dq.len(), q0.len());
    assert_eq!(qh.len(), q0.len());
    assert_eq!(qr.len(), 2);
}
