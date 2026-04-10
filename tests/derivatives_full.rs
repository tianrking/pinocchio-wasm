use pinocchio_wasm::algo::{
    ContactPoint, constrained_dynamics_derivatives_locked_joints, impulse_dynamics_derivatives,
    rnea_second_order_derivatives,
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
fn second_order_and_constraint_impulse_derivatives_smoke() {
    let model = planar_two_link();
    let mut ws = Workspace::new(&model);
    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let qdd = [0.3, -0.2];

    let so = rnea_second_order_derivatives(
        &model,
        &q,
        &qd,
        &qdd,
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
    )
    .expect("second order");
    let n = model.nv();
    assert_eq!(so.d2_out_dq2.len(), n * n * n);
    assert_eq!(so.d2_out_dv2.len(), n * n * n);
    assert_eq!(so.d2_out_du2.len(), n * n * n);

    let locked = [false, true];
    let cd = constrained_dynamics_derivatives_locked_joints(
        &model,
        &q,
        &qd,
        &[0.1, -0.2],
        &locked,
        Vec3::new(0.0, 0.0, -9.81),
        &mut ws,
    )
    .expect("constrained deriv");
    assert_eq!(cd.d_out_dq.len(), n);
    assert_eq!(cd.d_out_dv.len(), n);
    assert_eq!(cd.d_out_du.len(), n);

    let contacts = vec![ContactPoint {
        link_index: 2,
        point_local: Vec3::new(1.0, 0.0, 0.0),
        normal_world: Vec3::new(0.0, 1.0, 0.0),
        acceleration_bias: 0.0,
    }];
    let id = impulse_dynamics_derivatives(&model, &q, &qd, &contacts, 0.0, &mut ws)
        .expect("impulse deriv");
    assert_eq!(id.d_qd_plus_dq.len(), n);
    assert_eq!(id.d_qd_plus_dqd_minus.len(), n);
    assert_eq!(id.d_qd_plus_d_restitution.len(), n);
}
