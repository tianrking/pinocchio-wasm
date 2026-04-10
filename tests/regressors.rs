use pinocchio_wasm::algo::{
    center_of_mass_regressor, inverse_dynamics_regressor, inverse_dynamics_regressor_batch,
    kinetic_energy_regressor, potential_energy_regressor, select_independent_regressor_columns,
};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model};

fn model() -> Model {
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
fn regressor_family_smoke() {
    let m = model();
    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let qdd = [0.3, -0.2];
    let y = inverse_dynamics_regressor(&m, &q, &qd, &qdd, Vec3::new(0.0, 0.0, -9.81))
        .expect("id reg");
    let rows = m.nv();
    let cols = 10 * m.nlinks();
    assert_eq!(y.len(), rows * cols);

    let yb = inverse_dynamics_regressor_batch(
        &m,
        &[0.2, -0.3, 0.1, 0.2],
        &[0.4, 0.1, 0.2, -0.1],
        &[0.3, -0.2, 0.1, 0.0],
        2,
        Vec3::new(0.0, 0.0, -9.81),
    )
    .expect("id reg batch");
    assert_eq!(yb.len(), 2 * rows * cols);

    let ke = kinetic_energy_regressor(&m, &q, &qd).expect("ke reg");
    let pe = potential_energy_regressor(&m, &q, Vec3::new(0.0, 0.0, -9.81)).expect("pe reg");
    let com = center_of_mass_regressor(&m, &q).expect("com reg");
    assert_eq!(ke.len(), cols);
    assert_eq!(pe.len(), cols);
    assert_eq!(com.len(), 3 * cols);

    let basis = select_independent_regressor_columns(&y, rows, cols, 1e-10).expect("basis");
    assert!(basis.cols > 0);
    assert_eq!(basis.rows, rows);
    assert_eq!(basis.projected_row_major.len(), rows * basis.cols);
}
