use pinocchio_wasm::ffi::{
    pino_center_of_mass_regressor, pino_inverse_dynamics_regressor,
    pino_inverse_dynamics_regressor_batch, pino_kinetic_energy_regressor,
    pino_model_create_from_json, pino_model_free, pino_potential_energy_regressor,
    pino_regressor_select_independent_columns,
};

#[test]
fn ffi_regressor_family_smoke() {
    let json = r#"
    {
      "links": [
        {"name":"base","parent":null,"mass":0.1,"com":[0,0,0],"inertia":[[1,0,0],[0,1,0],[0,0,1]],"joint":null},
        {"name":"l1","parent":0,"mass":1.0,"com":[0.5,0,0],"inertia":[[1,0,0],[0,1,0],[0,0,1]],"joint":{"axis":[0,0,1],"origin":[0,0,0]}},
        {"name":"l2","parent":1,"mass":1.0,"com":[0.5,0,0],"inertia":[[1,0,0],[0,1,0],[0,0,1]],"joint":{"axis":[0,0,1],"origin":[1,0,0]}}
      ]
    }
    "#;
    let model = pino_model_create_from_json(json.as_ptr(), json.len());
    assert!(!model.is_null());

    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let qdd = [0.3, -0.2];
    let g = [0.0, 0.0, -9.81];
    let rows = 2usize;
    let cols = 30usize;
    let mut y = vec![0.0_f64; rows * cols];
    let s = pino_inverse_dynamics_regressor(
        model,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        g.as_ptr(),
        y.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut yb = vec![0.0_f64; 2 * rows * cols];
    let s = pino_inverse_dynamics_regressor_batch(
        model,
        [0.2, -0.3, 0.1, 0.2].as_ptr(),
        [0.4, 0.1, 0.2, -0.1].as_ptr(),
        [0.3, -0.2, 0.1, 0.0].as_ptr(),
        2,
        g.as_ptr(),
        yb.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut ke = vec![0.0_f64; cols];
    let mut pe = vec![0.0_f64; cols];
    let mut com = vec![0.0_f64; 3 * cols];
    let s = pino_kinetic_energy_regressor(model, q.as_ptr(), qd.as_ptr(), ke.as_mut_ptr());
    assert_eq!(s, 0);
    let s = pino_potential_energy_regressor(model, q.as_ptr(), g.as_ptr(), pe.as_mut_ptr());
    assert_eq!(s, 0);
    let s = pino_center_of_mass_regressor(model, q.as_ptr(), com.as_mut_ptr());
    assert_eq!(s, 0);

    let mut cnt = 0usize;
    let mut idx = vec![0usize; cols];
    let mut proj = vec![0.0_f64; rows * cols];
    let s = pino_regressor_select_independent_columns(
        y.as_ptr(),
        rows,
        cols,
        1e-10,
        &mut cnt,
        idx.as_mut_ptr(),
        proj.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(cnt > 0);

    pino_model_free(model);
}
