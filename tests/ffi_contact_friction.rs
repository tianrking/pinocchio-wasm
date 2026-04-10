use pinocchio_wasm::ffi::{
    pino_apply_contact_impulse_friction, pino_contact_constrained_dynamics_friction,
    pino_model_create_from_json, pino_model_free, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_contact_friction_smoke() {
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
    let ws = pino_workspace_new(model);
    assert!(!ws.is_null());

    let q = [0.0, 0.0];
    let qd = [1.0, 0.2];
    let tau = [0.0, 0.0];
    let g = [0.0, 0.0, 0.0];
    let links = [2_i32];
    let points = [1.0, 0.0, 0.0];
    let normals = [0.0, -1.0, 0.0];
    let accel_bias = [0.0];
    let mu = [0.5];

    let mut qdd_out = [0.0; 2];
    let mut ln_out = [0.0; 1];
    let mut lt_out = [0.0; 2];
    let mut f_out = [0.0; 3];
    let s = pino_contact_constrained_dynamics_friction(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        tau.as_ptr(),
        g.as_ptr(),
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        accel_bias.as_ptr(),
        mu.as_ptr(),
        qdd_out.as_mut_ptr(),
        ln_out.as_mut_ptr(),
        lt_out.as_mut_ptr(),
        f_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(qdd_out.iter().all(|x| x.is_finite()));
    assert!(ln_out[0] >= -1e-9);

    let mut qd_plus = [0.0; 2];
    let mut in_out = [0.0; 1];
    let mut it_out = [0.0; 2];
    let mut iw_out = [0.0; 3];
    let s = pino_apply_contact_impulse_friction(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        0.0,
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        mu.as_ptr(),
        qd_plus.as_mut_ptr(),
        in_out.as_mut_ptr(),
        it_out.as_mut_ptr(),
        iw_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(qd_plus.iter().all(|x| x.is_finite()));
    assert!(in_out[0] >= -1e-9);

    pino_workspace_free(ws);
    pino_model_free(model);
}
