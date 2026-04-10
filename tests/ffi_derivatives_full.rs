use pinocchio_wasm::ffi::{
    pino_constrained_dynamics_derivatives_locked_joints, pino_impulse_dynamics_derivatives,
    pino_model_create_from_json, pino_model_free, pino_rnea_second_order_derivatives,
    pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_derivatives_family_smoke() {
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

    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let qdd = [0.3, -0.2];
    let g = [0.0, 0.0, -9.81];
    let mut d2q = [0.0_f64; 8];
    let mut d2v = [0.0_f64; 8];
    let mut d2u = [0.0_f64; 8];
    let s = pino_rnea_second_order_derivatives(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        g.as_ptr(),
        d2q.as_mut_ptr(),
        d2v.as_mut_ptr(),
        d2u.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let locked = [0_i32, 1_i32];
    let mut dq = [0.0_f64; 4];
    let mut dv = [0.0_f64; 4];
    let mut du = [0.0_f64; 4];
    let s = pino_constrained_dynamics_derivatives_locked_joints(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        locked.as_ptr(),
        g.as_ptr(),
        dq.as_mut_ptr(),
        dv.as_mut_ptr(),
        du.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let links = [2_i32];
    let points = [1.0, 0.0, 0.0];
    let normals = [0.0, 1.0, 0.0];
    let mut idq = [0.0_f64; 4];
    let mut idv = [0.0_f64; 4];
    let mut idr = [0.0_f64; 2];
    let s = pino_impulse_dynamics_derivatives(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        0.0,
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        idq.as_mut_ptr(),
        idv.as_mut_ptr(),
        idr.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    pino_workspace_free(ws);
    pino_model_free(model);
}
