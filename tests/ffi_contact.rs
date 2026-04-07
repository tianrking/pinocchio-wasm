use pinocchio_wasm::ffi::{
    pino_apply_contact_impulse, pino_apply_contact_impulse_batch,
    pino_contact_constrained_dynamics, pino_contact_constrained_dynamics_batch,
    pino_model_create_from_json, pino_model_free, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_contact_and_impulse_smoke() {
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
    let qd = [0.8, -0.2];
    let tau = [0.0, 0.0];
    let gravity = [0.0, 0.0, 0.0];
    let links = [2_i32];
    let points = [1.0, 0.0, 0.0];
    let normals = [0.0, -1.0, 0.0];
    let bias = [0.0];
    let mut qdd_out = [0.0; 2];
    let mut lambda_out = [0.0; 1];

    let s = pino_contact_constrained_dynamics(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        tau.as_ptr(),
        gravity.as_ptr(),
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        bias.as_ptr(),
        qdd_out.as_mut_ptr(),
        lambda_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(qdd_out.iter().all(|v| v.is_finite()));
    assert!(lambda_out[0].is_finite());

    let mut qd_plus = [0.0; 2];
    let mut impulse = [0.0; 1];
    let s = pino_apply_contact_impulse(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        0.0,
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        qd_plus.as_mut_ptr(),
        impulse.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(qd_plus.iter().all(|v| v.is_finite()));
    assert!(impulse[0].is_finite());

    let q_batch = [0.0, 0.0, 0.1, -0.2];
    let qd_batch = [0.8, -0.2, 1.1, 0.2];
    let tau_batch = [0.0, 0.0, 0.0, 0.0];
    let mut qdd_batch_out = [0.0; 4];
    let mut lambda_batch_out = [0.0; 2];
    let s = pino_contact_constrained_dynamics_batch(
        model,
        ws,
        q_batch.as_ptr(),
        qd_batch.as_ptr(),
        tau_batch.as_ptr(),
        2,
        gravity.as_ptr(),
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        bias.as_ptr(),
        qdd_batch_out.as_mut_ptr(),
        lambda_batch_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut qd_plus_batch = [0.0; 4];
    let mut impulse_batch = [0.0; 2];
    let s = pino_apply_contact_impulse_batch(
        model,
        ws,
        q_batch.as_ptr(),
        qd_batch.as_ptr(),
        2,
        0.0,
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        qd_plus_batch.as_mut_ptr(),
        impulse_batch.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    pino_workspace_free(ws);
    pino_model_free(model);
}
