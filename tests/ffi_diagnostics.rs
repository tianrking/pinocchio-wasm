use pinocchio_wasm::ffi::{
    pino_bias_forces_batch, pino_coriolis_torques, pino_crba_batch, pino_gravity_torques,
    pino_gravity_torques_batch, pino_model_create_from_json, pino_model_free, pino_workspace_free,
    pino_workspace_new,
};

#[test]
fn ffi_diagnostics_smoke() {
    let json = r#"
    {
      "links": [
        {
          "name": "base",
          "parent": null,
          "mass": 0.1,
          "com": [0.0,0.0,0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": null
        },
        {
          "name": "l1",
          "parent": 0,
          "mass": 1.0,
          "com": [0.5,0.0,0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": {"axis": [0.0,0.0,1.0], "origin": [0.0,0.0,0.0]}
        },
        {
          "name": "l2",
          "parent": 1,
          "mass": 1.0,
          "com": [0.5,0.0,0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": {"axis": [0.0,0.0,1.0], "origin": [1.0,0.0,0.0]}
        }
      ]
    }
    "#;

    let model = pino_model_create_from_json(json.as_ptr(), json.len());
    assert!(!model.is_null());
    let ws = pino_workspace_new(model);
    assert!(!ws.is_null());

    let q = [0.2, -0.3];
    let qd = [0.4, 0.1];
    let g = [0.0, 0.0, -9.81];

    let mut g_out = [0.0; 2];
    let s = pino_gravity_torques(model, ws, q.as_ptr(), g.as_ptr(), g_out.as_mut_ptr());
    assert_eq!(s, 0);

    let mut c_out = [0.0; 2];
    let s = pino_coriolis_torques(model, ws, q.as_ptr(), qd.as_ptr(), c_out.as_mut_ptr());
    assert_eq!(s, 0);

    let q_batch = [0.1, -0.2, 0.3, -0.1];
    let qd_batch = [0.2, 0.1, 0.0, -0.3];
    let mut b_batch = [0.0; 4];
    let s = pino_bias_forces_batch(
        model,
        ws,
        q_batch.as_ptr(),
        qd_batch.as_ptr(),
        2,
        g.as_ptr(),
        b_batch.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut g_batch = [0.0; 4];
    let s = pino_gravity_torques_batch(
        model,
        ws,
        q_batch.as_ptr(),
        2,
        g.as_ptr(),
        g_batch.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut m_batch = [0.0; 8];
    let s = pino_crba_batch(model, ws, q_batch.as_ptr(), 2, m_batch.as_mut_ptr());
    assert_eq!(s, 0);

    assert!(b_batch.iter().all(|x| x.is_finite()));
    assert!(g_batch.iter().all(|x| x.is_finite()));
    assert!(m_batch.iter().all(|x| x.is_finite()));

    pino_workspace_free(ws);
    pino_model_free(model);
}
