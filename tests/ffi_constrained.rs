use pinocchio_wasm::ffi::{
    pino_constrained_aba_locked_joints, pino_constrained_aba_locked_joints_batch,
    pino_model_create_from_json, pino_model_free, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_constrained_smoke() {
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

    let q = [0.1, -0.2];
    let qd = [0.3, 0.4];
    let tau = [1.0, 0.2];
    let locked = [0_i32, 1_i32];
    let g = [0.0, 0.0, -9.81];
    let mut qdd = [0.0; 2];

    let s = pino_constrained_aba_locked_joints(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        tau.as_ptr(),
        locked.as_ptr(),
        g.as_ptr(),
        qdd.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert_eq!(qdd[1], 0.0);

    let q_batch = [0.1, -0.2, 0.2, -0.1];
    let qd_batch = [0.3, 0.4, 0.1, -0.2];
    let tau_batch = [1.0, 0.2, 0.5, -0.1];
    let mut qdd_batch = [0.0; 4];
    let s = pino_constrained_aba_locked_joints_batch(
        model,
        ws,
        q_batch.as_ptr(),
        qd_batch.as_ptr(),
        tau_batch.as_ptr(),
        2,
        locked.as_ptr(),
        g.as_ptr(),
        qdd_batch.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert_eq!(qdd_batch[1], 0.0);
    assert_eq!(qdd_batch[3], 0.0);

    pino_workspace_free(ws);
    pino_model_free(model);
}
