use pinocchio_wasm::ffi::{
    pino_forward_kinematics_poses, pino_forward_kinematics_poses_batch,
    pino_model_create_from_json, pino_model_free, pino_model_nlinks, pino_workspace_free,
    pino_workspace_new,
};

#[test]
fn ffi_fk_poses_smoke() {
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
        }
      ]
    }
    "#;

    let model = pino_model_create_from_json(json.as_ptr(), json.len());
    assert!(!model.is_null());
    let ws = pino_workspace_new(model);
    assert!(!ws.is_null());

    let nl = pino_model_nlinks(model);
    let q = [0.2_f64];
    let mut t = vec![0.0; 3 * nl];
    let mut r = vec![0.0; 9 * nl];
    let s = pino_forward_kinematics_poses(model, ws, q.as_ptr(), t.as_mut_ptr(), r.as_mut_ptr());
    assert_eq!(s, 0);

    let q_batch = [0.2_f64, -0.3_f64];
    let mut tb = vec![0.0; 2 * 3 * nl];
    let mut rb = vec![0.0; 2 * 9 * nl];
    let s = pino_forward_kinematics_poses_batch(
        model,
        ws,
        q_batch.as_ptr(),
        2,
        tb.as_mut_ptr(),
        rb.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(tb.iter().all(|x| x.is_finite()));
    assert!(rb.iter().all(|x| x.is_finite()));

    pino_workspace_free(ws);
    pino_model_free(model);
}
