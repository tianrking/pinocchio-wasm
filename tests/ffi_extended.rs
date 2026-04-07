use pinocchio_wasm::ffi::{
    pino_aba_batch, pino_model_create_from_json, pino_model_create_from_urdf, pino_model_free,
    pino_model_nq, pino_rnea_batch, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_create_model_from_json_and_urdf() {
    let json = r#"
    {
      "links": [
        {
          "name": "base",
          "parent": null,
          "mass": 0.1,
          "com": [0.0, 0.0, 0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": null
        },
        {
          "name": "link1",
          "parent": 0,
          "mass": 1.0,
          "com": [0.5, 0.0, 0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": {"axis": [0.0,0.0,1.0], "origin": [0.0,0.0,0.0]}
        }
      ]
    }
    "#;
    let m_json = pino_model_create_from_json(json.as_ptr(), json.len());
    assert!(!m_json.is_null());
    assert_eq!(pino_model_nq(m_json), 1);
    pino_model_free(m_json);

    let urdf = r#"
    <robot name="r">
      <link name="base"/>
      <link name="l1"/>
      <joint name="j1" type="revolute">
        <parent link="base"/>
        <child link="l1"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
    "#;
    let m_urdf = pino_model_create_from_urdf(urdf.as_ptr(), urdf.len());
    assert!(!m_urdf.is_null());
    assert_eq!(pino_model_nq(m_urdf), 1);
    pino_model_free(m_urdf);
}

#[test]
fn ffi_batch_rnea_aba_smoke() {
    let json = r#"
    {
      "links": [
        {
          "name": "base",
          "parent": null,
          "mass": 0.1,
          "com": [0.0, 0.0, 0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": null
        },
        {
          "name": "link1",
          "parent": 0,
          "mass": 1.0,
          "com": [0.5, 0.0, 0.0],
          "inertia": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
          "joint": {"axis": [0.0,0.0,1.0], "origin": [0.0,0.0,0.0]}
        },
        {
          "name": "link2",
          "parent": 1,
          "mass": 1.0,
          "com": [0.5, 0.0, 0.0],
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

    let nq = pino_model_nq(model);
    let steps = 2usize;
    let total = nq * steps;

    let q = vec![0.1, -0.3, 0.2, -0.1];
    let qd = vec![0.0, 0.1, 0.2, 0.1];
    let qdd = vec![0.4, -0.2, 0.1, -0.3];
    let tau = vec![1.0, -0.2, 0.8, 0.1];
    let g = [0.0, 0.0, -9.81];

    let mut tau_out = vec![0.0; total];
    let s = pino_rnea_batch(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        steps,
        g.as_ptr(),
        tau_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut qdd_out = vec![0.0; total];
    let s = pino_aba_batch(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        tau.as_ptr(),
        steps,
        g.as_ptr(),
        qdd_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    assert!(tau_out.iter().all(|x| x.is_finite()));
    assert!(qdd_out.iter().all(|x| x.is_finite()));

    pino_workspace_free(ws);
    pino_model_free(model);
}
