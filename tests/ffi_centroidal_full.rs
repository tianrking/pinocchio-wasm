use pinocchio_wasm::ffi::{
    pino_centroidal_full_terms, pino_centroidal_full_terms_with_contacts, pino_centroidal_map,
    pino_centroidal_map_derivatives, pino_centroidal_momentum, pino_centroidal_momentum_rate,
    pino_model_create_from_json, pino_model_free, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_centroidal_full_smoke() {
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

    let mut ag = [0.0_f64; 12];
    let s = pino_centroidal_map(model, ws, q.as_ptr(), ag.as_mut_ptr());
    assert_eq!(s, 0);

    let mut dag = [0.0_f64; 24];
    let s = pino_centroidal_map_derivatives(model, ws, q.as_ptr(), dag.as_mut_ptr());
    assert_eq!(s, 0);

    let mut m = [0.0_f64; 6];
    let mut com = [0.0_f64; 3];
    let s = pino_centroidal_momentum(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        m.as_mut_ptr(),
        com.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut hdot = [0.0_f64; 6];
    let s = pino_centroidal_momentum_rate(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        hdot.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut ag2 = [0.0_f64; 12];
    let mut dag2 = [0.0_f64; 24];
    let mut m2 = [0.0_f64; 6];
    let mut hdot2 = [0.0_f64; 6];
    let s = pino_centroidal_full_terms(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        ag2.as_mut_ptr(),
        dag2.as_mut_ptr(),
        m2.as_mut_ptr(),
        hdot2.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let links = [2_i32];
    let points = [1.0, 0.0, 0.0];
    let forces = [0.0, 10.0, 0.0];
    let mut cw = [0.0_f64; 6];
    let s = pino_centroidal_full_terms_with_contacts(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        1,
        links.as_ptr(),
        points.as_ptr(),
        forces.as_ptr(),
        ag2.as_mut_ptr(),
        dag2.as_mut_ptr(),
        m2.as_mut_ptr(),
        hdot2.as_mut_ptr(),
        cw.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    pino_workspace_free(ws);
    pino_model_free(model);
}
