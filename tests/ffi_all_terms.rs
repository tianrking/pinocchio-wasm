use pinocchio_wasm::ffi::{
    pino_compute_all_terms, pino_contact_jacobian_normal, pino_model_create_from_json,
    pino_model_free, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_all_terms_and_contact_jac_smoke() {
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

    let q = [0.2, -0.4];
    let qd = [0.7, 0.1];
    let g = [0.0, 0.0, -9.81];
    let mut mass = [0.0; 4];
    let mut bias = [0.0; 2];
    let mut grav = [0.0; 2];
    let mut cor = [0.0; 2];
    let mut com = [0.0; 3];
    let mut ke = 0.0;
    let mut pe = 0.0;

    let s = pino_compute_all_terms(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        g.as_ptr(),
        mass.as_mut_ptr(),
        bias.as_mut_ptr(),
        grav.as_mut_ptr(),
        cor.as_mut_ptr(),
        com.as_mut_ptr(),
        &mut ke,
        &mut pe,
    );
    assert_eq!(s, 0);
    assert!(mass.iter().all(|x| x.is_finite()));
    assert!(bias.iter().all(|x| x.is_finite()));
    assert!(grav.iter().all(|x| x.is_finite()));
    assert!(cor.iter().all(|x| x.is_finite()));
    assert!(com.iter().all(|x| x.is_finite()));
    assert!(ke.is_finite());
    assert!(pe.is_finite());

    let links = [2_i32];
    let points = [1.0, 0.0, 0.0];
    let normals = [0.0, 1.0, 0.0];
    let mut jac = [0.0; 2];
    let s = pino_contact_jacobian_normal(
        model,
        ws,
        q.as_ptr(),
        1,
        links.as_ptr(),
        points.as_ptr(),
        normals.as_ptr(),
        jac.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(jac.iter().all(|x| x.is_finite()));

    pino_workspace_free(ws);
    pino_model_free(model);
}
