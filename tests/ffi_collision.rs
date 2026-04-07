use pinocchio_wasm::ffi::{
    pino_collision_min_distance, pino_collision_min_distance_batch, pino_collision_model_create,
    pino_collision_model_free, pino_model_create_from_json, pino_model_free, pino_workspace_free,
    pino_workspace_new,
};

#[test]
fn ffi_collision_distance_smoke() {
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

    let links = [1_i32, 2_i32];
    let centers = [0.5, 0.0, 0.0, 0.5, 0.0, 0.0];
    let radii = [0.2, 0.2];
    let coll = pino_collision_model_create(2, links.as_ptr(), centers.as_ptr(), radii.as_ptr());
    assert!(!coll.is_null());

    let q = [0.0, 0.0];
    let mut d = 0.0;
    let mut pair = [0_i32; 2];
    let s = pino_collision_min_distance(model, coll, ws, q.as_ptr(), &mut d, pair.as_mut_ptr());
    assert_eq!(s, 0);
    assert!(d.is_finite());

    let q_batch = [0.0, 0.0, 0.3, -0.2, -0.4, 0.5];
    let mut out = [0.0_f64; 3];
    let s =
        pino_collision_min_distance_batch(model, coll, ws, q_batch.as_ptr(), 3, out.as_mut_ptr());
    assert_eq!(s, 0);
    assert!(out.iter().all(|x| x.is_finite()));

    pino_collision_model_free(coll);
    pino_workspace_free(ws);
    pino_model_free(model);
}
