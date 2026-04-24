use pinocchio_wasm::ffi::{
    pino_collision_min_distance, pino_collision_min_distance_batch,
    pino_collision_min_distance_detailed, pino_collision_min_distance_detailed_batch,
    pino_collision_model_create, pino_collision_model_create_geometries, pino_collision_model_free,
    pino_collision_query_details, pino_model_create_from_json, pino_model_free,
    pino_workspace_free, pino_workspace_new,
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

    let mut detailed_d = 0.0;
    let mut detailed_pair = [0_i32; 2];
    let mut normal = [0.0; 3];
    let mut pa = [0.0; 3];
    let mut pb = [0.0; 3];
    let mut penetration = 0.0;
    let mut colliding = 0_i32;
    let s = pino_collision_min_distance_detailed(
        model,
        coll,
        ws,
        q.as_ptr(),
        &mut detailed_d,
        detailed_pair.as_mut_ptr(),
        normal.as_mut_ptr(),
        pa.as_mut_ptr(),
        pb.as_mut_ptr(),
        &mut penetration,
        &mut colliding,
    );
    assert_eq!(s, 0);
    assert!(detailed_d.is_finite());
    assert!(penetration >= 0.0);

    let mut d_batch = [0.0_f64; 3];
    let mut p_batch = [0.0_f64; 3];
    let s = pino_collision_min_distance_detailed_batch(
        model,
        coll,
        ws,
        q_batch.as_ptr(),
        3,
        d_batch.as_mut_ptr(),
        p_batch.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(d_batch.iter().all(|x| x.is_finite()));
    assert!(p_batch.iter().all(|x| *x >= 0.0));

    pino_collision_model_free(coll);

    let types = [1_i32, 2_i32, 3_i32, 4_i32];
    let links2 = [1_i32, 2_i32, 2_i32, 1_i32];
    let centers2 = [
        0.5, 0.0, 0.0, //
        0.5, 0.0, 0.0, //
        0.2, 0.0, 0.0, //
        0.3, 0.0, 0.0,
    ];
    let params2 = [
        0.2, 0.1, 0.1, //
        0.3, 0.1, 0.0, //
        0.2, 0.1, 0.0, //
        0.1, 0.1, 0.1,
    ];
    let filter = [1_i32, 0_i32];
    let coll2 = pino_collision_model_create_geometries(
        4,
        types.as_ptr(),
        links2.as_ptr(),
        centers2.as_ptr(),
        params2.as_ptr(),
        filter.as_ptr(),
    );
    assert!(!coll2.is_null());

    let mut d2 = 0.0;
    let mut p2 = [0_i32; 2];
    let s = pino_collision_min_distance(model, coll2, ws, q.as_ptr(), &mut d2, p2.as_mut_ptr());
    assert_eq!(s, 0);
    assert!(d2.is_finite());

    let mut count = 0usize;
    let mut pairs_flat = [0_i32; 16];
    let mut dist = [0.0_f64; 8];
    let mut normal_flat = [0.0_f64; 24];
    let mut pa_flat = [0.0_f64; 24];
    let mut pb_flat = [0.0_f64; 24];
    let mut pen = [0.0_f64; 8];
    let mut colliding = [0_i32; 8];
    let s = pino_collision_query_details(
        model,
        coll2,
        ws,
        q.as_ptr(),
        8,
        &mut count,
        pairs_flat.as_mut_ptr(),
        dist.as_mut_ptr(),
        normal_flat.as_mut_ptr(),
        pa_flat.as_mut_ptr(),
        pb_flat.as_mut_ptr(),
        pen.as_mut_ptr(),
        colliding.as_mut_ptr(),
    );
    assert_eq!(s, 0);
    assert!(count > 0);
    assert!(dist[..count].iter().all(|x| x.is_finite()));
    assert!(pen[..count].iter().all(|x| *x >= 0.0));
    pino_collision_model_free(coll2);
    pino_workspace_free(ws);
    pino_model_free(model);
}
