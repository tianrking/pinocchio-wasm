use pinocchio_wasm::ffi::{
    pino_dealloc, pino_model_create_from_json, pino_model_free, pino_model_to_json,
};

#[test]
fn ffi_model_to_json_smoke() {
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

    let mut out_ptr: *mut u8 = core::ptr::null_mut();
    let mut out_len: usize = 0;
    let code = pino_model_to_json(model, &mut out_ptr, &mut out_len);
    assert_eq!(code, 0);
    assert!(!out_ptr.is_null());
    assert!(out_len > 0);

    let out = unsafe { core::slice::from_raw_parts(out_ptr, out_len) };
    let out_str = core::str::from_utf8(out).expect("utf8");
    assert!(out_str.contains("links"));

    pino_dealloc(out_ptr, out_len);
    pino_model_free(model);
}
