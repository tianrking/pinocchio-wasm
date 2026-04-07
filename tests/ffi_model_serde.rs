use pinocchio_wasm::ffi::{
    pino_dealloc, pino_model_create_from_json, pino_model_free, pino_model_to_json,
    pino_model_to_mjcf, pino_model_to_sdf, pino_model_to_urdf,
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

    let name = b"r";

    let mut urdf_ptr: *mut u8 = core::ptr::null_mut();
    let mut urdf_len: usize = 0;
    let code = pino_model_to_urdf(
        model,
        name.as_ptr(),
        name.len(),
        &mut urdf_ptr,
        &mut urdf_len,
    );
    assert_eq!(code, 0);
    assert!(urdf_len > 0);
    pino_dealloc(urdf_ptr, urdf_len);

    let mut sdf_ptr: *mut u8 = core::ptr::null_mut();
    let mut sdf_len: usize = 0;
    let code = pino_model_to_sdf(model, name.as_ptr(), name.len(), &mut sdf_ptr, &mut sdf_len);
    assert_eq!(code, 0);
    assert!(sdf_len > 0);
    pino_dealloc(sdf_ptr, sdf_len);

    let mut mjcf_ptr: *mut u8 = core::ptr::null_mut();
    let mut mjcf_len: usize = 0;
    let code = pino_model_to_mjcf(
        model,
        name.as_ptr(),
        name.len(),
        &mut mjcf_ptr,
        &mut mjcf_len,
    );
    assert_eq!(code, 0);
    assert!(mjcf_len > 0);
    pino_dealloc(mjcf_ptr, mjcf_len);

    pino_model_free(model);
}
