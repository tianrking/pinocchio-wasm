use pinocchio_wasm::ffi::{pino_alloc, pino_dealloc};

#[test]
fn ffi_alloc_dealloc_smoke() {
    let size = 64usize;
    let ptr = pino_alloc(size);
    assert!(!ptr.is_null());

    unsafe {
        let bytes = core::slice::from_raw_parts_mut(ptr, size);
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
    }

    pino_dealloc(ptr, size);
}
