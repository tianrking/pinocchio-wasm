use pinocchio_wasm::autodiff::{Dual, jacobian_fd};
use pinocchio_wasm::codegen::{generate_c_header_summary, generate_js_loader};
use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::visualization::ascii_tree;
use pinocchio_wasm::{Joint, Link, Model};

fn model() -> Model {
    let i = Mat3::identity();
    Model::new(vec![
        Link::root("base", 1.0, Vec3::zero(), i),
        Link::child(
            "l1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.1, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model")
}

#[test]
fn autodiff_and_fd_smoke() {
    let x = Dual::new(0.2, 1.0);
    let y = x.sin() * x + x.cos();
    assert!(y.value.is_finite());
    assert!(y.grad.is_finite());

    let j = jacobian_fd(&[1.0, 2.0], 2, |x| Ok(vec![x[0] * x[0], x[0] + x[1]])).expect("jac fd");
    assert_eq!(j.len(), 4);
}

#[test]
fn codegen_and_visualization_smoke() {
    let m = model();
    let c = generate_c_header_summary(&m);
    let js = generate_js_loader("Pinocchio");
    let tree = ascii_tree(&m);
    assert!(c.contains("nlinks"));
    assert!(js.contains("loadPinocchio"));
    assert!(tree.contains("base"));
}
