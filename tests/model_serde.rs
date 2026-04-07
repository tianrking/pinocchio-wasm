use pinocchio_wasm::core::math::{Mat3, Vec3};
use pinocchio_wasm::{Joint, Link, Model};

#[test]
fn model_json_roundtrip() {
    let i = Mat3::identity();
    let model = Model::new(vec![
        Link::root("base", 0.1, Vec3::new(0.0, 0.0, 0.0), i),
        Link::child(
            "l1",
            0,
            Joint::revolute(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
            1.0,
            Vec3::new(0.5, 0.0, 0.0),
            i,
        ),
    ])
    .expect("model");

    let s = model.to_json_string().expect("to json");
    let rt = Model::from_json_str(&s).expect("from json");
    assert_eq!(rt.nlinks(), model.nlinks());
    assert_eq!(rt.nq(), model.nq());
}
