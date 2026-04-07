use pinocchio_wasm::algo::{center_of_mass, rnea};
use pinocchio_wasm::core::math::Vec3;
use pinocchio_wasm::{Model, Workspace};

#[test]
fn build_model_from_json_and_run_algorithms() {
    let json = r#"
    {
      "links": [
        {
          "name": "base",
          "parent": null,
          "mass": 0.1,
          "com": [0.0, 0.0, 0.0],
          "inertia": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
          "joint": null
        },
        {
          "name": "link1",
          "parent": 0,
          "mass": 1.0,
          "com": [0.5, 0.0, 0.0],
          "inertia": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
          "joint": {
            "axis": [0.0, 0.0, 1.0],
            "origin": [0.0, 0.0, 0.0]
          }
        },
        {
          "name": "link2",
          "parent": 1,
          "mass": 1.0,
          "com": [0.5, 0.0, 0.0],
          "inertia": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
          "joint": {
            "axis": [0.0, 0.0, 1.0],
            "origin": [1.0, 0.0, 0.0]
          }
        }
      ]
    }
    "#;

    let model = Model::from_json_str(json).expect("json model");
    assert_eq!(model.nlinks(), 3);
    assert_eq!(model.nq(), 2);

    let mut ws = Workspace::new(&model);
    let q = vec![0.3, -0.4];
    let qd = vec![0.2, 0.1];
    let qdd = vec![0.5, -0.3];

    let tau = rnea(&model, &q, &qd, &qdd, Vec3::new(0.0, 0.0, -9.81), &mut ws).expect("rnea");
    assert_eq!(tau.len(), model.nq());

    let com = center_of_mass(&model, &q, &mut ws).expect("com");
    assert!(com.norm2() > 0.0);
}

#[test]
fn reject_invalid_json_model() {
    let bad = r#"{"links": []}"#;
    let err = Model::from_json_str(bad).expect_err("should reject");
    assert!(format!("{err}").contains("json model"));
}
