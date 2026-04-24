use pinocchio_wasm::ffi::{
    pino_aba, pino_center_of_mass, pino_crba, pino_energy, pino_model_create, pino_model_free,
    pino_model_nq, pino_rnea, pino_workspace_free, pino_workspace_new,
};

#[test]
fn ffi_roundtrip_smoke() {
    let nlinks = 3usize;
    let parents = vec![-1, 0, 1];
    let axes = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
    let origins = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0];
    let masses = vec![0.1, 1.0, 1.0];
    let coms = vec![0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0];
    let mut inertias = vec![0.0; 9 * nlinks];
    for i in 0..nlinks {
        inertias[9 * i] = 1.0;
        inertias[9 * i + 4] = 1.0;
        inertias[9 * i + 8] = 1.0;
    }

    let joint_types = vec![0i32; nlinks]; // all revolute
    let model = pino_model_create(
        nlinks,
        parents.as_ptr(),
        axes.as_ptr(),
        origins.as_ptr(),
        masses.as_ptr(),
        coms.as_ptr(),
        inertias.as_ptr(),
        joint_types.as_ptr(),
    );
    assert!(!model.is_null());

    let nq = pino_model_nq(model);
    assert_eq!(nq, 2);

    let ws = pino_workspace_new(model);
    assert!(!ws.is_null());

    let q = vec![0.2, -0.4];
    let qd = vec![0.5, 0.1];
    let qdd = vec![0.9, -0.7];
    let tau = vec![1.0, -0.2];
    let g = vec![0.0, 0.0, -9.81];

    let mut tau_out = vec![0.0; nq];
    let s = pino_rnea(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        qdd.as_ptr(),
        g.as_ptr(),
        tau_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut qdd_out = vec![0.0; nq];
    let s = pino_aba(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        tau.as_ptr(),
        g.as_ptr(),
        qdd_out.as_mut_ptr(),
    );
    assert_eq!(s, 0);

    let mut mass = vec![0.0; nq * nq];
    let s = pino_crba(model, ws, q.as_ptr(), mass.as_mut_ptr());
    assert_eq!(s, 0);

    let mut com = vec![0.0; 3];
    let s = pino_center_of_mass(model, ws, q.as_ptr(), com.as_mut_ptr());
    assert_eq!(s, 0);

    let mut ke = 0.0;
    let mut pe = 0.0;
    let s = pino_energy(
        model,
        ws,
        q.as_ptr(),
        qd.as_ptr(),
        g.as_ptr(),
        &mut ke,
        &mut pe,
    );
    assert_eq!(s, 0);
    assert!(ke >= 0.0);
    assert!(pe.is_finite());

    pino_workspace_free(ws);
    pino_model_free(model);
}
