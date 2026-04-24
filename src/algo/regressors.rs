use crate::core::error::{PinocchioError, Result};
use crate::core::math::{Mat3, Vec3};
use crate::model::{Model, Workspace};

use super::com_energy::{center_of_mass, kinetic_energy, potential_energy};
use super::dynamics::rnea;

#[derive(Debug, Clone)]
pub struct RegressorBasisResult {
    pub selected_columns: Vec<usize>,
    pub projected_row_major: Vec<f64>,
    pub rows: usize,
    pub cols: usize,
}

fn inertial_params_of_link(model: &Model, link_idx: usize) -> [f64; 10] {
    let l = &model.links[link_idx];
    [
        l.mass,
        l.mass * l.com_local.x,
        l.mass * l.com_local.y,
        l.mass * l.com_local.z,
        l.inertia_local_com.m[0][0],
        l.inertia_local_com.m[1][1],
        l.inertia_local_com.m[2][2],
        l.inertia_local_com.m[0][1],
        l.inertia_local_com.m[0][2],
        l.inertia_local_com.m[1][2],
    ]
}

fn set_inertial_params_of_link(model: &mut Model, link_idx: usize, p: [f64; 10]) {
    let mass = p[0].max(1e-6);
    model.links[link_idx].mass = mass;
    model.links[link_idx].com_local = Vec3::new(p[1] / mass, p[2] / mass, p[3] / mass);
    model.links[link_idx].inertia_local_com =
        Mat3::new([[p[4], p[7], p[8]], [p[7], p[5], p[9]], [p[8], p[9], p[6]]]);
}

pub fn inverse_dynamics_regressor(
    model: &Model,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    gravity: Vec3,
) -> Result<Vec<f64>> {
    let n = model.nv();
    model.check_state_dims(q, qd, Some(qdd))?;
    let p = 10 * model.nlinks();
    let mut y = vec![0.0; n * p];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base_param = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base_param;
            let mut p_minus = base_param;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let tau_plus = rnea(&plus, q, qd, qdd, gravity, &mut ws_plus)?;
            let tau_minus = rnea(&minus, q, qd, qdd, gravity, &mut ws_minus)?;
            let col = link_idx * 10 + k;
            for row in 0..n {
                y[row * p + col] = (tau_plus[row] - tau_minus[row]) / (2.0 * eps);
            }
        }
    }
    Ok(y)
}

pub fn inverse_dynamics_regressor_batch(
    model: &Model,
    q_batch: &[f64],
    qd_batch: &[f64],
    qdd_batch: &[f64],
    batch_size: usize,
    gravity: Vec3,
) -> Result<Vec<f64>> {
    let n = model.nv();
    let p = 10 * model.nlinks();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected || qd_batch.len() != expected || qdd_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len().min(qd_batch.len()).min(qdd_batch.len()),
        });
    }
    let mut out = vec![0.0; batch_size * n * p];
    for step in 0..batch_size {
        let b = step * n;
        let y = inverse_dynamics_regressor(
            model,
            &q_batch[b..b + n],
            &qd_batch[b..b + n],
            &qdd_batch[b..b + n],
            gravity,
        )?;
        let ob = step * n * p;
        out[ob..ob + n * p].copy_from_slice(&y);
    }
    Ok(out)
}

pub fn kinetic_energy_regressor(model: &Model, q: &[f64], qd: &[f64]) -> Result<Vec<f64>> {
    let nparam = 10 * model.nlinks();
    let mut out = vec![0.0; nparam];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base;
            let mut p_minus = base;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let e_plus = kinetic_energy(&plus, q, qd, &mut ws_plus)?;
            let e_minus = kinetic_energy(&minus, q, qd, &mut ws_minus)?;
            out[link_idx * 10 + k] = (e_plus - e_minus) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn potential_energy_regressor(model: &Model, q: &[f64], gravity: Vec3) -> Result<Vec<f64>> {
    let nparam = 10 * model.nlinks();
    let mut out = vec![0.0; nparam];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base;
            let mut p_minus = base;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let e_plus = potential_energy(&plus, q, gravity, &mut ws_plus)?;
            let e_minus = potential_energy(&minus, q, gravity, &mut ws_minus)?;
            out[link_idx * 10 + k] = (e_plus - e_minus) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn center_of_mass_regressor(model: &Model, q: &[f64]) -> Result<Vec<f64>> {
    let p = 10 * model.nlinks();
    let mut out = vec![0.0; 3 * p];
    let eps = 1e-6;
    for link_idx in 0..model.nlinks() {
        let base = inertial_params_of_link(model, link_idx);
        for k in 0..10 {
            let mut plus = model.clone();
            let mut minus = model.clone();
            let mut p_plus = base;
            let mut p_minus = base;
            p_plus[k] += eps;
            p_minus[k] -= eps;
            set_inertial_params_of_link(&mut plus, link_idx, p_plus);
            set_inertial_params_of_link(&mut minus, link_idx, p_minus);
            let mut ws_plus = Workspace::new(&plus);
            let mut ws_minus = Workspace::new(&minus);
            let c_plus = center_of_mass(&plus, q, &mut ws_plus)?;
            let c_minus = center_of_mass(&minus, q, &mut ws_minus)?;
            let col = link_idx * 10 + k;
            out[col] = (c_plus.x - c_minus.x) / (2.0 * eps);
            out[p + col] = (c_plus.y - c_minus.y) / (2.0 * eps);
            out[2 * p + col] = (c_plus.z - c_minus.z) / (2.0 * eps);
        }
    }
    Ok(out)
}

pub fn select_independent_regressor_columns(
    regressor_row_major: &[f64],
    rows: usize,
    cols: usize,
    tolerance: f64,
) -> Result<RegressorBasisResult> {
    if regressor_row_major.len() != rows * cols {
        return Err(PinocchioError::DimensionMismatch {
            expected: rows * cols,
            got: regressor_row_major.len(),
        });
    }
    if tolerance <= 0.0 {
        return Err(PinocchioError::InvalidModel("tolerance must be > 0"));
    }
    let mut selected = Vec::<usize>::new();
    let mut q_cols: Vec<Vec<f64>> = Vec::new();

    for c in 0..cols {
        let mut v = vec![0.0; rows];
        for r in 0..rows {
            v[r] = regressor_row_major[r * cols + c];
        }
        for qv in &q_cols {
            let mut alpha = 0.0;
            let mut qnorm2 = 0.0;
            for r in 0..rows {
                alpha += v[r] * qv[r];
                qnorm2 += qv[r] * qv[r];
            }
            if qnorm2 > 1e-18 {
                let s = alpha / qnorm2;
                for r in 0..rows {
                    v[r] -= s * qv[r];
                }
            }
        }
        let norm2 = v.iter().map(|x| x * x).sum::<f64>();
        if norm2.sqrt() > tolerance {
            selected.push(c);
            q_cols.push(v);
        }
    }

    let k = selected.len();
    let mut projected = vec![0.0; rows * k];
    for (j, &col) in selected.iter().enumerate() {
        for r in 0..rows {
            projected[r * k + j] = regressor_row_major[r * cols + col];
        }
    }
    Ok(RegressorBasisResult {
        selected_columns: selected,
        projected_row_major: projected,
        rows,
        cols: k,
    })
}
