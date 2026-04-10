use crate::core::error::{PinocchioError, Result};
use crate::core::math::{self, Mat3, Transform, Vec3};
use crate::model::{Model, Workspace};
use super::kinematics::forward_kinematics_poses;
use super::jacobian::frame_jacobian;

/// Result of an inverse kinematics solve.
#[derive(Debug, Clone)]
pub struct IKResult {
    /// Joint angles that achieve the target (or best effort).
    pub q: Vec<f64>,
    /// Whether the solver converged within the tolerance.
    pub converged: bool,
    /// Final task-space error norm.
    pub err: f64,
    /// Number of iterations performed.
    pub iterations: usize,
}

/// Solve full SE(3) inverse kinematics using damped least squares.
///
/// Finds joint angles `q` such that the frame at `target_link` reaches
/// the target placement `(target_pos, target_rot)`.
///
/// Returns the joint angles, convergence status, final error, and iteration count.
pub fn inverse_kinematics(
    model: &Model,
    q_init: &[f64],
    target_link: usize,
    target_pos: Vec3,
    target_rot: Mat3,
    ws: &mut Workspace,
    max_iter: usize,
    eps: f64,
    damping: f64,
) -> Result<IKResult> {
    let n = model.nv();
    if target_link >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: target_link,
            len: model.nlinks(),
        });
    }
    model.check_state_dims(q_init, &vec![0.0; n], None)?;

    let target_tf = Transform::new(target_rot, target_pos);
    let mut q = q_init.to_vec();

    for iter in 0..max_iter {
        // 1. Forward kinematics
        let poses = forward_kinematics_poses(model, &q, ws)?;
        let current_tf = poses[target_link];

        // 2. SE(3) error in world frame
        let err_tf = target_tf.multiply(current_tf.inverse());
        let err_vec = math::log6(err_tf);
        let err_norm = err_vec.iter().map(|v| v * v).sum::<f64>().sqrt();

        if err_norm < eps {
            return Ok(IKResult {
                q,
                converged: true,
                err: err_norm,
                iterations: iter + 1,
            });
        }

        // 3. Jacobian (reuse existing): 6×n, row-major [linear; angular]
        //    frame_jacobian returns [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z] per column
        let jac = frame_jacobian(model, &q, target_link, ws)?;

        // 4. Damped least squares: dq = Jᵀ(JJᵀ + λ²I)⁻¹ err
        //    Jacobian layout: rows = [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z] × n cols
        //    Error layout:    [ang_x, ang_y, ang_z, lin_x, lin_y, lin_z]
        //    Need to reorder: swap angular/linear in error to match Jacobian rows.
        let err_reordered = [
            err_vec[3], err_vec[4], err_vec[5], // linear
            err_vec[0], err_vec[1], err_vec[2], // angular
        ];

        // JJᵀ (6×6)
        let mut jjt = [[0.0; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut s = 0.0;
                for k in 0..n {
                    // jac layout: row-major, 6 rows × n cols
                    s += jac[i * n + k] * jac[j * n + k];
                }
                jjt[i][j] = s;
            }
        }
        // Add damping
        let d2 = damping * damping;
        for i in 0..6 {
            jjt[i][i] += d2;
        }

        // Solve JJᵀ x = err using Gaussian elimination
        let mut augmented = [[0.0; 7]; 6];
        for i in 0..6 {
            for j in 0..6 {
                augmented[i][j] = jjt[i][j];
            }
            augmented[i][6] = err_reordered[i];
        }

        let x = if let Some(sol) = solve_6x6(&augmented) {
            sol
        } else {
            // Fallback: gradient descent step
            let mut dq = vec![0.0; n];
            for j in 0..n {
                for i in 0..6 {
                    dq[j] += jac[i * n + j] * err_reordered[i];
                }
            }
            let scale = 1.0 / (d2 + err_norm);
            for j in 0..n {
                dq[j] *= scale;
            }
            clamp_and_integrate(&mut q, &dq, n);
            continue;
        };

        // dq = Jᵀ x
        let mut dq = vec![0.0; n];
        for j in 0..n {
            for i in 0..6 {
                dq[j] += jac[i * n + j] * x[i];
            }
        }

        clamp_and_integrate(&mut q, &dq, n);
    }

    // Final error check
    let poses = forward_kinematics_poses(model, &q, ws)?;
    let current_tf = poses[target_link];
    let err_tf = target_tf.multiply(current_tf.inverse());
    let err_vec = math::log6(err_tf);
    let err_norm = err_vec.iter().map(|v| v * v).sum::<f64>().sqrt();

    Ok(IKResult {
        q,
        converged: err_norm < eps * 10.0,
        err: err_norm,
        iterations: max_iter,
    })
}

/// Solve position-only inverse kinematics (3D target, ignoring orientation).
pub fn inverse_kinematics_position(
    model: &Model,
    q_init: &[f64],
    target_link: usize,
    target_pos: Vec3,
    ws: &mut Workspace,
    max_iter: usize,
    eps: f64,
    damping: f64,
) -> Result<IKResult> {
    let n = model.nv();
    if target_link >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: target_link,
            len: model.nlinks(),
        });
    }
    model.check_state_dims(q_init, &vec![0.0; n], None)?;

    let mut q = q_init.to_vec();

    for iter in 0..max_iter {
        let poses = forward_kinematics_poses(model, &q, ws)?;
        let current_pos = poses[target_link].translation;
        let err = target_pos - current_pos;
        let err_norm = err.norm();

        if err_norm < eps {
            return Ok(IKResult {
                q,
                converged: true,
                err: err_norm,
                iterations: iter + 1,
            });
        }

        // Full Jacobian, extract only linear rows (rows 0-2)
        let jac = frame_jacobian(model, &q, target_link, ws)?;

        // JP is 3×n: rows 0-2 of the Jacobian
        // JP JPᵀ (3×3)
        let mut jpjpt = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..n {
                    s += jac[i * n + k] * jac[j * n + k];
                }
                jpjpt[i][j] = s;
            }
        }
        let d2 = damping * damping;
        for i in 0..3 {
            jpjpt[i][i] += d2;
        }

        let err_arr = [err.x, err.y, err.z];
        let mut augmented = [[0.0; 4]; 3];
        for i in 0..3 {
            for j in 0..3 {
                augmented[i][j] = jpjpt[i][j];
            }
            augmented[i][3] = err_arr[i];
        }

        let x = if let Some(sol) = solve_3x3(&augmented) {
            sol
        } else {
            let mut dq = vec![0.0; n];
            for j in 0..n {
                for i in 0..3 {
                    dq[j] += jac[i * n + j] * err_arr[i];
                }
            }
            let scale = 1.0 / (d2 + err_norm);
            for j in 0..n {
                dq[j] *= scale;
            }
            clamp_and_integrate(&mut q, &dq, n);
            continue;
        };

        let mut dq = vec![0.0; n];
        for j in 0..n {
            for i in 0..3 {
                dq[j] += jac[i * n + j] * x[i];
            }
        }

        clamp_and_integrate(&mut q, &dq, n);
    }

    let poses = forward_kinematics_poses(model, &q, ws)?;
    let err = target_pos - poses[target_link].translation;
    let err_norm = err.norm();

    Ok(IKResult {
        q,
        converged: err_norm < eps * 10.0,
        err: err_norm,
        iterations: max_iter,
    })
}

/// Step clamping and simple revolute joint integration.
fn clamp_and_integrate(q: &mut [f64], dq: &[f64], n: usize) {
    let dq_norm: f64 = dq.iter().map(|v| v * v).sum::<f64>().sqrt();
    if dq_norm < 1e-12 {
        return;
    }
    let max_step = 0.5;
    let scale = if dq_norm > max_step {
        max_step / dq_norm
    } else {
        1.0
    };
    for i in 0..n {
        q[i] += dq[i] * scale;
    }
}

/// Solve 6×6 linear system via Gaussian elimination with partial pivoting.
fn solve_6x6(aug: &[[f64; 7]; 6]) -> Option<[f64; 6]> {
    let mut a = *aug;
    // Forward elimination
    for col in 0..6 {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..6 {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            return None;
        }
        if max_row != col {
            a.swap(col, max_row);
        }
        let pivot = a[col][col];
        for row in (col + 1)..6 {
            let factor = a[row][col] / pivot;
            for j in col..7 {
                a[row][j] -= factor * a[col][j];
            }
        }
    }
    // Back substitution
    let mut x = [0.0; 6];
    for i in (0..6).rev() {
        let mut s = a[i][6];
        for j in (i + 1)..6 {
            s -= a[i][j] * x[j];
        }
        if a[i][i].abs() < 1e-12 {
            return None;
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

/// Solve 3×3 linear system via Gaussian elimination with partial pivoting.
fn solve_3x3(aug: &[[f64; 4]; 3]) -> Option<[f64; 3]> {
    let mut a = *aug;
    for col in 0..3 {
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..3 {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            return None;
        }
        if max_row != col {
            a.swap(col, max_row);
        }
        let pivot = a[col][col];
        for row in (col + 1)..3 {
            let factor = a[row][col] / pivot;
            for j in col..4 {
                a[row][j] -= factor * a[col][j];
            }
        }
    }
    let mut x = [0.0; 3];
    for i in (0..3).rev() {
        let mut s = a[i][3];
        for j in (i + 1)..3 {
            s -= a[i][j] * x[j];
        }
        if a[i][i].abs() < 1e-12 {
            return None;
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}
