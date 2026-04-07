use crate::algo::forward_kinematics;
use crate::core::error::{PinocchioError, Result};
use crate::core::math::Vec3;
use crate::model::{Model, Workspace};

#[derive(Debug, Clone, Copy)]
pub struct Sphere {
    pub link_index: usize,
    pub center_local: Vec3,
    pub radius: f64,
}

#[derive(Debug, Clone)]
pub struct CollisionModel {
    pub spheres: Vec<Sphere>,
    pub pairs: Vec<(usize, usize)>,
}

#[derive(Debug, Clone, Copy)]
pub struct DistanceResult {
    pub pair: (usize, usize),
    pub distance: f64,
    pub point_a: Vec3,
    pub point_b: Vec3,
}

impl CollisionModel {
    pub fn new(spheres: Vec<Sphere>, pairs: Vec<(usize, usize)>) -> Result<Self> {
        if spheres.is_empty() {
            return Err(PinocchioError::InvalidModel(
                "collision model must contain at least one sphere",
            ));
        }
        for s in &spheres {
            if s.radius < 0.0 {
                return Err(PinocchioError::InvalidModel(
                    "sphere radius must be non-negative",
                ));
            }
        }
        for (a, b) in &pairs {
            if *a >= spheres.len() || *b >= spheres.len() || *a == *b {
                return Err(PinocchioError::InvalidModel(
                    "invalid collision pair indices",
                ));
            }
        }
        Ok(Self { spheres, pairs })
    }

    pub fn with_all_pairs(spheres: Vec<Sphere>) -> Result<Self> {
        let mut pairs = Vec::new();
        for i in 0..spheres.len() {
            for j in (i + 1)..spheres.len() {
                pairs.push((i, j));
            }
        }
        Self::new(spheres, pairs)
    }
}

fn sphere_world_center(model: &Model, ws: &Workspace, sphere: Sphere) -> Result<Vec3> {
    if sphere.link_index >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: sphere.link_index,
            len: model.nlinks(),
        });
    }
    Ok(ws.world_pose[sphere.link_index].transform_point(sphere.center_local))
}

fn pair_distance(model: &Model, ws: &Workspace, a: Sphere, b: Sphere) -> Result<(f64, Vec3, Vec3)> {
    let ca = sphere_world_center(model, ws, a)?;
    let cb = sphere_world_center(model, ws, b)?;
    let d = cb - ca;
    let norm = d.norm();
    let signed = norm - a.radius - b.radius;
    if norm <= 1e-12 {
        let pa = ca + Vec3::new(a.radius, 0.0, 0.0);
        let pb = cb - Vec3::new(b.radius, 0.0, 0.0);
        return Ok((signed, pa, pb));
    }
    let u = d * (1.0 / norm);
    let pa = ca + u * a.radius;
    let pb = cb - u * b.radius;
    Ok((signed, pa, pb))
}

pub fn minimum_distance(
    model: &Model,
    collision: &CollisionModel,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<DistanceResult> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let mut best = DistanceResult {
        pair: (0, 0),
        distance: f64::INFINITY,
        point_a: Vec3::zero(),
        point_b: Vec3::zero(),
    };
    for &(i, j) in &collision.pairs {
        let sa = collision.spheres[i];
        let sb = collision.spheres[j];
        let (dist, pa, pb) = pair_distance(model, ws, sa, sb)?;
        if dist < best.distance {
            best = DistanceResult {
                pair: (i, j),
                distance: dist,
                point_a: pa,
                point_b: pb,
            };
        }
    }
    Ok(best)
}

pub fn minimum_distance_batch(
    model: &Model,
    collision: &CollisionModel,
    q_batch: &[f64],
    batch_size: usize,
    ws: &mut Workspace,
    distances_out: &mut [f64],
) -> Result<()> {
    let n = model.nv();
    let expected = batch_size
        .checked_mul(n)
        .ok_or(PinocchioError::InvalidModel("batch size overflow"))?;
    if q_batch.len() != expected {
        return Err(PinocchioError::DimensionMismatch {
            expected,
            got: q_batch.len(),
        });
    }
    if distances_out.len() != batch_size {
        return Err(PinocchioError::DimensionMismatch {
            expected: batch_size,
            got: distances_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let res = minimum_distance(model, collision, &q_batch[b..b + n], ws)?;
        distances_out[step] = res.distance;
    }
    Ok(())
}
