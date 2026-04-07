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

#[derive(Debug, Clone, Copy)]
pub enum Geometry {
    Sphere {
        link_index: usize,
        center_local: Vec3,
        radius: f64,
    },
    Box {
        link_index: usize,
        center_local: Vec3,
        half_extents: Vec3,
    },
    Capsule {
        link_index: usize,
        center_local: Vec3,
        half_length: f64,
        radius: f64,
    },
    Cylinder {
        link_index: usize,
        center_local: Vec3,
        half_length: f64,
        radius: f64,
    },
    MeshApprox {
        link_index: usize,
        center_local: Vec3,
        half_extents: Vec3,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PairFilter {
    pub ignore_same_link: bool,
    pub ignore_parent_child: bool,
}

impl Default for PairFilter {
    fn default() -> Self {
        Self {
            ignore_same_link: true,
            ignore_parent_child: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct CollisionModel {
    pub geometries: Vec<Geometry>,
    pub pairs: Vec<(usize, usize)>,
    pub filter: PairFilter,
}

#[derive(Debug, Clone, Copy)]
pub struct DistanceResult {
    pub pair: (usize, usize),
    pub distance: f64,
    pub point_a: Vec3,
    pub point_b: Vec3,
}

#[derive(Debug, Clone, Copy)]
pub struct CollisionResult {
    pub pair: (usize, usize),
    pub distance: f64,
    pub normal_world: Vec3,
    pub point_a: Vec3,
    pub point_b: Vec3,
    pub penetration_depth: f64,
    pub is_colliding: bool,
}

#[derive(Debug, Clone, Copy)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn overlaps(self, other: Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
}

impl Geometry {
    fn link_index(self) -> usize {
        match self {
            Geometry::Sphere { link_index, .. }
            | Geometry::Box { link_index, .. }
            | Geometry::Capsule { link_index, .. }
            | Geometry::Cylinder { link_index, .. }
            | Geometry::MeshApprox { link_index, .. } => link_index,
        }
    }

    fn center_local(self) -> Vec3 {
        match self {
            Geometry::Sphere { center_local, .. }
            | Geometry::Box { center_local, .. }
            | Geometry::Capsule { center_local, .. }
            | Geometry::Cylinder { center_local, .. }
            | Geometry::MeshApprox { center_local, .. } => center_local,
        }
    }

    fn approx_radius(self) -> f64 {
        match self {
            Geometry::Sphere { radius, .. } => radius,
            Geometry::Box { half_extents, .. } | Geometry::MeshApprox { half_extents, .. } => {
                half_extents.norm()
            }
            Geometry::Capsule {
                half_length,
                radius,
                ..
            } => half_length + radius,
            Geometry::Cylinder {
                half_length,
                radius,
                ..
            } => (half_length * half_length + radius * radius).sqrt(),
        }
    }

    fn aabb_extents_world(self, ws: &Workspace) -> Vec3 {
        match self {
            Geometry::Sphere { radius, .. } => Vec3::new(radius, radius, radius),
            Geometry::Box {
                link_index,
                half_extents,
                ..
            }
            | Geometry::MeshApprox {
                link_index,
                half_extents,
                ..
            } => {
                let r = ws.world_pose[link_index].rotation.m;
                Vec3::new(
                    r[0][0].abs() * half_extents.x
                        + r[0][1].abs() * half_extents.y
                        + r[0][2].abs() * half_extents.z,
                    r[1][0].abs() * half_extents.x
                        + r[1][1].abs() * half_extents.y
                        + r[1][2].abs() * half_extents.z,
                    r[2][0].abs() * half_extents.x
                        + r[2][1].abs() * half_extents.y
                        + r[2][2].abs() * half_extents.z,
                )
            }
            Geometry::Capsule {
                link_index,
                half_length,
                radius,
                ..
            }
            | Geometry::Cylinder {
                link_index,
                half_length,
                radius,
                ..
            } => {
                let a = ws.world_pose[link_index]
                    .rotation
                    .mul_vec(Vec3::new(0.0, 0.0, 1.0));
                Vec3::new(
                    a.x.abs() * half_length + radius,
                    a.y.abs() * half_length + radius,
                    a.z.abs() * half_length + radius,
                )
            }
        }
    }
}

impl CollisionModel {
    pub fn new(geometries: Vec<Geometry>, pairs: Vec<(usize, usize)>) -> Result<Self> {
        Self::new_with_filter(geometries, pairs, PairFilter::default())
    }

    pub fn new_with_filter(
        geometries: Vec<Geometry>,
        pairs: Vec<(usize, usize)>,
        filter: PairFilter,
    ) -> Result<Self> {
        if geometries.is_empty() {
            return Err(PinocchioError::InvalidModel(
                "collision model must contain at least one geometry",
            ));
        }
        for g in &geometries {
            validate_geometry(*g)?;
        }
        for (a, b) in &pairs {
            if *a >= geometries.len() || *b >= geometries.len() || *a == *b {
                return Err(PinocchioError::InvalidModel(
                    "invalid collision pair indices",
                ));
            }
        }
        Ok(Self {
            geometries,
            pairs,
            filter,
        })
    }

    pub fn with_all_pairs(geometries: Vec<Geometry>) -> Result<Self> {
        let mut pairs = Vec::new();
        for i in 0..geometries.len() {
            for j in (i + 1)..geometries.len() {
                pairs.push((i, j));
            }
        }
        Self::new(geometries, pairs)
    }

    pub fn from_spheres(spheres: Vec<Sphere>) -> Result<Self> {
        let geometries: Vec<Geometry> = spheres
            .iter()
            .map(|s| Geometry::Sphere {
                link_index: s.link_index,
                center_local: s.center_local,
                radius: s.radius,
            })
            .collect();
        let mut pairs = Vec::new();
        for i in 0..geometries.len() {
            for j in (i + 1)..geometries.len() {
                pairs.push((i, j));
            }
        }
        Self::new_with_filter(
            geometries,
            pairs,
            PairFilter {
                ignore_same_link: true,
                ignore_parent_child: false,
            },
        )
    }
}

fn validate_geometry(g: Geometry) -> Result<()> {
    match g {
        Geometry::Sphere { radius, .. } => {
            if radius < 0.0 {
                return Err(PinocchioError::InvalidModel(
                    "sphere radius must be non-negative",
                ));
            }
        }
        Geometry::Box { half_extents, .. } | Geometry::MeshApprox { half_extents, .. } => {
            if half_extents.x < 0.0 || half_extents.y < 0.0 || half_extents.z < 0.0 {
                return Err(PinocchioError::InvalidModel(
                    "half extents must be non-negative",
                ));
            }
        }
        Geometry::Capsule {
            half_length,
            radius,
            ..
        }
        | Geometry::Cylinder {
            half_length,
            radius,
            ..
        } => {
            if half_length < 0.0 || radius < 0.0 {
                return Err(PinocchioError::InvalidModel(
                    "capsule/cylinder dimensions must be non-negative",
                ));
            }
        }
    }
    Ok(())
}

fn geom_world_center(model: &Model, ws: &Workspace, geom: Geometry) -> Result<Vec3> {
    let link_index = geom.link_index();
    if link_index >= model.nlinks() {
        return Err(PinocchioError::IndexOutOfBounds {
            index: link_index,
            len: model.nlinks(),
        });
    }
    Ok(ws.world_pose[link_index].transform_point(geom.center_local()))
}

fn geom_aabb(model: &Model, ws: &Workspace, geom: Geometry) -> Result<Aabb> {
    let c = geom_world_center(model, ws, geom)?;
    let e = geom.aabb_extents_world(ws);
    Ok(Aabb {
        min: Vec3::new(c.x - e.x, c.y - e.y, c.z - e.z),
        max: Vec3::new(c.x + e.x, c.y + e.y, c.z + e.z),
    })
}

fn is_parent_child(model: &Model, a_link: usize, b_link: usize) -> bool {
    model.links[a_link].parent == Some(b_link) || model.links[b_link].parent == Some(a_link)
}

fn normal_and_points_from_proxy_sphere(
    model: &Model,
    ws: &Workspace,
    a: Geometry,
    b: Geometry,
) -> Result<(f64, Vec3, Vec3, Vec3, f64)> {
    let ca = geom_world_center(model, ws, a)?;
    let cb = geom_world_center(model, ws, b)?;
    let ra = a.approx_radius();
    let rb = b.approx_radius();

    let d = cb - ca;
    let nrm = d.norm();
    let normal = if nrm <= 1e-12 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        d * (1.0 / nrm)
    };

    let signed = nrm - ra - rb;
    let pa = ca + normal * ra;
    let pb = cb - normal * rb;
    let penetration = (-signed).max(0.0);
    Ok((signed, normal, pa, pb, penetration))
}

pub fn collision_details(
    model: &Model,
    collision: &CollisionModel,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<Vec<CollisionResult>> {
    let n = model.nv();
    let qd = vec![0.0; n];
    let qdd = vec![0.0; n];
    forward_kinematics(model, q, &qd, &qdd, Vec3::zero(), ws)?;

    let mut out = Vec::new();
    for &(i, j) in &collision.pairs {
        let ga = collision.geometries[i];
        let gb = collision.geometries[j];

        let la = ga.link_index();
        let lb = gb.link_index();
        if collision.filter.ignore_same_link && la == lb {
            continue;
        }
        if collision.filter.ignore_parent_child && is_parent_child(model, la, lb) {
            continue;
        }

        let aabb_a = geom_aabb(model, ws, ga)?;
        let aabb_b = geom_aabb(model, ws, gb)?;
        let _broadphase_overlaps = aabb_a.overlaps(aabb_b);

        let (distance, normal, pa, pb, penetration_depth) =
            normal_and_points_from_proxy_sphere(model, ws, ga, gb)?;

        out.push(CollisionResult {
            pair: (i, j),
            distance,
            normal_world: normal,
            point_a: pa,
            point_b: pb,
            penetration_depth,
            is_colliding: penetration_depth > 0.0,
        });
    }

    Ok(out)
}

pub fn minimum_distance(
    model: &Model,
    collision: &CollisionModel,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<DistanceResult> {
    let details = collision_details(model, collision, q, ws)?;

    let mut best = DistanceResult {
        pair: (0, 0),
        distance: f64::INFINITY,
        point_a: Vec3::zero(),
        point_b: Vec3::zero(),
    };

    for c in details {
        if c.distance < best.distance {
            best = DistanceResult {
                pair: c.pair,
                distance: c.distance,
                point_a: c.point_a,
                point_b: c.point_b,
            };
        }
    }

    Ok(best)
}

pub fn minimum_distance_detailed(
    model: &Model,
    collision: &CollisionModel,
    q: &[f64],
    ws: &mut Workspace,
) -> Result<CollisionResult> {
    let details = collision_details(model, collision, q, ws)?;

    let mut best = CollisionResult {
        pair: (0, 0),
        distance: f64::INFINITY,
        normal_world: Vec3::new(1.0, 0.0, 0.0),
        point_a: Vec3::zero(),
        point_b: Vec3::zero(),
        penetration_depth: 0.0,
        is_colliding: false,
    };

    for c in details {
        if c.distance < best.distance {
            best = c;
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

pub fn minimum_distance_detailed_batch(
    model: &Model,
    collision: &CollisionModel,
    q_batch: &[f64],
    batch_size: usize,
    ws: &mut Workspace,
    distances_out: &mut [f64],
    penetration_out: &mut [f64],
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
    if penetration_out.len() != batch_size {
        return Err(PinocchioError::DimensionMismatch {
            expected: batch_size,
            got: penetration_out.len(),
        });
    }

    for step in 0..batch_size {
        let b = step * n;
        let res = minimum_distance_detailed(model, collision, &q_batch[b..b + n], ws)?;
        distances_out[step] = res.distance;
        penetration_out[step] = res.penetration_depth;
    }
    Ok(())
}
