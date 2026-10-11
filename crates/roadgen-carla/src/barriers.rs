//! The barriers a map stands beside its roads, as solids.
//!
//! A barrier's band is the ground it occupies; its height is how far its top stands
//! above that. A berm — a ridge of broken rock along a haul road's edge — has its two
//! sides rising from the band's edges at the angle loose rock rests at, and a flat
//! top where the band is wide enough to leave one, otherwise a sharp crest down the
//! middle. A wall or a rail is built: it stands upright over the whole band, in
//! concrete or in steel.
//!
//! A barrier that is part of a structure — a bridge's parapet — is drawn with its
//! structure instead, in the structure's own mesh and material (see
//! [`raise`]), so it is drawn once and built of what its bridge is built of.
//!
//! CARLA's import has no class for any of these, so they are terrain.

use roadgen_core::geometry::SamplingConfig;
use roadgen_core::geometry::{Point3, Polyline3};
use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry};

use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::Ordinals;
use crate::tags::{mesh_name, Role};

/// The slope of a berm's sides, degrees: the angle of repose of broken rock.
const REPOSE: f64 = 37.0;
/// How far apart the ridge's sections are, metres.
const STEP: f64 = 2.0;

/// Every barrier on the map that stands on its own, as a mesh of its own: all but
/// those that are part of a structure on the map, which [`raise`] draws with it.
pub fn berms(map: &Map, map_name: &str, ordinals: &mut Ordinals) -> Vec<Mesh> {
    let sampling = map.metadata.sampling;
    let mut meshes = Vec::new();
    for object in map.objects.iter() {
        let MapObjectKind::Barrier {
            kind,
            height,
            structure,
        } = &object.kind
        else {
            continue;
        };
        if structure
            .as_ref()
            .is_some_and(|id| map.structures.contains(id))
        {
            continue;
        }
        // Earth slopes at its angle of repose; anything built stands upright.
        let (slope, material) = match kind.as_str() {
            "berm" | "bund" | "windrow" | "embankment" => (Some(REPOSE), materials::GRAVEL),
            "wall" | "jersey" | "concrete" => (None, materials::CURB),
            _ => (None, materials::STEEL),
        };
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Terrain, ordinals.take(Role::Terrain)),
            Role::Terrain,
            material,
        );
        solid(&mut mesh, object, *height, slope, sampling);
        meshes.push(mesh);
    }
    meshes
}

/// A barrier that is part of a structure, built upright into the structure's
/// `mesh`: a bridge's parapet, on its curb.
pub fn raise(mesh: &mut Mesh, object: &MapObject, sampling: SamplingConfig) {
    if let MapObjectKind::Barrier { height, .. } = &object.kind {
        solid(mesh, object, *height, None, sampling);
    }
}

/// A barrier's solid, added to `mesh`: its sides rising from its band's edges at
/// `slope` degrees, or upright where it has none, its top, and its two ends.
fn solid(
    mesh: &mut Mesh,
    object: &MapObject,
    height: f64,
    slope: Option<f64>,
    sampling: SamplingConfig,
) {
    let ObjectGeometry::Band { left, right } = &object.geometry else {
        return;
    };
    let (Ok(left), Ok(right)) = (left.to_polyline(sampling), right.to_polyline(sampling)) else {
        return;
    };
    let count = ((left.horizontal_length().max(right.horizontal_length()) / STEP).ceil() as usize)
        .max(1)
        + 1;
    let (left, right) = (resample(&left, count), resample(&right, count));
    let run = slope.map_or(0.0, |angle| height / angle.to_radians().tan());
    let (mut top_left, mut top_right) = (Vec::with_capacity(count), Vec::with_capacity(count));
    for (l, r) in left.iter().zip(&right) {
        let width = l.horizontal_distance_to(*r);
        // In from each edge by the sides' run, or to the middle if they meet.
        let inset = if width > 0.0 {
            (run / width).min(0.5)
        } else {
            0.5
        };
        let base = |t: f64| l.lerp(*r, t);
        let lift = |p: Point3| Point3::new(p.x, p.y, p.z + height);
        top_left.push(lift(base(inset)));
        top_right.push(lift(base(1.0 - inset)));
    }
    // Each strip anticlockwise seen from outside: from above, left to right.
    mesh.strip(&left, &top_left, 0);
    mesh.strip(&top_left, &top_right, 0);
    mesh.strip(&top_right, &right, 0);
    // The two ends, closed so the ridge is a solid from any side.
    let last = count - 1;
    // The start faces back along the band and the end forwards along it.
    mesh.face(&[left[0], right[0], top_right[0], top_left[0]], 0);
    mesh.face(
        &[left[last], top_left[last], top_right[last], right[last]],
        0,
    );
}

/// `count` points evenly along a polyline's plan length.
fn resample(line: &Polyline3, count: usize) -> Vec<Point3> {
    let points = line.points();
    let total = line.horizontal_length();
    let mut out = Vec::with_capacity(count);
    let mut segment = 0;
    let mut start = 0.0;
    for index in 0..count {
        let target = total * index as f64 / (count - 1).max(1) as f64;
        while segment + 2 < points.len()
            && start + points[segment].horizontal_distance_to(points[segment + 1]) < target
        {
            start += points[segment].horizontal_distance_to(points[segment + 1]);
            segment += 1;
        }
        let (a, b) = (points[segment], points[(segment + 1).min(points.len() - 1)]);
        let length = a.horizontal_distance_to(b);
        let t = if length > 0.0 {
            ((target - start) / length).clamp(0.0, 1.0)
        } else {
            0.0
        };
        out.push(a.lerp(b, t));
    }
    out
}
