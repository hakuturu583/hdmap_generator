//! The bridges and tunnels a map's roads are carried on, as solids.
//!
//! The road surface on a structure is drawn like any other road's; what this adds
//! is what holds it up or closes it in. A bridge gets the underside of its deck, a
//! parapet along each side standing on the deck's curb, and its abutments and piers
//! down to the land. A tunnel gets its walls, its ceiling and the walkway each side,
//! all facing in, and a portal face round each mouth facing out.
//!
//! None of it is a road a vehicle drives on, and CARLA's import has no class for a
//! bridge a map can reach, so it is all terrain — the same as a berm — in concrete,
//! or in steel for a steel bridge.

use roadgen_core::geometry::{Frame3, Point3};
use roadgen_core::map::Map;
use roadgen_core::structure::{
    road_extent, Bridge, RoadSpan, StructureKind, StructureMaterial, Tunnel,
};

use crate::ground::Ground;
use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::Ordinals;
use crate::tags::{mesh_name, Role};

/// How tall a bridge's parapet stands above its deck, metres.
pub const PARAPET: f64 = 1.1;
/// How far apart the sections of a structure's solids are, metres.
const STEP: f64 = 2.0;
/// How far a pier goes down where nothing says where the ground is, metres.
const FALLBACK_DEPTH: f64 = 10.0;
/// The wall over a tunnel's mouth, above its lining, metres.
const HEADWALL: f64 = 1.0;

/// One station of a carried road: its banked frame and the road's two edges.
struct Row {
    frame: Frame3,
    left: f64,
    right: f64,
}

impl Row {
    fn at(&self, lateral: f64, rise: f64) -> Point3 {
        self.frame.to_global([0.0, lateral, rise])
    }
}

fn rows(map: &Map, span: &RoadSpan) -> Vec<Row> {
    let Some(road) = map.road(&span.road) else {
        return Vec::new();
    };
    let count = ((span.to - span.from) / STEP).ceil().max(1.0) as usize;
    (0..=count)
        .filter_map(|index| {
            let station = span.from + (span.to - span.from) * index as f64 / count as f64;
            let frame = road.frame_at(station, map.metadata.sampling).ok()?;
            let (left, right) = road_extent(map, &span.road, station)?;
            Some(Row { frame, left, right })
        })
        .collect()
}

/// What says whether a point of some road is on a bridge or in a tunnel: inside a
/// structure's outline in plan and within a couple of metres of the height of the
/// road it carries there. A road passing under a bridge is inside its outline too,
/// but a deck's depth and a clearance below it.
pub struct Carried {
    outlines: Vec<Outline>,
}

/// One structure's outline: in plan, with its heights, and its bounds.
type Outline = (Vec<[f64; 2]>, Vec<Point3>, [f64; 4]);

impl Carried {
    pub fn of(map: &Map) -> Carried {
        Carried {
            outlines: map
                .structures
                .iter()
                .map(|structure| {
                    let points = structure.outline.points().to_vec();
                    let ring: Vec<[f64; 2]> = points.iter().map(|p| [p.x, p.y]).collect();
                    let bounds = roadgen_core::geometry::plan::bounds(&ring);
                    (ring, points, bounds)
                })
                .collect(),
        }
    }

    pub fn contains(&self, point: &Point3) -> bool {
        self.outlines.iter().any(|(ring, points, b)| {
            point.x >= b[0]
                && point.x <= b[2]
                && point.y >= b[1]
                && point.y <= b[3]
                && roadgen_core::geometry::plan::ring_contains(ring, [point.x, point.y])
                && points
                    .iter()
                    .min_by(|a, b| {
                        a.horizontal_distance_to(*point)
                            .total_cmp(&b.horizontal_distance_to(*point))
                    })
                    .is_some_and(|nearest| (nearest.z - point.z).abs() < 2.0)
        })
    }
}

/// Every structure on the map, each as a mesh of its own.
pub fn meshes(
    map: &Map,
    ground: Option<&Ground>,
    map_name: &str,
    ordinals: &mut Ordinals,
) -> Vec<Mesh> {
    let mut out = Vec::new();
    for structure in map.structures.iter() {
        let material = match &structure.kind {
            StructureKind::Bridge(Bridge {
                material: StructureMaterial::Steel,
                ..
            }) => materials::STEEL,
            _ => materials::ROAD_CONCRETE,
        };
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Terrain, ordinals.take(Role::Terrain)),
            Role::Terrain,
            material,
        );
        match &structure.kind {
            StructureKind::Bridge(bridge) => {
                for span in &structure.carries {
                    deck(&mut mesh, &rows(map, span), bridge);
                }
                for support in &bridge.supports {
                    let ring = support.footprint.points();
                    let top = support.footprint.lowest();
                    let centre = support.footprint.centroid();
                    let base = support.base.unwrap_or_else(|| {
                        ground.map_or(top - FALLBACK_DEPTH, |ground| {
                            ground.height(centre.x, centre.y, top - FALLBACK_DEPTH)
                        })
                    });
                    column(&mut mesh, ring, base.min(top), top);
                }
            }
            StructureKind::Tunnel(tunnel) => {
                for span in &structure.carries {
                    bore(&mut mesh, &rows(map, span), tunnel);
                }
            }
        }
        out.push(mesh);
    }
    out
}

/// A deck's parapets and underside. The curb under each parapet is the parapet's
/// own footprint.
fn deck(mesh: &mut Mesh, rows: &[Row], bridge: &Bridge) {
    if rows.len() < 2 {
        return;
    }
    let curb = bridge.curb;
    let depth = bridge.deck_depth;
    let line = |lateral: &dyn Fn(&Row) -> f64, rise: f64| -> Vec<Point3> {
        rows.iter().map(|row| row.at(lateral(row), rise)).collect()
    };
    let left_inner = |row: &Row| row.left;
    let left_outer = |row: &Row| row.left + curb;
    let right_inner = |row: &Row| row.right;
    let right_outer = |row: &Row| row.right - curb;
    // The parapet's inner faces, facing the road; its top; its outer face, which
    // runs down the side of the deck to its underside.
    mesh.strip(&line(&left_inner, PARAPET), &line(&left_inner, 0.0), 0);
    mesh.strip(&line(&right_inner, 0.0), &line(&right_inner, PARAPET), 0);
    mesh.strip(&line(&left_outer, PARAPET), &line(&left_inner, PARAPET), 0);
    mesh.strip(
        &line(&right_inner, PARAPET),
        &line(&right_outer, PARAPET),
        0,
    );
    mesh.strip(&line(&left_outer, -depth), &line(&left_outer, PARAPET), 0);
    mesh.strip(&line(&right_outer, PARAPET), &line(&right_outer, -depth), 0);
    // The underside, facing down.
    mesh.strip(&line(&right_outer, -depth), &line(&left_outer, -depth), 0);
}

/// A column standing on `base` up to `top`: its sides, facing out, and its top.
fn column(mesh: &mut Mesh, ring: &[Point3], base: f64, top: f64) {
    for index in 0..ring.len() {
        let (a, b) = (ring[index], ring[(index + 1) % ring.len()]);
        let at = |p: Point3, z: f64| Point3::new(p.x, p.y, z);
        mesh.face(&[at(a, base), at(b, base), at(b, top), at(a, top)], 0);
    }
}

/// A bore's walls, ceiling and walkways, facing in, and a portal at each mouth
/// facing out.
fn bore(mesh: &mut Mesh, rows: &[Row], tunnel: &Tunnel) {
    if rows.len() < 2 {
        return;
    }
    let walkway = tunnel.walkway;
    let height = tunnel.clear_height;
    let line = |lateral: &dyn Fn(&Row) -> f64, rise: f64| -> Vec<Point3> {
        rows.iter().map(|row| row.at(lateral(row), rise)).collect()
    };
    let left_edge = |row: &Row| row.left;
    let right_edge = |row: &Row| row.right;
    let left_wall = |row: &Row| row.left + walkway;
    let right_wall = |row: &Row| row.right - walkway;
    // Walkways, facing up.
    mesh.strip(&line(&left_wall, 0.0), &line(&left_edge, 0.0), 0);
    mesh.strip(&line(&right_edge, 0.0), &line(&right_wall, 0.0), 0);
    // Walls, facing the road.
    mesh.strip(&line(&left_wall, height), &line(&left_wall, 0.0), 0);
    mesh.strip(&line(&right_wall, 0.0), &line(&right_wall, height), 0);
    // The ceiling, facing down.
    mesh.strip(&line(&right_wall, height), &line(&left_wall, height), 0);

    // The portals: the face round each mouth, its two piers and the lintel over
    // the opening, facing out of the tunnel.
    let top = height + tunnel.lining + HEADWALL;
    let lining = tunnel.lining;
    for (row, outward) in [(&rows[0], false), (&rows[rows.len() - 1], true)] {
        let (high, low) = (row.left + walkway, row.right - walkway);
        let quads = [
            (high, high + lining, 0.0, top),
            (low - lining, low, 0.0, top),
            (low, high, height, top),
        ];
        for (from, to, bottom, upper) in quads {
            // Anticlockwise seen from outside: at the start the viewer looks along
            // the road and its left is the road's left; at the end, the other way.
            let ring = if outward {
                [
                    row.at(from, bottom),
                    row.at(to, bottom),
                    row.at(to, upper),
                    row.at(from, upper),
                ]
            } else {
                [
                    row.at(to, bottom),
                    row.at(from, bottom),
                    row.at(from, upper),
                    row.at(to, upper),
                ]
            };
            mesh.face(&ring, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;
    use roadgen_core::structure::{BridgeSpec, TunnelSpec};

    /// Each triangle's facing, with its first corner.
    fn faces(mesh: &Mesh) -> Vec<(roadgen_core::geometry::Vector3, Point3)> {
        mesh.triangles
            .iter()
            .filter_map(|t| {
                let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
                let n = (b - a).cross(c - a).normalize().ok()?.get();
                Some((n, a))
            })
            .collect()
    }

    fn map(tunnel: bool) -> ValidatedMap {
        let mut builder = MapBuilder::default();
        let lanes = vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ];
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(300.0, 0.0, 10.0),
                    lanes,
                )
                .unwrap()
                .with_name("r"),
            )
            .unwrap();
        let span = vec![RoadSpan::new(road, 100.0, 200.0)];
        if tunnel {
            builder.add_tunnel(span, TunnelSpec::default()).unwrap();
        } else {
            builder.add_bridge(span, BridgeSpec::default()).unwrap();
        }
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn a_bridge_has_an_underside_that_faces_down_and_piers_to_the_ground() {
        let map = map(false);
        let meshes = meshes(&map, None, "m", &mut Ordinals::default());
        assert_eq!(meshes.len(), 1);
        let mesh = &meshes[0];
        assert_eq!(crate::tags::label_of(&mesh.name), crate::Label::Terrain);
        // Under the deck: 10 − 2.5.
        let underside = faces(mesh)
            .iter()
            .filter(|(n, a)| n.z < -0.99 && (a.z - 7.5).abs() < 1e-6)
            .count();
        assert!(underside > 0, "a deck seen from below");
        // A pier reaches down below the deck.
        let lowest = mesh
            .positions
            .iter()
            .map(|p| p.z)
            .fold(f64::INFINITY, f64::min);
        assert!((lowest - (7.5 - FALLBACK_DEPTH)).abs() < 1e-6, "{lowest}");
    }

    #[test]
    fn a_tunnel_is_closed_over_its_road_and_open_at_its_mouths() {
        let map = map(true);
        let meshes = meshes(&map, None, "m", &mut Ordinals::default());
        let mesh = &meshes[0];
        let faces = faces(mesh);
        // A ceiling over the road, facing down at 10 + 5.
        assert!(faces
            .iter()
            .any(|(n, a)| n.z < -0.99 && (a.z - 15.0).abs() < 1e-6));
        // The portal faces point out of the tunnel: −x at the start, +x at the end,
        // and none of them the other way.
        let facing = |want: f64, x: f64| {
            faces
                .iter()
                .any(|(n, a)| (n.x - want).abs() < 1e-6 && (a.x - x).abs() < 1e-6)
        };
        assert!(!facing(1.0, 100.0) && !facing(-1.0, 200.0));
        assert!(facing(-1.0, 100.0));
        assert!(facing(1.0, 200.0));
        // Nothing of the tunnel stands where the road is: the opening is clear.
        for p in &mesh.positions {
            let in_opening = p.y.abs() < 3.5 && p.z > 10.0 + 1e-6 && p.z < 15.0 - 1e-6;
            assert!(!in_opening, "{p:?}");
        }
    }
}
