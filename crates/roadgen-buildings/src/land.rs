//! Where a building may stand, and where one may not.
//!
//! # Buildings line streets
//!
//! A road network is the only thing this generator knows about the land, so a town
//! is laid out the way a town is: buildings face the street, in rows, set back from
//! the kerb. Nothing is placed in the middle of a block, because the middle of a
//! block is not something a road network describes — it is gardens, yards, car parks
//! and the backs of the buildings that face the next street over, and inventing
//! masses there would be inventing a land-use map rather than deriving one.
//!
//! So: every road has two frontages, each frontage is cut into lots along its length,
//! and each lot is a rectangle standing on the ground with its short edge at the
//! street. That rectangle is what the shape grammar is given.
//!
//! # And nothing stands in the road
//!
//! The road surface, widened by the setback, is the one region a building may never
//! reach into. It is built here as a strip of triangles per road — the actual
//! generated cross-section, sampled along the reference line, not an approximation
//! from a nominal width — and every candidate is tested against it. That is what
//! keeps a lot out of the road it faces at a bend, out of the road that crosses it,
//! and out of every junction, without any of those three being a special case.

use roadgen_core::geometry::Point3;
use roadgen_core::id::RoadId;
use roadgen_core::map::{Map, Road};
use roadgen_core::topology::LateralSide;

use crate::plane::{Index, Point2};

/// One place a building may stand: a rectangle on the ground, facing a street.
///
/// The frame is the one a CGA shape grammar expects, which is Y-up rather than the
/// IR's Z-up: local **X** runs along the frontage, local **Y** is up, and local **Z**
/// points away from the road — so the face at `Z = 0`, the one `Comp(Faces)` calls
/// `Front`, is the face that looks at the street.
#[derive(Debug, Clone, PartialEq)]
pub struct Lot {
    pub road: RoadId,
    pub side: LateralSide,
    /// Position along the frontage, counting from the road's start.
    pub index: usize,
    /// Where the middle of the lot faces the road, metres along its reference line.
    /// What the building's [`Frontage`](roadgen_core::buildings::Frontage) records.
    pub station: f64,
    /// The scope's `(0, 0, 0)` corner: on the ground, on the frontage line.
    pub origin: Point3,
    /// Local +X in the horizontal plane, unit length.
    pub along: Point2,
    /// Local +Z in the horizontal plane, unit length: away from the road.
    pub away: Point2,
    /// Metres of frontage.
    pub width: f64,
    /// Metres back from the frontage.
    pub depth: f64,
}

impl Lot {
    /// The lot's four corners on the ground, anticlockwise.
    pub fn ground_plan(&self) -> Vec<Point2> {
        let origin = [self.origin.x, self.origin.y];
        let along = [self.along[0] * self.width, self.along[1] * self.width];
        let away = [self.away[0] * self.depth, self.away[1] * self.depth];
        vec![
            origin,
            [origin[0] + along[0], origin[1] + along[1]],
            [
                origin[0] + along[0] + away[0],
                origin[1] + along[1] + away[1],
            ],
            [origin[0] + away[0], origin[1] + away[1]],
        ]
    }
}

/// How lots are cut out of a road's frontage. Every distance is in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Layout {
    /// From the edge of the road surface to the front of a lot.
    pub setback: f64,
    /// Frontage a single lot takes.
    pub lot_width: f64,
    /// How far back a lot reaches.
    pub lot_depth: f64,
    /// Left clear between neighbouring lots.
    pub lot_gap: f64,
    /// Left clear at each end of a frontage, so that buildings do not crowd the
    /// junction the road runs into.
    pub corner_clearance: f64,
}

impl Default for Layout {
    fn default() -> Self {
        Layout {
            setback: 4.0,
            lot_width: 14.0,
            lot_depth: 16.0,
            lot_gap: 3.0,
            corner_clearance: 10.0,
        }
    }
}

/// The road surface plus its setback: the region no building may reach into.
#[derive(Debug, Clone)]
pub struct Blocked {
    index: Index,
}

impl Blocked {
    /// Builds the blocked region of `map`, widened by `setback` metres on each side.
    pub fn of(map: &Map, setback: f64) -> Blocked {
        // Cells a little wider than a lot is long, so a candidate touches few of them
        // and each holds few triangles.
        let mut index = Index::new(25.0);
        for road in map.roads.iter() {
            let Some(edges) = corridor_edges(map, road, setback) else {
                continue;
            };
            for pair in edges.windows(2) {
                let (near, far) = (pair[0], pair[1]);
                // Two triangles rather than a quad: a quad spanning a sharp bend is
                // not convex, and a separating-axis test on a non-convex shape
                // quietly answers a different question.
                index.insert(vec![flat(near.0), flat(near.1), flat(far.1)]);
                index.insert(vec![flat(near.0), flat(far.1), flat(far.0)]);
            }
        }
        Blocked { index }
    }

    pub fn hits(&self, plan: &[Point2]) -> bool {
        self.index.hits(plan)
    }

    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }
}

fn flat(point: Point3) -> Point2 {
    [point.x, point.y]
}

/// The outer edge of `road`'s surface on each side, widened by `margin`, sampled
/// along the reference line.
///
/// Each entry is the (left, right) pair at one station. `None` when the road has no
/// geometry to sample, which a connector between two coincident points can have.
fn corridor_edges(map: &Map, road: &Road, margin: f64) -> Option<Vec<(Point3, Point3)>> {
    let stations = map.vertex_stations(&road.id).ok()?;
    if stations.len() < 2 {
        return None;
    }
    let mut edges = Vec::with_capacity(stations.len());
    for station in stations {
        let Some((left, right)) = section_edges(map, road, station) else {
            continue;
        };
        let Some(frame) = frame_at(map, road, station) else {
            continue;
        };
        edges.push((frame.offset(left + margin), frame.offset(right - margin)));
    }
    (edges.len() >= 2).then_some(edges)
}

/// The road local frame at `station`, banked the way the surface is.
fn frame_at(map: &Map, road: &Road, station: f64) -> Option<roadgen_core::geometry::Frame3> {
    Some(
        road.reference_line
            .sample_at(station, map.metadata.sampling)
            .ok()?
            .frame()
            .ok()?
            .banked(road.superelevation.evaluate(station)),
    )
}

/// How far the cross-section reaches to each side of the reference line at
/// `station`: `(left, right)` as signed lateral offsets, so `right` is negative.
///
/// Read off the generated lanes rather than off the road's spec, so a road that
/// tapers, gains a lane or carries a lane offset is measured where it actually is.
fn section_edges(map: &Map, road: &Road, station: f64) -> Option<(f64, f64)> {
    let section = section_at(road, station)?;
    let mut left = 0.0;
    let mut right = 0.0;
    for lane in road.lanes.iter().filter_map(|id| map.lanes.get(id)) {
        if lane.section != section {
            continue;
        }
        match lane.side {
            LateralSide::Left => left += lane.width_at(station),
            LateralSide::Right => right += lane.width_at(station),
        }
    }
    let centre = road.lane_offset.evaluate(station);
    Some((centre + left, centre - right))
}

/// Which cross-section covers `station`: the last one that starts at or before it.
///
/// `None` only for a station before the road begins, which nothing here asks for.
fn section_at(road: &Road, station: f64) -> Option<usize> {
    road.sections
        .iter()
        .rposition(|section| section.station <= station + 1e-9)
}

/// Cuts every road's two frontages into lots, in the map's own order.
///
/// Junction connectors get none: a connector is the IR's way of drawing a movement
/// through a junction, and the land beside a movement is the junction itself.
pub fn lots(map: &Map, layout: &Layout) -> Vec<Lot> {
    let mut lots = Vec::new();
    for road in map.roads.iter() {
        if road.is_connector() {
            continue;
        }
        for side in [LateralSide::Left, LateralSide::Right] {
            lots.extend(frontage_lots(map, road, side, layout));
        }
    }
    lots
}

/// The lots along one side of one road.
fn frontage_lots(map: &Map, road: &Road, side: LateralSide, layout: &Layout) -> Vec<Lot> {
    let Ok(length) = road.horizontal_length() else {
        return Vec::new();
    };
    let pitch = layout.lot_width + layout.lot_gap;
    let usable = length - 2.0 * layout.corner_clearance;
    if pitch <= 0.0
        || layout.lot_width <= 0.0
        || layout.lot_depth <= 0.0
        || usable < layout.lot_width
    {
        return Vec::new();
    }

    // Centred on the road rather than started at one end, so that the row reads the
    // same from either end and a road one lot too short for a second lot puts its
    // single lot in the middle instead of against the junction.
    let count = ((usable + layout.lot_gap) / pitch).floor().max(0.0) as usize;
    if count == 0 {
        return Vec::new();
    }
    let occupied = count as f64 * pitch - layout.lot_gap;
    let first = layout.corner_clearance + (usable - occupied) / 2.0;

    let mut lots = Vec::with_capacity(count);
    for index in 0..count {
        let from = first + index as f64 * pitch;
        let to = from + layout.lot_width;
        let Some(lot) = frontage_lot(map, road, side, layout, index, from, to) else {
            continue;
        };
        lots.push(lot);
    }
    lots
}

/// One lot, spanning the stations `from..to` of one frontage.
fn frontage_lot(
    map: &Map,
    road: &Road,
    side: LateralSide,
    layout: &Layout,
    index: usize,
    from: f64,
    to: f64,
) -> Option<Lot> {
    let corner = |station: f64| -> Option<Point3> {
        let (left, right) = section_edges(map, road, station)?;
        let frame = frame_at(map, road, station)?;
        Some(frame.offset(match side {
            LateralSide::Left => left + layout.setback,
            LateralSide::Right => right - layout.setback,
        }))
    };
    let start = corner(from)?;
    let end = corner(to)?;

    // The lot stands on level ground at the mean height of its frontage. A box
    // hanging off a graded road one corner at a time is not a building, and the
    // heights of the two ends of fourteen metres of frontage differ by centimetres.
    let ground = (start.z + end.z) / 2.0;

    // Local +X runs whichever way makes the frame right-handed with +Y up and +Z
    // away from the road: along the road on the right-hand frontage, against it on
    // the left. Either way the lot's origin is the corner the other three are
    // measured from, so it is the end that +X starts at.
    let (origin, far) = match side {
        LateralSide::Right => (start, end),
        LateralSide::Left => (end, start),
    };
    let along = unit([far.x - origin.x, far.y - origin.y])?;
    // `away = along × up`, which is what leaves `[along, up, away]` right-handed.
    let away = [along[1], -along[0]];

    let width = ((far.x - origin.x).powi(2) + (far.y - origin.y).powi(2)).sqrt();
    Some(Lot {
        road: road.id.clone(),
        side,
        index,
        station: (from + to) / 2.0,
        origin: Point3::new(origin.x, origin.y, ground),
        along,
        away,
        width,
        depth: layout.lot_depth,
    })
}

fn unit(vector: Point2) -> Option<Point2> {
    let length = (vector[0] * vector[0] + vector[1] * vector[1]).sqrt();
    (length > 1e-9).then(|| [vector[0] / length, vector[1] / length])
}

#[cfg(test)]
mod tests {
    use roadgen_core::prelude::*;
    use roadgen_core::topology::Direction;
    use roadgen_core::units::PositiveWidth;

    use super::*;

    fn straight(length: f64) -> Map {
        let mut builder = MapBuilder::default();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(length, 0.0, 0.0),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
                    ],
                )
                .unwrap()
                .with_name("main"),
            )
            .unwrap();
        builder.finish().unwrap().into_map()
    }

    #[test]
    fn a_frontage_is_cut_into_lots_on_both_sides() {
        let map = straight(200.0);
        let layout = Layout::default();
        let lots = lots(&map, &layout);

        assert!(lots.iter().any(|lot| lot.side == LateralSide::Left));
        assert!(lots.iter().any(|lot| lot.side == LateralSide::Right));
        for lot in &lots {
            assert!((lot.width - layout.lot_width).abs() < 1e-6);
            assert_eq!(lot.depth, layout.lot_depth);
        }
    }

    #[test]
    fn a_lot_faces_the_street_and_reaches_away_from_it() {
        let map = straight(200.0);
        let layout = Layout::default();
        let lots = lots(&map, &layout);

        // The road runs along +x with 3.5 m of lane each side, so the near edge of a
        // lot is 3.5 + setback from the centreline, and `away` points outwards.
        for lot in &lots {
            let expected = 3.5 + layout.setback;
            assert!((lot.origin.y.abs() - expected).abs() < 1e-6, "{lot:?}");
            let outwards = lot.origin.y.signum();
            assert!((lot.away[1] - outwards).abs() < 1e-6, "{lot:?}");
            assert!(lot.away[0].abs() < 1e-6, "{lot:?}");
            // Right-handed: along × up = away.
            let cross = [lot.along[1] * 1.0, -lot.along[0] * 1.0];
            assert!((cross[0] - lot.away[0]).abs() < 1e-9);
            assert!((cross[1] - lot.away[1]).abs() < 1e-9);
        }
    }

    #[test]
    fn a_road_too_short_for_a_lot_gets_none() {
        let map = straight(20.0);
        assert!(lots(&map, &Layout::default()).is_empty());
    }

    #[test]
    fn no_lot_reaches_into_the_road_it_faces() {
        let map = straight(200.0);
        let layout = Layout::default();
        let blocked = Blocked::of(&map, layout.setback);
        assert!(!blocked.is_empty());
        for lot in lots(&map, &layout) {
            assert!(!blocked.hits(&lot.ground_plan()), "{lot:?}");
        }
    }

    #[test]
    fn the_blocked_region_covers_the_road_surface() {
        let map = straight(200.0);
        let blocked = Blocked::of(&map, 0.0);
        // A metre square on the centreline is in the road.
        assert!(blocked.hits(&[[100.0, -0.5], [101.0, -0.5], [101.0, 0.5], [100.0, 0.5]]));
        // One well outside it is not.
        assert!(!blocked.hits(&[[100.0, 40.0], [101.0, 40.0], [101.0, 41.0], [100.0, 41.0]]));
    }
}
