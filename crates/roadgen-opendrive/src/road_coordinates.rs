//! Finding where a position sits in a road's own coordinates.
//!
//! OpenDRIVE places a signal or an object at `(s, t, zOffset)` — along the reference
//! line, across it, and above the road surface — where the IR holds a position in
//! space. Nothing else in this project needs that conversion, so it lives here rather
//! than in the IR: it is a thing the format asks for.
//!
//! The projection itself is the road local frame, the same one the generator laid the
//! cross-section out with. What has to be worked out first is *which* station, and
//! that is a nearest-point search along the reference line.

use roadgen_core::geometry::Point3;
use roadgen_core::map::{Map, Road};

/// A position in one road's coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoadPosition {
    /// Along the reference line, metres.
    pub s: f64,
    /// Across it, metres, positive to the left.
    pub t: f64,
    /// Above the road surface, metres.
    pub height: f64,
}

/// Where `point` sits in `road`'s coordinates, or `None` if the road has no usable
/// geometry.
pub fn locate(map: &Map, road: &Road, point: Point3) -> Option<RoadPosition> {
    let config = map.metadata.sampling;
    let stations = map.vertex_stations(&road.id).ok()?;
    if stations.len() < 2 {
        return None;
    }

    // Which stretch of the reference line the point is nearest to. A vertex-by-vertex
    // search is enough to pick the segment; the station within it comes from
    // projecting onto the chord, and the frame refines the rest.
    let mut best: Option<(f64, f64)> = None;
    for pair in stations.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let (Ok(a), Ok(b)) = (
            road.reference_line.sample_at(from, config),
            road.reference_line.sample_at(to, config),
        ) else {
            continue;
        };
        let along = b.point - a.point;
        let length_squared = along.dot(along);
        let u = if length_squared > 0.0 {
            ((point - a.point).dot(along) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let nearest = a.point + along * u;
        let distance = nearest.distance_to(point);
        if best.is_none_or(|(_, previous)| distance < previous) {
            best = Some((from + (to - from) * u, distance));
        }
    }

    // One pass through the frame: `to_local` gives the along-track error as well as
    // the lateral and vertical offsets, so feeding it back lands on the station the
    // point is actually abeam of.
    let mut station = best?.0;
    let mut local = [0.0; 3];
    for _ in 0..2 {
        let frame = road.frame_at(station, config).ok()?;
        local = frame.to_local(point);
        station = (station + local[0]).clamp(0.0, road.horizontal_length().ok()?);
    }

    Some(RoadPosition {
        s: station,
        t: local[1],
        height: local[2],
    })
}
