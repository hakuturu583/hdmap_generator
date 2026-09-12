//! The ego track: a vehicle driven along the generated map.
//!
//! ClipGT is a *scene* format, not a map format, and its reader will not look at a
//! directory at all unless it holds an egomotion table. So a map on its own cannot be
//! written as a clip; something has to drive through it.
//!
//! What is generated here is the simplest honest thing: a route followed lane by
//! lane, travelled at a constant speed, sampled at the clip's frame rate. The pose at
//! each sample is the road's own frame — which means the vehicle rolls with a banked
//! surface rather than staying pinned upright, and climbs with the grade. None of
//! that is in the IR as such; it is read back out of the geometry the IR already has.

use roadgen_core::geometry::{Frame3, Point3, UnitVector3, Vector3};
use roadgen_core::map::Map;
use roadgen_core::topology::Direction;
use roadgen_core::{LaneId, ValidatedMap};

use crate::error::ExportError;

/// One pose of the ego vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// Microseconds from the start of the clip.
    pub timestamp_micros: i64,
    pub position: Point3,
    /// The vehicle's orientation as `(x, y, z, w)`, mapping its own
    /// forward-left-up axes into the map's.
    pub orientation: [f64; 4],
}

/// A point on the driven path, with the roll of the surface under it.
#[derive(Debug, Clone, Copy)]
struct Node {
    point: Point3,
    roll: f64,
}

/// The lanes a vehicle can drive in one run, starting from `start`.
///
/// Follows the first successor at every branch and stops when the route would
/// repeat a lane, so a ring road terminates rather than looping forever.
pub fn route_from(map: &Map, start: &LaneId) -> Vec<LaneId> {
    let mut route = vec![start.clone()];
    while let Some(next) = map
        .successors(route.last().expect("non-empty"))
        .first()
        .cloned()
    {
        if route.contains(&next) {
            break;
        }
        route.push(next);
    }
    route
}

/// A lane to set off from: the first drivable lane of a road that is not a junction
/// connector, in the order the caller added them.
pub fn default_start(map: &Map) -> Option<LaneId> {
    map.lanes
        .iter()
        .find(|lane| {
            lane.lane_type.is_drivable()
                && map
                    .road(&lane.road)
                    .is_some_and(|road| !road.is_connector())
        })
        .map(|lane| lane.id.clone())
}

/// Drives `route` at `speed` metres per second, sampling at `frame_rate` hertz.
pub fn track(
    map: &ValidatedMap,
    route: &[LaneId],
    speed: f64,
    frame_rate: f64,
) -> Result<Vec<Pose>, ExportError> {
    if route.is_empty() {
        return Err(ExportError::NoRoute("the route is empty".into()));
    }
    if !(speed.is_finite() && speed > 0.0) {
        return Err(ExportError::NoRoute(format!(
            "a speed of {speed} m/s gets nowhere"
        )));
    }
    if !(frame_rate.is_finite() && frame_rate > 0.0) {
        return Err(ExportError::NoRoute(format!(
            "a frame rate of {frame_rate} Hz has no frames"
        )));
    }

    let nodes = path(map, route)?;
    if nodes.len() < 2 {
        return Err(ExportError::NoRoute(
            "the route is shorter than one segment".into(),
        ));
    }

    // Distance to each node, so a sample at a given time can be placed by arc length
    // rather than by counting vertices — which are not evenly spaced.
    let mut travelled = vec![0.0];
    for pair in nodes.windows(2) {
        let last = travelled.last().copied().expect("non-empty");
        travelled.push(last + pair[0].point.distance_to(pair[1].point));
    }
    let total = travelled.last().copied().expect("non-empty");

    let step = speed / frame_rate;
    let frames = (total / step).floor() as usize + 1;
    let micros_per_frame = 1e6 / frame_rate;

    let mut poses = Vec::with_capacity(frames);
    let mut segment = 0usize;
    for frame in 0..frames {
        let distance = (frame as f64 * step).min(total);
        while segment + 2 < nodes.len() && travelled[segment + 1] < distance {
            segment += 1;
        }
        let (from, to) = (nodes[segment], nodes[segment + 1]);
        let span = travelled[segment + 1] - travelled[segment];
        let fraction = if span > 0.0 {
            ((distance - travelled[segment]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let position = from.point.lerp(to.point, fraction);
        let roll = from.roll + (to.roll - from.roll) * fraction;
        let heading = UnitVector3::try_new(to.point - from.point)?;
        let frame3 = Frame3::from_tangent(position, heading)?.banked(roll);

        poses.push(Pose {
            timestamp_micros: (frame as f64 * micros_per_frame).round() as i64,
            position,
            orientation: orientation_of(&frame3),
        });
    }
    Ok(poses)
}

/// The path the route traces, in travel order, with the surface roll at each vertex.
fn path(map: &ValidatedMap, route: &[LaneId]) -> Result<Vec<Node>, ExportError> {
    let config = map.metadata.sampling;
    let mut nodes: Vec<Node> = Vec::new();
    for id in route {
        let lane = map
            .lane(id)
            .ok_or_else(|| ExportError::NoRoute(format!("{id} is not a lane of this map")))?;
        let road = map
            .road(&lane.road)
            .ok_or_else(|| ExportError::NoRoute(format!("{id} has no road")))?;

        // Sampling the reference-line-order centreline gives each vertex's station,
        // which is what the superelevation profile is written against; the travel
        // order is then a reversal of the list for a backward lane.
        let mut samples = lane.centerline.samples(config)?;
        let start = lane.station_range.0;
        if lane.direction == Direction::Backward {
            samples.reverse();
        }
        for sample in samples {
            let node = Node {
                point: sample.point,
                roll: road.superelevation.evaluate(start + sample.station),
            };
            match nodes.last() {
                // Lanes joined through a junction share their endpoint; keeping both
                // would put a zero-length segment in the middle of the path.
                Some(previous) if previous.point.distance_to(node.point) < 1e-6 => continue,
                _ => nodes.push(node),
            }
        }
    }
    Ok(nodes)
}

/// The quaternion `(x, y, z, w)` of the rotation taking forward-left-up onto `frame`.
pub(crate) fn orientation_of(frame: &Frame3) -> [f64; 4] {
    let (f, l, u) = (frame.tangent.get(), frame.left.get(), frame.up.get());
    // Columns of the rotation matrix are the body axes in map coordinates, so
    // `m[row][column]` reads m[world axis][body axis].
    let m = [[f.x, l.x, u.x], [f.y, l.y, u.y], [f.z, l.z, u.z]];
    let trace = m[0][0] + m[1][1] + m[2][2];

    // The largest of the four components is computed first and the rest divided
    // through it; picking the wrong one divides by something near zero.
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        ]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[1][0] - m[0][1]) / s,
        ]
    }
}

/// Rotates `vector` by the quaternion `(x, y, z, w)`. Only the tests need this, but
/// it is the definition the exporter is asserting, so it lives beside it.
pub fn rotate(quaternion: [f64; 4], vector: Vector3) -> Vector3 {
    let [x, y, z, w] = quaternion;
    let q = Vector3::new(x, y, z);
    let t = q.cross(vector) * 2.0;
    vector + t * w + q.cross(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::geometry::UnitVector3;

    fn frame(tangent: Vector3) -> Frame3 {
        Frame3::from_tangent(Point3::ORIGIN, UnitVector3::try_new(tangent).unwrap()).unwrap()
    }

    #[test]
    fn the_orientation_maps_the_vehicles_axes_onto_the_roads() {
        for tangent in [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.2),
            Vector3::new(0.0, -1.0, -0.3),
        ] {
            let frame = frame(tangent);
            let quaternion = orientation_of(&frame);
            // Forward is x, left is y, up is z — in the vehicle's own frame.
            let forward = rotate(quaternion, Vector3::new(1.0, 0.0, 0.0));
            let left = rotate(quaternion, Vector3::new(0.0, 1.0, 0.0));
            let up = rotate(quaternion, Vector3::new(0.0, 0.0, 1.0));
            assert!((forward - frame.tangent.get()).norm() < 1e-9, "{tangent:?}");
            assert!((left - frame.left.get()).norm() < 1e-9, "{tangent:?}");
            assert!((up - frame.up.get()).norm() < 1e-9, "{tangent:?}");
        }
    }

    #[test]
    fn a_banked_frame_tips_the_vehicle_with_the_road() {
        let upright = orientation_of(&frame(Vector3::new(1.0, 0.0, 0.0)));
        let banked = orientation_of(&frame(Vector3::new(1.0, 0.0, 0.0)).banked(0.1));
        let up = rotate(banked, Vector3::new(0.0, 0.0, 1.0));
        assert!(up.z < 1.0, "a rolled vehicle's up is no longer vertical");
        assert!((up.y.abs() - 0.1_f64.sin()).abs() < 1e-9);
        assert!(rotate(upright, Vector3::new(0.0, 0.0, 1.0)).z > 0.999);
    }

    #[test]
    fn the_orientation_is_a_unit_quaternion() {
        let quaternion = orientation_of(&frame(Vector3::new(0.3, -0.9, 0.1)).banked(-0.05));
        let norm: f64 = quaternion.iter().map(|value| value * value).sum();
        assert!((norm - 1.0).abs() < 1e-12);
    }
}
