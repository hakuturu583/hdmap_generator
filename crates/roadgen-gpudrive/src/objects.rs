//! The agents: vehicles driven along the generated map, as GPUDrive tracks.
//!
//! GPUDrive is a simulator of *scenes*, not a map viewer. Its scenes come from logged
//! driving, so an object is a track — where the agent was at each timestep, which way
//! it faced and how fast it was going — and the simulator either replays that track or
//! hands control of the agent to a policy and scores it against the goal the track
//! ended at.
//!
//! A generated map has no logs. What is written here is the simplest honest stand-in:
//! a route followed lane by lane at a constant speed, sampled at the scene's timestep.
//! It is a plan-view track, because GPUDrive is a plane — the distances are measured
//! in plan view too, so that an agent asked for 10 m/s travels ten metres of ground a
//! second rather than ten metres of a climbing road.

use roadgen_core::geometry::Point3;
use roadgen_core::{LaneId, ValidatedMap};

use crate::error::ExportError;
use crate::scene::{Object, ObjectKind, Vector2};
use crate::{Agent, Route};

/// Two positions closer than this are the same place: lanes joined through a junction
/// share their endpoint, and keeping both would put a zero-length step in the path.
const WELD_TOLERANCE: f64 = 1e-6;

/// The lanes an agent drives, whether it named them, named a start, or named nothing.
pub fn route_of(map: &ValidatedMap, agent: &Agent) -> Result<Vec<LaneId>, ExportError> {
    let route = match &agent.route {
        Some(Route::Lanes(lanes)) => {
            for lane in lanes {
                if map.lane(lane).is_none() {
                    return Err(ExportError::NoRoute(format!(
                        "{lane} is not a lane of this map"
                    )));
                }
            }
            lanes.clone()
        }
        Some(Route::From(start)) => {
            if map.lane(start).is_none() {
                return Err(ExportError::NoRoute(format!(
                    "{start} is not a lane of this map"
                )));
            }
            map.route_from(start)
        }
        None => {
            let start = map.default_start().ok_or_else(|| {
                ExportError::NoRoute("the map has no drivable lane outside a junction".into())
            })?;
            map.route_from(&start)
        }
    };
    if route.is_empty() {
        return Err(ExportError::NoRoute("the route is empty".into()));
    }
    Ok(route)
}

/// Drives `agent` over `steps` timesteps of `time_step` seconds each, and writes the
/// result as the track GPUDrive reads.
///
/// The agent sets off at the start of its route and stops where the route runs out —
/// it stands at its goal for whatever is left of the episode rather than vanishing,
/// because an agent that is there for the whole scene is what the other agents have to
/// deal with. Every timestep is therefore valid.
pub fn track(
    map: &ValidatedMap,
    agent: &Agent,
    id: u32,
    steps: usize,
    time_step: f64,
) -> Result<Object, ExportError> {
    if !(time_step.is_finite() && time_step > 0.0) {
        return Err(ExportError::NoRoute(format!(
            "a timestep of {time_step} s gets nowhere"
        )));
    }
    if steps == 0 {
        return Err(ExportError::NoRoute(
            "a scene of no timesteps has nothing to drive through it".into(),
        ));
    }
    if !(agent.speed.is_finite() && agent.speed >= 0.0) {
        return Err(ExportError::NoRoute(format!(
            "{} m/s is not a speed",
            agent.speed
        )));
    }

    let route = route_of(map, agent)?;
    let path = path(map, &route)?;
    if path.len() < 2 {
        return Err(ExportError::NoRoute(
            "the route is shorter than one segment".into(),
        ));
    }

    // Distance to each vertex, so a sample is placed by arc length rather than by
    // counting vertices, which are not evenly spaced.
    let mut travelled = Vec::with_capacity(path.len());
    travelled.push(0.0);
    for pair in path.windows(2) {
        let last = travelled.last().copied().expect("non-empty");
        travelled.push(last + pair[0].horizontal_distance_to(pair[1]));
    }
    let total = travelled.last().copied().expect("non-empty");

    let mut position = Vec::with_capacity(steps);
    let mut heading = Vec::with_capacity(steps);
    let mut velocity = Vec::with_capacity(steps);
    let mut segment = 0usize;
    for step in 0..steps {
        let distance = (step as f64 * agent.speed * time_step).min(total);
        while segment + 2 < path.len() && travelled[segment + 1] < distance {
            segment += 1;
        }
        let (from, to) = (path[segment], path[segment + 1]);
        let span = travelled[segment + 1] - travelled[segment];
        let fraction = if span > 0.0 {
            ((distance - travelled[segment]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let here = from.lerp(to, fraction);
        let along = to - from;
        let yaw = along.y.atan2(along.x);
        // Once the route has run out the agent is standing at its goal, so it has a
        // heading but no velocity.
        let moving = distance < total;
        position.push(Vector2::new(here.x, here.y));
        heading.push(yaw);
        velocity.push(if moving {
            Vector2::new(agent.speed * yaw.cos(), agent.speed * yaw.sin())
        } else {
            Vector2::new(0.0, 0.0)
        });
    }

    let goal = path.last().copied().expect("non-empty");
    Ok(Object {
        position,
        width: agent.width,
        length: agent.length,
        height: agent.height,
        id,
        heading,
        velocity,
        valid: vec![true; steps],
        goal_position: Vector2::new(goal.x, goal.y),
        kind: agent.kind,
        mark_as_expert: agent.mark_as_expert,
    })
}

/// The route's path, in travel order.
///
/// The heights ride along unused: what the track is paced and aimed by is plan-view
/// distance, because GPUDrive is a plane, and flattening at the last moment keeps this
/// function saying only what order the lanes come in.
fn path(map: &ValidatedMap, route: &[LaneId]) -> Result<Vec<Point3>, ExportError> {
    let config = map.metadata.sampling;
    let mut path: Vec<Point3> = Vec::new();
    for id in route {
        let lane = map
            .lane(id)
            .ok_or_else(|| ExportError::NoRoute(format!("{id} is not a lane of this map")))?;
        for point in lane.travel_polyline(config)?.points() {
            match path.last() {
                Some(previous) if previous.horizontal_distance_to(*point) < WELD_TOLERANCE => {
                    continue
                }
                _ => path.push(*point),
            }
        }
    }
    Ok(path)
}

/// The default size of an agent of each kind, metres.
///
/// The IR describes roads, not traffic, so there is nothing in it to take a vehicle's
/// dimensions from. These are ordinary sizes for the three kinds GPUDrive knows —
/// a family car, a person, a bicycle and rider — and a scenario that cares says so.
pub fn default_size(kind: ObjectKind) -> (f64, f64, f64) {
    match kind {
        // (length, width, height)
        ObjectKind::Vehicle => (4.6, 2.0, 1.6),
        ObjectKind::Pedestrian => (0.6, 0.6, 1.8),
        ObjectKind::Cyclist => (1.8, 0.7, 1.7),
    }
}
