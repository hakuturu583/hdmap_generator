//! Lanes, and the roads they are gathered into.
//!
//! Two lanelets are side by side when one's right boundary *is* the other's left —
//! the same linestring, running the same way — which is how Lanelet2 draws lanes
//! that share a line of paint. A run of lanelets side by side is one road with one
//! cross-section. Its reference line is the boundary on the side the lanes are laid
//! out from: the right-hand boundary of the rightmost lane when traffic keeps left
//! (forward lanes sit left of the reference line), the left-hand boundary of the
//! leftmost when it keeps right. Every lane then runs forward, and its width is
//! measured across the reference line at each of its vertices.
//!
//! A lanelet tagged `turn_direction` is a road of its own, whatever it shares a
//! boundary with: it is a junction connector, and a connector carries one lane.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use ll2_core::id::Id;

use roadgen_core::geometry::{
    Bezier3, Curve3, Frame3, LaneHeight, Point3, Poly3Piece, Poly3Profile, Sample, SamplingConfig,
    Taper, WidthProfile,
};
use roadgen_core::id::{LaneId, RoadId};
use roadgen_core::map::{CrossSection, Lane, Map, Road};
use roadgen_core::semantics::{BoundaryMarking, LaneType, MarkingColor, RoadMarking, RoadType};
use roadgen_core::topology::{Direction, LateralSide, RoadLink};
use roadgen_core::units::{PositiveWidth, SpeedLimit};

use super::source::{Lanelet, Source};
use super::Approximations;
use crate::error::ImportError;

/// The narrowest a lane is read as, metres. A `WidthProfile` cannot reach zero, and
/// a lanelet whose boundaries meet — or cross — somewhere is held at this.
const MIN_WIDTH: f64 = 0.01;

/// How far across the reference line a boundary is looked for, metres. A lane is a
/// few metres wide; a crossing further out than this is some other stretch of a
/// boundary that bends back.
const MAX_REACH: f64 = 50.0;

/// What the lanes became, for the steps that follow.
pub(crate) struct Built {
    pub lane_of: HashMap<Id, LaneId>,
    pub road_of: HashMap<Id, RoadId>,
    /// The lanelets of each road, from the reference line outwards.
    pub lanelets_of: BTreeMap<RoadId, Vec<Id>>,
    /// Lanelets that became junction connectors.
    pub connectors: BTreeSet<Id>,
}

/// The IR lane type of a lanelet subtype, if it has one.
fn lane_type(subtype: &str) -> Option<LaneType> {
    Some(match subtype {
        "road" | "highway" | "play_street" | "bus_lane" => LaneType::Driving,
        "road_shoulder" => LaneType::Shoulder,
        "walkway" => LaneType::Sidewalk,
        "bicycle_lane" => LaneType::Biking,
        _ => return None,
    })
}

pub(crate) fn build(
    source: &Source,
    map: &mut Map,
    approximations: &mut Approximations,
) -> Result<Built, ImportError> {
    let mut built = Built {
        lane_of: HashMap::new(),
        road_of: HashMap::new(),
        lanelets_of: BTreeMap::new(),
        connectors: BTreeSet::new(),
    };

    let mut through: Vec<&Lanelet> = Vec::new();
    let mut turning: Vec<&Lanelet> = Vec::new();
    for lanelet in source.lanelets.values() {
        match lane_type(&lanelet.subtype) {
            Some(_) if lanelet.turn => turning.push(lanelet),
            Some(_) => through.push(lanelet),
            // A crosswalk is furniture, and read as such.
            None if lanelet.subtype == "crosswalk" => {}
            None => approximations.count(format!(
                "the IR has no lane for a `{}` lanelet, so {{n}} of them are left out",
                lanelet.subtype
            )),
        }
        if !lanelet.one_way
            && lane_type(&lanelet.subtype).is_some_and(|kind| kind != LaneType::Sidewalk)
        {
            approximations.count(
                "an IR lane runs one way, so {n} lanelets that may be driven both ways are \
                 read as running the way they are drawn",
            );
        }
    }

    let side = map.metadata.handedness.side_for(Direction::Forward);
    for group in side_by_side(&through, side) {
        make_road(source, map, &group, false, &mut built, approximations);
    }
    for lanelet in turning {
        make_road(source, map, &[lanelet], true, &mut built, approximations);
    }
    Ok(built)
}

/// Runs of lanelets side by side, each ordered outwards from the side the lanes are
/// laid out from.
fn side_by_side<'a>(lanelets: &[&'a Lanelet], side: LateralSide) -> Vec<Vec<&'a Lanelet>> {
    // Keyed by the way *and* which way round it runs, so a line two lanelets share
    // running opposite ways — the middle of a two-way street drawn as one line —
    // does not make them neighbours: they are not the same cross-section.
    fn key(way: Id, nodes: &[Id]) -> (Id, Id, Id) {
        (way, nodes[0], nodes[nodes.len() - 1])
    }
    let by_left: HashMap<(Id, Id, Id), &'a Lanelet> = lanelets
        .iter()
        .map(|lanelet| (key(lanelet.left_way, &lanelet.left), *lanelet))
        .collect();
    let by_right: HashMap<(Id, Id, Id), &'a Lanelet> = lanelets
        .iter()
        .map(|lanelet| (key(lanelet.right_way, &lanelet.right), *lanelet))
        .collect();
    // The neighbour further out from the reference line, and the one nearer it.
    let outer = |lanelet: &Lanelet| -> Option<&'a Lanelet> {
        match side {
            LateralSide::Left => by_right.get(&key(lanelet.left_way, &lanelet.left)),
            LateralSide::Right => by_left.get(&key(lanelet.right_way, &lanelet.right)),
        }
        .copied()
    };
    let inner = |lanelet: &Lanelet| -> Option<&'a Lanelet> {
        match side {
            LateralSide::Left => by_left.get(&key(lanelet.right_way, &lanelet.right)),
            LateralSide::Right => by_right.get(&key(lanelet.left_way, &lanelet.left)),
        }
        .copied()
    };

    let mut placed: BTreeSet<Id> = BTreeSet::new();
    let mut groups = Vec::new();
    for lanelet in lanelets {
        if inner(lanelet).is_some() || placed.contains(&lanelet.id) {
            continue;
        }
        let mut group = vec![*lanelet];
        placed.insert(lanelet.id);
        let mut current = *lanelet;
        while let Some(next) = outer(current) {
            if !placed.insert(next.id) {
                break;
            }
            group.push(next);
            current = next;
        }
        groups.push(group);
    }
    // Whatever is left sits in a ring of neighbours with no innermost lane, which a
    // map cannot draw; each is a road of its own rather than lost.
    for lanelet in lanelets {
        if placed.insert(lanelet.id) {
            groups.push(vec![*lanelet]);
        }
    }
    groups
}

/// Builds one road from lanelets ordered outwards from its reference line, or
/// leaves them out and says why.
fn make_road(
    source: &Source,
    map: &mut Map,
    group: &[&Lanelet],
    connector: bool,
    built: &mut Built,
    approximations: &mut Approximations,
) {
    let id = RoadId::new(group[0].id.to_string());
    match road(source, map, &id, group, connector) {
        // Lanes side by side whose ends are staggered along the road — one stops
        // twenty metres before its neighbour — cannot share one pair of ends, and
        // lanes on either side of a crown cannot share one tilt. Each run of them
        // that can becomes a road of its own.
        Ok(built_road) if built_road.miss > MAX_MISS && group.len() > 1 => {
            let fits = |run: &[&Lanelet]| {
                road(source, map, &id, run, connector).is_ok_and(|built| built.miss <= MAX_MISS)
            };
            let mut runs = Vec::new();
            let mut start = 0;
            while start < group.len() {
                let mut end = start + 1;
                while end < group.len() && fits(&group[start..=end]) {
                    end += 1;
                }
                runs.push(start..end);
                start = end;
            }
            for run in runs {
                make_road(source, map, &group[run], connector, built, approximations);
            }
        }
        Ok(BuiltRoad { road, lanes, miss }) => {
            if miss > MAX_MISS {
                approximations.count(
                    "OpenDRIVE ends a road's lanes square to its reference line and tilts \
                     them as one, so {n} lanelets whose end is drawn along the way they run, \
                     or whose surface is not one tilt, come out more than 10 cm from where \
                     the file draws them there; their boundaries in the IR, and in Lanelet2, \
                     are the file's",
                );
            }
            for (lanelet, lane) in group.iter().zip(&lanes) {
                built.lane_of.insert(lanelet.id, lane.id.clone());
                built.road_of.insert(lanelet.id, id.clone());
                if connector {
                    built.connectors.insert(lanelet.id);
                }
            }
            built
                .lanelets_of
                .insert(id.clone(), group.iter().map(|lanelet| lanelet.id).collect());
            // The ids are the lanelets', which the file keeps unique.
            map.roads.insert(id, road).ok();
            for lane in lanes {
                map.lanes.insert(lane.id.clone(), lane).ok();
            }
        }
        // One lanelet that cannot be measured across should not take its
        // neighbours with it: they are tried as roads of their own.
        Err(_) if group.len() > 1 => {
            for lanelet in group {
                make_road(source, map, &[lanelet], connector, built, approximations);
            }
        }
        Err(error) => {
            let ids: Vec<String> = group.iter().map(|lanelet| lanelet.id.to_string()).collect();
            approximations.note(format!(
                "lanelets {} could not be read as a road and are left out: {error}",
                ids.join(", ")
            ));
        }
    }
}

/// A road, and how far OpenDRIVE would move the furthest of its lane edges: along
/// the road, where a lane ends off the reference line's normal at that end — every
/// lane ends on it in OpenDRIVE — up and down, where an edge stands off the one
/// tilt a cross-section can have, or anywhere, where the lanes laid out along the
/// normals stray from the boundaries between the stations they were measured at.
struct BuiltRoad {
    road: Road,
    lanes: Vec<Lane>,
    miss: f64,
}

/// How far OpenDRIVE may move a lane edge before the lanes are split into separate
/// roads, metres.
const MAX_MISS: f64 = 0.1;

/// How far the reference line turns to meet each end square, as fractions of the
/// full turn: tried in this order until the lanes it lays out stay on the file's
/// boundaries.
const LEANS: [f64; 4] = [1.0, 0.5, 0.25, 0.0];

/// Below this the laid-out lanes are as good as they get, metres.
const GOOD_ENOUGH: f64 = 0.02;

/// Builds the road with the reference line that best describes its lanes.
///
/// Meeting a road's end square is what puts every lane's end where the file draws
/// it, but a reference line that has to turn hard to do it — the short end of a
/// tight turn through a junction — fans its normals so fast that the lanes laid
/// out along them fold into a loop before the end. So less of the turn is tried,
/// down to none, and the road that strays least from the file is kept.
///
/// A junction connector's reference line may also run down the middle of its lane
/// or along its far boundary: a turn tighter than the lane is wide folds every
/// normal from its outer boundary before they reach the inner one, and from the
/// inside or the middle they do not. It is only a connector's that may: an
/// ordinary road links to the next where their reference lines meet, and the
/// boundary on the reference side is where they do.
fn road(
    source: &Source,
    map: &Map,
    id: &RoadId,
    group: &[&Lanelet],
    connector: bool,
) -> Result<BuiltRoad, ImportError> {
    let side = map.metadata.handedness.side_for(Direction::Forward);
    // The cross-section's edges, outwards from the reference side: edge 0 the
    // innermost boundary, edge k the outer boundary of the k-th lane.
    let mut edges: Vec<Vec<Point3>> = vec![source.polyline(inner_and_outer(group[0], side).0)];
    for lanelet in group {
        edges.push(source.polyline(inner_and_outer(lanelet, side).1));
    }
    let mut bases = vec![edges[0].clone()];
    if connector && group.len() == 1 {
        bases.push(midline(&edges[0], &edges[1]));
        bases.push(edges[1].clone());
    }

    let mut best: Option<BuiltRoad> = None;
    let mut failure = None;
    'search: for base in &bases {
        for lean in LEANS {
            match road_along(source, map, id, group, &edges, base, lean) {
                Ok(built) => {
                    let done = built.miss <= GOOD_ENOUGH;
                    if best.as_ref().is_none_or(|kept| built.miss < kept.miss) {
                        best = Some(built);
                    }
                    if done {
                        break 'search;
                    }
                }
                Err(error) => failure = failure.or(Some(error)),
            }
        }
    }
    best.ok_or_else(|| failure.expect("every reference line was tried"))
}

/// Builds the road along one reference line: a smooth line through `base`, turned
/// `lean` of the way to meeting each end square.
fn road_along(
    source: &Source,
    map: &Map,
    id: &RoadId,
    group: &[&Lanelet],
    edges: &[Vec<Point3>],
    base: &[Point3],
    lean: f64,
) -> Result<BuiltRoad, ImportError> {
    let config = map.metadata.sampling;
    let side = map.metadata.handedness.side_for(Direction::Forward);

    let reference_line = smooth_reference(edges, base, side, config, lean)?;
    let length = reference_line.horizontal_length()?;
    let mut road = Road {
        id: id.clone(),
        name: Some(format!(
            "lanelet {}",
            group
                .iter()
                .map(|lanelet| lanelet.id.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )),
        reference_line,
        lane_offset: Poly3Profile::default(),
        lanes: Vec::new(),
        sections: Vec::new(),
        junction: None,
        link: RoadLink::default(),
        road_type: road_type(group[0].location.as_deref()),
        speed_limit: None,
        superelevation: Poly3Profile::default(),
    };

    // The road's cross-fall. Lanelet2 boundaries carry their own heights, and a
    // carriageway is rarely level across: its outer lanes sit lower, or higher, than
    // the line it is laid out from. OpenDRIVE lays every lane out along the reference
    // line's normal, so unless that normal is tilted the way the road is, every lane
    // comes out at the reference line's height — and a connector joining an outer
    // lane meets it with a step. So the tilt is measured at every station, as the
    // slope that best fits the edges' heights across the road, and becomes the
    // road's superelevation.
    let required = measuring_stations(&road.reference_line.samples(config)?, edges, length);
    let samples = road.reference_line.samples_including(config, &required)?;
    let at_end = |station: f64| station <= 0.0 || station >= length - 1e-9;
    let edge_at = |frame: &Frame3, station: f64, edge: &[Point3]| -> Option<Point3> {
        if station <= 0.0 {
            Some(edge[0])
        } else if station >= length - 1e-9 {
            edge.last().copied()
        } else {
            crossing(frame, edge).map(|(_, point)| point)
        }
    };
    let mut rolls: Vec<(f64, f64)> = Vec::new();
    let mut unevenness: f64 = 0.0;
    for sample in &samples {
        let frame = sample.frame()?;
        let (lx, ly) = (frame.left.get().x, frame.left.get().y);
        let (mut moment, mut spread) = (0.0, 0.0);
        for edge in edges {
            if let Some(point) = edge_at(&frame, sample.station, edge) {
                let across = (point.x - frame.origin.x) * lx + (point.y - frame.origin.y) * ly;
                moment += across * (point.z - frame.origin.z);
                spread += across * across;
            }
        }
        if spread > MIN_SPACING * MIN_SPACING {
            let slope = moment / spread;
            rolls.push((sample.station, slope.atan()));
            // How far the edges stand off the plane that slope describes: a
            // carriageway crowned between its lanes is not one tilt.
            for edge in edges {
                if let Some(point) = edge_at(&frame, sample.station, edge) {
                    let across = (point.x - frame.origin.x) * lx + (point.y - frame.origin.y) * ly;
                    let off = (point.z - frame.origin.z - slope * across).abs();
                    unevenness = unevenness.max(off);
                }
            }
        }
    }
    road.superelevation = linear_profile(&rolls)?;

    // Where each edge crosses the reference line's normal, now tilted, all along
    // it. At the two ends it is where the edge itself starts and stops, measured
    // across the road the way validation measures it, so the widths agree with the
    // boundaries there exactly. Edge 0 is the reference line's own boundary, which
    // the smooth line runs through at every vertex and bows away from a little in
    // between; how far is the lane offset.
    let start_frame = road.frame_at(0.0, config)?;
    let end_frame = road.frame_at(length, config)?;
    let mut offsets: Vec<Vec<(f64, f64)>> = vec![Vec::new(); edges.len()];
    for sample in &samples {
        let station = sample.station;
        let frame = if station <= 0.0 {
            start_frame
        } else if station >= length - 1e-9 {
            end_frame
        } else {
            sample
                .frame()?
                .banked(road.superelevation.evaluate(station))
        };
        for (k, edge) in edges.iter().enumerate() {
            let offset = if at_end(station) {
                edge_at(&frame, station, edge).map(|point| across(&frame, point))
            } else {
                crossing(&frame, edge).map(|(offset, _)| offset)
            };
            if let Some(offset) = offset {
                offsets[k].push((station, offset));
            }
        }
    }
    road.lane_offset = linear_profile(&offsets[0])?;

    let mut lanes = Vec::new();
    for (index, lanelet) in group.iter().enumerate() {
        let ordinal = index + 1;
        let (inner, outer) = (&offsets[index], &offsets[index + 1]);
        // Both edges have a value at both ends; in between, only at stations where
        // both were found.
        let mut knots = Vec::new();
        for (station, outer_offset) in outer {
            let Some((_, inner_offset)) = inner
                .iter()
                .find(|(other, _)| (other - station).abs() < 1e-9)
            else {
                continue;
            };
            let width = side.sign() * (outer_offset - inner_offset);
            let at_end = *station == 0.0 || *station == length;
            if at_end && width <= 0.0 {
                // Validation measures a lane's ends across the reference line, and
                // these boundaries end on the wrong sides of each other there: a
                // lanelet that turns a corner with its end drawn along the way it
                // runs, which no cross-section can describe.
                return Err(ImportError::Unsupported(format!(
                    "lanelet {}, whose end is drawn along the way it runs: measured across \
                     the road there, its left boundary is {:.3} m to the right of its right",
                    lanelet.id, -width
                )));
            }
            knots.push((*station, PositiveWidth::new(width.max(MIN_WIDTH))?));
        }
        let width = WidthProfile::new(knots, Taper::Linear)?;

        let left = source.polyline(&lanelet.left);
        let right = source.polyline(&lanelet.right);
        let (left_edge, right_edge) = match side {
            LateralSide::Left => (ordinal as i32, ordinal as i32 - 1),
            LateralSide::Right => (-(ordinal as i32 - 1), -(ordinal as i32)),
        };
        let centerline = Curve3::polyline(midline(&left, &right))?;
        lanes.push(Lane {
            id: LaneId::of_road(id, index),
            road: id.clone(),
            index,
            side,
            ordinal,
            direction: Direction::Forward,
            lane_type: lane_type(&lanelet.subtype).unwrap_or(LaneType::Driving),
            width,
            // The boundaries are the file's, heights and all; the lift OpenDRIVE
            // would need to reproduce them off the one tilted surface is not
            // measured yet.
            height: LaneHeight::flat(),
            speed_limit: lanelet
                .speed_limit_kph
                .map(SpeedLimit::from_kph)
                .transpose()?,
            section: 0,
            station_range: (0.0, length),
            left_edge,
            right_edge,
            left_offset: across(&start_frame, left[0]),
            right_offset: across(&start_frame, right[0]),
            left_marking: marking(source, lanelet.left_way),
            right_marking: marking(source, lanelet.right_way),
            left_boundary: Curve3::polyline(left)?,
            right_boundary: Curve3::polyline(right)?,
            centerline,
        });
    }

    let limits: BTreeSet<u64> = lanes
        .iter()
        .filter_map(|lane| lane.speed_limit.map(|limit| limit.kph().to_bits()))
        .collect();
    if limits.len() == 1 && lanes.iter().all(|lane| lane.speed_limit.is_some()) {
        road.speed_limit = lanes[0].speed_limit;
    }
    road.lanes = lanes.iter().map(|lane| lane.id.clone()).collect();
    road.sections = vec![CrossSection {
        station: 0.0,
        lanes: road.lanes.clone(),
    }];
    let miss = edges
        .iter()
        .flat_map(|edge| {
            [
                (start_frame, edge[0]),
                (end_frame, *edge.last().expect("an edge has points")),
            ]
        })
        .map(|(frame, point)| (point - frame.origin).dot(frame.tangent.get()).abs())
        .fold(0.0, f64::max);
    let strays = strays(&road, &lanes, edges, config)?;
    Ok(BuiltRoad {
        road,
        lanes,
        miss: miss.max(unevenness).max(strays),
    })
}

/// How far the lane edges the road describes — its reference line, tilted by its
/// superelevation, with the lane offset and widths stacked along the normal, which
/// is what OpenDRIVE receives — stray from the file's boundaries, metres, in 3D.
///
/// Checked at every sample and halfway between each two: on the samples the edges
/// were measured, so a fold shows only between them, where the normals have turned
/// further than the straight runs of width can follow.
fn strays(
    road: &Road,
    lanes: &[Lane],
    edges: &[Vec<Point3>],
    config: SamplingConfig,
) -> Result<f64, ImportError> {
    let coarse = road.reference_line.samples(config)?;
    let halfway: Vec<f64> = coarse
        .windows(2)
        .map(|pair| (pair[0].station + pair[1].station) / 2.0)
        .collect();
    let samples = road.reference_line.samples_including(config, &halfway)?;
    let mut worst: f64 = 0.0;
    for sample in &samples {
        let station = sample.station;
        let frame = sample
            .frame()?
            .banked(road.superelevation.evaluate(station));
        let mut offset = road.lane_offset.evaluate(station);
        for (k, edge) in edges.iter().enumerate() {
            if k > 0 {
                let lane = &lanes[k - 1];
                offset += lane.side.sign() * lane.width.evaluate(station).metres();
            }
            let point = frame.origin + frame.left.get() * offset;
            worst = worst.max(distance_to(edge, point));
        }
    }
    Ok(worst)
}

/// The distance from a point to a polyline, in 3D.
fn distance_to(polyline: &[Point3], point: Point3) -> f64 {
    polyline
        .windows(2)
        .map(|pair| {
            let (a, b) = (pair[0], pair[1]);
            let along = b - a;
            let length_squared = along.dot(along);
            let u = if length_squared > 0.0 {
                ((point - a).dot(along) / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            a.lerp(b, u).distance_to(point)
        })
        .fold(f64::INFINITY, f64::min)
}

/// Stations to measure the widths at, strictly between the ends: every vertex of
/// the reference line, and the station abeam every vertex of every edge — a
/// width is a straight run between two of these, so a boundary's corner needs a
/// station of its own or the run cuts across it.
fn measuring_stations(samples: &[Sample], edges: &[Vec<Point3>], length: f64) -> Vec<f64> {
    let mut stations: Vec<f64> = samples.iter().map(|sample| sample.station).collect();
    for edge in edges {
        for point in edge {
            if let Some(station) = station_of(samples, *point) {
                stations.push(station);
            }
        }
    }
    stations.retain(|station| *station > 1e-6 && *station < length - 1e-6);
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| (*a - *b).abs() < 1e-3);
    stations
}

/// The station on the reference line nearest a point, in plan.
fn station_of(samples: &[Sample], point: Point3) -> Option<f64> {
    let mut best: Option<(f64, f64)> = None;
    for pair in samples.windows(2) {
        let (a, b) = (pair[0].point, pair[1].point);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let length_squared = dx * dx + dy * dy;
        if length_squared <= 0.0 {
            continue;
        }
        let u = (((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared).clamp(0.0, 1.0);
        let distance = (a.x + dx * u - point.x).hypot(a.y + dy * u - point.y);
        if best.is_none_or(|(_, previous)| distance < previous) {
            best = Some((
                pair[0].station + (pair[1].station - pair[0].station) * u,
                distance,
            ));
        }
    }
    best.map(|(station, _)| station)
}

/// How far a road's end may lean before the reference line stops turning to meet
/// it square, radians. Beyond this the end is drawn along the way the road runs
/// more than across it, and turning the line that far would fold the cross-section.
const MAX_END_LEAN: f64 = std::f64::consts::FRAC_PI_4;

/// Vertices of the reference boundary closer together than this are one vertex,
/// metres; a cubic through two nearly coincident points only wobbles.
const MIN_SPACING: f64 = 0.05;

/// The reference line: a smooth curve through the vertices of edge 0.
///
/// OpenDRIVE lays every lane out along the reference line's normal. A polyline has
/// a normal that jumps at every vertex, and it is written as one `<line>` per
/// segment, so every lane but the one on the line would break at every vertex, by
/// its offset times the angle turned. So the line is a chain of cubics through the
/// same vertices, their tangents the directions from each vertex's neighbour to
/// the next (Catmull–Rom): continuous, and within centimetres of the boundary.
///
/// At its two ends the tangent is square to the road's end — the line from where
/// edge 0 ends to where the outermost edge ends. A lanelet's end is where it meets
/// the next lanelet, and it is rarely drawn square to the lane; OpenDRIVE ends
/// every lane on the normal at the end of the reference line, so a square normal is
/// the only one on which every lane ends where the file says. Two roads that meet
/// share that end, so they turn to the same direction there and stay continuous.
fn smooth_reference(
    edges: &[Vec<Point3>],
    base: &[Point3],
    side: LateralSide,
    config: SamplingConfig,
    lean: f64,
) -> Result<Curve3, ImportError> {
    let mut points: Vec<Point3> = vec![base[0]];
    for point in &base[1..base.len() - 1] {
        if point.horizontal_distance_to(*points.last().unwrap()) >= MIN_SPACING {
            points.push(*point);
        }
    }
    let last = *base.last().unwrap();
    if points.len() > 1 && last.horizontal_distance_to(*points.last().unwrap()) < MIN_SPACING {
        points.pop();
    }
    points.push(last);
    if points.len() < 2 {
        return Ok(Curve3::polyline(base.iter().copied())?);
    }

    let n = points.len();
    let (innermost, outermost) = (&edges[0], edges.last().unwrap());
    // Horizontal direction and grade of the tangent at each vertex.
    let mut tangents: Vec<(f64, f64, f64)> = Vec::with_capacity(n);
    for i in 0..n {
        let (from, to) = (points[i.saturating_sub(1)], points[(i + 1).min(n - 1)]);
        let run = from.horizontal_distance_to(to);
        let (dx, dy) = ((to.x - from.x) / run, (to.y - from.y) / run);
        tangents.push((dx, dy, (to.z - from.z) / run));
    }
    for (index, near, far) in [
        (0, innermost[0], outermost[0]),
        (
            n - 1,
            *innermost.last().unwrap(),
            *outermost.last().unwrap(),
        ),
    ] {
        let (dx, dy, grade) = tangents[index];
        let (cx, cy) = (far.x - near.x, far.y - near.y);
        let across = cx.hypot(cy);
        if across < MIN_SPACING {
            continue;
        }
        // The end runs from edge 0 out to the outermost edge, which is to the
        // left of the road when the lanes are on its left.
        let (lx, ly) = (side.sign() * cx / across, side.sign() * cy / across);
        let (tx, ty) = (ly, -lx);
        let turn = (dx * ty - dy * tx).atan2(dx * tx + dy * ty);
        if turn.abs() <= MAX_END_LEAN {
            // `lean` of the way from the boundary's own direction to square.
            let (sin, cos) = (turn * lean).sin_cos();
            tangents[index] = (dx * cos - dy * sin, dx * sin + dy * cos, grade);
        }
    }

    let mut pieces = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        let (start, end) = (points[i], points[i + 1]);
        let handle = start.horizontal_distance_to(end) / 3.0;
        // Smooth in plan, straight in height: the boundary climbs straight from one
        // vertex to the next, and a cubic height would bow away from it between.
        let along = |point: Point3, (dx, dy, _): (f64, f64, f64), sign: f64, height: f64| {
            Point3::new(
                point.x + sign * dx * handle,
                point.y + sign * dy * handle,
                height,
            )
        };
        let climb = end.z - start.z;
        let control = [
            start,
            along(start, tangents[i], 1.0, start.z + climb / 3.0),
            along(end, tangents[i + 1], -1.0, start.z + 2.0 * climb / 3.0),
            end,
        ];
        pieces.push(Curve3::Bezier(Bezier3::new(control, config)?));
    }
    Ok(Curve3::composite(pieces)?)
}

/// A lanelet's boundary nearer the reference line, and the one further out.
fn inner_and_outer(lanelet: &Lanelet, side: LateralSide) -> (&[Id], &[Id]) {
    match side {
        LateralSide::Left => (&lanelet.right, &lanelet.left),
        LateralSide::Right => (&lanelet.left, &lanelet.right),
    }
}

fn road_type(location: Option<&str>) -> RoadType {
    match location {
        Some("nonurban") => RoadType::Rural,
        // Autoware's `private` is a car park or a private drive.
        Some("private") => RoadType::LowSpeed,
        _ => RoadType::Town,
    }
}

/// How far to the left of the frame's origin a point is, across the road.
fn across(frame: &Frame3, point: Point3) -> f64 {
    (point - frame.origin).dot(frame.left.get())
}

/// Where the frame's lateral axis, in plan, crosses a polyline: the offset to the
/// left of the origin along that axis, nearest first, and the point of the polyline
/// it crosses at. `None` when it does not cross within reach.
fn crossing(frame: &Frame3, polyline: &[Point3]) -> Option<(f64, Point3)> {
    let (ox, oy) = (frame.origin.x, frame.origin.y);
    let left = frame.left.get();
    let (lx, ly) = (left.x, left.y);
    let mut best: Option<(f64, Point3)> = None;
    for pair in polyline.windows(2) {
        let (ax, ay) = (pair[0].x, pair[0].y);
        let (dx, dy) = (pair[1].x - ax, pair[1].y - ay);
        let denominator = lx * dy - ly * dx;
        if denominator.abs() < 1e-12 {
            continue;
        }
        let (wx, wy) = (ax - ox, ay - oy);
        let t = (wx * dy - wy * dx) / denominator;
        let u = (wx * ly - wy * lx) / denominator;
        if (-1e-9..=1.0 + 1e-9).contains(&u)
            && t.abs() <= MAX_REACH
            && best.is_none_or(|(previous, _)| t.abs() < previous.abs())
        {
            best = Some((t, pair[0].lerp(pair[1], u.clamp(0.0, 1.0))));
        }
    }
    best
}

/// A profile running straight from each `(station, value)` to the next.
fn linear_profile(knots: &[(f64, f64)]) -> Result<Poly3Profile, ImportError> {
    Ok(Poly3Profile::new(knots.windows(2).filter_map(|pair| {
        let ((from, a), (to, b)) = (pair[0], pair[1]);
        (to - from > 0.0).then(|| Poly3Piece::new(from, a, (b - a) / (to - from), 0.0, 0.0))
    }))?)
}

/// The line halfway between two boundaries: both walked at the same fraction of
/// their length, so the ends are the midpoints of the ends — which is what makes two
/// lanelets that share their end points share their centreline's end too.
fn midline(left: &[Point3], right: &[Point3]) -> Vec<Point3> {
    let count = left.len().max(right.len()).max(2);
    (0..count)
        .map(|i| {
            let fraction = i as f64 / (count - 1) as f64;
            at_fraction(left, fraction).lerp(at_fraction(right, fraction), 0.5)
        })
        .collect()
}

fn at_fraction(polyline: &[Point3], fraction: f64) -> Point3 {
    let lengths: Vec<f64> = polyline
        .windows(2)
        .map(|pair| pair[0].distance_to(pair[1]))
        .collect();
    let total: f64 = lengths.iter().sum();
    if total <= 0.0 || fraction <= 0.0 {
        return polyline[0];
    }
    if fraction >= 1.0 {
        return *polyline.last().unwrap();
    }
    let mut remaining = fraction * total;
    for (index, length) in lengths.iter().enumerate() {
        if remaining <= *length && *length > 0.0 {
            return polyline[index].lerp(polyline[index + 1], remaining / length);
        }
        remaining -= length;
    }
    *polyline.last().unwrap()
}

/// The paint on a boundary, from its way's tags.
fn marking(source: &Source, way: Id) -> BoundaryMarking {
    let color = match source.way_tag(way, "color") {
        Some("yellow") | Some("orange") => MarkingColor::Yellow,
        _ => MarkingColor::White,
    };
    let marking = match (source.way_tag(way, "type"), source.way_tag(way, "subtype")) {
        (Some("virtual"), _) => RoadMarking::None,
        (Some("line_thin" | "line_thick"), subtype) => match subtype {
            Some("dashed") => RoadMarking::Broken,
            Some("solid_solid") => RoadMarking::SolidSolid,
            Some("dashed_solid") => RoadMarking::BrokenSolid,
            Some("solid_dashed") => RoadMarking::SolidBroken,
            _ => RoadMarking::Solid,
        },
        (Some("curbstone" | "road_border" | "guard_rail" | "wall" | "fence"), _) => {
            RoadMarking::Curbstone
        }
        _ => RoadMarking::None,
    };
    BoundaryMarking::new(marking, color)
}
