//! A motorway corridor: the line a motorway follows, the rules it is built to, and
//! a plan of the roads that make it up.
//!
//! A corridor is a part, like a pit: it shapes the land — its at-grade stretches
//! are cut and filled to meet the country, its others carried on bridges or through
//! tunnels — and it does not put a road on the map by itself. What it does instead
//! is *plan* one: [`CorridorSpec::plan`] works out every road a motorway of this
//! line needs — both carriageways, split where a slip road leaves or joins, with
//! the speed-change lane before each; the slip roads; the crossroad an interchange
//! is built on, lifted over the motorway or dropped under it; the junctions and
//! signals at the ramp terminals — and hands back a [`CorridorPlan`], points and
//! lanes and connections, for the caller to look at, change, and put on the map
//! with [`CorridorPlan::apply`]. The shape of every road stays the caller's to
//! decide; the plan is a first draft of it.
//!
//! # The rules
//!
//! The defaults follow Japan's Road Structure Ordinance (道路構造令) for the
//! design speed asked for — the least radius of a bend, the steepest grade, the
//! radius of a vertical curve, lane and shoulder widths, the median — as recorded in
//! [`DesignStandard`]. They are defaults: every value can be replaced, and a road
//! drawn outside them is reported in the plan's notes rather than refused.

use roadgen_core::builder::{LaneRef, LaneSpec, MapBuilder, RoadSpec};
use roadgen_core::geometry::{Curve3, Point3, Taper, VerticalProfile, WidthProfile};
use roadgen_core::id::{JunctionId, RoadId};
use roadgen_core::map::TrafficHandedness;
use roadgen_core::semantics::{
    BoundaryMarking, FormOfWay, LaneType, MarkingColor, RoadMarking, RoadType,
};
use roadgen_core::structure::{
    BridgeForm, BridgeSpec, RoadSpan, TunnelForm, TunnelSpec, CLEARANCE, OVERPASS_DECK,
};
use roadgen_core::topology::{Direction, LaneEnd};
use roadgen_core::units::{PositiveWidth, SpeedLimit};

/// The geometry a road is designed to, by design speed.
#[derive(Debug, Clone, PartialEq)]
pub struct DesignStandard {
    pub design_speed: f64,
    /// The least radius of a bend, metres.
    pub min_radius: f64,
    /// The steepest grade, as a fraction.
    pub max_grade: f64,
    /// The radius of a vertical curve over a crest and through a sag, metres.
    pub crest_radius: f64,
    pub sag_radius: f64,
    pub lane_width: f64,
    /// The shoulder on the outside of the carriageway, and on the median side.
    pub outer_shoulder: f64,
    pub inner_shoulder: f64,
    pub median: f64,
    /// A deceleration lane's full-width length and an acceleration lane's, and
    /// the taper at the open end of each, metres.
    pub deceleration: f64,
    pub acceleration: f64,
    pub taper: f64,
}

impl DesignStandard {
    /// The defaults for a design speed of `kph`: the row of the table at or below
    /// it, of 120, 100, 80, 60, 50 and 40 km/h.
    pub fn for_speed(kph: f64) -> Result<DesignStandard, String> {
        if !(kph.is_finite() && kph >= 30.0) {
            return Err(format!(
                "a design speed of {kph} km/h is not one a road is designed to"
            ));
        }
        // The Road Structure Ordinance (道路構造令) for a type 1 road, its standard
        // values rather than its special ones: the radius (art. 15), the grade for
        // an ordinary road (art. 20), the vertical curves (art. 22), the lane
        // (art. 5, 3.5 m; 3.75 m only as a special value), the shoulders (art. 8)
        // and the median (art. 6), as MLIT's tables of them give them. The speed-
        // change lanes are not in the Ordinance but in its commentary, and are
        // round figures of the same order, not checked against it.
        //
        // speed, radius, grade, crest, sag, lane, outer, inner, median, decel, accel, taper
        const ROWS: [[f64; 12]; 6] = [
            [
                120.0, 710.0, 0.02, 11000.0, 4000.0, 3.5, 2.5, 1.25, 4.5, 110.0, 200.0, 70.0,
            ],
            [
                100.0, 460.0, 0.03, 6500.0, 3000.0, 3.5, 2.5, 1.25, 4.5, 90.0, 180.0, 60.0,
            ],
            [
                80.0, 280.0, 0.04, 3000.0, 2000.0, 3.5, 2.5, 0.75, 3.0, 80.0, 160.0, 50.0,
            ],
            [
                60.0, 150.0, 0.05, 1400.0, 1000.0, 3.5, 1.75, 0.75, 3.0, 70.0, 120.0, 45.0,
            ],
            [
                50.0, 100.0, 0.06, 800.0, 700.0, 3.25, 1.5, 0.75, 1.75, 60.0, 100.0, 40.0,
            ],
            [
                40.0, 60.0, 0.07, 450.0, 450.0, 3.0, 1.5, 0.75, 1.75, 50.0, 80.0, 35.0,
            ],
        ];
        let row = ROWS
            .iter()
            .find(|row| kph >= row[0])
            .unwrap_or(&ROWS[ROWS.len() - 1]);
        Ok(DesignStandard {
            design_speed: kph,
            min_radius: row[1],
            max_grade: row[2],
            crest_radius: row[3],
            sag_radius: row[4],
            lane_width: row[5],
            outer_shoulder: row[6],
            inner_shoulder: row[7],
            median: row[8],
            deceleration: row[9],
            acceleration: row[10],
            taper: row[11],
        })
    }
}

/// Which side of a corridor's centre line, looking along it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub fn parse(text: &str) -> Option<Side> {
        match text {
            "left" => Some(Side::Left),
            "right" => Some(Side::Right),
            _ => None,
        }
    }

    fn sign(self) -> f64 {
        match self {
            Side::Left => 1.0,
            Side::Right => -1.0,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Side::Left => "left",
            Side::Right => "right",
        }
    }
}

/// Whether a corridor's structures are chosen from the land, or only where said.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StructureMode {
    #[default]
    Auto,
    Explicit,
}

/// How traffic off a slip road is let onto the crossroad.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Control {
    #[default]
    Signal,
    Yield,
}

/// What an interchange is built as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterchangeKind {
    /// Four slip roads, a terminal on the crossroad each side.
    Diamond,
    /// Two: off before the crossroad and on after it, both on one carriageway's
    /// side, or the mirror — `before` says which way round.
    HalfDiamond { side: Side },
}

impl InterchangeKind {
    pub fn as_str(self) -> &'static str {
        match self {
            InterchangeKind::Diamond => "diamond",
            InterchangeKind::HalfDiamond { .. } => "half_diamond",
        }
    }
}

/// An interchange on a corridor, with the crossroad it is built on.
#[derive(Debug, Clone, PartialEq)]
pub struct InterchangeSite {
    pub name: String,
    pub kind: InterchangeKind,
    /// The centre-line station the crossroad crosses at.
    pub at: f64,
    /// The crossroad's line, at ground level; the plan lifts it over the corridor
    /// or drops it under.
    pub crossroad: Vec<Point3>,
    pub crossroad_lanes: Vec<LaneSpec>,
    /// Over the motorway, or under it.
    pub over: bool,
    pub control: Control,
    /// How far from the centre line each ramp terminal is, along the crossroad,
    /// metres.
    pub terminal: f64,
    /// How far before (and after) the crossroad a slip road leaves (and joins),
    /// metres along the centre line.
    pub reach: f64,
}

/// A lone slip road: off a carriageway to a point, or from a point onto one. Its
/// open end is the caller's to join to a street.
#[derive(Debug, Clone, PartialEq)]
pub struct RampSite {
    pub name: String,
    /// `true` for one that joins the carriageway.
    pub on: bool,
    pub side: Side,
    /// The centre-line station of the nose where it leaves or joins.
    pub at: f64,
    /// Its open end.
    pub end: Point3,
}

/// A motorway corridor.
#[derive(Debug, Clone, PartialEq)]
pub struct CorridorSpec {
    pub name: String,
    /// The centre line in plan, between the two carriageways; corners are rounded.
    pub centre: Vec<Point3>,
    pub standard: DesignStandard,
    /// Points of vertical intersection along the centre line, `(station, height)`;
    /// empty to take the heights of `centre`'s own points.
    pub profile: Vec<(f64, f64)>,
    /// Driving lanes in each direction.
    pub lanes: usize,
    pub structures: StructureMode,
    /// Centre-line stretches lifted onto a viaduct, and how far above the profile.
    pub elevated: Vec<(f64, f64)>,
    pub elevated_height: f64,
    /// Centre-line stretches dropped into a cut-and-cover tunnel, and how far under.
    pub sunken: Vec<(f64, f64)>,
    pub sunken_depth: f64,
    pub interchanges: Vec<InterchangeSite>,
    pub ramps: Vec<RampSite>,
    /// The length over which the motorway becomes an ordinary road at its far
    /// end, its speed stepped down, when it does.
    pub end_transition: Option<f64>,
}

impl CorridorSpec {
    pub fn new(
        name: impl Into<String>,
        centre: Vec<Point3>,
        design_speed: f64,
    ) -> Result<Self, String> {
        Ok(CorridorSpec {
            name: name.into(),
            centre,
            standard: DesignStandard::for_speed(design_speed)?,
            profile: Vec::new(),
            lanes: 2,
            structures: StructureMode::Auto,
            elevated: Vec::new(),
            elevated_height: 9.0,
            sunken: Vec::new(),
            sunken_depth: 10.0,
            interchanges: Vec::new(),
            ramps: Vec::new(),
            end_transition: None,
        })
    }

    pub fn key(&self) -> String {
        super::slug(&self.name)
    }

    pub fn check(&self) -> Result<(), String> {
        if self.key().is_empty() {
            return Err(format!(
                "a corridor named {:?} has nothing to name its roads after",
                self.name
            ));
        }
        if self.centre.len() < 2 {
            return Err("a corridor's centre line needs two points at least".into());
        }
        if self.lanes == 0 || self.lanes > 6 {
            return Err(format!(
                "a corridor of {} lanes each way is not one",
                self.lanes
            ));
        }
        Ok(())
    }
}

// -----------------------------------------------------------------------------------
// The centre line
// -----------------------------------------------------------------------------------

/// How far apart the plan's points are, metres.
const STEP: f64 = 5.0;
/// The length of a junction where a slip road leaves or joins, metres.
const GAP: f64 = 20.0;

/// The centre line, rounded and sampled: station, position, heading.
#[derive(Debug, Clone)]
pub struct Track {
    rows: Vec<(f64, f64, f64, f64)>,
}

impl Track {
    /// Rounds each corner of `points` with the largest arc up to one and a half
    /// times the least radius that fits, noting any tighter than the least.
    pub fn new(
        points: &[Point3],
        min_radius: f64,
        notes: &mut Vec<String>,
    ) -> Result<Track, String> {
        let plan: Vec<[f64; 2]> = points.iter().map(|p| [p.x, p.y]).collect();
        let mut pieces: Vec<Curve3> = Vec::new();
        let mut cursor = Point3::new(plan[0][0], plan[0][1], 0.0);
        let unit = |a: [f64; 2], b: [f64; 2]| {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let n = dx.hypot(dy);
            (n > 1e-9).then(|| [dx / n, dy / n])
        };
        for index in 1..plan.len() {
            let (a, b) = (plan[index - 1], plan[index]);
            let Some(d_in) = unit(a, b) else {
                return Err("a corridor's centre line repeats a point".into());
            };
            let end = if index + 1 < plan.len() {
                let c = plan[index + 1];
                let Some(d_out) = unit(b, c) else {
                    return Err("a corridor's centre line repeats a point".into());
                };
                let cross = d_in[0] * d_out[1] - d_in[1] * d_out[0];
                let dot = (d_in[0] * d_out[0] + d_in[1] * d_out[1]).clamp(-1.0, 1.0);
                let angle = cross.atan2(dot);
                if angle.abs() < 1e-4 {
                    None
                } else {
                    let room = (((b[0] - a[0]).hypot(b[1] - a[1])) / 2.0)
                        .min((c[0] - b[0]).hypot(c[1] - b[1]) / 2.0);
                    let wanted = min_radius * 1.5;
                    let half = (angle.abs() / 2.0).tan();
                    let radius = wanted.min(room / half);
                    if radius < min_radius - 1e-6 {
                        notes.push(format!(
                            "a bend of the centre line at ({:.0}, {:.0}) has a radius of {radius:.0} m, \
                             under the {min_radius:.0} m the design speed asks for",
                            b[0], b[1]
                        ));
                    }
                    let tangent = radius * half;
                    Some((tangent, radius, angle, d_out))
                }
            } else {
                None
            };
            let line_end = match end {
                Some((tangent, ..)) => [b[0] - d_in[0] * tangent, b[1] - d_in[1] * tangent],
                None => b,
            };
            let to = Point3::new(line_end[0], line_end[1], 0.0);
            if cursor.horizontal_distance_to(to) > 1e-6 {
                pieces.push(Curve3::line(cursor, to).map_err(|e| e.to_string())?);
            }
            cursor = to;
            if let Some((_, radius, angle, _)) = end {
                let heading = d_in[1].atan2(d_in[0]);
                let curvature = angle.signum() / radius;
                let arc = roadgen_core::geometry::Arc3::new(
                    cursor,
                    heading,
                    curvature,
                    radius * angle.abs(),
                    0.0,
                )
                .map_err(|e| e.to_string())?;
                let curve = Curve3::Arc(arc);
                cursor = curve.end_point();
                pieces.push(curve);
            }
        }
        let curve = Curve3::composite(pieces).map_err(|e| e.to_string())?;
        let length = curve.horizontal_length().map_err(|e| e.to_string())?;
        let count = (length / STEP).ceil().max(1.0) as usize;
        let config = roadgen_core::geometry::SamplingConfig::default();
        let mut rows = Vec::with_capacity(count + 1);
        for index in 0..=count {
            let s = length * index as f64 / count as f64;
            let sample = curve.sample_at(s, config).map_err(|e| e.to_string())?;
            let tangent = sample.tangent.get();
            rows.push((
                s,
                sample.point.x,
                sample.point.y,
                tangent.y.atan2(tangent.x),
            ));
        }
        Ok(Track { rows })
    }

    pub fn length(&self) -> f64 {
        self.rows.last().map_or(0.0, |row| row.0)
    }

    /// Position and heading at station `s`.
    pub fn at(&self, s: f64) -> (f64, f64, f64) {
        let s = s.clamp(0.0, self.length());
        let index = self
            .rows
            .partition_point(|row| row.0 < s)
            .clamp(1, self.rows.len() - 1);
        let (a, b) = (self.rows[index - 1], self.rows[index]);
        let t = if b.0 > a.0 {
            (s - a.0) / (b.0 - a.0)
        } else {
            0.0
        };
        let mut dh = b.3 - a.3;
        while dh > std::f64::consts::PI {
            dh -= std::f64::consts::TAU;
        }
        while dh < -std::f64::consts::PI {
            dh += std::f64::consts::TAU;
        }
        (a.1 + (b.1 - a.1) * t, a.2 + (b.2 - a.2) * t, a.3 + dh * t)
    }

    /// The point `lateral` metres to the left of the centre line at `s`.
    pub fn offset(&self, s: f64, lateral: f64) -> [f64; 2] {
        let (x, y, h) = self.at(s);
        [x - h.sin() * lateral, y + h.cos() * lateral]
    }

    /// The station nearest a point, and the point's offset to the left there.
    pub fn locate(&self, p: [f64; 2]) -> (f64, f64) {
        let mut best = (0.0, f64::INFINITY, 0.0);
        for pair in self.rows.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let (dx, dy) = (b.1 - a.1, b.2 - a.2);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((p[0] - a.1) * dx + (p[1] - a.2) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (x, y) = (a.1 + dx * t, a.2 + dy * t);
            let d = (p[0] - x).hypot(p[1] - y);
            if d < best.1 {
                let len = len2.sqrt().max(1e-9);
                let lateral = (dx * (p[1] - a.2) - dy * (p[0] - a.1)) / len;
                best = (a.0 + (b.0 - a.0) * t, d, lateral);
            }
        }
        (best.0, best.2)
    }
}

/// The point `s` metres along a polyline whose vertices are at `stations`.
fn along(points: &[Point3], stations: &[f64], s: f64) -> Point3 {
    let index = stations
        .partition_point(|x| *x < s)
        .clamp(1, stations.len() - 1);
    let t = (s - stations[index - 1]) / (stations[index] - stations[index - 1]).max(1e-9);
    points[index - 1].lerp(points[index], t)
}

/// A smooth bump: zero before `from`, `height` from `from + ramp` to `to − ramp`,
/// zero after `to`, with cosine transitions.
fn bump(s: f64, from: f64, to: f64, ramp: f64, height: f64) -> f64 {
    if s <= from || s >= to {
        return 0.0;
    }
    let rise = |x: f64| 0.5 - 0.5 * (std::f64::consts::PI * (x / ramp).clamp(0.0, 1.0)).cos();
    height * rise(s - from).min(rise(to - s))
}

// -----------------------------------------------------------------------------------
// The plan
// -----------------------------------------------------------------------------------

/// One road of a plan.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadPlan {
    pub name: String,
    pub points: Vec<Point3>,
    pub lanes: Vec<LaneSpec>,
    /// Further cross-sections: `(station along the road, lanes)`.
    pub cross_sections: Vec<(f64, Vec<LaneSpec>)>,
    pub road_type: RoadType,
    pub form: FormOfWay,
    pub speed_limit: Option<f64>,
    /// Whether the road is the corridor's own — shaped into the land by it — or
    /// one it crosses.
    pub corridor: bool,
}

/// A movement a plan asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionPlan {
    /// One lane to one lane: `(road, global lane index)`.
    Lanes {
        from: (String, usize),
        to: (String, usize),
        junction: String,
    },
    /// Every lane that pairs across two road ends, as `connect` does.
    Ends {
        a: String,
        a_end: roadgen_core::topology::RoadEnd,
        b: String,
        b_end: roadgen_core::topology::RoadEnd,
        junction: String,
    },
}

/// A signal or a give-way at a lane's end.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlPlan {
    pub road: String,
    pub lane: usize,
    pub end: LaneEnd,
    pub control: Control,
    /// The lanes it gives way to, for a give-way.
    pub priority: Vec<(String, usize)>,
}

/// A structure a plan asks for, by road name and station.
#[derive(Debug, Clone, PartialEq)]
pub struct StructurePlan {
    pub spans: Vec<(String, f64, f64)>,
    pub bridge: Option<BridgeForm>,
    pub tunnel: Option<TunnelForm>,
}

/// An interchange a plan gathers.
#[derive(Debug, Clone, PartialEq)]
pub struct InterchangePlan {
    pub name: String,
    pub kind: String,
    pub junctions: Vec<String>,
    pub roads: Vec<String>,
}

/// Every road, movement and structure a corridor asks for.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CorridorPlan {
    pub corridor: String,
    pub roads: Vec<RoadPlan>,
    pub junctions: Vec<String>,
    pub connections: Vec<ConnectionPlan>,
    pub controls: Vec<ControlPlan>,
    pub structures: Vec<StructurePlan>,
    pub interchanges: Vec<InterchangePlan>,
    /// Roads the land is fitted to rather than read from, besides the corridor's
    /// own: a crossroad lifted onto an embankment to clear the motorway.
    pub fitted: Vec<String>,
    /// The ground's height where it is known: under a crossroad, as drawn.
    pub anchors: Vec<Point3>,
    /// Where the plan departs from the design standard.
    pub notes: Vec<String>,
}

fn width(metres: f64) -> PositiveWidth {
    PositiveWidth::new(metres.max(0.01)).expect("a positive width")
}

fn marking(marking: RoadMarking) -> BoundaryMarking {
    BoundaryMarking::new(marking, MarkingColor::White)
}

/// A one-way cross-section, outwards from the reference line: the median-side
/// shoulder, the driving lanes, an optional speed-change lane, the outer shoulder.
fn one_way(
    standard: &DesignStandard,
    lanes: usize,
    extra: Option<WidthProfile>,
    handedness: TrafficHandedness,
) -> Vec<LaneSpec> {
    // The lanes lie on the side forward traffic keeps to: the right under
    // right-hand traffic, where a lane's left boundary is its inner one.
    let inner_is_left = handedness == TrafficHandedness::RightHand;
    let edges = |inner: RoadMarking, outer: RoadMarking| {
        if inner_is_left {
            (marking(inner), marking(outer))
        } else {
            (marking(outer), marking(inner))
        }
    };
    let mut out = Vec::new();
    let (l, r) = edges(RoadMarking::None, RoadMarking::None);
    out.push(
        LaneSpec::new(width(standard.inner_shoulder), Direction::Forward)
            .with_type(LaneType::Shoulder)
            .with_markings(l, r),
    );
    let count = lanes + usize::from(extra.is_some());
    for index in 0..count {
        let inner = if index == 0 {
            RoadMarking::Solid
        } else {
            RoadMarking::Broken
        };
        let outer = if index + 1 == count {
            RoadMarking::Solid
        } else {
            RoadMarking::Broken
        };
        let (l, r) = edges(inner, outer);
        let mut lane =
            LaneSpec::new(width(standard.lane_width), Direction::Forward).with_markings(l, r);
        if index == lanes {
            if let Some(profile) = &extra {
                lane = lane.with_width_profile(profile.clone());
            }
        }
        out.push(lane);
    }
    let (l, r) = edges(RoadMarking::None, RoadMarking::None);
    out.push(
        LaneSpec::new(width(standard.outer_shoulder), Direction::Forward)
            .with_type(LaneType::Shoulder)
            .with_markings(l, r),
    );
    out
}

/// A carriageway's cross-section width, shoulders included.
fn carriageway_width(standard: &DesignStandard, lanes: usize) -> f64 {
    standard.inner_shoulder + standard.lane_width * lanes as f64 + standard.outer_shoulder
}

/// A cubic in plan from `a` heading `ha` to `b` heading `hb`, sampled every
/// [`STEP`], with heights eased from `za` to `zb`.
fn link(a: [f64; 2], ha: f64, za: f64, b: [f64; 2], hb: f64, zb: f64) -> Vec<Point3> {
    let chord = (b[0] - a[0]).hypot(b[1] - a[1]);
    let handle = chord * 0.4;
    let p = [
        a,
        [a[0] + ha.cos() * handle, a[1] + ha.sin() * handle],
        [b[0] - hb.cos() * handle, b[1] - hb.sin() * handle],
        b,
    ];
    let count = (chord / STEP).ceil().max(2.0) as usize;
    (0..=count)
        .map(|index| {
            let t = index as f64 / count as f64;
            let u = 1.0 - t;
            let w = [u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t];
            let x = w.iter().zip(&p).map(|(w, p)| w * p[0]).sum();
            let y = w.iter().zip(&p).map(|(w, p)| w * p[1]).sum();
            let ease = t * t * (3.0 - 2.0 * t);
            Point3::new(x, y, za + (zb - za) * ease)
        })
        .collect()
}

/// Plan length of a polyline.
fn length_of(points: &[Point3]) -> f64 {
    points
        .windows(2)
        .map(|pair| pair[0].horizontal_distance_to(pair[1]))
        .sum()
}

/// The steepest grade along a polyline.
fn steepest(points: &[Point3]) -> f64 {
    points
        .windows(2)
        .filter_map(|pair| {
            let run = pair[0].horizontal_distance_to(pair[1]);
            (run > 1e-6).then(|| ((pair[1].z - pair[0].z) / run).abs())
        })
        .fold(0.0, f64::max)
}

/// A stretch of one carriageway between two junctions, with the speed-change lane
/// it opens with, the one it ends with, or both.
#[derive(Debug, Clone)]
struct Piece {
    /// Travel stations along the carriageway (centre-line stations, measured in the
    /// direction traffic goes).
    from: f64,
    to: f64,
    accelerates: bool,
    decelerates: bool,
}

/// Where a slip road leaves or joins a carriageway.
#[derive(Debug, Clone)]
struct Nose {
    /// Travel station of the nose.
    at: f64,
    on: bool,
}

/// One carriageway of the corridor.
struct Carriageway {
    side: Side,
    /// Whether traffic goes the centre line's way.
    along: bool,
    /// The reference line's offset to the left of the centre line.
    offset: f64,
    noses: Vec<Nose>,
}

impl Carriageway {
    /// The way traffic heads at centre-line station `s`.
    fn heading(&self, track: &Track, s: f64) -> f64 {
        let (_, _, h) = track.at(s);
        if self.along {
            h
        } else {
            h + std::f64::consts::PI
        }
    }
}

impl CorridorSpec {
    /// The height of the centre line at `s`: the profile, lifted over its elevated
    /// stretches and dropped through its sunken ones.
    fn height_at(&self, profile: &VerticalProfile, s: f64) -> f64 {
        let ramp_for = |height: f64| {
            (height * std::f64::consts::PI / (2.0 * 0.8 * self.standard.max_grade.max(0.02)))
                .max(120.0)
        };
        let mut z = profile.height(s);
        for &(from, to) in &self.elevated {
            z += bump(
                s,
                from,
                to,
                ramp_for(self.elevated_height),
                self.elevated_height,
            );
        }
        for &(from, to) in &self.sunken {
            z -= bump(s, from, to, ramp_for(self.sunken_depth), self.sunken_depth);
        }
        z
    }

    /// Works out the roads, junctions, movements and structures the corridor asks
    /// for. See the module documentation.
    pub fn plan(&self, handedness: TrafficHandedness) -> Result<CorridorPlan, String> {
        self.check()?;
        let key = self.key();
        let standard = &self.standard;
        let mut plan = CorridorPlan {
            corridor: self.name.clone(),
            ..CorridorPlan::default()
        };
        let track = Track::new(&self.centre, standard.min_radius, &mut plan.notes)?;
        let length = track.length();

        // The vertical alignment: the profile given, or the centre points' heights
        // at their stations.
        let pvis: Vec<(f64, f64)> = if self.profile.is_empty() {
            self.centre
                .iter()
                .map(|p| (track.locate([p.x, p.y]).0, p.z))
                .collect()
        } else {
            self.profile.clone()
        };
        let mut pvis = pvis;
        pvis.sort_by(|a, b| a.0.total_cmp(&b.0));
        pvis.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-6);
        if pvis.len() < 2 {
            pvis = vec![
                (0.0, pvis.first().map_or(0.0, |p| p.1)),
                (length, pvis.first().map_or(0.0, |p| p.1)),
            ];
        }
        let radii: Vec<f64> = (0..pvis.len())
            .map(|index| {
                if index == 0 || index + 1 == pvis.len() {
                    return 0.0;
                }
                let before =
                    (pvis[index].1 - pvis[index - 1].1) / (pvis[index].0 - pvis[index - 1].0);
                let after =
                    (pvis[index + 1].1 - pvis[index].1) / (pvis[index + 1].0 - pvis[index].0);
                if after < before {
                    standard.crest_radius
                } else {
                    standard.sag_radius
                }
            })
            .collect();
        let profile = VerticalProfile::with_radii(pvis, radii).map_err(|e| e.to_string())?;
        if profile.steepest() > standard.max_grade + 1e-9 {
            plan.notes.push(format!(
                "the profile climbs {:.1} %, steeper than the {:.1} % the design speed allows",
                profile.steepest() * 100.0,
                standard.max_grade * 100.0
            ));
        }
        let z = |s: f64| self.height_at(&profile, s);

        let cw_width = carriageway_width(standard, self.lanes);
        // What a slip road is designed to.
        let ramp_standard = DesignStandard::for_speed(40.0)?;
        let forward_right = handedness == TrafficHandedness::RightHand;
        let mut carriageways: Vec<Carriageway> = [Side::Left, Side::Right]
            .into_iter()
            .map(|side| Carriageway {
                side,
                along: (side == Side::Right) == forward_right,
                offset: side.sign() * standard.median / 2.0,
                noses: Vec::new(),
            })
            .collect();
        // The side a carriageway's lanes lie on, as a sign to the left of travel.
        let outward = if forward_right { -1.0 } else { 1.0 };
        let travel = |c: &Carriageway, s: f64| if c.along { s } else { length - s };
        let centre_of = |c: &Carriageway, t: f64| if c.along { t } else { length - t };

        // Interchanges and ramps: where each slip road leaves or joins.
        struct Access {
            ramp: String,
            carriageway: usize,
            on: bool,
            nose: f64,
            // Its open end, and the height and heading there.
            end: [f64; 2],
            end_z: f64,
            end_heading: f64,
        }
        let mut accesses: Vec<Access> = Vec::new();
        for site in &self.interchanges {
            let ic = super::slug(&site.name);
            if ic.is_empty() {
                return Err(format!(
                    "an interchange named {:?} has nothing to name it after",
                    site.name
                ));
            }
            if site.crossroad.len() < 2 {
                return Err(format!(
                    "{}: a crossroad needs two points at least",
                    site.name
                ));
            }
            // The crossroad: where it crosses the centre line, and its profile up
            // over the motorway or down under it.
            let mut stations = vec![0.0];
            for pair in site.crossroad.windows(2) {
                stations.push(stations.last().unwrap() + pair[0].horizontal_distance_to(pair[1]));
            }
            let cross_length = *stations.last().unwrap();
            // The crossroad as drawn at station `s`: where, and the ground's height.
            let drawn = |s: f64| along(&site.crossroad, &stations, s);
            let point_at = |s: f64| {
                let p = drawn(s);
                [p.x, p.y]
            };
            let ground_at = |s: f64| drawn(s).z;
            // Its offset from the centre line every metre, once, to find where it is
            // any given distance to one side.
            let count = cross_length.ceil().max(1.0) as usize;
            let offsets: Vec<(f64, f64)> = (0..=count)
                .map(|step| {
                    let s = cross_length * step as f64 / count as f64;
                    (s, track.locate(point_at(s)).1)
                })
                .collect();
            let at_lateral = |target: f64| -> Option<f64> {
                offsets.windows(2).find_map(|pair| {
                    let ((ps, pl), (s, l)) = (pair[0], pair[1]);
                    let (pl, l) = (pl - target, l - target);
                    (pl.signum() != l.signum()).then(|| ps + (s - ps) * pl / (pl - l))
                })
            };
            let c_station = at_lateral(0.0).ok_or_else(|| {
                format!("{}: the crossroad does not cross the corridor", site.name)
            })?;
            let (centre_station, _) = track.locate(point_at(c_station));
            let motorway_z = z(centre_station);
            let half_corridor = standard.median / 2.0 + cw_width + 3.0;
            // Over the motorway: an overpass's deck and the clearance under it;
            // under it: the clearance under the motorway's own overpass.
            let target = if site.over {
                motorway_z + CLEARANCE + OVERPASS_DECK
            } else {
                motorway_z - OVERPASS_DECK - 0.5 - CLEARANCE
            };
            // The overpass reaches out far enough for the land to come down from
            // its abutments to the motorway at a fill's slope.
            let half_corridor =
                half_corridor + super::highway::abutment_reach((target - motorway_z).abs());
            let flat = half_corridor + 5.0;
            // Level over the motorway, down at an approach grade each side, and on
            // the ground it was drawn on beyond that: an overpass's approach is an
            // embankment only as long as it has to be.
            let approach_grade = 0.05;
            let mut cross_pvis = vec![(0.0, ground_at(0.0))];
            let before = c_station - flat;
            let after = c_station + flat;
            let foot_before = before - (target - ground_at(before.max(0.0))).abs() / approach_grade;
            let foot_after =
                after + (target - ground_at(after.min(cross_length))).abs() / approach_grade;
            if foot_before > 1.0 {
                cross_pvis.push((foot_before, ground_at(foot_before)));
            }
            if before > 1.0 {
                cross_pvis.push((before, target));
            }
            if after < cross_length - 1.0 {
                cross_pvis.push((after, target));
            }
            if foot_after < cross_length - 1.0 {
                cross_pvis.push((foot_after, ground_at(foot_after)));
            }
            cross_pvis.push((cross_length, ground_at(cross_length)));
            let cross_profile =
                VerticalProfile::new(cross_pvis, 1000.0).map_err(|e| e.to_string())?;
            if cross_profile.steepest() > 0.08 {
                plan.notes.push(format!(
                    "{}: the crossroad climbs {:.1} % to clear the motorway; give it more room",
                    site.name,
                    cross_profile.steepest() * 100.0
                ));
            }
            let crossroad_point = |s: f64| {
                let p = point_at(s);
                Point3::new(p[0], p[1], cross_profile.height(s))
            };

            // The terminals: one each side, where the ramps meet the crossroad.
            let sides: Vec<Side> = match site.kind {
                InterchangeKind::Diamond => vec![Side::Left, Side::Right],
                InterchangeKind::HalfDiamond { side } => vec![side],
            };
            let mut cuts: Vec<(f64, Side)> = Vec::new();
            for &side in &sides {
                let s = at_lateral(side.sign() * site.terminal).ok_or_else(|| {
                    format!(
                        "{}: the crossroad does not reach {:.0} m to the {} of the corridor",
                        site.name,
                        site.terminal,
                        side.as_str()
                    )
                })?;
                cuts.push((s, side));
            }
            cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
            // The crossroad in pieces between the terminals.
            let mut bounds = vec![0.0];
            for (s, _) in &cuts {
                bounds.push(s - GAP / 2.0);
                bounds.push(s + GAP / 2.0);
            }
            bounds.push(cross_length);
            let mut cross_names = Vec::new();
            for (index, pair) in bounds.chunks(2).enumerate() {
                let (from, to) = (pair[0], pair[1]);
                if to - from < 5.0 {
                    return Err(format!(
                        "{}: the crossroad is too short for its terminals",
                        site.name
                    ));
                }
                let count = ((to - from) / STEP).ceil().max(1.0) as usize;
                let points: Vec<Point3> = (0..=count)
                    .map(|k| crossroad_point(from + (to - from) * k as f64 / count as f64))
                    .collect();
                let name = format!("{ic}_cross_{index}");
                // The piece over (or under) the corridor is carried on a bridge.
                if site.over && from < c_station && to > c_station {
                    plan.structures.push(StructurePlan {
                        spans: vec![(
                            name.clone(),
                            (c_station - from - half_corridor).max(1.0),
                            (c_station - from + half_corridor).min(to - from - 1.0),
                        )],
                        bridge: Some(BridgeForm::Overpass),
                        tunnel: None,
                    });
                }
                plan.fitted.push(name.clone());
                plan.roads.push(RoadPlan {
                    name: name.clone(),
                    points,
                    lanes: site.crossroad_lanes.clone(),
                    cross_sections: Vec::new(),
                    road_type: RoadType::Town,
                    form: FormOfWay::Unspecified,
                    speed_limit: None,
                    corridor: false,
                });
                cross_names.push(name);
            }
            // The ground the crossroad was drawn on, away from the motorway: what
            // its embankment stands on.
            let mut s = 0.0;
            while s <= cross_length {
                if (s - c_station).abs() > half_corridor {
                    let p = point_at(s);
                    plan.anchors.push(Point3::new(p[0], p[1], ground_at(s)));
                }
                s += 10.0;
            }
            let mut ic_roads = cross_names.clone();
            let mut ic_junctions = Vec::new();

            // The ramps at each terminal, and the movements there.
            for (index, (s, side)) in cuts.iter().enumerate() {
                let junction = format!("{ic}_{}", side.as_str());
                plan.junctions.push(junction.clone());
                ic_junctions.push(junction.clone());
                let terminal = crossroad_point(*s);
                let (before, after) = (&cross_names[index], &cross_names[index + 1]);
                // The carriageway on this side.
                let c = carriageways
                    .iter()
                    .position(|c| c.side == *side)
                    .expect("both sides");
                let carriageway = &carriageways[c];
                let heading_travel = carriageway.heading(&track, site.at);
                let back = |d: f64| {
                    [
                        terminal.x - heading_travel.cos() * d,
                        terminal.y - heading_travel.sin() * d,
                    ]
                };
                let ahead = |d: f64| {
                    [
                        terminal.x + heading_travel.cos() * d,
                        terminal.y + heading_travel.sin() * d,
                    ]
                };
                // Lanes lie on the outward side of a ramp's reference line, so the
                // line is shifted back by half the ramp's width to centre the ramp on
                // the terminal.
                let ramp_width = carriageway_width(&ramp_standard, 1);
                let normal = heading_travel + std::f64::consts::FRAC_PI_2;
                let shift = |p: [f64; 2]| {
                    [
                        p[0] - normal.cos() * outward * ramp_width / 2.0,
                        p[1] - normal.sin() * outward * ramp_width / 2.0,
                    ]
                };
                let setback = 14.0;
                let off_name = format!("{ic}_{}_off", side.as_str());
                let on_name = format!("{ic}_{}_on", side.as_str());
                let travel_at = travel(carriageway, site.at);
                accesses.push(Access {
                    ramp: off_name.clone(),
                    carriageway: c,
                    on: false,
                    nose: travel_at - site.reach,
                    end: shift(back(setback)),
                    end_z: terminal.z,
                    end_heading: heading_travel,
                });
                accesses.push(Access {
                    ramp: on_name.clone(),
                    carriageway: c,
                    on: true,
                    nose: travel_at + site.reach,
                    end: shift(ahead(setback)),
                    end_z: terminal.z,
                    end_heading: heading_travel,
                });
                ic_roads.push(off_name.clone());
                ic_roads.push(on_name.clone());

                // The crossroad's lanes by direction: the innermost driving lane
                // each way takes the turns to and from the ramps.
                let first = |direction: Direction| {
                    site.crossroad_lanes.iter().position(|lane| {
                        lane.direction == direction && lane.lane_type == LaneType::Driving
                    })
                };
                let (Some(forward), Some(backward)) =
                    (first(Direction::Forward), first(Direction::Backward))
                else {
                    return Err(format!(
                        "{}: the crossroad needs a driving lane each way",
                        site.name
                    ));
                };
                use roadgen_core::topology::RoadEnd;
                plan.connections.push(ConnectionPlan::Ends {
                    a: before.clone(),
                    a_end: RoadEnd::End,
                    b: after.clone(),
                    b_end: RoadEnd::Start,
                    junction: junction.clone(),
                });
                let ramp_lane = 1;
                for (to, lane) in [(before, backward), (after, forward)] {
                    plan.connections.push(ConnectionPlan::Lanes {
                        from: (off_name.clone(), ramp_lane),
                        to: (to.clone(), lane),
                        junction: junction.clone(),
                    });
                }
                for (from, lane) in [(before, forward), (after, backward)] {
                    plan.connections.push(ConnectionPlan::Lanes {
                        from: (from.clone(), lane),
                        to: (on_name.clone(), ramp_lane),
                        junction: junction.clone(),
                    });
                }
                // Traffic off the ramp is controlled; the crossroad is signalled too
                // where the ramp is.
                plan.controls.push(ControlPlan {
                    road: off_name.clone(),
                    lane: ramp_lane,
                    end: LaneEnd::End,
                    control: site.control,
                    priority: vec![(before.clone(), forward), (after.clone(), backward)],
                });
                if site.control == Control::Signal {
                    plan.controls.push(ControlPlan {
                        road: before.clone(),
                        lane: forward,
                        end: LaneEnd::End,
                        control: Control::Signal,
                        priority: Vec::new(),
                    });
                    plan.controls.push(ControlPlan {
                        road: after.clone(),
                        lane: backward,
                        end: LaneEnd::Start,
                        control: Control::Signal,
                        priority: Vec::new(),
                    });
                }
            }
            plan.interchanges.push(InterchangePlan {
                name: site.name.clone(),
                kind: site.kind.as_str().into(),
                junctions: ic_junctions,
                roads: ic_roads,
            });
        }
        for ramp in &self.ramps {
            let name = super::slug(&ramp.name);
            if name.is_empty() {
                return Err(format!(
                    "a ramp named {:?} has nothing to name it after",
                    ramp.name
                ));
            }
            let c = carriageways
                .iter()
                .position(|c| c.side == ramp.side)
                .expect("both sides");
            let heading = carriageways[c].heading(&track, ramp.at);
            accesses.push(Access {
                ramp: name,
                carriageway: c,
                on: ramp.on,
                nose: travel(&carriageways[c], ramp.at),
                end: [ramp.end.x, ramp.end.y],
                end_z: ramp.end.z,
                end_heading: heading,
            });
        }
        for access in &accesses {
            carriageways[access.carriageway].noses.push(Nose {
                at: access.nose,
                on: access.on,
            });
        }

        // Each carriageway in pieces between its noses.
        for c in &mut carriageways {
            c.noses.sort_by(|a, b| a.at.total_cmp(&b.at));
        }
        let speed = standard.design_speed;
        let mut piece_names: Vec<Vec<(String, Piece)>> = Vec::new();
        for c in &carriageways {
            let mut pieces = Vec::new();
            let mut from = 0.0;
            let mut accelerates = false;
            for nose in &c.noses {
                if nose.at - GAP / 2.0 - from < 30.0 {
                    return Err(format!(
                        "{}: the slip road at {:.0} m on the {} carriageway is too close to another, \
                         or to the corridor's start",
                        self.name,
                        centre_of(c, nose.at),
                        c.side.as_str()
                    ));
                }
                pieces.push(Piece {
                    from,
                    to: nose.at - GAP / 2.0,
                    accelerates,
                    decelerates: !nose.on,
                });
                from = nose.at + GAP / 2.0;
                accelerates = nose.on;
            }
            if length - from < 30.0 {
                return Err(format!(
                    "{}: a slip road is too close to the corridor's end",
                    self.name
                ));
            }
            pieces.push(Piece {
                from,
                to: length,
                accelerates,
                decelerates: false,
            });
            let named = pieces
                .into_iter()
                .enumerate()
                .map(|(index, piece)| (format!("{key}_{}_{index}", c.side.as_str()), piece))
                .collect();
            piece_names.push(named);
        }

        // The pieces as roads.
        let point_on = |c: &Carriageway, t: f64, extra: f64| -> Point3 {
            let s = centre_of(c, t);
            // To the left of travel is to the left of the centre line along it,
            // and to its right against it.
            let travel_left = if c.along { 1.0 } else { -1.0 };
            let p = track.offset(s, c.offset + travel_left * extra);
            Point3::new(p[0], p[1], z(s))
        };
        let heading_on = |c: &Carriageway, t: f64| c.heading(&track, centre_of(c, t));
        for (c, pieces) in carriageways.iter().zip(&piece_names) {
            for (name, piece) in pieces {
                let count = ((piece.to - piece.from) / STEP).ceil().max(1.0) as usize;
                let points: Vec<Point3> = (0..=count)
                    .map(|k| {
                        point_on(
                            c,
                            piece.from + (piece.to - piece.from) * k as f64 / count as f64,
                            0.0,
                        )
                    })
                    .collect();
                let road_length = length_of(&points);
                // Speed-change lanes: full width over their length and tapering at
                // the open end, in a cross-section of their own.
                let accel_end = if piece.accelerates {
                    (standard.acceleration + standard.taper).min(road_length - 1.0)
                } else {
                    0.0
                };
                let decel_start = if piece.decelerates {
                    (road_length - standard.deceleration - standard.taper).max(1.0)
                } else {
                    road_length
                };
                let lane_width = standard.lane_width;
                let accel = || {
                    WidthProfile::new(
                        [
                            (0.0, width(lane_width)),
                            ((accel_end - standard.taper).max(0.5), width(lane_width)),
                            (accel_end, width(0.5)),
                        ],
                        Taper::Smooth,
                    )
                    .ok()
                };
                let decel = || {
                    WidthProfile::new(
                        [
                            (decel_start, width(0.5)),
                            (
                                (decel_start + standard.taper).min(road_length),
                                width(lane_width),
                            ),
                        ],
                        Taper::Smooth,
                    )
                    .ok()
                };
                let plain = one_way(standard, self.lanes, None, handedness);
                let (lanes, cross_sections) =
                    if piece.accelerates && piece.decelerates && accel_end >= decel_start {
                        // The two meet: one auxiliary lane all the way.
                        (
                            one_way(
                                standard,
                                self.lanes,
                                Some(WidthProfile::constant(width(lane_width))),
                                handedness,
                            ),
                            Vec::new(),
                        )
                    } else {
                        let mut sections = Vec::new();
                        let first = if piece.accelerates {
                            sections.push((accel_end, plain.clone()));
                            one_way(standard, self.lanes, accel(), handedness)
                        } else {
                            plain.clone()
                        };
                        if piece.decelerates {
                            sections.push((
                                decel_start,
                                one_way(standard, self.lanes, decel(), handedness),
                            ));
                        }
                        (first, sections)
                    };
                plan.roads.push(RoadPlan {
                    name: name.clone(),
                    points,
                    lanes,
                    cross_sections,
                    road_type: RoadType::Motorway,
                    form: FormOfWay::Motorway,
                    speed_limit: Some(speed),
                    corridor: true,
                });
            }
        }

        // The slip roads, and the junction where each leaves or joins.
        for access in &accesses {
            let c = &carriageways[access.carriageway];
            let pieces = &piece_names[access.carriageway];
            let index = pieces
                .iter()
                .position(|(_, piece)| (piece.to - (access.nose - GAP / 2.0)).abs() < 1e-6)
                .expect("a piece ends at every nose");
            let (before, after) = (&pieces[index].0, &pieces[index + 1].0);
            let lanes_before = plan
                .roads
                .iter()
                .find(|r| &r.name == before)
                .expect("planned");
            // Where the lanes of the piece's last cross-section begin in its list.
            let offset_last = match lanes_before.cross_sections.split_last() {
                None => 0,
                Some((_, earlier)) => {
                    lanes_before.lanes.len() + earlier.iter().map(|(_, l)| l.len()).sum::<usize>()
                }
            };
            // A slip road runs beside the plain carriageway, half a metre clear of it.
            let edge = cw_width + 0.5;
            let junction = format!("{}_{}", key, access.ramp);
            plan.junctions.push(junction.clone());
            let (start_t, end_t) = (access.nose - GAP / 2.0, access.nose + GAP / 2.0);
            let points = if access.on {
                let end = point_on(c, start_t, outward * edge);
                let heading = heading_on(c, start_t);
                link(
                    access.end,
                    access.end_heading,
                    access.end_z,
                    [end.x, end.y],
                    heading,
                    end.z,
                )
            } else {
                let start = point_on(c, end_t, outward * edge);
                let heading = heading_on(c, end_t);
                link(
                    [start.x, start.y],
                    heading,
                    start.z,
                    access.end,
                    access.end_heading,
                    access.end_z,
                )
            };
            if steepest(&points) > ramp_standard.max_grade + 1e-9 {
                plan.notes.push(format!(
                    "{}: the slip road climbs {:.1} %, steeper than {:.0} %",
                    access.ramp,
                    steepest(&points) * 100.0,
                    ramp_standard.max_grade * 100.0
                ));
            }
            plan.roads.push(RoadPlan {
                name: access.ramp.clone(),
                points,
                lanes: one_way(&ramp_standard, 1, None, handedness),
                cross_sections: Vec::new(),
                road_type: RoadType::Motorway,
                form: FormOfWay::SlipRoad,
                speed_limit: Some(60.0_f64.min(speed)),
                corridor: true,
            });
            // Through: driving lane k to driving lane k. Off: the deceleration lane
            // onto the slip road; on: the slip road onto the acceleration lane.
            for k in 1..=self.lanes {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (before.clone(), offset_last + k),
                    to: (after.clone(), k),
                    junction: junction.clone(),
                });
            }
            if access.on {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (access.ramp.clone(), 1),
                    to: (after.clone(), self.lanes + 1),
                    junction: junction.clone(),
                });
            } else {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (before.clone(), offset_last + self.lanes + 1),
                    to: (access.ramp.clone(), 1),
                    junction: junction.clone(),
                });
            }
            if let Some(ic) = plan
                .interchanges
                .iter_mut()
                .find(|ic| ic.roads.contains(&access.ramp))
            {
                ic.junctions.push(junction);
            }
        }

        // The carriageways' own structures: the elevated and sunken stretches.
        for (stretches, bridge) in [(&self.elevated, true), (&self.sunken, false)] {
            for &(from, to) in stretches.iter() {
                // Only where the lift is most of the way up: the ends of the ramp up
                // to it are earthworks.
                let margin = if bridge {
                    self.elevated_height
                } else {
                    self.sunken_depth
                } / (0.8 * standard.max_grade.max(0.02))
                    * 0.35;
                let (from, to) = (from + margin, to - margin);
                if to - from < 20.0 {
                    continue;
                }
                for (c, pieces) in carriageways.iter().zip(&piece_names) {
                    let (t0, t1) = {
                        let (a, b) = (travel(c, from), travel(c, to));
                        (a.min(b), a.max(b))
                    };
                    for (name, piece) in pieces {
                        let (a, b) = (t0.max(piece.from), t1.min(piece.to));
                        if b - a < 10.0 {
                            continue;
                        }
                        let road = plan
                            .roads
                            .iter()
                            .find(|r| &r.name == name)
                            .expect("planned");
                        // Travel stations to stations along the road's own points.
                        let scale = length_of(&road.points) / (piece.to - piece.from).max(1e-9);
                        plan.structures.push(StructurePlan {
                            spans: vec![(
                                name.clone(),
                                (a - piece.from) * scale,
                                (b - piece.from) * scale,
                            )],
                            bridge: bridge.then_some(BridgeForm::Viaduct),
                            tunnel: (!bridge).then_some(TunnelForm::CutAndCover),
                        });
                    }
                }
            }
        }

        // The far end becoming an ordinary road: lower limits in steps, and the
        // last stretch no motorway.
        if let Some(transition) = self.end_transition {
            for (c, pieces) in carriageways.iter().zip(&piece_names) {
                // The end the corridor's far end is at, in travel terms.
                let at_end = if c.along {
                    pieces.last()
                } else {
                    pieces.first()
                };
                let Some((name, _)) = at_end else { continue };
                if let Some(road) = plan.roads.iter_mut().find(|r| &r.name == name) {
                    let total = length_of(&road.points);
                    if total < transition {
                        plan.notes.push(format!(
                            "{name}: shorter than the {transition:.0} m transition at the corridor's end"
                        ));
                    }
                    road.form = FormOfWay::MultipleCarriageway;
                    road.road_type = RoadType::Rural;
                    road.speed_limit = Some(60.0_f64.min(speed));
                }
            }
        }
        Ok(plan)
    }
}

impl CorridorPlan {
    /// Puts the plan on the map: its roads, junctions, movements, signals,
    /// structures and interchanges, and records which roads are the corridor's.
    pub fn apply(
        &self,
        builder: &mut MapBuilder,
        places: &mut super::Places,
    ) -> Result<Vec<RoadId>, String> {
        let mut ids = Vec::new();
        for road in &self.roads {
            let mut spec = RoadSpec::new(
                Curve3::polyline(road.points.clone()).map_err(|e| format!("{}: {e}", road.name))?,
                road.lanes.clone(),
            )
            .with_name(road.name.clone())
            .with_type(road.road_type)
            .with_form(road.form);
            if let Some(kph) = road.speed_limit {
                spec = spec.with_speed_limit(SpeedLimit::from_kph(kph).map_err(|e| e.to_string())?);
            }
            for (station, lanes) in &road.cross_sections {
                spec = spec.with_cross_section(*station, lanes.clone());
            }
            let id = builder
                .add_road(spec)
                .map_err(|e| format!("{}: {e}", road.name))?;
            if road.corridor {
                places
                    .corridor_roads
                    .push((id.clone(), self.corridor.clone()));
            } else if self.fitted.contains(&road.name) {
                places.fitted_roads.push(id.clone());
            }
            ids.push(id);
        }
        places.anchors.extend(self.anchors.iter().copied());
        let road = |name: &str| RoadId::new(name);
        let mut junctions = std::collections::HashMap::new();
        for name in &self.junctions {
            junctions.insert(name.clone(), builder.add_junction(Some(name)));
        }
        let junction = |name: &str| -> Result<JunctionId, String> {
            junctions
                .get(name)
                .cloned()
                .ok_or_else(|| format!("the plan names no junction {name}"))
        };
        for connection in &self.connections {
            match connection {
                ConnectionPlan::Lanes {
                    from,
                    to,
                    junction: j,
                } => {
                    builder
                        .connect_lanes(
                            &LaneRef::new(road(&from.0), from.1),
                            &LaneRef::new(road(&to.0), to.1),
                            Some(&junction(j)?),
                        )
                        .map_err(|e| e.to_string())?;
                }
                ConnectionPlan::Ends {
                    a,
                    a_end,
                    b,
                    b_end,
                    junction: j,
                } => {
                    builder
                        .connect_ends(&road(a), *a_end, &road(b), *b_end, Some(&junction(j)?))
                        .map_err(|e| e.to_string())?;
                }
            }
        }
        for control in &self.controls {
            let lane = LaneRef::new(road(&control.road), control.lane);
            let stop = builder
                .add_stop_line(&lane, control.end)
                .map_err(|e| e.to_string())?;
            match control.control {
                Control::Signal => {
                    let light = builder
                        .add_traffic_light(&lane, control.end, 5.0)
                        .map_err(|e| e.to_string())?;
                    builder.add_traffic_light_rule(vec![light], Some(stop), vec![lane]);
                }
                Control::Yield => {
                    builder.add_right_of_way(
                        control
                            .priority
                            .iter()
                            .map(|(name, index)| LaneRef::new(road(name), *index))
                            .collect(),
                        vec![lane],
                        Some(stop),
                    );
                }
            }
        }
        for structure in &self.structures {
            let spans: Vec<RoadSpan> = structure
                .spans
                .iter()
                .map(|(name, from, to)| RoadSpan::new(road(name), *from, *to))
                .collect();
            if let Some(form) = structure.bridge {
                builder
                    .add_bridge(spans, BridgeSpec::for_form(form))
                    .map_err(|e| e.to_string())?;
            } else if let Some(form) = structure.tunnel {
                builder
                    .add_tunnel(
                        spans,
                        TunnelSpec {
                            form,
                            ..TunnelSpec::default()
                        },
                    )
                    .map_err(|e| e.to_string())?;
            }
        }
        for interchange in &self.interchanges {
            builder
                .add_interchange(
                    interchange.name.clone(),
                    interchange.kind.clone(),
                    interchange
                        .junctions
                        .iter()
                        .map(|name| junction(name))
                        .collect::<Result<_, _>>()?,
                    interchange.roads.iter().map(|name| road(name)).collect(),
                )
                .map_err(|e| e.to_string())?;
        }
        Ok(ids)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bend_is_rounded_at_the_radius_the_speed_asks_for() {
        let mut notes = Vec::new();
        let track = Track::new(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1500.0, 0.0, 0.0),
                Point3::new(1500.0, 1500.0, 0.0),
            ],
            460.0,
            &mut notes,
        )
        .unwrap();
        assert!(notes.is_empty());
        // A quarter turn of 690 m radius cuts the corner by r(√2 − 1).
        let corner = track
            .rows
            .iter()
            .map(|row| (row.1 - 1500.0).hypot(row.2))
            .fold(f64::INFINITY, f64::min);
        assert!(
            (corner - 690.0 * (2f64.sqrt() - 1.0)).abs() < 1.0,
            "{corner}"
        );
    }

    #[test]
    fn a_bend_too_tight_for_the_speed_is_noted() {
        let mut notes = Vec::new();
        Track::new(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(200.0, 0.0, 0.0),
                Point3::new(200.0, 200.0, 0.0),
            ],
            460.0,
            &mut notes,
        )
        .unwrap();
        assert_eq!(notes.len(), 1);
    }

    #[test]
    fn the_standard_for_a_speed_is_the_row_at_or_below_it() {
        assert_eq!(DesignStandard::for_speed(110.0).unwrap().min_radius, 460.0);
        assert_eq!(DesignStandard::for_speed(120.0).unwrap().min_radius, 710.0);
        assert!(DesignStandard::for_speed(10.0).is_err());
    }
}
