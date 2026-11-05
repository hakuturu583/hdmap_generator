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
use roadgen_core::geometry::plan::{bounds, polyline_distance, ring_contains, within_bounds};
use roadgen_core::geometry::{
    Curve3, Point3, SamplingConfig, Taper, VerticalProfile, WidthProfile,
};
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

use super::interchange::{self, Directional, FreeFlow, Stack};

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

    pub(super) fn sign(self) -> f64 {
        match self {
            Side::Left => 1.0,
            Side::Right => -1.0,
        }
    }

    pub(super) fn as_str(self) -> &'static str {
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
    /// A partial cloverleaf: a diamond whose turn on to the motorway across the
    /// crossroad's oncoming traffic is a loop instead. Each side has its off
    /// slip road and a loop on, both meeting the crossroad at one terminal
    /// before the crossing, and a slip road on after it for traffic from the
    /// near side.
    Parclo,
    /// A crossroad that ends at the motorway, every movement free-flowing: it
    /// comes from the side opposite `side`, where a slip road off merges into it
    /// and one on leaves it, crosses the motorway on its overpass as the stem,
    /// and on `side` splits, with no terminal, into a loop off the far
    /// carriageway and a semi-direct slip road on to it that goes round the loop.
    Trumpet { side: Side },
    /// A full cloverleaf: in each quadrant a loop, for the turn across the
    /// crossroad on (before it) or off (after it), and outside the loop a slip
    /// road for the turn the other way. The loops and the slip roads leave and
    /// join a collector–distributor road beside each carriageway, so the
    /// weaving between a loop on and a loop off is kept off the through lanes;
    /// the collector–distributor road leaves the carriageway once and joins it
    /// once. On the crossroad every slip road merges or diverges: nothing is
    /// signalled and nothing gives way.
    Cloverleaf,
    /// A junction of two motorways where one ends at the other — a directional
    /// T. The crossroad is the other motorway's line: it comes from `side` and
    /// ends short of the corridor, and the plan builds it as two carriageways of
    /// the corridor's standard and lanes, its own `crossroad_lanes` unused. Its
    /// carriageway in splits into a slip road on to the near carriageway and a
    /// flyover across the motorway on to the far one; its carriageway out is
    /// joined by a slip road off the near carriageway and a flyover off the far
    /// one. The two flyovers cross each other beside the motorway, one stacked
    /// over the other: three levels.
    Directional { side: Side },
    /// A junction of two motorways that cross — a four-level stack. The
    /// crossroad is the other motorway's line, drawn right across the corridor;
    /// the plan builds it as two carriageways of the corridor's standard and
    /// lanes, its own `crossroad_lanes` unused, lifted a level over the
    /// corridor — or carried over as it is, standing higher already — and on a
    /// viaduct where it crosses it. Every one of the eight turns has a slip road of its own, off
    /// one motorway with a deceleration lane and on to the other with an
    /// acceleration lane: the four turns that cross nothing a quarter turn in
    /// their quadrant, outside everything else; the four that cross both
    /// motorways a flyover round the middle, the two on to the corridor a level
    /// over the other motorway and the two off it a level higher still. The
    /// corridor is the bottom level, the other motorway the second, and the
    /// flyovers the third and the fourth.
    Stack,
}

impl InterchangeKind {
    pub fn as_str(self) -> &'static str {
        match self {
            InterchangeKind::Diamond => "diamond",
            InterchangeKind::HalfDiamond { .. } => "half_diamond",
            InterchangeKind::Parclo => "parclo",
            InterchangeKind::Trumpet { .. } => "trumpet",
            InterchangeKind::Cloverleaf => "cloverleaf",
            InterchangeKind::Directional { .. } => "directional",
            InterchangeKind::Stack => "stack",
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

/// What a corridor does where it runs through a town, when it finds the town's
/// stretches itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Urban {
    /// Lifted onto a viaduct, by [`CorridorSpec::elevated_height`].
    Elevated,
    /// Dropped into a cut-and-cover tunnel, by [`CorridorSpec::sunken_depth`].
    Sunken,
}

impl Urban {
    pub fn parse(text: &str) -> Option<Urban> {
        match text {
            "elevated" => Some(Urban::Elevated),
            "sunken" => Some(Urban::Sunken),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Urban::Elevated => "elevated",
            Urban::Sunken => "sunken",
        }
    }
}

/// The town a corridor runs through, as it finds its urban stretches by: the
/// streets on the map, and the areas built on.
///
/// The plan is drawn before it is put on the map, so the town is handed to it
/// rather than read from the map: [`Town::of`] gathers it from a map being
/// built. A street is any road on it that is no corridor's — neither a
/// carriageway or slip road of one nor a crossroad one has lifted or dropped —
/// and no site's; the corridor's own crossroads and slip roads are in its plan,
/// not yet on the map, and so are never among them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Town {
    /// Each street's line in plan, and the farthest its edge can stand off the
    /// line, metres.
    pub streets: Vec<(Vec<[f64; 2]>, f64)>,
    /// The outlines in plan of the areas built on: the pads buildings stand on.
    pub areas: Vec<Vec<[f64; 2]>>,
}

impl Town {
    /// The town on a map being built: its streets, and the pads with buildings
    /// on them.
    pub fn of(builder: &MapBuilder, places: &super::Places) -> Town {
        let theirs = |id: &RoadId| {
            places.corridor_roads.iter().any(|(road, _)| road == id)
                || places.site_roads.iter().any(|(road, _)| road == id)
                || places.fitted_roads.contains(id)
        };
        let config = SamplingConfig::new(STEP).expect("a positive step");
        let streets = builder
            .roads()
            .filter(|(id, _)| !theirs(id))
            .filter_map(|(_, spec)| {
                let line: Vec<[f64; 2]> = spec
                    .reference_line
                    .samples(config)
                    .ok()?
                    .iter()
                    .map(|sample| [sample.point.x, sample.point.y])
                    .collect();
                // The wider side's lanes, each at its widest: no edge stands
                // farther off the line than that.
                let handedness = builder.metadata().handedness;
                let reach = spec
                    .cross_sections
                    .iter()
                    .map(|section| {
                        let mut sides = [0.0_f64; 2];
                        for lane in &section.lanes {
                            let widest = lane
                                .width
                                .knots()
                                .iter()
                                .map(|k| k.1.metres())
                                .fold(0.0, f64::max);
                            let side = roadgen_core::layout::side_of(lane, handedness);
                            sides[usize::from(
                                side == roadgen_core::topology::LateralSide::Right,
                            )] += widest;
                        }
                        sides[0].max(sides[1])
                    })
                    .fold(0.0, f64::max);
                Some((line, reach))
            })
            .collect();
        let areas = places
            .pads
            .iter()
            .filter(|pad| pad.buildings.is_some())
            .map(|pad| pad.outline.iter().map(|p| [p.x, p.y]).collect())
            .collect();
        Town { streets, areas }
    }

    /// Whether `p` is within `reach` of the town: of a street's edge, or of a
    /// built-up area or in it.
    fn near(&self, p: [f64; 2], reach: f64) -> bool {
        self.streets.iter().any(|(line, half)| {
            within_bounds(&bounds(line), p, reach + half)
                && polyline_distance(p, line) <= reach + half
        }) || self.areas.iter().any(|ring| {
            let mut closed = ring.clone();
            closed.extend(ring.first().copied());
            within_bounds(&bounds(ring), p, reach)
                && (ring_contains(ring, p) || polyline_distance(p, &closed) <= reach)
        })
    }
}

/// How near the town a stretch of the centre line counts as running through it,
/// metres: from the centre line to a street's edge or a built-up area.
pub const URBAN_REACH: f64 = 60.0;
/// The shortest stretch between two urban stretches the profile comes back down
/// (or up) to grade for, metres; shorter, the two are one.
pub const URBAN_GAP: f64 = 200.0;
/// How far before the town, and after it, the profile is at its full height or
/// depth, metres, beyond the climb or descent at the design grade.
const URBAN_LEVEL: f64 = 20.0;

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
    /// Whether the two carriageways share one deck where both are on a bridge or a
    /// viaduct over the same stretch — the elevated stretches, and the bridges the
    /// land asks for — as an urban viaduct does: one structure carrying both, its
    /// piers down the median. Otherwise each carriageway has a deck of its own.
    pub shared_deck: bool,
    /// Centre-line stretches dropped into a cut-and-cover tunnel, and how far under.
    pub sunken: Vec<(f64, f64)>,
    pub sunken_depth: f64,
    /// Whether the stretches through the town are found by the corridor itself,
    /// and lifted onto a viaduct or dropped into a cut-and-cover tunnel: the
    /// stretches near the town's streets and built-up areas, with the room to
    /// climb or descend at the design grade either side. They are found only by
    /// [`CorridorSpec::plan_in`], which is given the town, and added to the
    /// stretches `elevated` and `sunken` give by hand. See [`Town`].
    pub urban: Option<Urban>,
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
            shared_deck: false,
            sunken: Vec::new(),
            sunken_depth: 10.0,
            urban: None,
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
pub(super) const STEP: f64 = 5.0;
/// The length of a junction where a slip road leaves or joins, metres.
pub(super) const GAP: f64 = 20.0;
/// The radius of a loop slip road along the middle of its lane, metres: the
/// Road Structure Ordinance's least radius (art. 15) for 30 km/h, the speed a
/// loop is designed to — slower than the 40 km/h of the other slip roads, whose
/// 60 m would need the terminals half as far out again.
pub(super) const LOOP_RADIUS: f64 = 30.0;
/// The room kept between a loop and the off slip road outside it, metres.
pub(super) const LOOP_CLEARANCE: f64 = 2.0;
/// How much farther out than the terminal an off slip road may end, metres, to
/// make room for the loop inside it: the junction at the terminal reaches
/// [`GAP`] / 2 along the crossroad each side, and the off slip road's lanes stay
/// within it.
const LOOP_PUSH: f64 = 3.5;

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

/// `stretches` sorted, and any nearer each other than `gap` made one.
fn merged(mut stretches: Vec<(f64, f64)>, gap: f64) -> Vec<(f64, f64)> {
    stretches.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (from, to) in stretches {
        match out.last_mut() {
            Some(last) if from - last.1 < gap || from <= last.1 => last.1 = last.1.max(to),
            _ => out.push((from, to)),
        }
    }
    out
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
    /// One road's end on to the next road's start, lane for lane, with no
    /// junction between: a road that changes what it is part of the way along.
    Follows { from: String, to: String },
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
pub(super) fn one_way(
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
pub(super) fn carriageway_width(standard: &DesignStandard, lanes: usize) -> f64 {
    standard.inner_shoulder + standard.lane_width * lanes as f64 + standard.outer_shoulder
}

/// A cubic in plan from `a` heading `ha` to `b` heading `hb`, sampled every
/// [`STEP`], with heights eased from `za` to `zb`.
pub(super) fn link(a: [f64; 2], ha: f64, za: f64, b: [f64; 2], hb: f64, zb: f64) -> Vec<Point3> {
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

/// Where the lanes of a road's last cross-section begin in its list of lanes.
fn last_section(road: &RoadPlan) -> usize {
    match road.cross_sections.split_last() {
        None => 0,
        Some((_, earlier)) => {
            road.lanes.len() + earlier.iter().map(|(_, l)| l.len()).sum::<usize>()
        }
    }
}

/// Plan length of a polyline.
pub(super) fn length_of(points: &[Point3]) -> f64 {
    points
        .windows(2)
        .map(|pair| pair[0].horizontal_distance_to(pair[1]))
        .sum()
}

/// The steepest grade along a polyline.
pub(super) fn steepest(points: &[Point3]) -> f64 {
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
    /// The limit on it, km/h, and whether it is still the motorway: a stage of
    /// the transition at the corridor's far end is slower, and its last is an
    /// ordinary road.
    speed: f64,
    motorway: bool,
}

/// Where a slip road leaves or joins a carriageway.
#[derive(Debug, Clone)]
pub(super) struct Nose {
    /// Travel station of the nose.
    pub at: f64,
    pub on: bool,
}

/// One carriageway of the corridor, or a collector–distributor road beside one.
pub(super) struct Carriageway {
    pub side: Side,
    /// Whether traffic goes the centre line's way.
    pub along: bool,
    /// The reference line's offset to the left of the centre line.
    pub offset: f64,
    pub noses: Vec<Nose>,
    /// What it is built to, and its driving lanes: the motorway's, or a slip
    /// road's for a collector–distributor road.
    pub standard: DesignStandard,
    pub lanes: usize,
    /// The travel stations it runs between: the corridor's ends, or where a
    /// collector–distributor road leaves the carriageway and joins it again.
    pub span: (f64, f64),
    /// What its pieces are named after, numbered.
    pub name: String,
    /// How far a collector–distributor road's reference line moves out, away from
    /// the carriageway it runs beside, over [`DRIFT_LENGTH`] from each end: the
    /// room that keeps it apart from the through lanes. Nothing for a carriageway.
    pub drift: f64,
    pub collector: bool,
}

/// How far a collector–distributor road takes to move out from beside the
/// carriageway, and back, metres.
const DRIFT_LENGTH: f64 = 60.0;

impl Carriageway {
    /// The way traffic heads at centre-line station `s`.
    pub fn heading(&self, track: &Track, s: f64) -> f64 {
        let (_, _, h) = track.at(s);
        if self.along {
            h
        } else {
            h + std::f64::consts::PI
        }
    }

    /// How far its reference line has moved out at travel station `t`.
    fn drift_at(&self, t: f64) -> f64 {
        if self.drift == 0.0 {
            return 0.0;
        }
        let x = ((t - self.span.0).min(self.span.1 - t) / DRIFT_LENGTH).clamp(0.0, 1.0);
        self.drift * (0.5 - 0.5 * (std::f64::consts::PI * x).cos())
    }
}

/// How a slip road's points are made.
pub(super) enum Shape {
    /// A cubic from the nose to its open end, the heights eased.
    Link,
    /// The cubic to `lead`, the points between it and the open end — from the
    /// open end on a slip road on, to it on one off — at one even grade.
    Lead(Vec<Point3>),
    /// A lead whose heights go evenly from its open end to the access's end
    /// height where it meets the cubic, which runs on from that height: a loop
    /// that has done its climbing by the time it reaches the carriageway. On a
    /// slip road on only.
    Loop(Vec<Point3>),
    /// The road whole, heights and all, from its open end to the nose's end of
    /// the junction on a slip road on, and from there to the open end off.
    Whole(Vec<Point3>),
    /// No road of its own: the slip road is the first (off) or last (on) piece
    /// of a collector–distributor road.
    Collector,
    /// No road of its own here: the slip road is another plan's, which leaves
    /// or joins this one's carriageway — a stack's, between two motorways.
    Elsewhere,
}

/// A slip road another plan draws, leaving or joining one of this corridor's
/// carriageways: its name, the carriageway's side, and the nose, as a travel
/// station along that carriageway.
#[derive(Debug, Clone)]
pub(super) struct Join {
    pub ramp: String,
    pub side: Side,
    pub on: bool,
    pub nose: f64,
}

/// Where a slip road leaves or joins a carriageway, and the slip road.
pub(super) struct Access {
    pub ramp: String,
    pub carriageway: usize,
    pub on: bool,
    pub nose: f64,
    /// Its open end — or, beside a lead, the lead's end at the link — and the
    /// height and heading there.
    pub end: [f64; 2],
    pub end_z: f64,
    pub end_heading: f64,
    pub shape: Shape,
}

impl CorridorSpec {
    /// The length over which the profile climbs `height` onto an elevated stretch,
    /// or descends it into a sunken one: a cosine transition whose steepest grade
    /// is four fifths of the design standard's, and no shorter than 120 m.
    pub(super) fn lift(&self, height: f64) -> f64 {
        (height * std::f64::consts::PI / (2.0 * 0.8 * self.standard.max_grade.max(0.02))).max(120.0)
    }

    /// The stretches of the centre line through `town`, each from where the
    /// profile leaves its grade to where it is back on it.
    ///
    /// A station runs through the town when the centre line there is within
    /// [`URBAN_REACH`] of a street's edge or of a built-up area. Each run of such
    /// stations is widened by the climb (or descent) to the height of
    /// [`CorridorSpec::urban`]'s structure at the design grade — as
    /// [`CorridorSpec::elevated`] stretches are, which include their ramps — and
    /// by [`URBAN_LEVEL`] more, so the profile is at its full height over the
    /// town; runs nearer each other than [`URBAN_GAP`] are then one. A stretch
    /// whose climb does not fit before an end of the corridor is noted.
    pub fn urban_stretches(
        &self,
        town: &Town,
        notes: &mut Vec<String>,
    ) -> Result<Vec<(f64, f64)>, String> {
        let Some(urban) = self.urban else {
            return Ok(Vec::new());
        };
        let track = Track::new(&self.centre, self.standard.min_radius, &mut Vec::new())?;
        let length = track.length();
        let steps = (length / STEP).ceil().max(1.0) as usize;
        let mut runs: Vec<(f64, f64)> = Vec::new();
        for k in 0..=steps {
            let s = length * k as f64 / steps as f64;
            let (x, y, _) = track.at(s);
            if !town.near([x, y], URBAN_REACH) {
                continue;
            }
            match runs.last_mut() {
                Some(run) if s - run.1 <= STEP * 1.5 => run.1 = s,
                _ => runs.push((s, s)),
            }
        }
        let height = match urban {
            Urban::Elevated => self.elevated_height,
            Urban::Sunken => self.sunken_depth,
        };
        let lead = self.lift(height) + URBAN_LEVEL;
        let mut stretches = Vec::new();
        for (from, to) in runs {
            if from < lead || to > length - lead {
                notes.push(format!(
                    "{}: the town at {from:.0}–{to:.0} m is too near an end of the corridor \
                     for it to be {} at its grade there",
                    self.name,
                    urban.as_str()
                ));
            }
            stretches.push(((from - lead).max(0.0), (to + lead).min(length)));
        }
        Ok(merged(stretches, URBAN_GAP))
    }

    /// Plans the corridor through `town`: as [`CorridorSpec::plan`], with the
    /// stretches [`CorridorSpec::urban_stretches`] finds lifted or dropped.
    ///
    /// They are added to the stretches of the same kind given by hand, and where
    /// one overlaps another the two are one; one that overlaps a stretch of the
    /// other kind given by hand gives way to it, and is noted. Without
    /// [`CorridorSpec::urban`] this is [`CorridorSpec::plan`].
    pub fn plan_in(
        &self,
        handedness: TrafficHandedness,
        town: &Town,
    ) -> Result<CorridorPlan, String> {
        let Some(urban) = self.urban else {
            return self.plan(handedness);
        };
        self.check()?;
        let mut notes = Vec::new();
        let found = self.urban_stretches(town, &mut notes)?;
        let mut spec = self.clone();
        spec.urban = None;
        let (mine, theirs) = match urban {
            Urban::Elevated => (&mut spec.elevated, &self.sunken),
            Urban::Sunken => (&mut spec.sunken, &self.elevated),
        };
        for (from, to) in found {
            if theirs.iter().any(|&(a, b)| from < b && a < to) {
                notes.push(format!(
                    "{}: the town at {from:.0}–{to:.0} m is not {}: a stretch given by hand \
                     is the other way",
                    self.name,
                    urban.as_str()
                ));
                continue;
            }
            mine.push((from, to));
        }
        *mine = merged(std::mem::take(mine), 0.0);
        let mut plan = spec.plan(handedness)?;
        notes.append(&mut plan.notes);
        plan.notes = notes;
        Ok(plan)
    }

    /// The height of the centre line at `s`: the profile, lifted over its elevated
    /// stretches and dropped through its sunken ones.
    pub(super) fn height_at(&self, profile: &VerticalProfile, s: f64) -> f64 {
        let ramp_for = |height: f64| self.lift(height);
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

    /// Splits the piece at the corridor's far end into the stages of its
    /// transition to an ordinary road: the limit stepped down 20 km/h at a
    /// time to 60, each stage an equal share of `transition`, and the last no
    /// longer the motorway. `along` says which way traffic on the carriageway
    /// goes: towards the far end, whose piece is the last, or away from it,
    /// whose piece is the first. A speed-change lane on the piece stays on the
    /// stretch of it at the design speed.
    fn stage(
        &self,
        along: bool,
        transition: f64,
        pieces: &mut Vec<Piece>,
        notes: &mut Vec<String>,
    ) {
        let standard = &self.standard;
        let mut speeds = Vec::new();
        let mut limit = standard.design_speed - 20.0;
        while limit > 60.0 + 1e-6 {
            speeds.push(limit);
            limit -= 20.0;
        }
        speeds.push(60.0_f64.min(standard.design_speed));
        let piece = if along {
            pieces.pop()
        } else {
            (!pieces.is_empty()).then(|| pieces.remove(0))
        };
        let Some(piece) = piece else { return };
        // What the design-speed stretch keeps: its speed-change lane, or a road's
        // worth of length.
        let keep = if piece.accelerates {
            standard.acceleration + standard.taper + 10.0
        } else if piece.decelerates {
            standard.deceleration + standard.taper + 10.0
        } else {
            30.0
        };
        let room = (piece.to - piece.from - keep).max(0.0);
        if room + 1e-6 < transition {
            notes.push(format!(
                "{}: only {room:.0} m of the {transition:.0} m transition at the corridor's end fits",
                self.name
            ));
        }
        let length = transition.min(room);
        let share = length / speeds.len() as f64;
        let stage = |from: f64, to: f64, speed: f64, last: bool| Piece {
            from,
            to,
            accelerates: false,
            decelerates: false,
            speed,
            motorway: !last,
        };
        let count = speeds.len();
        if share < 10.0 {
            // No room for stages: the whole piece is the ordinary road.
            pieces.insert(
                if along { pieces.len() } else { 0 },
                Piece {
                    speed: speeds[count - 1],
                    motorway: false,
                    ..piece
                },
            );
            return;
        }
        if along {
            let start = piece.to - length;
            pieces.push(Piece { to: start, ..piece });
            for (k, &speed) in speeds.iter().enumerate() {
                let from = start + share * k as f64;
                pieces.push(stage(from, from + share, speed, k + 1 == count));
            }
        } else {
            let mut staged: Vec<Piece> = speeds
                .iter()
                .rev()
                .enumerate()
                .map(|(k, &speed)| {
                    let from = piece.from + share * k as f64;
                    stage(from, from + share, speed, k == 0)
                })
                .collect();
            staged.push(Piece {
                from: piece.from + length,
                ..piece
            });
            pieces.splice(0..0, staged);
        }
    }

    /// Works out the roads, junctions, movements and structures the corridor asks
    /// for. See the module documentation.
    pub fn plan(&self, handedness: TrafficHandedness) -> Result<CorridorPlan, String> {
        self.plan_joined(handedness, &[])
    }

    /// The centre line, rounded and sampled, and its vertical alignment: the
    /// profile given, or the centre points' heights at their stations. A bend
    /// tighter than the design speed allows, and a grade steeper, is noted.
    pub(super) fn alignment(
        &self,
        notes: &mut Vec<String>,
    ) -> Result<(Track, VerticalProfile), String> {
        let standard = &self.standard;
        let track = Track::new(&self.centre, standard.min_radius, notes)?;
        let length = track.length();
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
            notes.push(format!(
                "the profile climbs {:.1} %, steeper than the {:.1} % the design speed allows",
                profile.steepest() * 100.0,
                standard.max_grade * 100.0
            ));
        }
        Ok((track, profile))
    }

    /// Plans the corridor as [`CorridorSpec::plan`] does, with `joins` — slip
    /// roads another plan draws — leaving and joining its carriageways too.
    pub(super) fn plan_joined(
        &self,
        handedness: TrafficHandedness,
        joins: &[Join],
    ) -> Result<CorridorPlan, String> {
        self.check()?;
        let key = self.key();
        let standard = &self.standard;
        let mut plan = CorridorPlan {
            corridor: self.name.clone(),
            ..CorridorPlan::default()
        };
        if let Some(urban) = self.urban {
            plan.notes.push(format!(
                "{}: drawn without the town, so no stretch through it is {}; plan it in the \
                 town with plan_in",
                self.name,
                urban.as_str()
            ));
        }
        let (track, profile) = self.alignment(&mut plan.notes)?;
        let length = track.length();
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
                standard: standard.clone(),
                lanes: self.lanes,
                span: (0.0, length),
                name: format!("{key}_{}", side.as_str()),
                drift: 0.0,
                collector: false,
            })
            .collect();
        // The side a carriageway's lanes lie on, as a sign to the left of travel.
        let outward = if forward_right { -1.0 } else { 1.0 };
        let travel = |c: &Carriageway, s: f64| if c.along { s } else { length - s };
        let centre_of = |c: &Carriageway, t: f64| if c.along { t } else { length - t };

        // Interchanges and ramps: where each slip road leaves or joins.
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
            // The ramps are laid out round where the crossroad really crosses.
            if (centre_station - site.at).abs() > 1.0 {
                plan.notes.push(format!(
                    "{}: the crossroad crosses the corridor at {centre_station:.0} m, not at \
                     {:.0} m; the interchange is built where it crosses",
                    site.name, site.at
                ));
            }
            // A directional T's crossroad is a motorway that ends short of the
            // corridor, and nothing of it crosses.
            if let InterchangeKind::Directional { side: near } = site.kind {
                self.directional(
                    Directional {
                        site,
                        ic: &ic,
                        near,
                        c_station,
                        centre_station,
                        cross_length,
                        track: &track,
                        z: &z,
                        point_at: &point_at,
                        ground_at: &ground_at,
                        at_lateral: &at_lateral,
                        cw_width,
                        outward,
                        handedness,
                        ramp_standard: &ramp_standard,
                    },
                    &mut plan,
                    &carriageways,
                    &mut accesses,
                )?;
                continue;
            }
            // A stack's crossroad is a motorway that crosses on through, planned
            // as a corridor of its own.
            if site.kind == InterchangeKind::Stack {
                self.stack(
                    Stack {
                        site,
                        ic: &ic,
                        centre_station,
                        track: &track,
                        z: &z,
                        cw_width,
                        outward,
                        handedness,
                        ramp_standard: &ramp_standard,
                    },
                    &mut plan,
                    &carriageways,
                    &mut accesses,
                )?;
                continue;
            }
            let motorway_z = z(centre_station);
            // A cloverleaf's collector–distributor roads, an auxiliary lane wide,
            // pass under the crossroad too.
            let collector_room = if site.kind == InterchangeKind::Cloverleaf {
                0.5 + interchange::DRIFT + carriageway_width(&ramp_standard, 2)
            } else {
                0.0
            };
            let half_corridor = standard.median / 2.0 + cw_width + collector_room + 3.0;
            // Over the motorway: an overpass's deck and the clearance under it,
            // and half a metre to spare, which the vertical curves rounding the
            // ends of the level stretch eat into over what passes under near
            // them — a parclo's loops; under it: the clearance under the
            // motorway's own overpass.
            let target = if site.over {
                motorway_z + CLEARANCE + OVERPASS_DECK + 0.5
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

            let (centre_plan, centre_heading) = {
                let (x, y, h) = track.at(centre_station);
                ([x, y], h)
            };
            // A free-flowing interchange is laid out side by side: its slip roads,
            // and where they meet the crossroad.
            let free = matches!(
                site.kind,
                InterchangeKind::Cloverleaf | InterchangeKind::Trumpet { .. }
            );
            let mut layouts: Vec<(Side, interchange::SideLayout)> = Vec::new();
            if free {
                for side in [Side::Left, Side::Right] {
                    // The crossroad's traffic heading out on this side goes the way
                    // it was drawn if this side is past the crossing.
                    let after = at_lateral(side.sign() * 5.0).is_some_and(|s| s > c_station);
                    let width_of = |direction: Direction| {
                        site.crossroad_lanes
                            .iter()
                            .filter(|lane| lane.direction == direction)
                            .map(|lane| lane.width.evaluate(0.0).metres())
                            .sum::<f64>()
                    };
                    let (forward, backward) =
                        (width_of(Direction::Forward), width_of(Direction::Backward));
                    let dims = interchange::Dims {
                        edge: standard.median / 2.0 + cw_width,
                        ramp_width: carriageway_width(&ramp_standard, 1),
                        lane_middle: ramp_standard.inner_shoulder + ramp_standard.lane_width / 2.0,
                        ramp_radius: ramp_standard.min_radius,
                        loop_radius: LOOP_RADIUS,
                        out: if after { forward } else { backward },
                        inward: if after { backward } else { forward },
                        least: half_corridor + GAP / 2.0 + 1.0,
                        terminal: site.terminal,
                        reach: site.reach,
                    };
                    let layout = match site.kind {
                        InterchangeKind::Trumpet { side: far } if far == side => {
                            interchange::trumpet_far(&dims)
                        }
                        InterchangeKind::Trumpet { .. } => interchange::trumpet_near(&dims),
                        _ => interchange::cloverleaf(&dims),
                    }
                    .map_err(|error| format!("{}: {error}", site.name))?;
                    layouts.push((side, layout));
                }
            }
            // The junctions on the crossroad, by station: a terminal each side,
            // or where a free-flowing layout puts them.
            let sides: Vec<Side> = match site.kind {
                InterchangeKind::HalfDiamond { side } => vec![side],
                _ => vec![Side::Left, Side::Right],
            };
            let mut cuts: Vec<(f64, Side, usize)> = Vec::new();
            for &side in &sides {
                let spots: Vec<f64> = match layouts.iter().find(|(s, _)| *s == side) {
                    Some((_, layout)) => layout.junctions.iter().map(|j| j.b).collect(),
                    None => vec![site.terminal],
                };
                for (k, b) in spots.into_iter().enumerate() {
                    let s = at_lateral(side.sign() * b).ok_or_else(|| {
                        format!(
                            "{}: the crossroad does not reach {b:.0} m to the {} of the corridor",
                            site.name,
                            side.as_str()
                        )
                    })?;
                    cuts.push((s, side, k));
                }
            }
            cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
            // The crossroad in pieces between the terminals.
            let mut bounds = vec![0.0];
            for (s, _, _) in &cuts {
                bounds.push(s - GAP / 2.0);
                bounds.push(s + GAP / 2.0);
            }
            bounds.push(cross_length);
            // A crossroad that ends at the interchange stops at the junction it
            // ends at: the piece beyond is not built.
            let stop = cuts
                .iter()
                .position(|(_, side, k)| {
                    layouts
                        .iter()
                        .any(|(s, layout)| s == side && layout.junctions[*k].stop)
                })
                .map(|k| (cuts[k].0, if cuts[k].0 > c_station { k + 1 } else { k }));
            let beyond_stop =
                |s: f64| stop.is_some_and(|(at, _)| (s - at) * (at - c_station) > 0.0);
            let mut cross_names: Vec<Option<String>> = Vec::new();
            for (index, pair) in bounds.chunks(2).enumerate() {
                if stop.is_some_and(|(_, dropped)| dropped == index) {
                    cross_names.push(None);
                    continue;
                }
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
                cross_names.push(Some(name));
            }
            // The ground the crossroad was drawn on, away from the motorway: what
            // its embankment stands on.
            let mut s = 0.0;
            while s <= cross_length {
                if (s - c_station).abs() > half_corridor && !beyond_stop(s) {
                    let p = point_at(s);
                    plan.anchors.push(Point3::new(p[0], p[1], ground_at(s)));
                }
                s += 10.0;
            }
            let mut ic_roads: Vec<String> = cross_names.iter().flatten().cloned().collect();
            let mut ic_junctions = Vec::new();

            // The crossroad's lanes by direction: the innermost driving lane each
            // way takes the turns to and from the ramps.
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
            if free {
                self.free_flow(
                    FreeFlow {
                        site,
                        ic: &ic,
                        cuts: &cuts,
                        layouts: &layouts,
                        cross_names: &cross_names,
                        c_station,
                        centre_station,
                        centre_plan,
                        centre_heading,
                        track: &track,
                        cw_width,
                        outward,
                        ramp_standard: &ramp_standard,
                        at_lateral: &at_lateral,
                        crossroad_point: &crossroad_point,
                        roads: ic_roads,
                    },
                    &mut plan,
                    &mut carriageways,
                    &mut accesses,
                )?;
                continue;
            }

            // The ramps at each terminal, and the movements there.
            for (index, (s, side, _)) in cuts.iter().enumerate() {
                let junction = format!("{ic}_{}", side.as_str());
                plan.junctions.push(junction.clone());
                ic_junctions.push(junction.clone());
                let terminal = crossroad_point(*s);
                let (before, after) = (&cross_names[index], &cross_names[index + 1]);
                // Moving away from the corridor is moving up the crossroad when the
                // terminal is past the crossing, and down it otherwise.
                let outer_is_after = *s > c_station;
                let (inner, outer) = if outer_is_after {
                    (before, after)
                } else {
                    (after, before)
                };
                let Some(inner) = inner.clone() else {
                    return Err(format!(
                        "{}: the crossroad does not reach the corridor",
                        site.name
                    ));
                };
                let (out_lane, in_lane) = if outer_is_after {
                    (forward, backward)
                } else {
                    (backward, forward)
                };
                // A lane of a piece ends at the terminal at the piece's end when
                // the piece is before it.
                let end_at = |piece: &String| {
                    if Some(piece) == before.as_ref() {
                        LaneEnd::End
                    } else {
                        LaneEnd::Start
                    }
                };
                // Arriving at the terminal heading out, and heading in; leaving it
                // heading out, and heading in.
                let arrive_out = (inner.clone(), out_lane);
                let arrive_in = outer.clone().map(|outer| (outer, in_lane));
                let leave_out = outer.clone().map(|outer| (outer, out_lane));
                let leave_in = (inner.clone(), in_lane);

                // The carriageway on this side.
                let c = carriageways
                    .iter()
                    .position(|c| c.side == *side)
                    .expect("both sides");
                let carriageway = &carriageways[c];
                let heading_travel = carriageway.heading(&track, centre_station);
                let u = [heading_travel.cos(), heading_travel.sin()];
                let back = |d: f64| [terminal.x - u[0] * d, terminal.y - u[1] * d];
                let ahead = |d: f64| [terminal.x + u[0] * d, terminal.y + u[1] * d];
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
                // Towards the corridor from the terminal, square to it.
                let toward = {
                    let left = [-centre_heading.sin(), centre_heading.cos()];
                    let lateral = (terminal.x - centre_plan[0]) * left[0]
                        + (terminal.y - centre_plan[1]) * left[1];
                    let sign = -lateral.signum();
                    [left[0] * sign, left[1] * sign]
                };
                let setback = 14.0;
                let off_name = format!("{ic}_{}_off", side.as_str());
                let on_name = format!("{ic}_{}_on", side.as_str());
                let loop_name = format!("{ic}_{}_loop", side.as_str());
                let travel_at = travel(carriageway, centre_station);
                // What this terminal has: the slip roads off and on always, and a
                // loop on at a parclo.
                let looped = matches!(site.kind, InterchangeKind::Parclo);
                // Beside a loop the off slip road ends a ramp's width out, the
                // loop leaving the terminal a ramp's width in.
                let apart = ramp_width / 2.0 + 1.0;
                // The loop: its radius is what a loop is designed to, whatever
                // the room. It leaves the terminal a ramp's width in, against the
                // traffic, and turns towards the corridor to come out on the
                // merge line with the traffic, and on under the crossroad. Its
                // reference line is its outer edge, the lane inside it.
                let loop_start = {
                    let b = back(setback);
                    [
                        b[0] + toward[0] * (ramp_width + 1.0),
                        b[1] + toward[1] * (ramp_width + 1.0),
                    ]
                };
                let loop_radius =
                    LOOP_RADIUS + ramp_standard.inner_shoulder + ramp_standard.lane_width / 2.0;
                // How far the loop has to come in, from its start to the merge
                // line: twice its radius is a half turn; more room is taken up by
                // a straight towards the corridor between two quarter turns, and
                // less by starting out turned away from the corridor and going
                // round more than a half turn, which takes the loop out past its
                // start by the shortfall. Out there is the off slip road, which
                // ends farther out than the terminal if it has to.
                let drop = {
                    let (_, lateral) = track.locate(loop_start);
                    lateral.abs() - (standard.median / 2.0 + cw_width + 0.5)
                };
                let rise = (2.0 * loop_radius - drop).max(0.0);
                // The loop's outer edge starts a ramp's width and 2 m in from
                // the off slip road's inner one.
                let push = (rise - (ramp_width + 2.0 - LOOP_CLEARANCE)).max(0.0);
                if looped && push > LOOP_PUSH {
                    // Rounded up, so that the distance it names is enough.
                    let least = (site.terminal + push - LOOP_PUSH).ceil();
                    return Err(format!(
                        "{}: the terminal is too near the motorway for a loop of {LOOP_RADIUS:.0} m; \
                         put it {least:.0} m out or farther",
                        site.name
                    ));
                }
                let off_end = if looped {
                    let p = shift(back(setback));
                    let out = apart + push;
                    [p[0] - toward[0] * out, p[1] - toward[1] * out]
                } else {
                    shift(back(setback))
                };
                accesses.push(Access {
                    ramp: off_name.clone(),
                    carriageway: c,
                    on: false,
                    nose: travel_at - site.reach,
                    end: off_end,
                    end_z: terminal.z,
                    end_heading: heading_travel,
                    shape: Shape::Link,
                });
                ic_roads.push(off_name.clone());
                accesses.push(Access {
                    ramp: on_name.clone(),
                    carriageway: c,
                    on: true,
                    nose: travel_at + site.reach,
                    end: shift(ahead(setback)),
                    end_z: terminal.z,
                    end_heading: heading_travel,
                    shape: Shape::Link,
                });
                ic_roads.push(on_name.clone());
                if looped {
                    // The turns, as headings in the plane of along the traffic
                    // and towards the corridor, the loop turning from the one to
                    // the other: away from the corridor by `turned` at the start,
                    // against the traffic, towards the corridor, and with the
                    // traffic at the end.
                    let turned = if rise > 0.0 {
                        (drop / loop_radius - 1.0).clamp(-1.0, 1.0).acos()
                    } else {
                        0.0
                    };
                    let straight = (drop - 2.0 * loop_radius).max(0.0);
                    let half_pi = std::f64::consts::FRAC_PI_2;
                    // Points about a metre or two apart round the turns, so the
                    // curve fitted through them keeps the radius.
                    let mut local = vec![[0.0_f64, 0.0_f64]];
                    let mut heading = std::f64::consts::PI + turned;
                    let arc = |local: &mut Vec<[f64; 2]>, heading: &mut f64, to: f64| {
                        let at = *local.last().expect("a start");
                        // Turning clockwise here, the centre is to the right.
                        let centre = [
                            at[0] + loop_radius * heading.sin(),
                            at[1] - loop_radius * heading.cos(),
                        ];
                        let count = ((*heading - to) * loop_radius / 2.0).ceil().max(2.0) as usize;
                        for k in 1..=count {
                            let h = *heading + (to - *heading) * k as f64 / count as f64;
                            local.push([
                                centre[0] - loop_radius * h.sin(),
                                centre[1] + loop_radius * h.cos(),
                            ]);
                        }
                        *heading = to;
                    };
                    arc(&mut local, &mut heading, half_pi);
                    if straight > 0.0 {
                        let at = *local.last().expect("a start");
                        let count = (straight / STEP).ceil().max(1.0) as usize;
                        for k in 1..=count {
                            local.push([at[0], at[1] + straight * k as f64 / count as f64]);
                        }
                    }
                    arc(&mut local, &mut heading, 0.0);
                    let lead: Vec<Point3> = local
                        .iter()
                        .map(|[a, b]| {
                            Point3::new(
                                loop_start[0] + u[0] * a + toward[0] * b,
                                loop_start[1] + u[1] * a + toward[1] * b,
                                terminal.z,
                            )
                        })
                        .collect();
                    let end = *lead.last().expect("points");
                    // On to the nose 25 m on from where the loop starts, which
                    // is as far on as the loop goes: the merge is past the
                    // crossroad, not under it. The loop comes down round its
                    // turns to the motorway's level on the merge line, and runs
                    // on under the crossroad at that level, with all of the
                    // clearance under the overpass.
                    accesses.push(Access {
                        ramp: loop_name.clone(),
                        carriageway: c,
                        on: true,
                        nose: travel_at - setback + 25.0 + GAP / 2.0,
                        end: [end.x, end.y],
                        end_z: z(track.locate([end.x, end.y]).0),
                        end_heading: heading_travel,
                        shape: Shape::Loop(lead),
                    });
                    ic_roads.push(loop_name.clone());
                }

                use roadgen_core::topology::RoadEnd;
                if let (Some(before), Some(after)) = (before, after) {
                    plan.connections.push(ConnectionPlan::Ends {
                        a: before.clone(),
                        a_end: RoadEnd::End,
                        b: after.clone(),
                        b_end: RoadEnd::Start,
                        junction: junction.clone(),
                    });
                }
                let ramp_lane = 1;
                let lanes = |from: Option<(String, usize)>, to: Option<(String, usize)>| {
                    from.zip(to).map(|(from, to)| ConnectionPlan::Lanes {
                        from,
                        to,
                        junction: junction.clone(),
                    })
                };
                let off = Some((off_name.clone(), ramp_lane));
                // Off: out and in.
                let off_to = vec![Some(leave_in.clone()), leave_out.clone()];
                plan.connections
                    .extend(off_to.into_iter().filter_map(|to| lanes(off.clone(), to)));
                // On: from both arrivals by the slip road, unless a loop takes the
                // traffic arriving from across the corridor.
                let on = Some((on_name.clone(), ramp_lane));
                let from = if looped {
                    vec![arrive_in.clone()]
                } else {
                    vec![Some(arrive_out.clone()), arrive_in.clone()]
                };
                plan.connections
                    .extend(from.into_iter().filter_map(|from| lanes(from, on.clone())));
                if looped {
                    plan.connections.extend(lanes(
                        Some(arrive_out.clone()),
                        Some((loop_name.clone(), ramp_lane)),
                    ));
                }
                // Traffic off the ramp is controlled; the crossroad is signalled too
                // where the ramp is.
                let arrivals: Vec<(String, usize)> = std::iter::once(arrive_out.clone())
                    .chain(arrive_in.clone())
                    .collect();
                plan.controls.push(ControlPlan {
                    road: off_name.clone(),
                    lane: ramp_lane,
                    end: LaneEnd::End,
                    control: site.control,
                    priority: arrivals.clone(),
                });
                if site.control == Control::Signal {
                    for (road, lane) in arrivals {
                        let end = end_at(&road);
                        plan.controls.push(ControlPlan {
                            road,
                            lane,
                            end,
                            control: Control::Signal,
                            priority: Vec::new(),
                        });
                    }
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
                shape: Shape::Link,
            });
        }
        for join in joins {
            let c = carriageways
                .iter()
                .position(|c| c.side == join.side && !c.collector)
                .expect("both sides");
            accesses.push(Access {
                ramp: join.ramp.clone(),
                carriageway: c,
                on: join.on,
                nose: join.nose,
                end: [0.0, 0.0],
                end_z: 0.0,
                end_heading: 0.0,
                shape: Shape::Elsewhere,
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
            // A collector–distributor road is held to the slip roads' limit.
            let speed = if c.collector {
                60.0_f64.min(speed)
            } else {
                speed
            };
            let mut pieces = Vec::new();
            let mut from = c.span.0;
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
                    speed,
                    motorway: true,
                });
                from = nose.at + GAP / 2.0;
                accelerates = nose.on;
            }
            if c.span.1 - from < 30.0 {
                return Err(format!(
                    "{}: a slip road is too close to the corridor's end",
                    self.name
                ));
            }
            pieces.push(Piece {
                from,
                to: c.span.1,
                accelerates,
                decelerates: false,
                speed,
                motorway: true,
            });
            if let (Some(transition), false) = (self.end_transition, c.collector) {
                self.stage(c.along, transition, &mut pieces, &mut plan.notes);
            }
            let named = pieces
                .into_iter()
                .enumerate()
                .map(|(index, piece)| (format!("{}_{index}", c.name), piece))
                .collect();
            piece_names.push(named);
        }

        // The pieces as roads.
        let point_on = |c: &Carriageway, t: f64, extra: f64| -> Point3 {
            let s = centre_of(c, t);
            // To the left of travel is to the left of the centre line along it,
            // and to its right against it.
            let travel_left = if c.along { 1.0 } else { -1.0 };
            let p = track.offset(
                s,
                c.offset + travel_left * (extra + outward * c.drift_at(t)),
            );
            Point3::new(p[0], p[1], z(s))
        };
        let heading_on = |c: &Carriageway, t: f64| c.heading(&track, centre_of(c, t));
        for (c, pieces) in carriageways.iter().zip(&piece_names) {
            let standard = &c.standard;
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
                let plain = one_way(standard, c.lanes, None, handedness);
                let (lanes, cross_sections) = if piece.accelerates
                    && piece.decelerates
                    && accel_end >= decel_start
                {
                    // The two meet: one auxiliary lane all the way.
                    (
                        one_way(
                            standard,
                            c.lanes,
                            Some(WidthProfile::constant(width(lane_width))),
                            handedness,
                        ),
                        Vec::new(),
                    )
                } else {
                    let mut sections = Vec::new();
                    let first = if piece.accelerates {
                        sections.push((accel_end, plain.clone()));
                        one_way(standard, c.lanes, accel(), handedness)
                    } else {
                        plain.clone()
                    };
                    if piece.decelerates {
                        sections
                            .push((decel_start, one_way(standard, c.lanes, decel(), handedness)));
                    }
                    (first, sections)
                };
                plan.roads.push(RoadPlan {
                    name: name.clone(),
                    points,
                    lanes,
                    cross_sections,
                    road_type: if piece.motorway {
                        RoadType::Motorway
                    } else {
                        RoadType::Rural
                    },
                    // A collector–distributor road is a slip road all the way:
                    // entered only from the carriageway and its own slip roads.
                    form: if c.collector {
                        FormOfWay::SlipRoad
                    } else if piece.motorway {
                        FormOfWay::Motorway
                    } else {
                        FormOfWay::MultipleCarriageway
                    },
                    speed_limit: Some(piece.speed),
                    corridor: true,
                });
            }
            // Stages of the transition follow on from each other with no junction.
            for pair in pieces.windows(2) {
                if (pair[1].1.from - pair[0].1.to).abs() < 1e-6 {
                    plan.connections.push(ConnectionPlan::Follows {
                        from: pair[0].0.clone(),
                        to: pair[1].0.clone(),
                    });
                }
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
            let offset_last = last_section(lanes_before);
            // A slip road runs beside the plain carriageway, half a metre clear of it.
            let edge = carriageway_width(&c.standard, c.lanes) + 0.5;
            let junction = format!("{}_{}", key, access.ramp);
            plan.junctions.push(junction.clone());
            let (start_t, end_t) = (access.nose - GAP / 2.0, access.nose + GAP / 2.0);
            // Spreads the heights of `points` at one grade from the first to the
            // last.
            let even = |mut points: Vec<Point3>| {
                let total = length_of(&points).max(1e-9);
                let (start, end) = (points[0].z, points[points.len() - 1].z);
                let mut run = 0.0;
                for index in 0..points.len() {
                    if index > 0 {
                        run += points[index - 1].horizontal_distance_to(points[index]);
                    }
                    points[index].z = start + (end - start) * run / total;
                }
                points
            };
            let points = match (&access.shape, access.on) {
                (Shape::Collector | Shape::Elsewhere, _) => None,
                (Shape::Whole(points), _) => Some(points.clone()),
                (shape, true) => {
                    let end = point_on(c, start_t, outward * edge);
                    let heading = heading_on(c, start_t);
                    let tail = link(
                        access.end,
                        access.end_heading,
                        access.end_z,
                        [end.x, end.y],
                        heading,
                        end.z,
                    );
                    match shape {
                        // A loop, then the link: one even grade from the loop's
                        // open end down (or up) to the nose.
                        Shape::Lead(lead) if lead.len() > 1 => Some(even(
                            lead[..lead.len() - 1].iter().copied().chain(tail).collect(),
                        )),
                        // The loop at one even grade to where it meets the
                        // link, and the link on from there.
                        Shape::Loop(lead) if lead.len() > 1 => {
                            let mut lead = lead.clone();
                            lead.last_mut().expect("points").z = access.end_z;
                            let lead = even(lead);
                            Some(lead[..lead.len() - 1].iter().copied().chain(tail).collect())
                        }
                        _ => Some(tail),
                    }
                }
                (shape, false) => {
                    let start = point_on(c, end_t, outward * edge);
                    let heading = heading_on(c, end_t);
                    let head = link(
                        [start.x, start.y],
                        heading,
                        start.z,
                        access.end,
                        access.end_heading,
                        access.end_z,
                    );
                    match shape {
                        // The link, then the lead on to the open end.
                        Shape::Lead(lead) if lead.len() > 1 => Some(even(
                            head.into_iter().chain(lead[1..].iter().copied()).collect(),
                        )),
                        _ => Some(head),
                    }
                }
            };
            if let Some(points) = points {
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
            }
            // The slip road's lane: on a collector–distributor road's last piece,
            // the one of its last cross-section.
            let ramp_lane = match access.shape {
                Shape::Collector if access.on => {
                    let road = plan
                        .roads
                        .iter()
                        .find(|r| r.name == access.ramp)
                        .expect("planned");
                    last_section(road) + 1
                }
                _ => 1,
            };
            // Through: driving lane k to driving lane k. Off: the deceleration lane
            // onto the slip road; on: the slip road onto the acceleration lane.
            for k in 1..=c.lanes {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (before.clone(), offset_last + k),
                    to: (after.clone(), k),
                    junction: junction.clone(),
                });
            }
            if access.on {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (access.ramp.clone(), ramp_lane),
                    to: (after.clone(), c.lanes + 1),
                    junction: junction.clone(),
                });
            } else {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (before.clone(), offset_last + c.lanes + 1),
                    to: (access.ramp.clone(), ramp_lane),
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
                // to it are earthworks. A viaduct starts where the climb has lifted
                // the deck's underside 2 m clear of the profile — read off the
                // climb's own curve, so the land, rolling a little either side of
                // the profile, never stands above the deck.
                let margin = if bridge {
                    let need = BridgeSpec::for_form(BridgeForm::Viaduct).deck_depth + 2.0;
                    let share = (need / self.elevated_height).min(1.0);
                    (1.0 - 2.0 * share).acos() / std::f64::consts::PI
                        * self.lift(self.elevated_height)
                } else {
                    self.sunken_depth / (0.8 * standard.max_grade.max(0.02)) * 0.35
                };
                let (from, to) = (from + margin, to - margin);
                if to - from < 20.0 {
                    continue;
                }
                // Each carried stretch, with the centre-line stretch it runs beside.
                let mut stretches: Vec<Stretch> = Vec::new();
                for (side, (c, pieces)) in carriageways
                    .iter()
                    .zip(&piece_names)
                    .enumerate()
                    .filter(|(_, (c, _))| !c.collector)
                {
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
                        let (ca, cb) = (centre_of(c, a), centre_of(c, b));
                        stretches.push(Stretch {
                            side,
                            range: (ca.min(cb), ca.max(cb)),
                            road: name.clone(),
                            along: c.along,
                            start: piece.from,
                            // Travel stations to stations along the road's own
                            // points.
                            scale: length_of(&road.points) / (piece.to - piece.from).max(1e-9),
                            length,
                        });
                    }
                }
                let decks = if bridge && self.shared_deck {
                    shared_decks(stretches)
                } else {
                    stretches
                        .iter()
                        .map(|stretch| vec![stretch.span(stretch.range)])
                        .collect()
                };
                for spans in decks {
                    plan.structures.push(StructurePlan {
                        spans,
                        bridge: bridge.then_some(BridgeForm::Viaduct),
                        tunnel: (!bridge).then_some(TunnelForm::CutAndCover),
                    });
                }
            }
        }

        Ok(plan)
    }
}

/// A stretch of a road a structure carries: the road, and from and to along it.
type Span = (String, f64, f64);

/// A carriageway's piece where it runs along a stretch of the centre line.
#[derive(Debug, Clone)]
struct Stretch {
    /// Which carriageway it is a piece of.
    side: usize,
    /// The centre-line stretch it runs beside.
    range: (f64, f64),
    road: String,
    /// Whether the carriageway runs along the centre line or against it, where
    /// its piece starts in travel stations, and how much longer the road's own
    /// stations are, for a centre-line station to become one along the road.
    along: bool,
    start: f64,
    scale: f64,
    /// The centre line's length.
    length: f64,
}

impl Stretch {
    /// The span of its road beside the centre-line stretch `range`.
    fn span(&self, range: (f64, f64)) -> Span {
        let at = |centre: f64| {
            let travel = if self.along {
                centre
            } else {
                self.length - centre
            };
            (travel - self.start) * self.scale
        };
        let (a, b) = (at(range.0), at(range.1));
        (self.road.clone(), a.min(b), a.max(b))
    }
}

/// The shortest part a carried stretch is cut into to share a deck, metres: a
/// boundary nearer than this to an end or to another cut is not cut at.
const SHORTEST_PART: f64 = 10.0;

/// The carried stretches of an elevated stretch gathered onto shared decks.
///
/// Two stretches of different carriageways that overlap only in part — the two
/// broken by slip roads at different stations — are first cut where the other
/// begins and ends, so that every part that lies beside a part of the other is
/// within it. Then, the longest first, each deck takes in every other part whose
/// centre-line stretch lies within its own: a deck is laid out along its first
/// stretch, so one that reached past the first's ends would leave the rest
/// outside its outline. A carriageway broken by a slip road where the other is
/// not is carried whole by the unbroken one's deck; the parts beside nothing —
/// across from the other's slip-road junction — are decks of their own.
fn shared_decks(stretches: Vec<Stretch>) -> Vec<Vec<Span>> {
    let partly = |a: (f64, f64), b: (f64, f64)| {
        let overlap = a.1.min(b.1) - a.0.max(b.0);
        let within =
            |x: (f64, f64), y: (f64, f64)| x.0 >= y.0 - SHORTEST_PART && x.1 <= y.1 + SHORTEST_PART;
        overlap > 0.0 && !within(a, b) && !within(b, a)
    };
    let mut parts: Vec<(Stretch, (f64, f64))> = Vec::new();
    for stretch in &stretches {
        let (from, to) = stretch.range;
        let mut cuts: Vec<f64> = stretches
            .iter()
            .filter(|other| other.side != stretch.side && partly(stretch.range, other.range))
            .flat_map(|other| [other.range.0, other.range.1])
            .filter(|cut| *cut > from + SHORTEST_PART && *cut < to - SHORTEST_PART)
            .collect();
        cuts.sort_by(f64::total_cmp);
        let mut start = from;
        for cut in cuts {
            if cut - start >= SHORTEST_PART {
                parts.push((stretch.clone(), (start, cut)));
                start = cut;
            }
        }
        parts.push((stretch.clone(), (start, to)));
    }
    parts.sort_by(|a, b| (b.1 .1 - b.1 .0).total_cmp(&(a.1 .1 - a.1 .0)));
    // Each deck: its first part's carriageway and centre-line stretch, and the
    // spans it carries. Only the other carriageway's parts join it.
    let mut decks: Vec<(usize, (f64, f64), Vec<Span>)> = Vec::new();
    for (stretch, range) in parts {
        let span = stretch.span(range);
        match decks.iter_mut().find(|(side, first, _)| {
            *side != stretch.side
                && range.0 >= first.0 - SHORTEST_PART
                && range.1 <= first.1 + SHORTEST_PART
        }) {
            Some((_, _, spans)) => spans.push(span),
            None => decks.push((stretch.side, range, vec![span])),
        }
    }
    decks.into_iter().map(|(_, _, spans)| spans).collect()
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
                ConnectionPlan::Follows { from, to } => {
                    builder
                        .connect(&road(from), &road(to))
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
    fn the_far_end_steps_down_to_an_ordinary_road() {
        let mut spec = CorridorSpec::new(
            "E9",
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3000.0, 0.0, 0.0)],
            120.0,
        )
        .unwrap();
        spec.end_transition = Some(900.0);
        let plan = spec.plan(TrafficHandedness::RightHand).unwrap();
        assert!(plan.notes.is_empty(), "{:?}", plan.notes);
        for side in ["right", "left"] {
            // In the order traffic meets them.
            let roads: Vec<&RoadPlan> = plan
                .roads
                .iter()
                .filter(|r| r.name.contains(&format!("_{side}_")))
                .collect();
            let limits: Vec<f64> = roads.iter().map(|r| r.speed_limit.unwrap()).collect();
            let forms: Vec<FormOfWay> = roads.iter().map(|r| r.form).collect();
            // Away from the far end the stages come first, slowest first.
            let (limits, forms) = if limits[0] == 60.0 {
                (
                    limits.into_iter().rev().collect::<Vec<_>>(),
                    forms.into_iter().rev().collect::<Vec<_>>(),
                )
            } else {
                (limits, forms)
            };
            assert_eq!(limits, [120.0, 100.0, 80.0, 60.0], "{side}");
            assert_eq!(forms.last(), Some(&FormOfWay::MultipleCarriageway));
            assert!(forms[..3].iter().all(|f| *f == FormOfWay::Motorway));
            let stage = roads.iter().find(|r| r.speed_limit == Some(80.0)).unwrap();
            assert!((length_of(&stage.points) - 300.0).abs() < 1.0);
        }
        let follows = plan
            .connections
            .iter()
            .filter(|c| matches!(c, ConnectionPlan::Follows { .. }))
            .count();
        assert_eq!(follows, 6, "three joins on each carriageway");
    }

    #[test]
    fn a_town_at_the_corridors_start_is_noted_where_the_climb_to_it_does_not_fit() {
        let mut spec = CorridorSpec::new(
            "Shuto",
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2000.0, 0.0, 0.0)],
            60.0,
        )
        .unwrap();
        spec.urban = Some(Urban::Elevated);
        let street = |x: f64| (vec![[x, -300.0], [x, 300.0]], 6.0);
        let town = Town {
            streets: vec![street(50.0), street(1300.0)],
            areas: Vec::new(),
        };
        let mut notes = Vec::new();
        let stretches = spec.urban_stretches(&town, &mut notes).unwrap();
        // Two stretches, too far apart to be one; the first from the very start.
        assert_eq!(stretches.len(), 2, "{stretches:?}");
        assert_eq!(stretches[0].0, 0.0);
        assert!(stretches[1].0 > 800.0 && stretches[1].1 < 1800.0);
        assert_eq!(notes.len(), 1, "{notes:?}");
        // A built-up area counts as the town too, from inside it or near it.
        let town = Town {
            streets: Vec::new(),
            areas: vec![vec![
                [900.0, 30.0],
                [1100.0, 30.0],
                [1100.0, 200.0],
                [900.0, 200.0],
            ]],
        };
        let stretches = spec.urban_stretches(&town, &mut notes).unwrap();
        assert_eq!(stretches.len(), 1, "{stretches:?}");
    }

    /// The viaducts a corridor along y = 0, elevated from 200 m to 1400 m, asks
    /// for: each as the roads it carries.
    fn viaducts(shared_deck: bool, ramp: bool) -> Vec<Vec<String>> {
        let mut spec = CorridorSpec::new(
            "Shuto",
            vec![Point3::new(-800.0, 0.0, 0.0), Point3::new(900.0, 0.0, 0.0)],
            60.0,
        )
        .unwrap();
        spec.elevated = vec![(200.0, 1400.0)];
        spec.shared_deck = shared_deck;
        if ramp {
            // Off the right carriageway in the middle of the viaduct.
            spec.ramps.push(RampSite {
                name: "Off".into(),
                on: false,
                side: Side::Right,
                at: 800.0,
                end: Point3::new(100.0, -80.0, 9.0),
            });
        }
        let plan = spec.plan(TrafficHandedness::RightHand).unwrap();
        plan.structures
            .iter()
            .filter(|s| s.bridge == Some(BridgeForm::Viaduct))
            .map(|s| s.spans.iter().map(|(road, _, _)| road.clone()).collect())
            .collect()
    }

    #[test]
    fn each_carriageway_has_a_deck_of_its_own_unless_they_share_one() {
        assert_eq!(
            viaducts(false, false),
            vec![
                vec!["shuto_left_0".to_owned()],
                vec!["shuto_right_0".to_owned()]
            ]
        );
        assert_eq!(
            viaducts(true, false),
            vec![vec!["shuto_left_0".to_owned(), "shuto_right_0".to_owned()]]
        );
    }

    #[test]
    fn a_carriageway_broken_by_a_slip_road_shares_the_deck_of_the_unbroken_one() {
        assert_eq!(viaducts(false, true).len(), 3);
        assert_eq!(
            viaducts(true, true),
            vec![vec![
                "shuto_left_0".to_owned(),
                "shuto_right_0".to_owned(),
                "shuto_right_1".to_owned()
            ]]
        );
    }

    #[test]
    fn carriageways_broken_at_different_stations_share_a_deck_wherever_side_by_side() {
        let mut spec = CorridorSpec::new(
            "Shuto",
            vec![Point3::new(-800.0, 0.0, 0.0), Point3::new(900.0, 0.0, 0.0)],
            60.0,
        )
        .unwrap();
        spec.elevated = vec![(200.0, 1400.0)];
        spec.shared_deck = true;
        // Off the right carriageway at 700 m, and off the left at 1000 m.
        spec.ramps.push(RampSite {
            name: "West Off".into(),
            on: false,
            side: Side::Right,
            at: 700.0,
            end: Point3::new(0.0, -80.0, 9.0),
        });
        spec.ramps.push(RampSite {
            name: "East Off".into(),
            on: false,
            side: Side::Left,
            at: 1000.0,
            end: Point3::new(100.0, 80.0, 9.0),
        });
        let plan = spec.plan(TrafficHandedness::RightHand).unwrap();
        let roads = |name: &str| plan.roads.iter().find(|r| r.name == name).unwrap();
        // A span's stretch of the centre line, from the road's own points.
        let along = |(name, from, to): &(String, f64, f64)| {
            let road = roads(name);
            let x = |s: f64| {
                let mut walked = 0.0;
                for pair in road.points.windows(2) {
                    let step = pair[0].horizontal_distance_to(pair[1]);
                    if walked + step >= s {
                        return pair[0].x + (pair[1].x - pair[0].x) * (s - walked) / step;
                    }
                    walked += step;
                }
                road.points.last().unwrap().x
            };
            let (a, b) = (x(*from), x(*to));
            (a.min(b), a.max(b))
        };
        let decks: Vec<&StructurePlan> = plan
            .structures
            .iter()
            .filter(|s| s.bridge == Some(BridgeForm::Viaduct))
            .collect();
        let mut shared = 0;
        for deck in &decks {
            match deck.spans.as_slice() {
                [a, b] => {
                    // One of each carriageway, beside each other end to end
                    // but where one runs on past the other's slip-road junction.
                    assert_ne!(a.0.contains("left"), b.0.contains("left"), "{deck:?}");
                    let (a, b) = (along(a), along(b));
                    assert!(
                        (a.0 - b.0).abs() < GAP + 1.0 && (a.1 - b.1).abs() < GAP + 1.0,
                        "{a:?} {b:?}"
                    );
                    shared += 1;
                }
                // Alone only across from the other's slip-road junction.
                [one] => {
                    let (a, b) = along(one);
                    assert!(b - a < GAP + 1.0, "{one:?} on a deck of its own");
                }
                _ => panic!("{deck:?}"),
            }
        }
        // Before the first slip road, between the two, and after the second.
        assert_eq!(shared, 3, "{decks:#?}");
    }

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
    fn an_interchange_is_built_where_its_crossroad_crosses() {
        let mut spec = CorridorSpec::new(
            "E7",
            vec![
                Point3::new(-1500.0, 0.0, 0.0),
                Point3::new(1500.0, 0.0, 0.0),
            ],
            100.0,
        )
        .unwrap();
        // Said to be at 1000 m; the crossroad crosses at 1700 m, x = 200.
        let lane = |direction| LaneSpec::new(width(3.5), direction);
        spec.interchanges.push(InterchangeSite {
            name: "Higashi".into(),
            kind: InterchangeKind::Diamond,
            at: 1000.0,
            crossroad: vec![
                Point3::new(200.0, -500.0, 0.0),
                Point3::new(200.0, 500.0, 0.0),
            ],
            crossroad_lanes: vec![lane(Direction::Backward), lane(Direction::Forward)],
            over: true,
            control: Control::Signal,
            terminal: 80.0,
            reach: 320.0,
        });
        let plan = spec.plan(TrafficHandedness::RightHand).unwrap();
        assert!(
            plan.notes.iter().any(|note| note.contains("at 1700 m")),
            "{:?}",
            plan.notes
        );
        for road in plan
            .roads
            .iter()
            .filter(|r| r.name.starts_with("higashi_") && !r.name.contains("cross"))
        {
            for p in &road.points {
                assert!((p.x - 200.0).abs() < 400.0, "{} at {p:?}", road.name);
            }
        }
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
