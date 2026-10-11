//! The free-flow interchanges a corridor plans — the cloverleaf, the trumpet, the
//! directional T and the four-level stack — laid out quadrant by quadrant.
//!
//! Each side of the corridor is drawn in a frame of its own: `a` metres along the
//! traffic of that side's carriageway from where the crossroad crosses the centre
//! line, and `b` metres out from the centre line. In that frame the side looks the
//! same whichever side it is and whichever side traffic keeps to: the crossroad's
//! traffic heading out, away from the motorway, keeps to `a < 0`, the traffic
//! heading in to `a > 0`, and a slip road's lane lies on its left, so that a turn
//! to the left, anticlockwise, has the lane inside it and the reference line on its
//! outer edge. A loop's radius is what a loop is designed to — [`LOOP_RADIUS`]
//! along the middle of its lane — and every other turn of a slip road is as tight
//! as a slip road's design speed allows, so the layout is what the radii ask for.
//! The terminal distance only keeps a trumpet's junction on the near side, and the
//! end of a directional T's other motorway, from coming any nearer; a cloverleaf's
//! junctions and a trumpet's stem end are as near the motorway as the overpass
//! lets them be.
//!
//! The layout is square to the motorway: a crossroad that crosses on the slant
//! has its junctions where it reaches each distance out, and the slip roads still
//! leave and meet it as though it crossed square.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, SQRT_2};

use roadgen_core::builder::LaneSpec;
use roadgen_core::geometry::{Point3, VerticalProfile};
use roadgen_core::map::TrafficHandedness;
use roadgen_core::semantics::{FormOfWay, LaneType, RoadType};
use roadgen_core::structure::{BridgeForm, BridgeSpec, CLEARANCE, OVERPASS_DECK};
use roadgen_core::topology::{Direction, RoadEnd};

use super::corridor::{
    carriageway_width, length_of, link, one_way, Access, Carriageway, ConnectionPlan, CorridorPlan,
    CorridorSpec, DesignStandard, InterchangePlan, InterchangeSite, Join, RoadPlan, Shape, Side,
    StructurePlan, Track, GAP, LOOP_CLEARANCE, LOOP_RADIUS, STEP,
};

/// How far a collector–distributor road's reference line stands out from where
/// a slip road's would — half a metre clear of the carriageway — once it has
/// moved out: what keeps it apart from the through lanes, metres.
pub(super) const DRIFT: f64 = 2.5;
/// The straight from the end of a junction to where a slip road's own layout
/// begins, metres.
const LINK: f64 = 2.0;
/// The least length of a collector–distributor road from where it leaves the
/// carriageway to its first nose, and from its last nose to where it joins
/// again: the room to move out, and a deceleration lane.
const COLLECTOR_LEAD: f64 = 100.0;
/// How far a trumpet's semi-direct slip road runs on out from the stem before it
/// turns round the loop, metres.
const RISE: f64 = 12.0;

/// A side's frame: where the crossroad crosses, the way that side's carriageway's
/// traffic heads, and out from the centre line on that side.
#[derive(Debug, Clone, Copy)]
pub(super) struct Frame {
    origin: [f64; 2],
    u: [f64; 2],
    w: [f64; 2],
}

impl Frame {
    pub fn new(origin: [f64; 2], heading: f64, outward: [f64; 2]) -> Frame {
        Frame {
            origin,
            u: [heading.cos(), heading.sin()],
            w: outward,
        }
    }

    /// A point of the frame in the plane.
    pub fn place(&self, [a, b]: [f64; 2]) -> [f64; 2] {
        [
            self.origin[0] + self.u[0] * a + self.w[0] * b,
            self.origin[1] + self.u[1] * a + self.w[1] * b,
        ]
    }

    /// A point in the plane as a point of the frame.
    pub fn local(&self, p: [f64; 2]) -> [f64; 2] {
        let (x, y) = (p[0] - self.origin[0], p[1] - self.origin[1]);
        [x * self.u[0] + y * self.u[1], x * self.w[0] + y * self.w[1]]
    }

    /// A heading in the frame as a heading in the plane.
    pub fn heading(&self, local: f64) -> f64 {
        let (c, s) = (local.cos(), local.sin());
        let x = self.u[0] * c + self.w[0] * s;
        let y = self.u[1] * c + self.w[1] * s;
        y.atan2(x)
    }
}

/// A slip road's line in a side's frame, drawn as straights and arcs, and the way
/// it heads at its end.
#[derive(Debug, Clone)]
pub(super) struct Path {
    pub points: Vec<[f64; 2]>,
    pub heading: f64,
}

impl Path {
    pub fn from(a: f64, b: f64, heading: f64) -> Path {
        Path {
            points: vec![[a, b]],
            heading,
        }
    }

    pub fn end(&self) -> [f64; 2] {
        *self.points.last().expect("a start")
    }

    /// On straight ahead, a point every few metres.
    pub fn line(mut self, length: f64) -> Path {
        if length <= 1e-6 {
            return self;
        }
        let at = self.end();
        let count = (length / 5.0).ceil().max(1.0) as usize;
        let (c, s) = (self.heading.cos(), self.heading.sin());
        for k in 1..=count {
            let d = length * k as f64 / count as f64;
            self.points.push([at[0] + c * d, at[1] + s * d]);
        }
        self
    }

    /// Round an arc of `radius` through `sweep` radians, anticlockwise when
    /// positive, a point every metre or two so the curve fitted through them
    /// keeps the radius.
    pub fn arc(mut self, radius: f64, sweep: f64) -> Path {
        let at = self.end();
        let turn = sweep.signum();
        // The centre is to the left of travel for a turn to the left.
        let centre = [
            at[0] - turn * radius * self.heading.sin(),
            at[1] + turn * radius * self.heading.cos(),
        ];
        let count = (sweep.abs() * radius / 2.0).ceil().max(2.0) as usize;
        for k in 1..=count {
            let h = self.heading + sweep * k as f64 / count as f64;
            self.points.push([
                centre[0] + turn * radius * h.sin(),
                centre[1] - turn * radius * h.cos(),
            ]);
        }
        self.heading += sweep;
        self
    }

    /// The nearest it comes to the segment from `p` to `q`.
    fn distance_to(&self, p: [f64; 2], q: [f64; 2]) -> f64 {
        self.points
            .iter()
            .map(|x| segment_distance(*x, p, q))
            .fold(f64::INFINITY, f64::min)
    }
}

fn segment_distance(x: [f64; 2], p: [f64; 2], q: [f64; 2]) -> f64 {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let length = dx * dx + dy * dy;
    let t = if length < 1e-12 {
        0.0
    } else {
        (((x[0] - p[0]) * dx + (x[1] - p[1]) * dy) / length).clamp(0.0, 1.0)
    };
    (x[0] - p[0] - t * dx).hypot(x[1] - p[1] - t * dy)
}

/// What a side's layout is drawn to, metres.
#[derive(Debug, Clone)]
pub(super) struct Dims {
    /// The carriageway's outer edge, out from the centre line.
    pub edge: f64,
    /// A slip road's width, and how far the middle of its lane is in from its
    /// reference line.
    pub ramp_width: f64,
    pub lane_middle: f64,
    /// The radius of a slip road's turns and a loop's, along the middle of the
    /// lane.
    pub ramp_radius: f64,
    pub loop_radius: f64,
    /// The crossroad's width from its reference line on the side its outbound
    /// traffic keeps to, and on the inbound side.
    pub out: f64,
    pub inward: f64,
    /// The nearest to the centre line a junction on the crossroad may be: clear
    /// of the overpass and the land coming down from its abutments.
    pub least: f64,
    pub terminal: f64,
    pub reach: f64,
}

/// Which slip road a crossroad junction meets, and how.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Hookup {
    pub ramp: String,
    /// Whether it meets the crossroad's traffic heading out, away from the
    /// motorway, or heading in.
    pub outbound: bool,
    /// Whether it merges into that traffic, or diverges from it.
    pub merge: bool,
}

/// A junction on the crossroad: how far out, what meets it there, and whether the
/// crossroad stops at it.
#[derive(Debug, Clone)]
pub(super) struct CrossJunction {
    pub suffix: &'static str,
    pub b: f64,
    pub hookups: Vec<Hookup>,
    pub stop: bool,
}

/// What a slip road leaves or joins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Joins {
    Carriageway,
    Collector,
}

/// A slip road of a layout: its name after the side's, what it leaves or joins
/// and where, and its line in travel order, from the crossroad on a slip road on
/// and to it on one off.
#[derive(Debug, Clone)]
pub(super) struct RampLayout {
    pub suffix: String,
    pub joins: Joins,
    pub on: bool,
    /// The nose, as `a`.
    pub nose: f64,
    pub path: Path,
}

impl RampLayout {
    /// The slip road as an access to carriageway `carriageway`, whose travel
    /// station is `travel_at` where the crossroad crosses, laid out in `frame` and
    /// as high as the crossroad, `z`, where it meets it.
    fn access(
        &self,
        frame: &Frame,
        name: String,
        carriageway: usize,
        travel_at: f64,
        z: f64,
    ) -> Access {
        let lead: Vec<Point3> = self
            .path
            .points
            .iter()
            .map(|p| {
                let q = frame.place(*p);
                Point3::new(q[0], q[1], z)
            })
            .collect();
        // Its end at the carriageway runs with the traffic.
        let (end, end_heading) = if self.on {
            (lead[lead.len() - 1], frame.heading(self.path.heading))
        } else {
            (lead[0], frame.heading(0.0))
        };
        Access {
            ramp: name,
            carriageway,
            on: self.on,
            nose: travel_at + self.nose,
            end: [end.x, end.y],
            end_z: z,
            end_heading,
            shape: Shape::Lead(lead),
        }
    }
}

/// One side of a free-flow interchange.
#[derive(Debug, Clone)]
pub(super) struct SideLayout {
    pub ramps: Vec<RampLayout>,
    pub junctions: Vec<CrossJunction>,
    /// The collector–distributor road's ends, as `a`.
    pub collector: Option<(f64, f64)>,
}

impl Dims {
    /// A loop's reference line, its outer edge, about the loop's centre.
    fn loop_reference(&self) -> f64 {
        self.loop_radius + self.lane_middle
    }

    /// A slip road's reference line about the centre of a turn to the left, the
    /// lane inside it, and of a turn to the right, the lane outside it.
    fn left_turn(&self) -> f64 {
        self.ramp_radius + self.lane_middle
    }

    fn right_turn(&self) -> f64 {
        self.ramp_radius - self.lane_middle
    }

    /// Where the slip roads beside the crossroad run: half a metre clear of it,
    /// on each side, and no nearer its line than a crossroad of a lane and a
    /// pavement each way would put them.
    fn beside(&self) -> (f64, f64) {
        (-(self.out.max(6.0) + 0.5), self.inward.max(6.0) + 0.5)
    }
}

/// One side of a cloverleaf: a loop on before the crossroad and a loop off after
/// it, each with a slip road round its outside, all four off and on to a
/// collector–distributor road.
pub(super) fn cloverleaf(d: &Dims) -> Result<SideLayout, String> {
    let r = d.loop_reference();
    let ro = d.left_turn();
    // The collector–distributor road's reference line once it has moved out, and
    // the line its slip roads meet it on.
    let collector = d.edge + 0.5 + DRIFT;
    let merge = collector + d.ramp_width + 0.5;
    let (a_out, a_in) = d.beside();
    // The loops meet the crossroad a junction's length out from its junction,
    // which is as near the motorway as the overpass lets it be, the terminal
    // distance aside: farther out than a loop's half turn needs, a straight
    // towards the motorway between the turns takes up the room.
    let inner = d.least.max(merge + r - GAP / 2.0);
    let start = inner + GAP / 2.0;
    let straight = start - merge - r;
    let loop_on = Path::from(a_out, start, FRAC_PI_2)
        .arc(r, FRAC_PI_2)
        .arc(r, FRAC_PI_2)
        .line(straight)
        .arc(r, FRAC_PI_2);
    let loop_off = Path::from(a_in + r, merge, 0.0)
        .arc(r, FRAC_PI_2)
        .line(straight)
        .arc(r, FRAC_PI_2)
        .arc(r, FRAC_PI_2);
    // The slip roads outside the loops: a quarter turn in two halves with a
    // straight between, as far out as keeps them clear of the loop. The loop's
    // outer edge is its reference line, and the slip road's edge towards it is
    // the slip road's.
    let outer_off = |end: f64| {
        let delta = end - merge;
        Path::from(a_out - delta, merge, 0.0)
            .arc(ro, FRAC_PI_4)
            .line(SQRT_2 * (delta - ro))
            .arc(ro, FRAC_PI_4)
    };
    let outer_on = |end: f64| {
        let delta = end - merge;
        Path::from(a_in, end, 1.5 * PI)
            .arc(ro, FRAC_PI_4)
            .line(SQRT_2 * (delta - ro))
            .arc(ro, FRAC_PI_4)
    };
    let around_on = ([a_out - r, start], [a_out - r, start - straight]);
    let around_off = ([a_in + r, merge + r], [a_in + r, start]);
    let clear =
        |path: &Path, (p, q): ([f64; 2], [f64; 2])| path.distance_to(p, q) >= r + LOOP_CLEARANCE;
    // The outer junction a crossroad's piece of 10 m beyond the inner one at
    // least.
    let mut end = (merge + ro).max(inner + GAP + 10.0);
    while !(clear(&outer_off(end), around_on) && clear(&outer_on(end), around_off)) {
        end += 1.0;
    }
    let outer_off = outer_off(end);
    let outer_on = outer_on(end);
    let noses = [
        outer_off.points[0][0] - LINK - GAP / 2.0,
        loop_on.end()[0] + LINK + GAP / 2.0,
        loop_off.points[0][0] - LINK - GAP / 2.0,
        outer_on.end()[0] + LINK + GAP / 2.0,
    ];
    // Room on the collector–distributor road to move out and to slow down
    // before its first nose, and to speed up and move in after its last.
    let least = (GAP / 2.0 + COLLECTOR_LEAD + GAP / 2.0 + noses[3].max(-noses[0])).ceil();
    if d.reach < least {
        return Err(format!(
            "the collector–distributor roads of a cloverleaf need the room of a reach of \
             {least:.0} m or more"
        ));
    }
    let ramp = |suffix: &str, on: bool, nose: f64, path: Path| RampLayout {
        suffix: suffix.into(),
        joins: Joins::Collector,
        on,
        nose,
        path,
    };
    let hookup = |ramp: &str, outbound: bool, merge: bool| Hookup {
        ramp: ramp.into(),
        outbound,
        merge,
    };
    Ok(SideLayout {
        ramps: vec![
            ramp("off", false, noses[0], outer_off),
            ramp("loop_on", true, noses[1], loop_on),
            ramp("loop_off", false, noses[2], loop_off),
            ramp("on", true, noses[3], outer_on),
        ],
        junctions: vec![
            CrossJunction {
                suffix: "inner",
                b: inner,
                hookups: vec![
                    hookup("loop_on", true, false),
                    hookup("loop_off", false, true),
                ],
                stop: false,
            },
            CrossJunction {
                suffix: "outer",
                b: end + GAP / 2.0,
                hookups: vec![hookup("off", true, true), hookup("on", false, false)],
                stop: false,
            },
        ],
        collector: Some((-d.reach + GAP / 2.0, d.reach - GAP / 2.0)),
    })
}

/// How far out from the carriageway a quarter turn's slip road has drifted where
/// the turn begins, metres: a tenth of the way it runs beside the carriageway
/// from its nose, and no more than this.
const LIFT: f64 = 30.0;

/// The slip roads a quarter turn from the carriageway to the crossroad and back,
/// off before it to the traffic heading out and on after it from the traffic
/// heading in. Their noses are `reach` before and after the crossroad; each leaves
/// (or joins) its carriageway on a long easy S to where its turn begins, so it does
/// not run beside the through lanes all that way. Where they meet the crossroad,
/// a junction's length short of the junction, is `b` out at least.
fn quarter_turns(d: &Dims, b: f64) -> Result<(f64, RampLayout, RampLayout), String> {
    let ro = d.left_turn();
    let merge = d.edge + 0.5;
    let (a_out, a_in) = d.beside();
    let least = (GAP / 2.0 + LINK + ro + a_out.abs().max(a_in)).ceil();
    if d.reach < least {
        return Err(format!(
            "the slip roads need the room of a reach of {least:.0} m or more"
        ));
    }
    let lift = ((d.reach - least) * 0.1).min(LIFT);
    let b = b.max(merge + lift + ro);
    let straight = b - merge - lift - ro;
    let off = Path::from(a_out - ro, merge + lift, 0.0)
        .arc(ro, FRAC_PI_2)
        .line(straight);
    let on = Path::from(a_in, b, 1.5 * PI)
        .line(straight)
        .arc(ro, FRAC_PI_2);
    Ok((
        b,
        RampLayout {
            suffix: "off".into(),
            joins: Joins::Carriageway,
            on: false,
            nose: -d.reach,
            path: off,
        },
        RampLayout {
            suffix: "on".into(),
            joins: Joins::Carriageway,
            on: true,
            nose: d.reach,
            path: on,
        },
    ))
}

/// The side a trumpet's crossroad comes from: a slip road off merging into it and
/// one on diverging from it, at one junction.
pub(super) fn trumpet_near(d: &Dims) -> Result<SideLayout, String> {
    let b = d
        .least
        .max(d.terminal)
        .max(d.edge + 0.5 + d.left_turn() + GAP / 2.0);
    let (end, off, on) = quarter_turns(d, b - GAP / 2.0)?;
    Ok(SideLayout {
        ramps: vec![off, on],
        junctions: vec![CrossJunction {
            suffix: "near",
            b: end + GAP / 2.0,
            hookups: vec![
                Hookup {
                    ramp: "off".into(),
                    outbound: true,
                    merge: true,
                },
                Hookup {
                    ramp: "on".into(),
                    outbound: false,
                    merge: false,
                },
            ],
            stop: false,
        }],
        collector: None,
    })
}

/// The side a trumpet's stem ends on: it splits into a loop off the carriageway,
/// after the crossroad, and a semi-direct slip road on that runs out from the
/// stem, round the loop and back to the carriageway beyond it.
pub(super) fn trumpet_far(d: &Dims) -> Result<SideLayout, String> {
    let r = d.loop_reference();
    let ro = d.left_turn();
    let merge = d.edge + 0.5;
    let (a_out, a_in) = d.beside();
    let start = (d.least + GAP / 2.0).max(merge + r).max(merge + ro - RISE);
    let straight = start - merge - r;
    let looped = Path::from(a_in + r, merge, 0.0)
        .arc(r, FRAC_PI_2)
        .line(straight)
        .arc(r, FRAC_PI_2)
        .arc(r, FRAC_PI_2);
    let around = ([a_in + r, merge + r], [a_in + r, start]);
    let semi = |rs: f64| {
        Path::from(a_out, start, FRAC_PI_2)
            .line(RISE)
            .arc(rs, -PI)
            .line(start + RISE - merge - ro)
            .arc(ro, FRAC_PI_2)
    };
    // As tight round the loop as a slip road may turn, or wider to clear it.
    let mut rs = d.right_turn();
    while semi(rs).distance_to(around.0, around.1) < r + LOOP_CLEARANCE {
        rs += 1.0;
    }
    let semi = semi(rs);
    Ok(SideLayout {
        ramps: vec![
            RampLayout {
                suffix: "loop".into(),
                joins: Joins::Carriageway,
                on: false,
                nose: looped.points[0][0] - LINK - GAP / 2.0,
                path: looped,
            },
            RampLayout {
                suffix: "on".into(),
                joins: Joins::Carriageway,
                on: true,
                nose: semi.end()[0] + LINK + GAP / 2.0,
                path: semi,
            },
        ],
        junctions: vec![CrossJunction {
            suffix: "stem",
            b: start - GAP / 2.0,
            hookups: vec![
                Hookup {
                    ramp: "on".into(),
                    outbound: true,
                    merge: false,
                },
                Hookup {
                    ramp: "loop".into(),
                    outbound: false,
                    merge: true,
                },
            ],
            stop: true,
        }],
        collector: None,
    })
}

/// What [`CorridorSpec::free_flow`] wires up: an interchange's site, its layout,
/// and the crossroad's pieces.
pub(super) struct FreeFlow<'a> {
    pub site: &'a InterchangeSite,
    pub ic: &'a str,
    /// The crossroad's junctions by station: the side, and which of the side's.
    pub cuts: &'a [(f64, Side, usize)],
    pub layouts: &'a [(Side, SideLayout)],
    /// The crossroad's pieces between the junctions, those not built missing.
    pub cross_names: &'a [Option<String>],
    /// Where the crossroad crosses the centre line, along each.
    pub c_station: f64,
    pub centre_station: f64,
    pub centre_plan: [f64; 2],
    pub centre_heading: f64,
    pub track: &'a Track,
    pub cw_width: f64,
    pub outward: f64,
    pub ramp_standard: &'a DesignStandard,
    /// The crossroad's station that far to the left of the centre line, and the
    /// crossroad there, lifted or dropped.
    pub at_lateral: &'a dyn Fn(f64) -> Option<f64>,
    pub crossroad_point: &'a dyn Fn(f64) -> Point3,
    /// The interchange's roads so far: the crossroad's pieces.
    pub roads: Vec<String>,
}

/// The driving lanes of a cross-section that go `direction`, innermost first.
fn driving(lanes: &[LaneSpec], direction: Direction) -> Vec<usize> {
    lanes
        .iter()
        .enumerate()
        .filter(|(_, lane)| lane.direction == direction && lane.lane_type == LaneType::Driving)
        .map(|(index, _)| index)
        .collect()
}

impl CorridorSpec {
    /// Wires up a free-flowing interchange laid out side by side: the movements
    /// at each junction on the crossroad, every slip road as an access to its
    /// carriageway or collector–distributor road, and the collector–distributor
    /// roads themselves, as carriageways of their own.
    pub(super) fn free_flow(
        &self,
        cx: FreeFlow<'_>,
        plan: &mut CorridorPlan,
        carriageways: &mut Vec<Carriageway>,
        accesses: &mut Vec<Access>,
    ) -> Result<(), String> {
        let FreeFlow { site, ic, .. } = cx;
        let mut roads = cx.roads;
        let mut junctions = Vec::new();
        let forward = driving(&site.crossroad_lanes, Direction::Forward);
        let backward = driving(&site.crossroad_lanes, Direction::Backward);
        if forward.is_empty() || backward.is_empty() {
            return Err(format!(
                "{}: the crossroad needs a driving lane each way",
                site.name
            ));
        }
        let layout_of = |side: Side| {
            &cx.layouts
                .iter()
                .find(|(s, _)| *s == side)
                .expect("laid out")
                .1
        };
        // The movements at each junction on the crossroad: through it where the
        // crossroad goes on, and off and on to each slip road that meets it.
        for (index, &(s, side, k)) in cx.cuts.iter().enumerate() {
            let spot = &layout_of(side).junctions[k];
            let junction = format!("{ic}_{}_{}", side.as_str(), spot.suffix);
            plan.junctions.push(junction.clone());
            junctions.push(junction.clone());
            let (before, after) = (&cx.cross_names[index], &cx.cross_names[index + 1]);
            // Out, away from the corridor, is up the crossroad past the crossing.
            let outer_is_after = s > cx.c_station;
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
            let (out_lanes, in_lanes) = if outer_is_after {
                (&forward, &backward)
            } else {
                (&backward, &forward)
            };
            if let (Some(before), Some(after)) = (before, after) {
                plan.connections.push(ConnectionPlan::Ends {
                    a: before.clone(),
                    a_end: RoadEnd::End,
                    b: after.clone(),
                    b_end: RoadEnd::Start,
                    junction: junction.clone(),
                });
            }
            for hookup in &spot.hookups {
                let ramp = format!("{ic}_{}_{}", side.as_str(), hookup.ramp);
                let lanes = if hookup.outbound { out_lanes } else { in_lanes };
                // Where the crossroad goes on, a slip road meets its outermost lane
                // that way; where it stops, every lane.
                let lanes: Vec<usize> = if outer.is_some() {
                    lanes.last().copied().into_iter().collect()
                } else {
                    lanes.clone()
                };
                // Traffic heading out arrives on the inner piece and leaves on the
                // outer; heading in, the other way about.
                let (arrive, leave) = if hookup.outbound {
                    (Some(inner.clone()), outer.clone())
                } else {
                    (outer.clone(), Some(inner.clone()))
                };
                let piece = if hookup.merge { leave } else { arrive };
                let Some(piece) = piece else {
                    return Err(format!("{}: {ramp} has no crossroad to meet", site.name));
                };
                for lane in lanes {
                    let (from, to) = if hookup.merge {
                        ((ramp.clone(), 1), (piece.clone(), lane))
                    } else {
                        ((piece.clone(), lane), (ramp.clone(), 1))
                    };
                    plan.connections.push(ConnectionPlan::Lanes {
                        from,
                        to,
                        junction: junction.clone(),
                    });
                }
            }
        }

        // The slip roads, and the collector–distributor roads they leave and join.
        let length = cx.track.length();
        for (side, layout) in cx.layouts {
            let c = carriageways
                .iter()
                .position(|c| c.side == *side && !c.collector)
                .expect("both sides");
            let (along, offset) = (carriageways[c].along, carriageways[c].offset);
            let travel_at = if along {
                cx.centre_station
            } else {
                length - cx.centre_station
            };
            let heading = carriageways[c].heading(cx.track, cx.centre_station);
            let frame = Frame::new(
                cx.centre_plan,
                heading,
                [
                    -cx.centre_heading.sin() * side.sign(),
                    cx.centre_heading.cos() * side.sign(),
                ],
            );
            let collector = layout.collector.map(|(from, to)| {
                let name = format!("{ic}_{}_cd", side.as_str());
                let count = layout
                    .ramps
                    .iter()
                    .filter(|ramp| ramp.joins == Joins::Collector)
                    .count();
                // It leaves the carriageway as its first piece and joins it again
                // as its last, where a slip road would, half a metre clear of it.
                for (on, nose, piece) in [
                    (false, travel_at + from - GAP / 2.0, 0),
                    (true, travel_at + to + GAP / 2.0, count),
                ] {
                    accesses.push(Access {
                        ramp: format!("{name}_{piece}"),
                        carriageway: c,
                        on,
                        nose,
                        end: [0.0, 0.0],
                        end_z: 0.0,
                        end_heading: 0.0,
                        shape: Shape::Collector,
                    });
                }
                roads.extend((0..=count).map(|k| format!("{name}_{k}")));
                let travel_left = if along { 1.0 } else { -1.0 };
                carriageways.push(Carriageway {
                    side: *side,
                    along,
                    offset: offset + travel_left * cx.outward * (cx.cw_width + 0.5),
                    noses: Vec::new(),
                    standard: cx.ramp_standard.clone(),
                    lanes: 1,
                    span: (travel_at + from, travel_at + to),
                    name,
                    drift: DRIFT,
                    collector: true,
                });
                carriageways.len() - 1
            });
            for ramp in &layout.ramps {
                let name = format!("{ic}_{}_{}", side.as_str(), ramp.suffix);
                let points = &ramp.path.points;
                // At the crossroad, the slip road is as high as the crossroad.
                let at_cross = if ramp.on {
                    points[0]
                } else {
                    points[points.len() - 1]
                };
                let station = (cx.at_lateral)(side.sign() * at_cross[1]).ok_or_else(|| {
                    format!(
                        "{}: the crossroad does not reach {:.0} m to the {} of the corridor",
                        site.name,
                        at_cross[1],
                        side.as_str()
                    )
                })?;
                let z = (cx.crossroad_point)(station).z;
                let carriageway = match ramp.joins {
                    Joins::Carriageway => c,
                    Joins::Collector => collector.expect("a collector–distributor road"),
                };
                accesses.push(ramp.access(&frame, name.clone(), carriageway, travel_at, z));
                roads.push(name);
            }
        }
        plan.interchanges.push(InterchangePlan {
            name: site.name.clone(),
            kind: site.kind.as_str().into(),
            junctions,
            roads,
        });
        Ok(())
    }
}

/// What [`CorridorSpec::directional`] builds from: the site, where its crossroad
/// — the other motorway's line — crosses the centre line, and the corridor.
pub(super) struct Directional<'a> {
    pub site: &'a InterchangeSite,
    pub ic: &'a str,
    /// The side the other motorway comes from and ends on.
    pub near: Side,
    pub c_station: f64,
    pub centre_station: f64,
    pub cross_length: f64,
    pub track: &'a Track,
    /// The corridor's height at a centre-line station.
    pub z: &'a dyn Fn(f64) -> f64,
    /// The crossroad as drawn at one of its stations, and the ground there.
    pub point_at: &'a dyn Fn(f64) -> [f64; 2],
    pub ground_at: &'a dyn Fn(f64) -> f64,
    pub at_lateral: &'a dyn Fn(f64) -> Option<f64>,
    pub cw_width: f64,
    pub outward: f64,
    pub handedness: TrafficHandedness,
    pub ramp_standard: &'a DesignStandard,
}

/// How far out from the far carriageway's slip-road line a flyover comes down to
/// run beside it, before it eases in to its nose, metres.
const SWING: f64 = 10.0;
/// The steepest a flyover is laid out to climb: under a slip road's steepest, so
/// that the vertical curves stay within it.
const FLYOVER_GRADE: f64 = 0.06;
/// How far before and after what it crosses a flyover stays level, metres: room
/// for the vertical curves either side.
const LEVEL_MARGIN: f64 = 25.0;
/// How high above the straight between its ends a flyover is carried on a
/// viaduct rather than an embankment, metres.
const VIADUCT_HEIGHT: f64 = 5.0;

/// Plan stations along a polyline, from its start.
fn stations_of(points: &[Point3]) -> Vec<f64> {
    let mut out = vec![0.0];
    for pair in points.windows(2) {
        out.push(out[out.len() - 1] + pair[0].horizontal_distance_to(pair[1]));
    }
    out
}

/// Where two polylines cross in plan, as stations along each.
fn crossing(a: &[Point3], b: &[Point3]) -> Option<(f64, f64)> {
    let (sa, sb) = (stations_of(a), stations_of(b));
    for i in 0..a.len().saturating_sub(1) {
        for j in 0..b.len().saturating_sub(1) {
            let (p, q, r, t) = (a[i], a[i + 1], b[j], b[j + 1]);
            let (dx1, dy1) = (q.x - p.x, q.y - p.y);
            let (dx2, dy2) = (t.x - r.x, t.y - r.y);
            let denominator = dx1 * dy2 - dy1 * dx2;
            if denominator.abs() < 1e-12 {
                continue;
            }
            let (ex, ey) = (r.x - p.x, r.y - p.y);
            let u = (ex * dy2 - ey * dx2) / denominator;
            let v = (ex * dy1 - ey * dx1) / denominator;
            if (0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v) {
                return Some((
                    sa[i] + (sa[i + 1] - sa[i]) * u,
                    sb[j] + (sb[j + 1] - sb[j]) * v,
                ));
            }
        }
    }
    None
}

/// A flyover's heights: from `from` at its start up to `level` over the stretch
/// `level_at` of its stations, and down to `to` at its end, each grade rounded
/// with a vertical curve; and the two grades it climbs and descends at, infinite
/// where it has no room to.
fn flyover(
    stations: &[f64],
    from: f64,
    level: f64,
    level_at: (f64, f64),
    to: f64,
    curve: f64,
) -> Result<(Vec<f64>, f64, f64), String> {
    let total = stations[stations.len() - 1];
    let (lo, hi) = level_at;
    let grade = |rise: f64, run: f64| {
        if run > 1.0 {
            rise / run
        } else {
            f64::INFINITY
        }
    };
    let (climb, descent) = (grade(level - from, lo), grade(level - to, total - hi));
    if !(climb.is_finite() && descent.is_finite()) {
        return Ok((Vec::new(), climb, descent));
    }
    let profile = VerticalProfile::new(
        vec![(0.0, from), (lo, level), (hi, level), (total, to)],
        curve,
    )
    .map_err(|e| e.to_string())?;
    Ok((
        stations.iter().map(|s| profile.height(*s)).collect(),
        climb,
        descent,
    ))
}

impl CorridorSpec {
    /// Builds a directional T: the other motorway's two carriageways up to where
    /// they split and merge, the slip roads a quarter turn on to and off the near
    /// carriageway, and the two flyovers across the motorway to and from the far
    /// one, stacked where they cross — the slip road off the far carriageway a
    /// level over the one on to it — each carried on a viaduct where it stands
    /// high. The other motorway ends as far out as the flyovers need to come down
    /// to it at an easy grade.
    pub(super) fn directional(
        &self,
        cx: Directional<'_>,
        plan: &mut CorridorPlan,
        carriageways: &[Carriageway],
        accesses: &mut Vec<Access>,
    ) -> Result<(), String> {
        let Directional { site, ic, near, .. } = cx;
        let standard = &self.standard;
        let ramp = cx.ramp_standard;
        let far = match near {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        };
        let length = cx.track.length();
        let index_of = |side: Side| {
            carriageways
                .iter()
                .position(|c| c.side == side && !c.collector)
                .expect("both sides")
        };
        let (n, f) = (index_of(near), index_of(far));
        let travel = |c: &Carriageway, s: f64| if c.along { s } else { length - s };
        let near_at = travel(&carriageways[n], cx.centre_station);
        let far_at = travel(&carriageways[f], cx.centre_station);
        let (ox, oy, oh) = cx.track.at(cx.centre_station);
        let frame = Frame::new(
            [ox, oy],
            carriageways[n].heading(cx.track, cx.centre_station),
            [-oh.sin() * near.sign(), oh.cos() * near.sign()],
        );
        let median = standard.median / 2.0;
        let edge = median + cx.cw_width;
        let ramp_width = carriageway_width(ramp, 1);
        // The slip roads to and from the near carriageway leave the other
        // motorway outside the flyovers, 3 m clear of them.
        let beside = median + ramp_width + 3.0;
        let dims = Dims {
            edge,
            ramp_width,
            lane_middle: ramp.inner_shoulder + ramp.lane_width / 2.0,
            ramp_radius: ramp.min_radius,
            loop_radius: LOOP_RADIUS,
            out: beside,
            inward: beside,
            least: 0.0,
            terminal: site.terminal,
            reach: site.reach,
        };
        let rcw = dims.right_turn();
        let merge = edge + 0.5;
        // Where the flyovers turn: down the other motorway's line beside its
        // median, then a quarter turn to the right across the motorway, to come
        // out SWING beyond the far carriageway's slip-road line.
        let corner = rcw - merge - SWING;
        let motorway_z = (cx.z)(cx.centre_station);
        let level_on = motorway_z + CLEARANCE + OVERPASS_DECK + 0.5;
        let level_off = level_on + CLEARANCE + OVERPASS_DECK + 0.5;
        // Where the flyovers leave and join the far carriageway: the ends of the
        // junctions there, half a metre clear of it.
        let far_c = &carriageways[f];
        let nose_point = |t: f64| {
            let s = if far_c.along { t } else { length - t };
            let travel_left = if far_c.along { 1.0 } else { -1.0 };
            let p = cx.track.offset(
                s,
                far_c.offset + travel_left * cx.outward * (cx.cw_width + 0.5),
            );
            (
                Point3::new(p[0], p[1], (cx.z)(s)),
                far_c.heading(cx.track, s),
            )
        };
        let (join, join_heading) = nose_point(far_at + site.reach - GAP / 2.0);
        let (leave, leave_heading) = nose_point(far_at - site.reach + GAP / 2.0);
        // The other motorway at `b` out from the centre line: its station, and the
        // ground.
        let branch_at = |b: f64| {
            (cx.at_lateral)(near.sign() * b).ok_or_else(|| {
                format!(
                    "{}: the crossroad does not reach {b:.0} m to the {} of the corridor",
                    site.name,
                    near.as_str()
                )
            })
        };
        // The flyovers' lines, `b_end` out where they leave and join the other
        // motorway: the one on to the far carriageway, and the one off it.
        let lines = |b_end: f64| {
            let place = |path: &Path| -> Vec<Point3> {
                path.points
                    .iter()
                    .map(|p| {
                        let q = frame.place(*p);
                        Point3::new(q[0], q[1], 0.0)
                    })
                    .collect()
            };
            let on = Path::from(median, b_end, 1.5 * PI)
                .line(b_end - corner)
                .arc(rcw, -FRAC_PI_2);
            let mut on_points = place(&on);
            let last = on_points[on_points.len() - 1];
            on_points.extend(
                link(
                    [last.x, last.y],
                    frame.heading(on.heading),
                    0.0,
                    [join.x, join.y],
                    join_heading,
                    0.0,
                )
                .into_iter()
                .skip(1),
            );
            let off = Path::from(rcw - median, corner - rcw, PI)
                .arc(rcw, -FRAC_PI_2)
                .line(b_end - corner);
            let off_lead = place(&off);
            let mut off_points = link(
                [leave.x, leave.y],
                leave_heading,
                0.0,
                [off_lead[0].x, off_lead[0].y],
                frame.heading(PI),
                0.0,
            );
            off_points.extend(off_lead.into_iter().skip(1));
            (on_points, off_points)
        };
        // Where each must be at its level: over the carriageways, and where the
        // two cross.
        let level_at = |points: &[Point3], stations: &[f64], cross: f64| {
            let mut range = (cross, cross);
            for (p, s) in points.iter().zip(stations) {
                if cx.track.locate([p.x, p.y]).1.abs() < edge {
                    range = (range.0.min(*s), range.1.max(*s));
                }
            }
            (range.0 - LEVEL_MARGIN, range.1 + LEVEL_MARGIN)
        };
        // Farther out the other motorway ends, the longer the flyovers have to
        // climb from it: out as far as an easy grade needs.
        let (mut b_end, ..) = quarter_turns(&dims, site.terminal.max(corner + 10.0))?;
        let (on_points, on_z, off_points, off_z, ground) = loop {
            if b_end > 5000.0 {
                return Err(format!(
                    "{}: the flyovers cannot be laid out at an easy grade",
                    site.name
                ));
            }
            let ground = (cx.ground_at)(branch_at(b_end)?);
            let (on_points, off_points) = lines(b_end);
            let (on_stations, off_stations) = (stations_of(&on_points), stations_of(&off_points));
            let Some((on_cross, off_cross)) = crossing(&on_points, &off_points) else {
                return Err(format!("{}: the flyovers do not cross", site.name));
            };
            let curve = ramp.crest_radius;
            let (on_z, on_climb, on_descent) = flyover(
                &on_stations,
                ground,
                level_on,
                level_at(&on_points, &on_stations, on_cross),
                join.z,
                curve,
            )?;
            let (off_z, off_climb, off_descent) = flyover(
                &off_stations,
                leave.z,
                level_off,
                level_at(&off_points, &off_stations, off_cross),
                ground,
                curve,
            )?;
            // Grades either way: the branch's ground may stand above the flyovers'
            // levels, and a flyover down to it as steep as one up is as bad.
            let (on_climb, on_descent, off_climb, off_descent) = (
                on_climb.abs(),
                on_descent.abs(),
                off_climb.abs(),
                off_descent.abs(),
            );
            if on_descent > FLYOVER_GRADE || off_climb > FLYOVER_GRADE {
                let least = if on_descent.max(off_climb).is_finite() {
                    site.reach * on_descent.max(off_climb) / FLYOVER_GRADE
                } else {
                    site.reach * 2.0
                };
                return Err(format!(
                    "{}: the flyovers climb too steeply from the far carriageway; give it a \
                     reach of {least:.0} m or more",
                    site.name
                ));
            }
            if on_climb <= FLYOVER_GRADE && off_descent <= FLYOVER_GRADE {
                break (on_points, on_z, off_points, off_z, ground);
            }
            b_end += 10.0;
        };
        let (b_end, near_off, near_on) = quarter_turns(&dims, b_end)?;

        // The flyovers, on a viaduct where they stand high.
        let names = |side: Side, suffix: &str| format!("{ic}_{}_{suffix}", side.as_str());
        let (on_name, off_name) = (names(far, "on"), names(far, "off"));
        for (name, points, heights) in [
            (&on_name, &on_points, &on_z),
            (&off_name, &off_points, &off_z),
        ] {
            let stations = stations_of(points);
            let total = stations[stations.len() - 1];
            let (first, last) = (heights[0], heights[heights.len() - 1]);
            let high: Vec<f64> = stations
                .iter()
                .zip(heights)
                .filter(|(s, z)| **z - (first + (last - first) * **s / total) >= VIADUCT_HEIGHT)
                .map(|(s, _)| *s)
                .collect();
            if let (Some(from), Some(to)) = (high.first(), high.last()) {
                plan.structures.push(StructurePlan {
                    spans: vec![(name.clone(), *from, *to)],
                    bridge: Some(BridgeForm::Viaduct),
                    tunnel: None,
                });
            }
        }
        let with_heights = |points: &[Point3], heights: &[f64]| -> Vec<Point3> {
            points
                .iter()
                .zip(heights)
                .map(|(p, z)| Point3::new(p.x, p.y, *z))
                .collect()
        };
        for (name, on, nose, points) in [
            (
                &on_name,
                true,
                far_at + site.reach,
                with_heights(&on_points, &on_z),
            ),
            (
                &off_name,
                false,
                far_at - site.reach,
                with_heights(&off_points, &off_z),
            ),
        ] {
            accesses.push(Access {
                ramp: name.clone(),
                carriageway: f,
                on,
                nose,
                end: [0.0, 0.0],
                end_z: 0.0,
                end_heading: 0.0,
                shape: Shape::Whole(points),
            });
        }
        let (near_off_name, near_on_name) = (names(near, "off"), names(near, "on"));
        accesses.push(near_off.access(&frame, near_off_name.clone(), n, near_at, ground));
        accesses.push(near_on.access(&frame, near_on_name.clone(), n, near_at, ground));

        // The other motorway: its carriageways from where it was drawn from to
        // the junctions where they split and merge, a junction's length beyond
        // where the slip roads begin.
        let end_station = branch_at(b_end + GAP)?;
        let from_station = if branch_at(10.0)? > cx.c_station {
            cx.cross_length
        } else {
            0.0
        };
        let count = ((end_station - from_station).abs() / STEP).ceil().max(1.0) as usize;
        let u = frame.heading(0.0);
        let (inward_points, outward_points): (Vec<Point3>, Vec<Point3>) = (0..=count)
            .map(|k| {
                let s = from_station + (end_station - from_station) * k as f64 / count as f64;
                let p = (cx.point_at)(s);
                let ahead = (cx.point_at)((s + 1.0).min(cx.cross_length));
                let behind = (cx.point_at)((s - 1.0).max(0.0));
                let h = (ahead[1] - behind[1]).atan2(ahead[0] - behind[0]);
                // Square to the line, on the side the near carriageway's
                // traffic heads.
                let mut normal = [-h.sin(), h.cos()];
                if normal[0] * u.cos() + normal[1] * u.sin() < 0.0 {
                    normal = [-normal[0], -normal[1]];
                }
                let z = (cx.ground_at)(s);
                (
                    Point3::new(p[0] + normal[0] * median, p[1] + normal[1] * median, z),
                    Point3::new(p[0] - normal[0] * median, p[1] - normal[1] * median, z),
                )
            })
            .unzip();
        let mut outward_points = outward_points;
        outward_points.reverse();
        let branch_in = format!("{ic}_branch_in");
        let branch_out = format!("{ic}_branch_out");
        for (name, points) in [(&branch_in, inward_points), (&branch_out, outward_points)] {
            if length_of(&points) < 30.0 {
                return Err(format!(
                    "{}: the crossroad is too short for the junction, which is {:.0} m out",
                    site.name,
                    b_end + GAP / 2.0
                ));
            }
            plan.roads.push(RoadPlan {
                name: name.clone(),
                points,
                lanes: one_way(standard, self.lanes, None, cx.handedness),
                cross_sections: Vec::new(),
                road_type: RoadType::Motorway,
                form: FormOfWay::Motorway,
                speed_limit: Some(standard.design_speed),
                corridor: true,
            });
        }
        // Its inner lanes take the flyover, its outer the slip road to the near
        // carriageway: a lane in the middle of an odd number either.
        let lanes = self.lanes;
        let inner = 1..=lanes.div_ceil(2);
        let outer = lanes / 2 + 1..=lanes;
        let split = format!("{ic}_split");
        let merge_junction = format!("{ic}_merge");
        for (lanes, ramp) in [(inner.clone(), &on_name), (outer.clone(), &near_on_name)] {
            for k in lanes {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (branch_in.clone(), k),
                    to: (ramp.clone(), 1),
                    junction: split.clone(),
                });
            }
        }
        for (lanes, ramp) in [(inner, &off_name), (outer, &near_off_name)] {
            for k in lanes {
                plan.connections.push(ConnectionPlan::Lanes {
                    from: (ramp.clone(), 1),
                    to: (branch_out.clone(), k),
                    junction: merge_junction.clone(),
                });
            }
        }
        plan.junctions.push(split.clone());
        plan.junctions.push(merge_junction.clone());
        plan.interchanges.push(InterchangePlan {
            name: site.name.clone(),
            kind: site.kind.as_str().into(),
            junctions: vec![split, merge_junction],
            roads: vec![
                branch_in,
                branch_out,
                near_off_name,
                near_on_name,
                off_name,
                on_name,
            ],
        });
        Ok(())
    }
}

/// What [`CorridorSpec::stack`] builds from: the site, where its crossroad — the
/// other motorway's line — crosses the centre line, and the corridor.
pub(super) struct Stack<'a> {
    pub site: &'a InterchangeSite,
    pub ic: &'a str,
    pub centre_station: f64,
    pub track: &'a Track,
    /// The corridor's height at a centre-line station.
    pub z: &'a dyn Fn(f64) -> f64,
    pub cw_width: f64,
    pub outward: f64,
    pub handedness: TrafficHandedness,
    pub ramp_standard: &'a DesignStandard,
}

/// How far a slip road of a stack takes to move out from the carriageway it
/// leaves to the line it runs on beside it, and back in to the one it joins,
/// metres: an easy S.
const EASE: f64 = 100.0;
/// The room kept between two of a stack's slip roads side by side, and between
/// the two flyovers of a level where they pass through the middle, metres.
const APART: f64 = 3.0;
/// How far out from the middle a stack's flyover may cross anything, metres:
/// past it, nothing is looked for to keep it level over.
const MIDDLE: f64 = 300.0;

/// A carriageway as a stack reads it: the line it runs beside and its height
/// there, which way along the line its traffic goes, and how far to the left of
/// the line its reference line is.
struct Way<'a> {
    track: &'a Track,
    z: &'a dyn Fn(f64) -> f64,
    along: bool,
    offset: f64,
}

impl Way<'_> {
    /// A centre-line station as a travel station, or back.
    fn travel(&self, s: f64) -> f64 {
        if self.along {
            s
        } else {
            self.track.length() - s
        }
    }

    /// Where a slip road leaves or joins it at travel station `t`: half a metre
    /// clear of its edge, a carriageway `width` wide whose lanes lie `outward`
    /// of its reference line, at its height; and the way its traffic heads.
    fn beside(&self, t: f64, width: f64, outward: f64) -> (Point3, f64) {
        let s = self.travel(t);
        let travel_left = if self.along { 1.0 } else { -1.0 };
        let p = self
            .track
            .offset(s, self.offset + travel_left * outward * (width + 0.5));
        (Point3::new(p[0], p[1], (self.z)(s)), self.heading(s))
    }

    /// The way its traffic heads at centre-line station `s`.
    fn heading(&self, s: f64) -> f64 {
        let (_, _, h) = self.track.at(s);
        if self.along {
            h
        } else {
            h + PI
        }
    }
}

/// One of a stack's slip roads: its line, heights and all, from the nose it
/// leaves at to the nose it joins at, and the two noses — on the corridor's
/// carriageway, and on the other motorway's — as travel stations.
struct StackRamp {
    name: String,
    points: Vec<Point3>,
    flyover: bool,
    /// Whether it joins the corridor, from the other motorway, or leaves it.
    on: bool,
    carriageway: usize,
    nose: f64,
    other: Side,
    other_nose: f64,
}

impl CorridorSpec {
    /// Builds a four-level stack: the other motorway on through, a corridor of
    /// its own lifted a level over this one where it crosses, and the eight slip
    /// roads between the two. See [`InterchangeKind::Stack`].
    ///
    /// Each side is laid out in its frame, as a free-flow interchange's is, the
    /// other motorway running along `a = 0`. The flyovers turn round the middle
    /// on arcs whose centres stand on the diagonals, `c` out each way: the two
    /// of a level — on to each carriageway, or off each — pass either side of
    /// the middle, `d` from it, and cross the other level's two; each runs on
    /// beside both motorways `p` out from its line. The quarter turns are as
    /// much farther out as keeps them clear of the flyovers, and leave and join
    /// each motorway farther from the middle than the flyovers do, by a speed-
    /// change lane and an easy S: the outer turn leaves first and joins last,
    /// and nothing crosses it.
    ///
    /// [`InterchangeKind::Stack`]: super::corridor::InterchangeKind::Stack
    pub(super) fn stack(
        &self,
        cx: Stack<'_>,
        plan: &mut CorridorPlan,
        carriageways: &[Carriageway],
        accesses: &mut Vec<Access>,
    ) -> Result<(), String> {
        let Stack { site, ic, .. } = cx;
        let standard = &self.standard;
        let ramp = cx.ramp_standard;
        let median = standard.median / 2.0;
        let edge = median + cx.cw_width;
        // A level over another: the clearance under a viaduct's deck, and half a
        // metre to spare.
        let level = CLEARANCE + BridgeSpec::for_form(BridgeForm::Viaduct).deck_depth + 0.5;
        let (ox, oy, oh) = cx.track.at(cx.centre_station);

        // The other motorway: a corridor of this one's standard along the
        // crossroad, lifted where it crosses to a level over this one.
        let mut other = CorridorSpec::new(
            format!("{ic}_branch"),
            site.crossroad.clone(),
            standard.design_speed,
        )?;
        other.standard = standard.clone();
        other.lanes = self.lanes;
        other.structures = self.structures;
        other.shared_deck = self.shared_deck;
        let (other_track, other_profile) = other.alignment(&mut Vec::new())?;
        let other_length = other_track.length();
        let (across, lateral) = other_track.locate([ox, oy]);
        if lateral.abs() > 1.0 {
            return Err(format!(
                "{}: the crossroad does not cross the corridor",
                site.name
            ));
        }
        // Lifted to a level over the corridor where the two cross: over the
        // corridor's highest under the deck, from the crossroad's lowest there,
        // both on their grades.
        let deck = edge + 5.0;
        let highest = |z: &dyn Fn(f64) -> f64, from: f64, to: f64| {
            let steps = ((to - from) / 5.0).ceil().max(1.0) as usize;
            (0..=steps)
                .map(|k| z(from + (to - from) * k as f64 / steps as f64))
                .fold(f64::NEG_INFINITY, f64::max)
        };
        let under = highest(cx.z, cx.centre_station - deck, cx.centre_station + deck);
        let low = -highest(
            &|s: f64| -other_profile.height(s),
            across - deck,
            across + deck,
        );
        let lift = under + level - low;
        if lift < 0.0 {
            return Err(format!(
                "{}: the crossroad of a stack stands {:.1} m over the corridor where they                  cross, higher than the level the stack lifts it to; give it a profile                  nearer the corridor's",
                site.name,
                low - under
            ));
        }
        // Always on a bridge where it crosses, however little it is lifted: the
        // bridge is what says which of the two is carried over the other.
        let lift = lift.max(0.5);
        let half = edge + 10.0 + other.lift(lift);
        other.elevated = vec![(across - half, across + half)];
        other.elevated_height = lift;
        let other_z = |s: f64| other.height_at(&other_profile, s);
        let forward_right = cx.handedness == TrafficHandedness::RightHand;
        let other_way = |side: Side| Way {
            track: &other_track,
            z: &other_z,
            along: (side == Side::Right) == forward_right,
            offset: side.sign() * median,
        };
        let way = |c: usize| Way {
            track: cx.track,
            z: cx.z,
            along: carriageways[c].along,
            offset: carriageways[c].offset,
        };
        let index_of = |side: Side| {
            carriageways
                .iter()
                .position(|c| c.side == side && !c.collector)
                .expect("both sides")
        };
        let opposite = |side: Side| match side {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        };

        // The layout's dimensions, in a side's frame. A flyover's reference line
        // is on the inside of its turn, its lane outside, towards the middle.
        let ramp_width = carriageway_width(ramp, 1);
        let lane_middle = ramp.inner_shoulder + ramp.lane_width / 2.0;
        let d = ramp_width + APART;
        let radius = ((edge + 0.5 + SWING + d / SQRT_2) / (1.0 - 1.0 / SQRT_2))
            .max(ramp.min_radius - lane_middle);
        let c = (radius + d) / SQRT_2;
        let p = radius - c;
        let q = p + ramp_width + APART;
        let ro = ramp.min_radius + lane_middle;
        // The quarter turns leave and join a speed-change lane and an S farther
        // out than the flyovers.
        let spacing =
            GAP + standard.acceleration.max(standard.deceleration) + standard.taper + EASE + 10.0;

        // The layout for the flyovers' noses `reach` from the middle along each
        // motorway: nothing if they are too near to fit, or to climb at an easy
        // grade.
        let layout = |reach: f64| -> Result<Option<Vec<StackRamp>>, String> {
            let reach_q = reach + spacing;
            let mut ramps = Vec::new();
            for side in [Side::Left, Side::Right] {
                let far = opposite(side);
                let (n, f) = (index_of(side), index_of(far));
                let frame = Frame::new(
                    [ox, oy],
                    carriageways[n].heading(cx.track, cx.centre_station),
                    [-oh.sin() * side.sign(), oh.cos() * side.sign()],
                );
                // The other motorway's carriageway heading in towards the
                // corridor on this side, and the one heading out.
                let heads_in = |s: Side| {
                    let heading = other_way(s).heading(across);
                    heading.cos() * frame.w[0] + heading.sin() * frame.w[1] < 0.0
                };
                let (inbound, outbound) = if heads_in(Side::Left) {
                    (Side::Left, Side::Right)
                } else {
                    (Side::Right, Side::Left)
                };
                let (b_in, b_out) = (other_way(inbound), other_way(outbound));
                let (t_in, t_out) = (b_in.travel(across), b_out.travel(across));
                let (a_near, a_far) = (way(n), way(f));
                let (t_near, t_far) = (
                    a_near.travel(cx.centre_station),
                    a_far.travel(cx.centre_station),
                );
                let width = cx.cw_width;
                // An off slip road starts at the end of its junction, an on one
                // ends at its start.
                let leave = |w: &Way, t: f64| w.beside(t + GAP / 2.0, width, cx.outward);
                let join = |w: &Way, t: f64| w.beside(t - GAP / 2.0, width, cx.outward);
                let local = |end: &(Point3, f64)| frame.local([end.0.x, end.0.y]);
                // The line from `start` along `path` to `end`, eased in and out.
                let whole = |start: (Point3, f64), from: f64, path: &Path, end: (Point3, f64)| {
                    let placed: Vec<[f64; 2]> =
                        path.points.iter().map(|x| frame.place(*x)).collect();
                    let mut out = link(
                        [start.0.x, start.0.y],
                        start.1,
                        0.0,
                        placed[0],
                        frame.heading(from),
                        0.0,
                    );
                    out.extend(placed[1..].iter().map(|x| Point3::new(x[0], x[1], 0.0)));
                    out.extend(
                        link(
                            placed[placed.len() - 1],
                            frame.heading(path.heading),
                            0.0,
                            [end.0.x, end.0.y],
                            end.1,
                            0.0,
                        )
                        .into_iter()
                        .skip(1),
                    );
                    // As high as the carriageways at the noses; the heights
                    // between are worked out below.
                    let last = out.len() - 1;
                    out[0].z = start.0.z;
                    out[last].z = end.0.z;
                    out
                };

                // The flyover from the other motorway on to the far carriageway:
                // down beside it, round the middle, and on beside the far
                // carriageway.
                let start = leave(&b_in, t_in - reach);
                let end = join(&a_far, t_far + reach);
                let (lead_b, lead_a) = (local(&start)[1] - EASE, local(&end)[0] + EASE);
                if lead_b < c || lead_a > -c {
                    return Ok(None);
                }
                let path = Path::from(p, lead_b, 1.5 * PI)
                    .line(lead_b - c)
                    .arc(radius, -FRAC_PI_2)
                    .line(-c - lead_a);
                ramps.push(StackRamp {
                    name: format!("{ic}_{}_flyover_on", far.as_str()),
                    points: whole(start, 1.5 * PI, &path, end),
                    flyover: true,
                    on: true,
                    carriageway: f,
                    nose: t_far + reach,
                    other: inbound,
                    other_nose: t_in - reach,
                });
                // And the one off the far carriageway: in from beside it, round
                // the middle, and on out beside the other motorway.
                let start = leave(&a_far, t_far - reach);
                let end = join(&b_out, t_out + reach);
                let (lead_a, lead_b) = (local(&start)[0] - EASE, local(&end)[1] - EASE);
                if lead_a < c || lead_b < c {
                    return Ok(None);
                }
                let path = Path::from(lead_a, -p, PI)
                    .line(lead_a - c)
                    .arc(radius, -FRAC_PI_2)
                    .line(lead_b - c);
                ramps.push(StackRamp {
                    name: format!("{ic}_{}_flyover_off", far.as_str()),
                    points: whole(start, PI, &path, end),
                    flyover: true,
                    on: false,
                    carriageway: f,
                    nose: t_far - reach,
                    other: outbound,
                    other_nose: t_out + reach,
                });
                // The quarter turns: off the near carriageway out beside the
                // other motorway, and in from it on to the near carriageway.
                let start = leave(&a_near, t_near - reach_q);
                let end = join(&b_out, t_out + reach_q);
                let (lead_a, lead_b) = (local(&start)[0] + EASE, local(&end)[1] - EASE);
                if lead_a > -q - ro || lead_b < q + ro {
                    return Ok(None);
                }
                let path = Path::from(lead_a, q, 0.0)
                    .line(-q - ro - lead_a)
                    .arc(ro, FRAC_PI_2)
                    .line(lead_b - q - ro);
                ramps.push(StackRamp {
                    name: format!("{ic}_{}_off", side.as_str()),
                    points: whole(start, 0.0, &path, end),
                    flyover: false,
                    on: false,
                    carriageway: n,
                    nose: t_near - reach_q,
                    other: outbound,
                    other_nose: t_out + reach_q,
                });
                let start = leave(&b_in, t_in - reach_q);
                let end = join(&a_near, t_near + reach_q);
                let (lead_b, lead_a) = (local(&start)[1] - EASE, local(&end)[0] - EASE);
                if lead_b < q + ro || lead_a < q + ro {
                    return Ok(None);
                }
                let path = Path::from(q, lead_b, 1.5 * PI)
                    .line(lead_b - q - ro)
                    .arc(ro, FRAC_PI_2)
                    .line(lead_a - q - ro);
                ramps.push(StackRamp {
                    name: format!("{ic}_{}_on", side.as_str()),
                    points: whole(start, 1.5 * PI, &path, end),
                    flyover: false,
                    on: true,
                    carriageway: n,
                    nose: t_near + reach_q,
                    other: inbound,
                    other_nose: t_in - reach_q,
                });
            }

            // The heights. A flyover is level over both motorways and where it
            // crosses another flyover — on to the corridor the third level, off
            // it the fourth — and climbs and descends each side; a quarter turn
            // is at one grade all the way.
            let lines: Vec<Vec<Point3>> = ramps.iter().map(|r| r.points.clone()).collect();
            // The second level, the one the flyovers stand over: the other
            // motorway's, at its highest where a flyover crosses it, and a level
            // over the corridor's highest where one crosses that. On a grade a
            // motorway stands higher where a flyover crosses it than where the
            // two motorways cross, by more than a level has to spare.
            let mut second = under + level;
            for ramp_line in ramps.iter().filter(|ramp| ramp.flyover) {
                for point in &ramp_line.points {
                    if (point.x - ox).hypot(point.y - oy) > MIDDLE {
                        continue;
                    }
                    let (s, off) = cx.track.locate([point.x, point.y]);
                    if off.abs() < edge + 3.0 {
                        second = second.max((cx.z)(s) + level);
                    }
                    let (s, off) = other_track.locate([point.x, point.y]);
                    if off.abs() < edge + 3.0 {
                        second = second.max(other_z(s));
                    }
                }
            }
            for (index, ramp_line) in ramps.iter_mut().enumerate() {
                let stations = stations_of(&ramp_line.points);
                let (from, to) = (
                    ramp_line.points[0].z,
                    ramp_line.points[ramp_line.points.len() - 1].z,
                );
                if !ramp_line.flyover {
                    let total = stations[stations.len() - 1];
                    if ((to - from) / total).abs() > ramp.max_grade {
                        return Ok(None);
                    }
                    for (point, s) in ramp_line.points.iter_mut().zip(&stations) {
                        point.z = from + (to - from) * s / total;
                    }
                    continue;
                }
                let mut range: Option<(f64, f64)> = None;
                let mut widen = |s: f64| {
                    range = Some(range.map_or((s, s), |(lo, hi)| (lo.min(s), hi.max(s))));
                };
                for (point, s) in ramp_line.points.iter().zip(&stations) {
                    if (point.x - ox).hypot(point.y - oy) > MIDDLE {
                        continue;
                    }
                    let near_line =
                        |track: &Track| track.locate([point.x, point.y]).1.abs() < edge + 3.0;
                    if near_line(cx.track) || near_line(&other_track) {
                        widen(*s);
                    }
                }
                for (other_index, other_line) in lines.iter().enumerate() {
                    if other_index == index {
                        continue;
                    }
                    if let Some((s, _)) = crossing(&lines[index], other_line) {
                        widen(s);
                    }
                }
                let Some((lo, hi)) = range else {
                    return Err(format!(
                        "{}: a flyover of the stack crosses neither motorway",
                        site.name
                    ));
                };
                let top = second + level * if ramp_line.on { 1.0 } else { 2.0 };
                let (heights, climb, descent) = flyover(
                    &stations,
                    from,
                    top,
                    (lo - LEVEL_MARGIN, hi + LEVEL_MARGIN),
                    to,
                    ramp.crest_radius,
                )?;
                if climb.abs().max(descent.abs()) > FLYOVER_GRADE || heights.is_empty() {
                    return Ok(None);
                }
                for (point, z) in ramp_line.points.iter_mut().zip(heights) {
                    point.z = z;
                }
            }
            // The other motorway reaches far enough for the quarter turns, and
            // is lifted and back down again within it.
            let needed = (reach_q + GAP / 2.0 + 30.0).max(half);
            if across < needed || other_length - across < needed {
                return Err(format!(
                    "{}: the crossroad of a stack needs to reach {needed:.0} m each side of the \
                     corridor",
                    site.name
                ));
            }
            Ok(Some(ramps))
        };
        let mut reach = site.reach;
        let ramps = loop {
            if reach > 5000.0 {
                return Err(format!(
                    "{}: the flyovers of the stack cannot be laid out at an easy grade",
                    site.name
                ));
            }
            match layout(reach)? {
                Some(ramps) if reach == site.reach => break ramps,
                Some(_) => {
                    return Err(format!(
                        "{}: the flyovers of a stack need room to climb; give it a reach of \
                         {reach:.0} m or more",
                        site.name
                    ))
                }
                None => reach += 10.0,
            }
        };

        // The flyovers, on a viaduct where they stand high; and every slip road
        // as an access to the corridor's carriageway and a join to the other
        // motorway's.
        let mut joins = Vec::new();
        let mut roads = Vec::new();
        for ramp_line in ramps {
            let StackRamp {
                name,
                points,
                flyover,
                on,
                carriageway,
                nose,
                other: other_side,
                other_nose,
            } = ramp_line;
            if flyover {
                let stations = stations_of(&points);
                let total = stations[stations.len() - 1];
                let (first, last) = (points[0].z, points[points.len() - 1].z);
                let high: Vec<f64> = stations
                    .iter()
                    .zip(&points)
                    .filter(|(s, p)| p.z - (first + (last - first) * **s / total) >= VIADUCT_HEIGHT)
                    .map(|(s, _)| *s)
                    .collect();
                if let (Some(from), Some(to)) = (high.first(), high.last()) {
                    plan.structures.push(StructurePlan {
                        spans: vec![(name.clone(), *from, *to)],
                        bridge: Some(BridgeForm::Viaduct),
                        tunnel: None,
                    });
                }
            }
            joins.push(Join {
                ramp: name.clone(),
                side: other_side,
                on: !on,
                nose: other_nose,
            });
            accesses.push(Access {
                ramp: name.clone(),
                carriageway,
                on,
                nose,
                end: [0.0, 0.0],
                end_z: 0.0,
                end_heading: 0.0,
                shape: Shape::Whole(points),
            });
            roads.push(name);
        }

        // The other motorway, its carriageways split where the slip roads leave
        // and join, each with its speed-change lane.
        let other_plan = other.plan_joined(cx.handedness, &joins)?;
        let mut ic_roads: Vec<String> = other_plan.roads.iter().map(|r| r.name.clone()).collect();
        ic_roads.extend(roads);
        plan.roads.extend(other_plan.roads);
        plan.junctions.extend(other_plan.junctions.iter().cloned());
        plan.connections.extend(other_plan.connections);
        plan.controls.extend(other_plan.controls);
        plan.structures.extend(other_plan.structures);
        plan.fitted.extend(other_plan.fitted);
        plan.anchors.extend(other_plan.anchors);
        plan.notes.extend(other_plan.notes);
        plan.interchanges.push(InterchangePlan {
            name: site.name.clone(),
            kind: site.kind.as_str().into(),
            junctions: other_plan.junctions,
            roads: ic_roads,
        });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dims() -> Dims {
        Dims {
            edge: 13.0,
            ramp_width: 5.25,
            lane_middle: 2.25,
            ramp_radius: 60.0,
            loop_radius: 30.0,
            out: 6.0,
            inward: 6.0,
            least: 52.0,
            terminal: 80.0,
            reach: 320.0,
        }
    }

    #[test]
    fn an_arc_keeps_its_radius_and_turns_the_way_asked() {
        let path = Path::from(0.0, 0.0, 0.0).arc(10.0, FRAC_PI_2);
        let [a, b] = path.end();
        assert!(
            (a - 10.0).abs() < 1e-9 && (b - 10.0).abs() < 1e-9,
            "{a} {b}"
        );
        let path = Path::from(0.0, 0.0, 0.0).arc(10.0, -FRAC_PI_2);
        let [a, b] = path.end();
        assert!(
            (a - 10.0).abs() < 1e-9 && (b + 10.0).abs() < 1e-9,
            "{a} {b}"
        );
        assert!((path.heading + FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn a_cloverleafs_slip_roads_meet_the_crossroad_square_and_the_collector_level() {
        let d = dims();
        let side = cloverleaf(&d).unwrap();
        let merge = d.edge + 0.5 + DRIFT + d.ramp_width + 0.5;
        for ramp in &side.ramps {
            // The end at the collector–distributor road runs along it.
            let (at, heading) = if ramp.on {
                (ramp.path.end(), ramp.path.heading)
            } else {
                (ramp.path.points[0], 0.0)
            };
            assert!((at[1] - merge).abs() < 1e-6, "{}: {at:?}", ramp.suffix);
            assert!(heading.sin().abs() < 1e-9, "{}", ramp.suffix);
        }
        // Loop on before the loop off, each 30 m clear of the other's nose.
        let [off, loop_on, loop_off, on] = [0, 1, 2, 3].map(|k| side.ramps[k].nose);
        assert!(off < loop_on && loop_on + GAP + 30.0 <= loop_off && loop_off < on);
    }

    #[test]
    fn a_trumpets_semi_direct_slip_road_goes_round_the_loop() {
        let d = dims();
        let side = trumpet_far(&d).unwrap();
        let [looped, semi] = [&side.ramps[0], &side.ramps[1]];
        // It ends heading the way the carriageway's traffic does, beyond the
        // loop's nose.
        assert!(semi.path.heading.rem_euclid(2.0 * PI) < 1e-9);
        assert!(semi.nose > looped.nose + 100.0);
        // And it turned right half round, then left a quarter.
        assert!((semi.path.heading - (FRAC_PI_2 - PI + FRAC_PI_2)).abs() < 1e-9);
    }
}
