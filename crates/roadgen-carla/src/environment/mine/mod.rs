//! An open-pit mine: the [`Mine`](super::Environment::Mine) branch of a CARLA map's
//! environment.
//!
//! What it fills is the part of the IR a road network alone leaves empty — the shape
//! of the land, what the land is, and the place they add up to — and the roads that
//! the place needs. It writes no file: the package is written from the IR like any
//! other map's, and so is every other format.
//!
//! ```text
//!   MineSpec + seed
//!        │
//!   lay_out(&mut MapBuilder)     the roads: the ramp down the pit wall, the haul
//!        │                       roads to the dump and the stockpile, the junction
//!   MapBuilder::finish()
//!        │
//!   generate(&mut UnvalidatedMap) the land: benches, heaps, ground, as one TIN with
//!        │                        its breaklines; the land covers; the berms; the site
//!   validate()
//! ```
//!
//! Two calls rather than one because the land is built round the roads' *generated*
//! edges — the TIN takes them as breaklines, so the road and the ground under it
//! agree to the vertex — and a road has no generated edges until the builder has
//! finished.
//!
//! # How a pit is made
//!
//! A pit is designed outwards from its floor. The floor is an ellipse; each bench
//! is the ring of the floor pushed out along its normal — a face rising `bench_height`
//! at `face_angle`, from its toe to its crest, then a bench `bench_width` wide, then
//! the next face. Parallel curves of a convex outline never fold, so the rings stack
//! without having to be checked against each other.
//!
//! The ramp is a road that starts at the rim and spirals down the wall at a constant
//! grade. At each height it sits where the wall, on average, is at that height, so
//! the ground rises on its outer side and falls on its inner side — it is cut into
//! the wall above and built out over the bench below, which is what a ramp is. The
//! benches it passes through are interrupted where it passes: a breakline cannot run
//! through a road.
//!
//! # A site is a pure function
//!
//! A [`MineSpec`] with its seed is a mine, every time; the identifiers are derived
//! from the site's name and what each feature is, never from a counter.

pub mod geometry;

use std::f64::consts::TAU;
use std::fmt;

use roadgen_core::builder::{LaneSpec, MapBuilder, RoadSpec};
use roadgen_core::error::{BuildError, GeometryError};
use roadgen_core::geometry::{Curve3, Point3};
use roadgen_core::id::{JunctionId, ObjectId, RoadId, SiteId, TinSurfaceId};
use roadgen_core::map::Lane;
use roadgen_core::semantics::{
    BoundaryMarking, LaneType, MapObject, MapObjectKind, MarkingColor, ObjectGeometry, Pavement,
    PavementProfile, RoadMarking, RoadType, UnpavedSurface,
};
use roadgen_core::site::Site;
use roadgen_core::terrain::TinError;
use roadgen_core::topology::{Direction, LateralSide, RoadEnd};
use roadgen_core::units::{PositiveWidth, SpeedLimit};
use roadgen_core::validation::UnvalidatedMap;

use self::geometry::{Ellipse, Frame};
use super::land::{self, plan_of, smoothstep, Base, Land, Landforms, Level, Line};

/// The pit: its floor, and the benches stepped up from it to the rim.
#[derive(Debug, Clone, PartialEq)]
pub struct PitSpec {
    /// The floor's length along the pit's long axis, and its width across it.
    pub floor_length: f64,
    pub floor_width: f64,
    /// Benches from the floor to the rim. The pit is this many bench heights deep.
    pub benches: usize,
    pub bench_height: f64,
    /// How wide each bench is, from the crest of the face below to the toe of the
    /// face above.
    pub bench_width: f64,
    /// How steep each face is, degrees from the horizontal.
    pub face_angle: f64,
}

impl Default for PitSpec {
    fn default() -> Self {
        PitSpec {
            floor_length: 220.0,
            floor_width: 140.0,
            benches: 6,
            bench_height: 12.0,
            bench_width: 10.0,
            face_angle: 70.0,
        }
    }
}

/// The ramp down the pit wall.
#[derive(Debug, Clone, PartialEq)]
pub struct RampSpec {
    /// Rise over run: a haul ramp is usually between 0.08 and 0.10.
    pub grade: f64,
    /// Where on the rim it starts, degrees anticlockwise round the pit from the end
    /// of its long axis.
    pub start: f64,
    /// Which way round it winds on the way down, seen from above.
    pub clockwise: bool,
}

impl Default for RampSpec {
    fn default() -> Self {
        RampSpec {
            grade: 0.1,
            start: -90.0,
            clockwise: true,
        }
    }
}

/// The cross-section every haul road is built to, the ramp included.
#[derive(Debug, Clone, PartialEq)]
pub struct HaulRoadSpec {
    /// Each of the two driving lanes.
    pub lane_width: f64,
    /// Each of the two shoulders, outside the lanes. The berm stands on one.
    pub shoulder_width: f64,
    /// How high the safety berm stands on the pit side of the ramp: about half the
    /// height of the largest wheel that uses the road, by the usual rule.
    pub berm_height: f64,
    pub speed_limit_kph: f64,
    pub surface: Pavement,
}

impl Default for HaulRoadSpec {
    fn default() -> Self {
        HaulRoadSpec {
            lane_width: 7.5,
            shoulder_width: 3.0,
            berm_height: 1.5,
            speed_limit_kph: 40.0,
            surface: Pavement::Unpaved(UnpavedSurface::Gravel),
        }
    }
}

impl HaulRoadSpec {
    fn half_width(&self) -> f64 {
        self.lane_width + self.shoulder_width
    }
}

/// A heap of broken rock: the waste dump or the stockpile. A frustum, its sides at
/// the material's angle of repose.
#[derive(Debug, Clone, PartialEq)]
pub struct HeapSpec {
    /// Degrees anticlockwise from the pit's long axis, seen from the junction at
    /// the top of the ramp.
    pub bearing: f64,
    /// From that junction to the heap's centre, metres.
    pub distance: f64,
    /// The heap's footprint: the radius of its toe.
    pub radius: f64,
    pub height: f64,
    /// Degrees from the horizontal.
    pub repose: f64,
}

pub use super::land::GroundSpec;

/// Everything an open-pit mine is generated from.
#[derive(Debug, Clone, PartialEq)]
pub struct MineSpec {
    /// The site's name, which every identifier it makes is named after.
    pub name: String,
    /// The centre of the pit's floor in plan, at the height of the rim and of the
    /// country round it.
    pub centre: Point3,
    /// Radians anticlockwise from `+x` to the pit's long axis.
    pub heading: f64,
    pub pit: PitSpec,
    pub ramp: RampSpec,
    pub haul_road: HaulRoadSpec,
    pub waste_dump: HeapSpec,
    pub stockpile: HeapSpec,
    pub ground: GroundSpec,
    /// What stands on the yard beside the waste dump's haul road — the workshops,
    /// the offices — from the same shape grammar a town's buildings come from, or
    /// nothing.
    pub buildings: Option<roadgen_buildings::Rules>,
    pub seed: u64,
}

impl Default for MineSpec {
    fn default() -> Self {
        MineSpec {
            name: "mine".into(),
            centre: Point3::ORIGIN,
            heading: 0.0,
            pit: PitSpec::default(),
            ramp: RampSpec::default(),
            haul_road: HaulRoadSpec::default(),
            waste_dump: HeapSpec {
                bearing: -60.0,
                distance: 260.0,
                radius: 110.0,
                height: 30.0,
                repose: 37.0,
            },
            stockpile: HeapSpec {
                bearing: 20.0,
                distance: 210.0,
                radius: 38.0,
                height: 14.0,
                repose: 37.0,
            },
            ground: GroundSpec::default(),
            buildings: roadgen_buildings::Rules::preset("industrial"),
            seed: 0,
        }
    }
}

/// Why a mine could not be generated.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The spec asks for something no pit could be.
    Spec(String),
    Build(BuildError),
    Geometry(GeometryError),
    Tin(TinError),
    /// The map already holds a feature this site would be named the same as.
    Duplicate(String),
    /// The yard's buildings could not be generated.
    Buildings(roadgen_buildings::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Spec(detail) => write!(f, "mine spec: {detail}"),
            Error::Build(error) => write!(f, "{error}"),
            Error::Geometry(error) => write!(f, "{error}"),
            Error::Tin(error) => write!(f, "terrain: {error}"),
            Error::Duplicate(id) => write!(f, "{id} is already on the map"),
            Error::Buildings(error) => write!(f, "the yard's buildings: {error}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<BuildError> for Error {
    fn from(error: BuildError) -> Self {
        Error::Build(error)
    }
}

impl From<GeometryError> for Error {
    fn from(error: GeometryError) -> Self {
        Error::Geometry(error)
    }
}

impl From<super::land::Error> for Error {
    fn from(error: super::land::Error) -> Self {
        match error {
            super::land::Error::Spec(detail) => Error::Spec(detail),
            super::land::Error::Geometry(error) => Error::Geometry(error),
            super::land::Error::Tin(error) => Error::Tin(error),
            super::land::Error::Duplicate(id) => Error::Duplicate(id),
        }
    }
}

impl From<TinError> for Error {
    fn from(error: TinError) -> Self {
        Error::Tin(error)
    }
}

/// What [`lay_out`] put on the builder, and what [`generate`] needs to finish.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub spec: MineSpec,
    pub site: SiteId,
    pub ramp: RoadId,
    pub haul_roads: Vec<RoadId>,
    pub hub: JunctionId,
    layout: Layout,
}

impl Plan {
    /// The roads the mine laid out: the ramp, then the haul roads.
    pub fn roads(&self) -> impl Iterator<Item = &RoadId> {
        std::iter::once(&self.ramp).chain(&self.haul_roads)
    }
}

/// What generating the land came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Report {
    pub nodes: usize,
    pub triangles: usize,
    pub breaklines: usize,
    pub land_covers: usize,
    pub berms: usize,
    /// Buildings put on the yard.
    pub buildings: usize,
}

/// A heap, placed.
#[derive(Debug, Clone, PartialEq)]
struct Heap {
    key: &'static str,
    centre: [f64; 2],
    radius: f64,
    top: f64,
    height: f64,
}

/// The mine's geometry, worked out once from its spec.
#[derive(Debug, Clone, PartialEq)]
struct Layout {
    frame: Frame,
    floor: Ellipse,
    /// The rim's height, which is the country's.
    rim: f64,
    /// The floor's height.
    bottom: f64,
    /// How far out each face reaches in plan, and the offset of the rim's crest
    /// from the floor's outline.
    run: f64,
    top: f64,
    /// The ramp's reference line, from the junction's mouth to the floor.
    ramp: Vec<Point3>,
    hub: [f64; 2],
    heaps: Vec<Heap>,
    /// The yard beside the waste dump's haul road, anticlockwise.
    pad: Vec<Point3>,
    /// Where each haul road starts and ends.
    haul: Vec<(&'static str, Point3, Point3)>,
}

/// The yard: how far its near edge stands off the road's edge, how far back it
/// reaches, and how long it is along the road, metres.
const PAD_SETBACK: f64 = 2.0;
const PAD_DEPTH: f64 = 45.0;
const PAD_LENGTH: f64 = 80.0;

/// A point of the map's frame in a pit's frame: the inverse of [`Frame::place`].
fn frame_local(frame: &Frame, point: Point3) -> [f64; 2] {
    let (sin, cos) = frame.heading.sin_cos();
    let (dx, dy) = (point.x - frame.centre.x, point.y - frame.centre.y);
    [dx * cos + dy * sin, -dx * sin + dy * cos]
}

/// How far a heap's or a road's mouth stands from the junction centre, metres.
const HUB_RADIUS: f64 = 30.0;
/// How far along the rim, and how far out from it, the ramp's run in starts.
const LEAD_IN: f64 = 80.0;
const LEAD_OUT: f64 = 35.0;
/// How far round the rim the ramp eases from the rim into the wall.
const EASE: f64 = 40.0;

fn check(spec: &MineSpec) -> Result<(), Error> {
    let positive = |name: &str, value: f64| {
        if value.is_finite() && value > 0.0 {
            Ok(())
        } else {
            Err(Error::Spec(format!(
                "{name} must be a positive number, not {value}"
            )))
        }
    };
    let pit = &spec.pit;
    positive("floor_length", pit.floor_length)?;
    positive("floor_width", pit.floor_width)?;
    positive("bench_height", pit.bench_height)?;
    positive("bench_width", pit.bench_width)?;
    if pit.benches == 0 {
        return Err(Error::Spec("a pit has at least one bench".into()));
    }
    if !(10.0..89.0).contains(&pit.face_angle) {
        return Err(Error::Spec(format!(
            "a face at {} degrees is not one a bench stands at",
            pit.face_angle
        )));
    }
    if !(spec.ramp.grade > 0.0 && spec.ramp.grade <= 0.25) {
        return Err(Error::Spec(format!(
            "a ramp at grade {} is not one a haul truck climbs",
            spec.ramp.grade
        )));
    }
    let road = &spec.haul_road;
    positive("lane_width", road.lane_width)?;
    positive("shoulder_width", road.shoulder_width)?;
    positive("berm_height", road.berm_height)?;
    positive("speed_limit_kph", road.speed_limit_kph)?;
    for (name, heap) in [
        ("waste_dump", &spec.waste_dump),
        ("stockpile", &spec.stockpile),
    ] {
        positive(&format!("{name}.radius"), heap.radius)?;
        positive(&format!("{name}.height"), heap.height)?;
        if !(5.0..80.0).contains(&heap.repose) {
            return Err(Error::Spec(format!(
                "{name}: a repose of {} degrees",
                heap.repose
            )));
        }
        if heap.height / heap.repose.to_radians().tan() >= heap.radius {
            return Err(Error::Spec(format!(
                "{name}: {} m high at {} degrees needs a wider toe than {} m",
                heap.height, heap.repose, heap.radius
            )));
        }
        if heap.distance <= heap.radius + HUB_RADIUS + 20.0 {
            return Err(Error::Spec(format!(
                "{name}: its centre is too near the junction for a road to reach it"
            )));
        }
    }
    positive("ground.cell", spec.ground.cell)?;
    positive("ground.spacing", spec.ground.spacing)?;
    positive("ground.wavelength", spec.ground.wavelength)?;
    if !(spec.ground.margin >= 0.0 && spec.ground.relief >= 0.0) {
        return Err(Error::Spec(
            "the ground's margin and relief cannot be negative".into(),
        ));
    }
    if spec.name.trim().is_empty() {
        return Err(Error::Spec("a site needs a name".into()));
    }
    Ok(())
}

/// The name made safe to put in an identifier: lower case, words joined by `_`.
fn slug(name: &str) -> String {
    let mut out = String::new();
    for character in name.trim().chars() {
        if character.is_ascii_alphanumeric() {
            out.push(character.to_ascii_lowercase());
        } else if !out.ends_with('_') {
            out.push('_');
        }
    }
    out.trim_matches('_').to_owned()
}

impl Layout {
    fn of(spec: &MineSpec) -> Layout {
        let pit = &spec.pit;
        let frame = Frame {
            centre: spec.centre,
            heading: spec.heading,
        };
        let floor = Ellipse {
            a: pit.floor_length / 2.0,
            b: pit.floor_width / 2.0,
        };
        let rim = spec.centre.z;
        let depth = pit.benches as f64 * pit.bench_height;
        let bottom = rim - depth;
        let run = pit.bench_height / pit.face_angle.to_radians().tan();
        let top = pit.benches as f64 * (run + pit.bench_width) - pit.bench_width;

        // Where the wall is, on average, at height `z`: straight from the floor's
        // outline at the bottom to the rim's crest at the top. The ramp's outer
        // edge runs along it, so the ramp is cut into the wall above and stands out
        // over the bench below, and at the bottom it is on the floor.
        let half = spec.haul_road.half_width();
        let wall = |z: f64| ((z - bottom) / depth).clamp(0.0, 1.0) * top - half;
        let sense = if spec.ramp.clockwise { -1.0 } else { 1.0 };
        let theta0 = spec.ramp.start.to_radians();

        // Down the wall in small steps of the ellipse's parameter, keeping the
        // horizontal distance travelled, which is what the grade is over.
        let mut spiral: Vec<[f64; 3]> = Vec::new();
        let mut theta = theta0;
        let mut travelled = 0.0;
        let mut last = floor.at(theta, top - half);
        spiral.push([last[0], last[1], rim]);
        let mut since = 0.0;
        let step = 0.002;
        loop {
            theta += sense * step;
            let z_guess = rim - spec.ramp.grade * travelled;
            // Ease from the rim into the wall over the first stretch, so the road
            // turns into the pit rather than kinking into it.
            let ease = smoothstep(0.0, EASE, travelled);
            let offset = top - half + (wall(z_guess) - (top - half)) * ease;
            let here = floor.at(theta, offset);
            let length = (here[0] - last[0]).hypot(here[1] - last[1]);
            travelled += length;
            since += length;
            last = here;
            let z = rim - spec.ramp.grade * travelled;
            if z <= bottom {
                spiral.push([here[0], here[1], bottom]);
                break;
            }
            if since >= spec.ground.spacing.min(4.0) {
                spiral.push([here[0], here[1], z]);
                since = 0.0;
            }
            if (theta - theta0).abs() > 4.0 * TAU {
                break;
            }
        }

        // The run in to the rim from the junction's mouth, which stands out from
        // the rim so that the junction is clear of the pit: a curve that leaves the
        // mouth heading in and meets the rim along the ramp's own direction there.
        let tangent = floor.tangent(theta0).map(|value| value * sense);
        let normal = floor.normal(theta0);
        let rim_point = spiral[0];
        let mouth = [
            rim_point[0] - tangent[0] * LEAD_IN + normal[0] * (half + LEAD_OUT),
            rim_point[1] - tangent[1] * LEAD_IN + normal[1] * (half + LEAD_OUT),
        ];
        let control = [
            rim_point[0] - tangent[0] * LEAD_IN * 0.5,
            rim_point[1] - tangent[1] * LEAD_IN * 0.5,
        ];
        let heading_in = {
            let (dx, dy) = (control[0] - mouth[0], control[1] - mouth[1]);
            let length = dx.hypot(dy);
            [dx / length, dy / length]
        };
        let hub = [
            mouth[0] - heading_in[0] * HUB_RADIUS,
            mouth[1] - heading_in[1] * HUB_RADIUS,
        ];
        let mut ramp = Vec::new();
        let lead_steps = (LEAD_IN / 5.0).ceil() as usize;
        for index in 0..lead_steps {
            let t = index as f64 / lead_steps as f64;
            let (a, b, c) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t);
            ramp.push(frame.place(
                a * mouth[0] + b * control[0] + c * rim_point[0],
                a * mouth[1] + b * control[1] + c * rim_point[1],
                rim,
            ));
        }
        ramp.extend(spiral.iter().map(|p| frame.place(p[0], p[1], p[2])));

        // Bearings are measured in the pit's frame from the junction.
        let toward = |bearing: f64, distance: f64| {
            let (sin, cos) = bearing.to_radians().sin_cos();
            [hub[0] + cos * distance, hub[1] + sin * distance]
        };
        let mut heaps = Vec::new();
        let mut haul = Vec::new();
        for (key, heap) in [
            ("waste_dump", &spec.waste_dump),
            ("stockpile", &spec.stockpile),
        ] {
            let centre = toward(heap.bearing, heap.distance);
            let top_radius = heap.radius - heap.height / heap.repose.to_radians().tan();
            heaps.push(Heap {
                key,
                centre,
                radius: heap.radius,
                top: top_radius,
                height: heap.height,
            });
            let start = toward(heap.bearing, HUB_RADIUS);
            // The road stops short of the toe, where a truck turns to tip.
            let end = toward(heap.bearing, heap.distance - heap.radius - 15.0);
            haul.push((
                key,
                frame.place(start[0], start[1], rim),
                frame.place(end[0], end[1], rim),
            ));
        }

        // The yard: beside the waste dump's haul road, halfway along it and on the
        // side away from the pit, so that a building on it faces the road the way
        // a building in a town faces its street.
        let pad = {
            let (_, start, end) = &haul[0];
            let (start, end) = (frame_local(&frame, *start), frame_local(&frame, *end));
            let length = (end[0] - start[0]).hypot(end[1] - start[1]);
            let along = [(end[0] - start[0]) / length, (end[1] - start[1]) / length];
            let mut across = [-along[1], along[0]];
            let middle = [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0];
            if middle[0] * across[0] + middle[1] * across[1] < 0.0 {
                across = [-across[0], -across[1]];
            }
            let (near, far, half_long) = (
                half + PAD_SETBACK,
                half + PAD_SETBACK + PAD_DEPTH,
                PAD_LENGTH / 2.0,
            );
            [
                (-half_long, near),
                (half_long, near),
                (half_long, far),
                (-half_long, far),
            ]
            .iter()
            .map(|&(u, v)| {
                frame.place(
                    middle[0] + along[0] * u + across[0] * v,
                    middle[1] + along[1] * u + across[1] * v,
                    rim,
                )
            })
            .collect()
        };

        Layout {
            frame,
            floor,
            rim,
            bottom,
            run,
            top,
            ramp,
            hub: {
                let p = frame.place(hub[0], hub[1], rim);
                [p.x, p.y]
            },
            heaps: heaps
                .into_iter()
                .map(|heap| {
                    let p = frame.place(heap.centre[0], heap.centre[1], rim);
                    Heap {
                        centre: [p.x, p.y],
                        ..heap
                    }
                })
                .collect(),
            pad,
            haul,
        }
    }

    /// A ring of the pit, in the map's frame, at height `z`.
    fn pit_ring(&self, offset: f64, z: f64, spacing: f64) -> Vec<Point3> {
        self.floor
            .ring(offset, spacing)
            .into_iter()
            .map(|p| self.frame.place(p[0], p[1], z))
            .collect()
    }
}

fn heap_ring(heap: &Heap, radius: f64, z: f64, spacing: f64) -> Vec<Point3> {
    let count = (TAU * radius / spacing).ceil().max(12.0) as usize;
    (0..count)
        .map(|index| {
            let angle = TAU * index as f64 / count as f64;
            Point3::new(
                heap.centre[0] + radius * angle.cos(),
                heap.centre[1] + radius * angle.sin(),
                z,
            )
        })
        .collect()
}

fn haul_lanes(spec: &HaulRoadSpec) -> Result<Vec<LaneSpec>, Error> {
    let width =
        |metres: f64| PositiveWidth::new(metres).map_err(|error| Error::Spec(error.to_string()));
    // No paint: a haul road is graded gravel, and the edge a driver keeps off is
    // the berm, not a line.
    let bare = BoundaryMarking::new(RoadMarking::None, MarkingColor::White);
    let limit = SpeedLimit::from_kph(spec.speed_limit_kph)
        .map_err(|error| Error::Spec(error.to_string()))?;
    let lane =
        |metres: f64, direction: Direction, lane_type: LaneType| -> Result<LaneSpec, Error> {
            Ok(LaneSpec::new(width(metres)?, direction)
                .with_type(lane_type)
                .with_markings(bare, bare)
                .with_speed_limit(limit))
        };
    Ok(vec![
        lane(spec.lane_width, Direction::Forward, LaneType::Driving)?,
        lane(spec.shoulder_width, Direction::Forward, LaneType::Shoulder)?,
        lane(spec.lane_width, Direction::Backward, LaneType::Driving)?,
        lane(spec.shoulder_width, Direction::Backward, LaneType::Shoulder)?,
    ])
}

/// Puts the mine's roads on `builder`: the ramp, the haul roads, and the junction
/// at the top of the ramp where they meet.
pub fn lay_out(builder: &mut MapBuilder, spec: &MineSpec) -> Result<Plan, Error> {
    check(spec)?;
    let name = slug(&spec.name);
    if name.is_empty() {
        return Err(Error::Spec(format!(
            "{:?} has nothing to name features after",
            spec.name
        )));
    }
    let layout = Layout::of(spec);
    let pavement = PavementProfile::constant(spec.haul_road.surface);
    let limit = SpeedLimit::from_kph(spec.haul_road.speed_limit_kph)
        .map_err(|error| Error::Spec(error.to_string()))?;
    let road = |line: Curve3, key: &str| -> Result<RoadSpec, Error> {
        Ok(RoadSpec::new(line, haul_lanes(&spec.haul_road)?)
            .with_name(format!("{name}_{key}"))
            .with_type(RoadType::LowSpeed)
            .with_speed_limit(limit)
            .with_pavement(pavement.clone()))
    };

    let ramp = builder.add_road(road(
        Curve3::polyline(layout.ramp.iter().copied())?,
        "ramp",
    )?)?;
    let mut haul_roads = Vec::new();
    for (key, start, end) in &layout.haul {
        let id = builder.add_road(road(Curve3::line(*start, *end)?, &format!("haul_{key}"))?)?;
        haul_roads.push(id);
    }
    let hub = builder.add_junction(Some(&format!("{name}_hub")));
    let arms: Vec<&RoadId> = std::iter::once(&ramp).chain(&haul_roads).collect();
    for (index, from) in arms.iter().enumerate() {
        for to in &arms[index + 1..] {
            builder.connect_ends(from, RoadEnd::Start, to, RoadEnd::Start, Some(&hub))?;
        }
    }
    Ok(Plan {
        spec: spec.clone(),
        site: SiteId::new(&name),
        ramp,
        haul_roads,
        hub,
        layout,
    })
}

/// Fills `map` with the mine's land, land covers, berms and site. The roads have
/// to have been laid out on the builder `map` came from, with the same `plan`.
pub fn generate(map: &mut UnvalidatedMap, plan: &Plan) -> Result<Report, Error> {
    let spec = &plan.spec;
    let layout = &plan.layout;
    let name = plan.site.local_name().to_owned();
    let surface_id = TinSurfaceId::new(&name);
    {
        let existing = map.as_map();
        if existing.sites.contains(&plan.site) {
            return Err(Error::Duplicate(plan.site.to_string()));
        }
        if existing.tin_surfaces.contains(&surface_id) {
            return Err(Error::Duplicate(surface_id.to_string()));
        }
    }

    // The land, shaped by the pit and the heaps — the town's land is the same
    // land shaped by nothing — and round every road on the map.
    let spacing = spec.ground.spacing;
    let pit = &spec.pit;
    let mut forms = Landforms::default();
    for bench in 0..pit.benches {
        let toe = bench as f64 * (layout.run + pit.bench_width);
        let z = layout.bottom + bench as f64 * pit.bench_height;
        forms.lines.push(Line {
            key: format!("bench/{bench}/toe"),
            kind: "toe",
            points: layout.pit_ring(toe, z, spacing),
            closed: true,
        });
        forms.lines.push(Line {
            key: format!("bench/{bench}/crest"),
            kind: "crest",
            points: layout.pit_ring(toe + layout.run, z + pit.bench_height, spacing),
            closed: true,
        });
    }
    let rim = layout.pit_ring(layout.top, layout.rim, spacing);
    forms.holes.push(plan_of(&rim));
    forms.levels.push(Level {
        ring: plan_of(&layout.pit_ring(0.0, layout.bottom, spacing)),
        z: layout.bottom,
    });
    for heap in &layout.heaps {
        let toe = heap_ring(heap, heap.radius, layout.rim, spacing);
        forms.holes.push(plan_of(&toe));
        forms.levels.push(Level {
            ring: plan_of(&heap_ring(heap, heap.top, 0.0, spacing)),
            z: layout.rim + heap.height,
        });
        forms.lines.push(Line {
            key: format!("{}/toe", heap.key),
            kind: "toe",
            points: toe,
            closed: true,
        });
        forms.lines.push(Line {
            key: format!("{}/crest", heap.key),
            kind: "crest",
            points: heap_ring(heap, heap.top, layout.rim + heap.height, spacing),
            closed: true,
        });
    }
    forms.flats.push(plan_of(&layout.pad));

    // What stands on the yard, from the town's own shape grammar: lots along the
    // haul road it is beside, kept to the yard. Before the land, so that the land
    // keeps its roll off them.
    let mut report = Report::default();
    if let Some(rules) = &spec.buildings {
        let placement = roadgen_buildings::Placement {
            roads: Some(plan.haul_roads.clone()),
            within: vec![plan_of(&layout.pad)],
            keep: true,
        };
        let before = map.as_map().buildings.len();
        roadgen_buildings::generate_with(map, &rules.clone().with_seed(spec.seed), &placement)
            .map_err(Error::Buildings)?;
        report.buildings = map.as_map().buildings.len() - before;
    }

    let land = Land {
        name: name.clone(),
        site: Some(plan.site.clone()),
        base: Base::Level(layout.rim),
        ground: spec.ground.clone(),
        seed: spec.seed,
    };
    let (surface_id, built) = land::generate(map, &land, forms)?;
    report.nodes = built.nodes;
    report.triangles = built.triangles;
    report.breaklines = built.breaklines;

    // What the land is, each over the outline the land was shaped to.
    let target = map.as_map_mut();
    let site = Some(&plan.site);
    let surface = Some(&surface_id);
    let mut covers = vec![land::cover(
        target, &name, "pit", "pit", rim, site, surface,
    )?];
    for heap in &layout.heaps {
        let toe = heap_ring(heap, heap.radius, layout.rim, spacing);
        covers.push(land::cover(
            target, &name, heap.key, heap.key, toe, site, surface,
        )?);
    }
    covers.push(land::cover(
        target,
        &name,
        "pad",
        "pad",
        layout.pad.clone(),
        site,
        surface,
    )?);
    report.land_covers = covers.len();

    // The berm along the ramp's pit side: on the shoulder on the side the pit's
    // centre is, which is to the right of a road winding down clockwise.
    let pit_side = if spec.ramp.clockwise {
        LateralSide::Right
    } else {
        LateralSide::Left
    };
    let ramp_lanes: Vec<Lane> = target
        .lanes_of_section(&plan.ramp, 0)
        .into_iter()
        .cloned()
        .collect();
    let shoulder = ramp_lanes
        .iter()
        .find(|lane| lane.side == pit_side && lane.lane_type == LaneType::Shoulder);
    let beside = ramp_lanes
        .iter()
        .find(|lane| lane.side == pit_side && lane.lane_type == LaneType::Driving);
    if let (Some(shoulder), Some(beside)) = (shoulder, beside) {
        let id = ObjectId::new(format!("{name}/berm/ramp/{}", pit_side.as_str()));
        target
            .objects
            .insert(
                id.clone(),
                MapObject {
                    id,
                    kind: MapObjectKind::Barrier {
                        kind: "berm".into(),
                        height: spec.haul_road.berm_height,
                    },
                    geometry: ObjectGeometry::Band {
                        left: shoulder.left_boundary.clone(),
                        right: shoulder.right_boundary.clone(),
                    },
                    lanes: vec![beside.id.clone()],
                },
            )
            .map_err(|duplicate| Error::Duplicate(duplicate.0.to_string()))?;
        report.berms += 1;
    }

    let mut site_roads: Vec<RoadId> = plan.roads().cloned().collect();
    if let Some(junction) = target.junction(&plan.hub) {
        site_roads.extend(junction.connecting_roads.iter().cloned());
    }
    target
        .sites
        .insert(
            plan.site.clone(),
            Site {
                id: plan.site.clone(),
                name: Some(spec.name.clone()),
                kind: "open_pit_mine".into(),
                land_covers: covers,
                surfaces: vec![surface_id],
                roads: site_roads,
            },
        )
        .map_err(|duplicate| Error::Duplicate(duplicate.0.to_string()))?;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_name_becomes_an_identifier() {
        assert_eq!(slug("North Pit #2"), "north_pit_2");
        assert_eq!(slug("  "), "");
    }

    #[test]
    fn the_ramp_reaches_the_floor_at_its_grade() {
        let spec = MineSpec::default();
        let layout = Layout::of(&spec);
        let last = layout.ramp.last().unwrap();
        assert!((last.z - layout.bottom).abs() < 1e-9);
        // Level on the run in, then falling at the grade all the way down, but for
        // the last step, which stops on the floor.
        let falling: Vec<_> = layout
            .ramp
            .windows(2)
            .filter(|pair| pair[1].z < layout.rim)
            .collect();
        assert!(falling.len() > 10);
        for pair in &falling[..falling.len() - 1] {
            let grade = (pair[0].z - pair[1].z) / pair[0].horizontal_distance_to(pair[1]);
            assert!((grade - spec.ramp.grade).abs() < 1e-3, "{grade}");
        }
    }
}
