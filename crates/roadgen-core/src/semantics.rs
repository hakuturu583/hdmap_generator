//! What the road *means*, as opposed to where it is or what it connects to.
//!
//! These are IR concepts. Neither an OpenDRIVE `<signal>` nor a Lanelet2
//! `RegulatoryElement` appears here; each exporter maps these onto its own format's
//! vocabulary, and a format that cannot express one simply drops it.

use crate::geometry::{Curve3, Point3, UnitVector3};
use crate::id::{LaneId, ObjectId};
use crate::units::SpeedLimit;

/// What a lane is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LaneType {
    Driving,
    Shoulder,
    Border,
    Sidewalk,
    Biking,
    Parking,
    Restricted,
    /// Carries no traffic; used for the strip beyond the outermost boundary.
    None,
}

impl LaneType {
    /// Whether vehicles travel along this lane, which decides whether it becomes a
    /// lanelet and whether it takes part in connectivity.
    pub fn is_drivable(self) -> bool {
        matches!(self, LaneType::Driving | LaneType::Biking)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            LaneType::Driving => "driving",
            LaneType::Shoulder => "shoulder",
            LaneType::Border => "border",
            LaneType::Sidewalk => "sidewalk",
            LaneType::Biking => "biking",
            LaneType::Parking => "parking",
            LaneType::Restricted => "restricted",
            LaneType::None => "none",
        }
    }

    pub fn parse(value: &str) -> Option<LaneType> {
        Some(match value.to_ascii_lowercase().as_str() {
            "driving" => LaneType::Driving,
            "shoulder" => LaneType::Shoulder,
            "border" => LaneType::Border,
            "sidewalk" | "walking" => LaneType::Sidewalk,
            "biking" | "bicycle" => LaneType::Biking,
            "parking" => LaneType::Parking,
            "restricted" => LaneType::Restricted,
            "none" => LaneType::None,
            _ => return None,
        })
    }
}

/// What kind of road a stretch is, kept separate from its lanes' types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadType {
    Town,
    Rural,
    Motorway,
    LowSpeed,
    Pedestrian,
}

impl RoadType {
    pub fn as_str(self) -> &'static str {
        match self {
            RoadType::Town => "town",
            RoadType::Rural => "rural",
            RoadType::Motorway => "motorway",
            RoadType::LowSpeed => "lowSpeed",
            RoadType::Pedestrian => "pedestrian",
        }
    }

    /// The Lanelet2 `location` tag that goes with this road type.
    pub fn lanelet2_location(self) -> &'static str {
        match self {
            RoadType::Rural | RoadType::Motorway => "nonurban",
            _ => "urban",
        }
    }

    pub fn parse(value: &str) -> Option<RoadType> {
        Some(match value.to_ascii_lowercase().as_str() {
            "town" | "urban" => RoadType::Town,
            "rural" => RoadType::Rural,
            "motorway" | "highway" => RoadType::Motorway,
            "lowspeed" | "low_speed" => RoadType::LowSpeed,
            "pedestrian" => RoadType::Pedestrian,
            _ => return None,
        })
    }
}

/// What part a road plays in the network, kept apart from [`RoadType`].
///
/// GDF's Form of Way. A road type says what kind of place the road is in — a town,
/// the country, a motorway — which is what OpenDRIVE's `<type>` holds; the form says
/// what the road *is* to the network: a carriageway of a motorway, the slip road
/// onto it, a parallel collector. The two are different questions, and a slip road
/// in a town is both.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FormOfWay {
    /// Nothing said; what every road is unless the caller says otherwise.
    #[default]
    Unspecified,
    /// One carriageway of a road for motor vehicles only, entered and left only by
    /// slip roads and crossing nothing on the level.
    Motorway,
    /// A road with physically separated carriageways that is not a motorway.
    MultipleCarriageway,
    SingleCarriageway,
    /// A road built to enter or leave another: a ramp, a link between motorways.
    SlipRoad,
    /// A collector-distributor beside a carriageway, or a frontage road.
    ParallelRoad,
    ServiceRoad,
    Roundabout,
}

impl FormOfWay {
    pub fn as_str(self) -> &'static str {
        match self {
            FormOfWay::Unspecified => "unspecified",
            FormOfWay::Motorway => "motorway",
            FormOfWay::MultipleCarriageway => "multiple_carriageway",
            FormOfWay::SingleCarriageway => "single_carriageway",
            FormOfWay::SlipRoad => "slip_road",
            FormOfWay::ParallelRoad => "parallel_road",
            FormOfWay::ServiceRoad => "service_road",
            FormOfWay::Roundabout => "roundabout",
        }
    }

    pub fn parse(value: &str) -> Option<FormOfWay> {
        Some(match value.to_ascii_lowercase().as_str() {
            "unspecified" | "" => FormOfWay::Unspecified,
            "motorway" => FormOfWay::Motorway,
            "multiple_carriageway" | "dual_carriageway" => FormOfWay::MultipleCarriageway,
            "single_carriageway" => FormOfWay::SingleCarriageway,
            "slip_road" | "ramp" => FormOfWay::SlipRoad,
            "parallel_road" => FormOfWay::ParallelRoad,
            "service_road" => FormOfWay::ServiceRoad,
            "roundabout" => FormOfWay::Roundabout,
            _ => return None,
        })
    }

    /// A motorway or a slip road: entered only where a slip road joins, crossing
    /// nothing on the level, and with no frontage for a building to face.
    pub fn is_controlled_access(self) -> bool {
        matches!(self, FormOfWay::Motorway | FormOfWay::SlipRoad)
    }
}

/// What a road is surfaced with: whether it is paved, and with what.
///
/// Two levels, the way GDF puts it — a pavement status, then a surface type that
/// only means anything under it — so that a road that is unpaved and asphalt is a
/// value this type cannot hold rather than one validation has to catch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Pavement {
    Paved(PavedSurface),
    Unpaved(UnpavedSurface),
}

/// The surface of a paved road.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PavedSurface {
    Asphalt,
    Concrete,
    Cobblestone,
}

/// The surface of an unpaved road.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum UnpavedSurface {
    Gravel,
    Dirt,
    Sand,
    Rock,
}

impl Pavement {
    pub fn is_paved(self) -> bool {
        matches!(self, Pavement::Paved(_))
    }

    /// The surface type on its own, which is the word every consumer that has one
    /// field for it wants.
    pub fn surface_str(self) -> &'static str {
        match self {
            Pavement::Paved(PavedSurface::Asphalt) => "asphalt",
            Pavement::Paved(PavedSurface::Concrete) => "concrete",
            Pavement::Paved(PavedSurface::Cobblestone) => "cobblestone",
            Pavement::Unpaved(UnpavedSurface::Gravel) => "gravel",
            Pavement::Unpaved(UnpavedSurface::Dirt) => "dirt",
            Pavement::Unpaved(UnpavedSurface::Sand) => "sand",
            Pavement::Unpaved(UnpavedSurface::Rock) => "rock",
        }
    }

    /// Reads a surface type; its pavement status follows from it.
    pub fn parse(value: &str) -> Option<Pavement> {
        Some(match value.to_ascii_lowercase().as_str() {
            "asphalt" => Pavement::Paved(PavedSurface::Asphalt),
            "concrete" => Pavement::Paved(PavedSurface::Concrete),
            "cobblestone" | "sett" => Pavement::Paved(PavedSurface::Cobblestone),
            "gravel" => Pavement::Unpaved(UnpavedSurface::Gravel),
            "dirt" | "earth" | "ground" => Pavement::Unpaved(UnpavedSurface::Dirt),
            "sand" => Pavement::Unpaved(UnpavedSurface::Sand),
            "rock" => Pavement::Unpaved(UnpavedSurface::Rock),
            _ => return None,
        })
    }
}

/// A road's pavement along its length: each value holds from its station to the
/// next one's. Empty when nobody said, which is every road the IR held before
/// pavement existed, and which a consumer reads as "whatever it assumes".
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PavementProfile {
    /// `(station, pavement)`, ascending by station.
    spans: Vec<(f64, Pavement)>,
}

impl PavementProfile {
    /// Unknown everywhere.
    pub fn unknown() -> Self {
        PavementProfile::default()
    }

    /// The same pavement along the whole road.
    pub fn constant(pavement: Pavement) -> Self {
        PavementProfile {
            spans: vec![(0.0, pavement)],
        }
    }

    /// Pavements taking over at each station in turn. Fails unless the stations are
    /// finite and strictly ascending; a span that repeats the one before it is
    /// merged into it.
    pub fn new(spans: impl IntoIterator<Item = (f64, Pavement)>) -> Result<Self, String> {
        let mut kept: Vec<(f64, Pavement)> = Vec::new();
        for (station, pavement) in spans {
            if !station.is_finite() {
                return Err(format!("a pavement starts at station {station}"));
            }
            if let Some(&(previous, last)) = kept.last() {
                if station <= previous {
                    return Err(format!(
                        "pavement stations must ascend, and {station} follows {previous}"
                    ));
                }
                if last == pavement {
                    continue;
                }
            }
            kept.push((station, pavement));
        }
        Ok(PavementProfile { spans: kept })
    }

    pub fn is_unknown(&self) -> bool {
        self.spans.is_empty()
    }

    pub fn spans(&self) -> &[(f64, Pavement)] {
        &self.spans
    }

    /// The pavement at `station`: the last span that starts at or before it, or the
    /// first before the first span begins.
    pub fn at(&self, station: f64) -> Option<Pavement> {
        self.spans
            .iter()
            .rev()
            .find(|(start, _)| *start <= station + 1e-9)
            .or_else(|| self.spans.first())
            .map(|&(_, pavement)| pavement)
    }
}

/// The paint on a lane boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadMarking {
    /// Nothing is painted, but the boundary exists as a geometric line.
    None,
    Solid,
    Broken,
    SolidSolid,
    BrokenSolid,
    SolidBroken,
    /// A physical edge such as a kerb.
    Curbstone,
}

impl RoadMarking {
    pub fn as_str(self) -> &'static str {
        match self {
            RoadMarking::None => "none",
            RoadMarking::Solid => "solid",
            RoadMarking::Broken => "broken",
            RoadMarking::SolidSolid => "solid solid",
            RoadMarking::BrokenSolid => "broken solid",
            RoadMarking::SolidBroken => "solid broken",
            RoadMarking::Curbstone => "curb",
        }
    }

    pub fn parse(value: &str) -> Option<RoadMarking> {
        Some(
            match value.to_ascii_lowercase().replace('_', " ").as_str() {
                "none" => RoadMarking::None,
                "solid" => RoadMarking::Solid,
                "broken" | "dashed" => RoadMarking::Broken,
                "solid solid" => RoadMarking::SolidSolid,
                "broken solid" => RoadMarking::BrokenSolid,
                "solid broken" => RoadMarking::SolidBroken,
                "curb" | "curbstone" | "kerb" => RoadMarking::Curbstone,
                _ => return None,
            },
        )
    }
}

/// Colour of a road marking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MarkingColor {
    White,
    Yellow,
}

impl MarkingColor {
    pub fn as_str(self) -> &'static str {
        match self {
            MarkingColor::White => "white",
            MarkingColor::Yellow => "yellow",
        }
    }
}

/// A lane boundary's paint, as the IR records it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoundaryMarking {
    pub marking: RoadMarking,
    pub color: MarkingColor,
}

impl BoundaryMarking {
    pub fn new(marking: RoadMarking, color: MarkingColor) -> Self {
        BoundaryMarking { marking, color }
    }
}

impl Default for BoundaryMarking {
    fn default() -> Self {
        BoundaryMarking::new(RoadMarking::Solid, MarkingColor::White)
    }
}

/// Where a map object sits.
#[derive(Debug, Clone, PartialEq)]
pub enum ObjectGeometry {
    /// A single position, such as the base of a sign post.
    Point(Point3),
    /// A line across or along the road: a stop line, the bar of a traffic light.
    Line(Curve3),
    /// A strip with two edges, such as a crosswalk.
    Band { left: Curve3, right: Curve3 },
}

/// The colour a traffic light's lamp shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LightColor {
    Red,
    Yellow,
    Green,
}

impl LightColor {
    pub const ALL: [LightColor; 3] = [LightColor::Red, LightColor::Yellow, LightColor::Green];

    pub fn as_str(self) -> &'static str {
        match self {
            LightColor::Red => "red",
            LightColor::Yellow => "yellow",
            LightColor::Green => "green",
        }
    }

    pub fn parse(value: &str) -> Option<Self> {
        let value = value.trim();
        if value.eq_ignore_ascii_case("amber") {
            return Some(LightColor::Yellow);
        }
        Self::ALL
            .into_iter()
            .find(|color| color.as_str().eq_ignore_ascii_case(value))
    }
}

/// The way an arrow lamp points, as seen by the driver facing it: `Up` is straight
/// on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LightArrow {
    Up,
    Down,
    Left,
    Right,
    UpLeft,
    UpRight,
    DownLeft,
    DownRight,
}

impl LightArrow {
    pub const ALL: [LightArrow; 8] = [
        LightArrow::Up,
        LightArrow::Down,
        LightArrow::Left,
        LightArrow::Right,
        LightArrow::UpLeft,
        LightArrow::UpRight,
        LightArrow::DownLeft,
        LightArrow::DownRight,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            LightArrow::Up => "up",
            LightArrow::Down => "down",
            LightArrow::Left => "left",
            LightArrow::Right => "right",
            LightArrow::UpLeft => "up_left",
            LightArrow::UpRight => "up_right",
            LightArrow::DownLeft => "down_left",
            LightArrow::DownRight => "down_right",
        }
    }

    pub fn parse(value: &str) -> Option<Self> {
        let value = value.trim();
        Self::ALL
            .into_iter()
            .find(|arrow| arrow.as_str().eq_ignore_ascii_case(value))
    }
}

/// One lamp of a traffic light: where it is, and what it shows when lit.
#[derive(Debug, Clone, PartialEq)]
pub struct LightBulb {
    pub position: Point3,
    pub color: LightColor,
    /// `None` for a round lamp.
    pub arrow: Option<LightArrow>,
}

/// What a traffic light looks like beyond where it stands: how tall its housing is
/// and which lamps it carries. A light placed by the builder knows neither, and a
/// consumer that needs them — a perception stack that looks for the lit lamp —
/// falls back on its own defaults.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LightHead {
    /// How far the housing reaches up from the object's geometry, which is its
    /// bottom edge, metres.
    pub height: Option<f64>,
    /// The lamps, in the order they are listed.
    pub bulbs: Vec<LightBulb>,
}

impl LightHead {
    /// The width of a standard head: a vertical housing with three round lamps.
    pub const STANDARD_WIDTH: f64 = 0.38;
    /// The diameter of each of a standard head's lamps.
    pub const STANDARD_LAMP: f64 = 0.30;
    /// How much taller a standard housing is than its column of lamps.
    pub const STANDARD_MARGIN: f64 = 0.15;

    /// A standard head whose housing's bottom edge is centred on `bottom`: three
    /// lamps one above the other, green at the bottom and red at the top, as a
    /// builder-placed light is built in every export that draws one.
    pub fn standard(bottom: Point3, up: UnitVector3) -> LightHead {
        let lamp = Self::STANDARD_LAMP;
        let margin = Self::STANDARD_MARGIN / 2.0;
        let bulbs = [LightColor::Green, LightColor::Yellow, LightColor::Red]
            .into_iter()
            .enumerate()
            .map(|(row, color)| LightBulb {
                position: bottom + up.scaled(margin + lamp / 2.0 + row as f64 * lamp),
                color,
                arrow: None,
            })
            .collect();
        LightHead {
            height: Some(3.0 * lamp + Self::STANDARD_MARGIN),
            bulbs,
        }
    }
}

/// Kinds of physical furniture the IR knows about.
#[derive(Debug, Clone, PartialEq)]
pub enum MapObjectKind {
    TrafficLight {
        head: LightHead,
    },
    /// `code` is the sign's identifier in whatever catalogue the caller uses; the IR
    /// does not define one.
    TrafficSign {
        code: String,
    },
    StopLine,
    Crosswalk,
    /// Something that stands along the road to keep traffic on it: a safety berm
    /// beside a haul road, a guard rail, a wall. `kind` says which, in the
    /// caller's words; `height` is how far its top stands above its base, metres.
    /// The geometry is a band along the ground it occupies.
    Barrier {
        kind: String,
        height: f64,
    },
}

impl MapObjectKind {
    /// A traffic light with nothing known of its housing or lamps.
    pub fn traffic_light() -> Self {
        MapObjectKind::TrafficLight {
            head: LightHead::default(),
        }
    }

    pub fn is_traffic_light(&self) -> bool {
        matches!(self, MapObjectKind::TrafficLight { .. })
    }

    /// A traffic light's housing and lamps; `None` for anything else.
    pub fn light_head(&self) -> Option<&LightHead> {
        match self {
            MapObjectKind::TrafficLight { head } => Some(head),
            _ => None,
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            MapObjectKind::TrafficLight { .. } => "traffic_light",
            MapObjectKind::TrafficSign { .. } => "traffic_sign",
            MapObjectKind::StopLine => "stop_line",
            MapObjectKind::Crosswalk => "crosswalk",
            MapObjectKind::Barrier { .. } => "barrier",
        }
    }
}

/// A piece of road furniture, and the lanes it applies to.
#[derive(Debug, Clone, PartialEq)]
pub struct MapObject {
    pub id: ObjectId,
    pub kind: MapObjectKind,
    pub geometry: ObjectGeometry,
    /// Lanes governed by the object, in the order the caller listed them.
    pub lanes: Vec<LaneId>,
}

/// A rule that governs movement, expressed over IR objects and lanes.
#[derive(Debug, Clone, PartialEq)]
pub enum TrafficRule {
    /// Lanes controlled by one or more traffic lights, with the stop lines they
    /// stop at: none, one, or — when the lanes reach their junction by more than
    /// one mouth — one per mouth, each across the rule's lanes it crosses.
    TrafficLight {
        lights: Vec<ObjectId>,
        stop_lines: Vec<ObjectId>,
        lanes: Vec<LaneId>,
    },
    /// `yielding` gives way to `right_of_way`.
    RightOfWay {
        right_of_way: Vec<LaneId>,
        yielding: Vec<LaneId>,
        stop_line: Option<ObjectId>,
    },
    /// A limit that applies to a set of lanes rather than to a whole road.
    SpeedLimit {
        limit: SpeedLimit,
        lanes: Vec<LaneId>,
    },
}

impl TrafficRule {
    /// Lanes the rule applies to, whatever its kind.
    pub fn lanes(&self) -> Vec<LaneId> {
        match self {
            TrafficRule::TrafficLight { lanes, .. } => lanes.clone(),
            TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                ..
            } => right_of_way.iter().chain(yielding).cloned().collect(),
            TrafficRule::SpeedLimit { lanes, .. } => lanes.clone(),
        }
    }

    pub fn kind_str(&self) -> &'static str {
        match self {
            TrafficRule::TrafficLight { .. } => "traffic_light",
            TrafficRule::RightOfWay { .. } => "right_of_way",
            TrafficRule::SpeedLimit { .. } => "speed_limit",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lane_types_round_trip_through_their_names() {
        for kind in [
            LaneType::Driving,
            LaneType::Sidewalk,
            LaneType::Shoulder,
            LaneType::None,
        ] {
            assert_eq!(LaneType::parse(kind.as_str()), Some(kind));
        }
        assert_eq!(LaneType::parse("nonsense"), None);
    }

    #[test]
    fn only_vehicle_lanes_are_drivable() {
        assert!(LaneType::Driving.is_drivable());
        assert!(!LaneType::Sidewalk.is_drivable());
    }
}
