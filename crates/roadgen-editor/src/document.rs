//! The document an editor keeps: what the user drew, not what was generated.
//!
//! A road here is a list of control points and a lane list. Everything else — lane
//! boundaries, connectors through junctions, markings — is generated from it by
//! [`crate::build`], so the document stays small enough to edit by hand and to keep
//! in version control, and the IR is never edited directly.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Which side of the road traffic keeps to.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Handedness {
    #[default]
    Right,
    Left,
}

/// How a road's reference line passes through its control points.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Shape {
    /// A smooth curve through every point: cubic Bézier pieces with Catmull-Rom
    /// tangents, so the road bends at a point rather than kinking.
    #[default]
    Smooth,
    /// Straight segments between the points.
    Polyline,
    /// Not drawn through points at all: from the first point, at `heading`, a
    /// chain of straights, arcs and clothoids — the road's [`ElementDoc`]s — each
    /// starting where the last one ended, pointing the way it pointed. What a road
    /// designer writes, and what OpenDRIVE holds.
    Alignment,
}

/// One piece of an alignment.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct ElementDoc {
    /// `line`, `arc`, or `spiral` (a clothoid).
    pub kind: String,
    /// Plan length, metres.
    pub length: f64,
    /// For an arc, its radius; for a spiral, the radius it ends at, whatever it
    /// started at. Metres, positive turning left and negative right. Unset for a
    /// spiral that ends straight; ignored for a line.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub radius: Option<f64>,
    /// How far the road climbs over the piece, metres.
    pub rise: f64,
}

impl Default for ElementDoc {
    fn default() -> Self {
        ElementDoc::line(50.0)
    }
}

impl ElementDoc {
    pub fn line(length: f64) -> Self {
        ElementDoc {
            kind: "line".to_owned(),
            length,
            radius: None,
            rise: 0.0,
        }
    }

    pub fn arc(length: f64, radius: f64) -> Self {
        ElementDoc {
            kind: "arc".to_owned(),
            radius: Some(radius),
            ..ElementDoc::line(length)
        }
    }

    pub fn spiral(length: f64, radius: Option<f64>) -> Self {
        ElementDoc {
            kind: "spiral".to_owned(),
            radius,
            ..ElementDoc::line(length)
        }
    }
}

/// The way a lane runs against its road's reference line.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Travel {
    #[default]
    Forward,
    Backward,
}

/// A side of the reference line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Side {
    Left,
    Right,
}

/// One end of a road.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum End {
    Start,
    #[default]
    End,
}

/// Paint along one edge of a lane.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Marking {
    /// `none`, `solid`, `broken`, `solid solid`, `broken solid`, `solid broken` or
    /// `curb` — the IR's own words.
    pub kind: String,
    /// `white` or `yellow`.
    pub color: String,
}

impl Default for Marking {
    fn default() -> Self {
        Marking::new("solid", "white")
    }
}

impl Marking {
    pub fn new(kind: &str, color: &str) -> Self {
        Marking {
            kind: kind.to_owned(),
            color: color.to_owned(),
        }
    }
}

/// One lane of a road, listed outwards from the reference line on its side.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct LaneDoc {
    /// Metres.
    pub width: f64,
    pub direction: Travel,
    /// `driving`, `shoulder`, `border`, `sidewalk`, `biking`, `parking`,
    /// `restricted` or `none`.
    pub lane_type: String,
    /// Which side of the reference line the lane is on. Left unset, the map's
    /// handedness decides from the direction.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub side: Option<Side>,
    pub left_marking: Marking,
    pub right_marking: Marking,
}

impl Default for LaneDoc {
    fn default() -> Self {
        LaneDoc::driving(3.5, Travel::Forward)
    }
}

impl LaneDoc {
    pub fn driving(width: f64, direction: Travel) -> Self {
        LaneDoc {
            width,
            direction,
            lane_type: "driving".to_owned(),
            side: None,
            left_marking: Marking::default(),
            right_marking: Marking::default(),
        }
    }

    pub fn with_type(mut self, lane_type: &str) -> Self {
        self.lane_type = lane_type.to_owned();
        self
    }
}

/// A road, as its control points and its lanes.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct RoadDoc {
    /// The road's key; the IR names it `road/<id>`.
    pub id: String,
    /// x, y, z in metres, at least two. An alignment uses only the first, where it
    /// starts, and keeps the rest for switching back.
    pub points: Vec<[f64; 3]>,
    pub shape: Shape,
    /// A smooth road's tangent handle at each point, by position: the vector in
    /// plan, metres, from the point to its forward control point, mirrored behind
    /// it. Unset, or short of the point, the handle is worked out from the
    /// neighbouring points.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Vec::is_empty"))]
    pub handles: Vec<Option<[f64; 2]>>,
    /// An alignment's direction at its start, degrees counter-clockwise from east.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "is_zero"))]
    pub heading: f64,
    /// An alignment's pieces, in order.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Vec::is_empty"))]
    pub elements: Vec<ElementDoc>,
    pub lanes: Vec<LaneDoc>,
    /// `town`, `rural`, `motorway`, `low_speed` or `pedestrian`.
    pub road_type: String,
    /// km/h.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub speed_limit: Option<f64>,
    /// Where the road's surface departs from its design: ruts, potholes, puddles,
    /// humps, roughness.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Vec::is_empty"))]
    pub surface: Vec<SurfaceDoc>,
    /// The bridges and tunnels the road runs on or through.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Vec::is_empty"))]
    pub structures: Vec<StructureDoc>,
    /// How the road and the land meet, where the document has land: empty for the
    /// land taking its height from the road; `fitted` for the land cut and filled
    /// to meet it; `structured` for bridges and tunnels chosen from the land where
    /// the road runs far above or below it.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "String::is_empty"))]
    pub ground: String,
}

/// The land round the roads: generated from a few shapes, not drawn triangle by
/// triangle.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct TerrainDoc {
    /// How far the land reaches past the roads, and how finely it is laid out,
    /// metres.
    pub margin: f64,
    pub cell: f64,
    pub spacing: f64,
    /// The roll of the country where nothing else shapes it: how high, metres,
    /// and over what distance.
    pub relief: f64,
    pub wavelength: f64,
    /// What makes one country unlike another of the same relief.
    pub seed: u64,
    pub hills: Vec<HillDoc>,
    pub rivers: Vec<RiverDoc>,
    pub heaps: Vec<HeapDoc>,
    pub pads: Vec<PadDoc>,
}

impl Default for TerrainDoc {
    fn default() -> Self {
        TerrainDoc {
            margin: 150.0,
            cell: 10.0,
            spacing: 5.0,
            relief: 6.0,
            wavelength: 180.0,
            seed: 0,
            hills: Vec::new(),
            rivers: Vec::new(),
            heaps: Vec::new(),
            pads: Vec::new(),
        }
    }
}

/// A hill or a ridge: a swell of the country along its crest, `height` metres
/// above the land round it, `width` metres across at its foot.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct HillDoc {
    pub name: String,
    /// One point for a hill, more for a ridge. Heights are ignored.
    pub crest: Vec<[f64; 3]>,
    pub width: f64,
    pub height: f64,
}

impl Default for HillDoc {
    fn default() -> Self {
        HillDoc {
            name: "hill".to_owned(),
            crest: Vec::new(),
            width: 120.0,
            height: 20.0,
        }
    }
}

/// A river: a channel along its centre line, `width` across at the water and
/// `depth` deep, banks rising at `bank_slope` (horizontal over vertical).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct RiverDoc {
    pub name: String,
    pub centre: Vec<[f64; 3]>,
    pub width: f64,
    pub depth: f64,
    pub bank_slope: f64,
}

impl Default for RiverDoc {
    fn default() -> Self {
        RiverDoc {
            name: "river".to_owned(),
            centre: Vec::new(),
            width: 20.0,
            depth: 3.0,
            bank_slope: 2.0,
        }
    }
}

/// A heap: a cone of tipped material, its toe a circle of `radius`, `height`
/// tall, its sides at the material's angle of `repose`, degrees.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct HeapDoc {
    pub name: String,
    /// `waste_dump`, `stockpile` — the generator's word, passed on.
    pub kind: String,
    pub centre: [f64; 3],
    pub radius: f64,
    pub height: f64,
    pub repose: f64,
}

impl Default for HeapDoc {
    fn default() -> Self {
        HeapDoc {
            name: "heap".to_owned(),
            kind: "stockpile".to_owned(),
            centre: [0.0; 3],
            radius: 40.0,
            height: 12.0,
            repose: 37.0,
        }
    }
}

/// A pad: land levelled flat inside its outline, at the outline's mean height.
#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct PadDoc {
    pub name: String,
    pub outline: Vec<[f64; 3]>,
    /// Whether the document's buildings go on the pad too: a yard's sheds.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "std::ops::Not::not"))]
    pub buildings: bool,
}

/// The buildings along the roads: generated from a shape grammar, lot by lot along
/// every frontage, not placed one by one.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct BuildingsDoc {
    /// `town`, `suburban`, `downtown` or `industrial`; or `custom` for `grammar`.
    pub preset: String,
    /// CGA rules written out in full, used when `preset` is `custom`.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "String::is_empty"))]
    pub grammar: String,
    /// What makes one street unlike another under the same rules.
    pub seed: u64,
}

impl Default for BuildingsDoc {
    fn default() -> Self {
        BuildingsDoc {
            preset: "town".to_owned(),
            grammar: String::new(),
            seed: 0,
        }
    }
}

/// A bridge or a tunnel over a stretch of one road.
///
/// Which of the fields count depends on `kind`; the rest are kept, so switching a
/// structure from one kind to the other and back loses nothing.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct StructureDoc {
    /// `bridge` or `tunnel`.
    pub kind: String,
    /// Where it starts and ends, metres of station along the road.
    pub from: f64,
    pub to: f64,
    /// What it is called, if anything.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "String::is_empty"))]
    pub name: String,
    /// A bridge: `bridge`, `viaduct` or `overpass`; a tunnel: `bored`,
    /// `cut_and_cover` or `underpass`. Empty for the first of each.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "String::is_empty"))]
    pub form: String,
    /// A bridge's: `concrete`, `steel`, `masonry` or `timber`.
    pub material: String,
    /// A bridge's deck depth, longest span between supports, and the kerb its
    /// parapet stands on past the road's edge, metres.
    pub deck_depth: f64,
    pub pier_spacing: f64,
    pub curb: f64,
    /// A tunnel's clear height and lining thickness, metres.
    pub clear_height: f64,
    pub lining: f64,
    /// Whether a tunnel is lit.
    pub lighting: bool,
    /// A tunnel's inspection walkway beside the road, metres.
    pub walkway: f64,
}

impl Default for StructureDoc {
    fn default() -> Self {
        StructureDoc {
            kind: "bridge".to_owned(),
            from: 0.0,
            to: 0.0,
            name: String::new(),
            form: String::new(),
            material: "concrete".to_owned(),
            deck_depth: 2.5,
            pier_spacing: 40.0,
            curb: 0.6,
            clear_height: 5.0,
            lining: 0.6,
            lighting: true,
            walkway: 0.75,
        }
    }
}

impl StructureDoc {
    /// A structure of `kind` from `from` to `to` along its road, as built usually.
    pub fn new(kind: &str, from: f64, to: f64) -> Self {
        StructureDoc {
            kind: kind.to_owned(),
            from,
            to,
            ..StructureDoc::default()
        }
    }
}

/// A rut worn along a path through one lane rather than straight down the road.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RutPath {
    /// The lane, by its position in the road's lane list.
    pub lane: usize,
    /// `[station, offset]` pairs, the offset metres to the left of the lane's
    /// middle; stations ascending. Joined smoothly.
    pub knots: Vec<[f64; 2]>,
}

/// One departure of a road's surface from its design.
///
/// The footprint is in the road's own coordinates: from `station` for `length`
/// metres along it, `width` metres across, centred `offset` metres to the left of
/// the reference line. Which of the other fields count depends on `kind`; the rest
/// are kept, so switching a feature's kind back and forth loses nothing.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct SurfaceDoc {
    /// `rut`, `pothole`, `puddle`, `bump` or `roughness`.
    pub kind: String,
    pub station: f64,
    pub length: f64,
    pub offset: f64,
    pub width: f64,
    /// How deep a rut, a pothole or a puddle goes, metres.
    pub depth: f64,
    /// How high a bump stands, metres.
    pub height: f64,
    /// A rut's two troughs: centre to centre, and each one's width, metres.
    pub track: f64,
    pub trough: f64,
    /// What a bump is: `bump`, `hump`, `table`, `cushion` or `heave`.
    pub bump: String,
    /// A bump's profile — `circular`, `parabolic`, `sinusoidal` or `flat` — or
    /// empty for the one its kind is usually built to.
    pub profile: String,
    /// The ramp length of a `flat` profile, metres.
    pub ramp: f64,
    /// Roughness's ISO 8608 class, `A` to `H`.
    pub class: String,
    /// What makes one pothole's rim, or one rough patch, unlike another.
    pub seed: u64,
    /// For a rut: the path it follows through a lane. Its footprint is then
    /// fitted to the path, and the four footprint fields above are ignored.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub path: Option<RutPath>,
}

impl Default for SurfaceDoc {
    fn default() -> Self {
        SurfaceDoc {
            kind: "pothole".to_owned(),
            station: 0.0,
            length: 1.0,
            offset: 0.0,
            width: 1.0,
            depth: 0.05,
            height: 0.075,
            track: 1.8,
            trough: 0.4,
            bump: "hump".to_owned(),
            profile: String::new(),
            ramp: 1.0,
            class: "C".to_owned(),
            seed: 0,
            path: None,
        }
    }
}

impl SurfaceDoc {
    /// A feature of `kind`, `length` by `width`, from `station` at `offset`.
    pub fn new(kind: &str, station: f64, length: f64, offset: f64, width: f64) -> Self {
        SurfaceDoc {
            kind: kind.to_owned(),
            station,
            length,
            offset,
            width,
            ..SurfaceDoc::default()
        }
    }
}

#[cfg(feature = "serde")]
fn is_zero(value: &f64) -> bool {
    *value == 0.0
}

impl Default for RoadDoc {
    fn default() -> Self {
        RoadDoc {
            id: String::new(),
            points: Vec::new(),
            shape: Shape::Smooth,
            handles: Vec::new(),
            heading: 0.0,
            elements: Vec::new(),
            lanes: vec![
                LaneDoc::driving(3.5, Travel::Forward),
                LaneDoc::driving(3.5, Travel::Backward),
            ],
            road_type: "town".to_owned(),
            speed_limit: None,
            surface: Vec::new(),
            structures: Vec::new(),
            ground: String::new(),
        }
    }
}

impl RoadDoc {
    /// The tangent handle set at point `index`, if one is.
    pub fn handle(&self, index: usize) -> Option<[f64; 2]> {
        self.handles.get(index).copied().flatten()
    }

    /// A two-way road laid out as an alignment from `start`.
    pub fn aligned(id: &str, start: [f64; 3], heading: f64, elements: Vec<ElementDoc>) -> Self {
        RoadDoc {
            id: id.to_owned(),
            points: vec![start],
            shape: Shape::Alignment,
            heading,
            elements,
            ..RoadDoc::default()
        }
    }

    /// A two-lane, two-way town road through `points`.
    pub fn two_way(id: &str, points: Vec<[f64; 3]>) -> Self {
        RoadDoc {
            id: id.to_owned(),
            points,
            ..RoadDoc::default()
        }
    }
}

/// Two road ends that meet: directly, or through a junction.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct LinkDoc {
    pub from: String,
    pub from_end: End,
    pub to: String,
    pub to_end: End,
    /// The junction the two meet in. Unset, the ends are joined directly, and must
    /// coincide; set, a connector is generated for every movement between them.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub junction: Option<String>,
}

impl Default for LinkDoc {
    fn default() -> Self {
        LinkDoc {
            from: String::new(),
            from_end: End::End,
            to: String::new(),
            to_end: End::Start,
            junction: None,
        }
    }
}

/// Everything the editor keeps.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Document {
    /// The format's version, for a reader of a later one.
    pub version: u32,
    pub name: String,
    pub handedness: Handedness,
    /// Latitude, longitude and altitude of the map's origin; the IR's default when
    /// unset.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub origin: Option<[f64; 3]>,
    pub roads: Vec<RoadDoc>,
    pub links: Vec<LinkDoc>,
    /// The land, when the document has any; none, and the map states no land.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub terrain: Option<TerrainDoc>,
    /// The buildings, when the document has any.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub buildings: Option<BuildingsDoc>,
}

impl Default for Document {
    fn default() -> Self {
        Document {
            version: Document::VERSION,
            name: "untitled".to_owned(),
            handedness: Handedness::Right,
            origin: None,
            roads: Vec::new(),
            links: Vec::new(),
            terrain: None,
            buildings: None,
        }
    }
}

impl Document {
    pub const VERSION: u32 = 1;

    pub fn road(&self, id: &str) -> Option<&RoadDoc> {
        self.roads.iter().find(|road| road.id == id)
    }

    /// A small town to start from: a crossroads, one arm of it bending away and
    /// carrying on past a straight joint into a road with a pavement over a bridge,
    /// another worn, with a hump before the crossing, and a third through a tunnel.
    pub fn example() -> Document {
        let z = 0.0;
        let mut east = RoadDoc::two_way(
            "east",
            vec![[12.0, 0.0, z], [60.0, 0.0, z], [100.0, 25.0, z]],
        );
        east.shape = Shape::Smooth;
        let mut avenue = RoadDoc::two_way(
            "avenue",
            vec![[100.0, 25.0, z], [128.0, 37.0, z], [160.0, 42.0, z]],
        );
        avenue
            .lanes
            .push(LaneDoc::driving(2.0, Travel::Forward).with_type("sidewalk"));
        avenue
            .lanes
            .push(LaneDoc::driving(2.0, Travel::Backward).with_type("sidewalk"));
        // A worn approach: a pothole and a puddle in its lanes, ruts in the
        // eastbound lane's wheel paths, and a hump to slow traffic before the
        // crossroads. Right-hand traffic: eastbound is to the right, at -1.75.
        // The southern approach dips through a short tunnel; the avenue leaves on a
        // bridge.
        let mut south = RoadDoc::two_way("south", vec![[0.0, -80.0, z], [0.0, -12.0, z]]);
        south.structures = vec![StructureDoc {
            name: "south tunnel".to_owned(),
            ..StructureDoc::new("tunnel", 15.0, 45.0)
        }];
        avenue.structures = vec![StructureDoc {
            name: "avenue bridge".to_owned(),
            ..StructureDoc::new("bridge", 12.0, 48.0)
        }];
        let mut west = RoadDoc::two_way("west", vec![[-80.0, 0.0, z], [-12.0, 0.0, z]]);
        west.surface = vec![
            SurfaceDoc {
                depth: 0.02,
                ..SurfaceDoc::new("rut", 4.0, 30.0, -1.75, 2.2)
            },
            SurfaceDoc {
                depth: 0.08,
                seed: 7,
                ..SurfaceDoc::new("pothole", 38.0, 1.2, -2.2, 0.9)
            },
            SurfaceDoc {
                depth: 0.03,
                ..SurfaceDoc::new("puddle", 24.0, 3.0, 1.9, 2.4)
            },
            SurfaceDoc {
                bump: "hump".to_owned(),
                height: 0.075,
                ..SurfaceDoc::new("bump", 52.0, 3.7, 0.0, 7.0)
            },
        ];
        Document {
            name: "example".to_owned(),
            roads: vec![
                west,
                east,
                south,
                RoadDoc::two_way("north", vec![[0.0, 12.0, z], [0.0, 70.0, z]]),
                avenue,
            ],
            links: vec![
                LinkDoc {
                    from: "west".into(),
                    from_end: End::End,
                    to: "east".into(),
                    to_end: End::Start,
                    junction: Some("cross".into()),
                },
                LinkDoc {
                    from: "south".into(),
                    from_end: End::End,
                    to: "north".into(),
                    to_end: End::Start,
                    junction: Some("cross".into()),
                },
                LinkDoc {
                    from: "west".into(),
                    from_end: End::End,
                    to: "north".into(),
                    to_end: End::Start,
                    junction: Some("cross".into()),
                },
                LinkDoc {
                    from: "south".into(),
                    from_end: End::End,
                    to: "east".into(),
                    to_end: End::Start,
                    junction: Some("cross".into()),
                },
                LinkDoc {
                    from: "east".into(),
                    from_end: End::End,
                    to: "avenue".into(),
                    to_end: End::Start,
                    junction: None,
                },
            ],
            ..Document::default()
        }
    }
}
