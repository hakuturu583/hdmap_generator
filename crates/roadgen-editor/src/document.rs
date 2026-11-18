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
    /// x, y, z in metres, at least two.
    pub points: Vec<[f64; 3]>,
    pub shape: Shape,
    pub lanes: Vec<LaneDoc>,
    /// `town`, `rural`, `motorway`, `low_speed` or `pedestrian`.
    pub road_type: String,
    /// km/h.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub speed_limit: Option<f64>,
}

impl Default for RoadDoc {
    fn default() -> Self {
        RoadDoc {
            id: String::new(),
            points: Vec::new(),
            shape: Shape::Smooth,
            lanes: vec![
                LaneDoc::driving(3.5, Travel::Forward),
                LaneDoc::driving(3.5, Travel::Backward),
            ],
            road_type: "town".to_owned(),
            speed_limit: None,
        }
    }
}

impl RoadDoc {
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
        }
    }
}

impl Document {
    pub const VERSION: u32 = 1;

    pub fn road(&self, id: &str) -> Option<&RoadDoc> {
        self.roads.iter().find(|road| road.id == id)
    }

    /// A small town to start from: a crossroads, one arm of it bending away and
    /// carrying on past a straight joint into a road with a pavement.
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
        Document {
            name: "example".to_owned(),
            roads: vec![
                RoadDoc::two_way("west", vec![[-80.0, 0.0, z], [-12.0, 0.0, z]]),
                east,
                RoadDoc::two_way("south", vec![[0.0, -80.0, z], [0.0, -12.0, z]]),
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
