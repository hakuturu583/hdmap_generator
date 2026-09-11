//! The Lanelet2 and Autoware tag vocabulary.
//!
//! Every key and value here comes from the Lanelet2 specification or from
//! `autoware_lanelet2_extension`. None of it is invented for this project: an
//! Autoware-specific tag that does not already exist upstream would be a tag
//! Autoware does not read.

use ll2_core::attribute::{Attribute, AttributeMap};

use roadgen_core::semantics::{LaneType, MapObjectKind, RoadMarking};

/// Builds an attribute map from `(key, value)` pairs.
pub fn attributes(pairs: impl IntoIterator<Item = (&'static str, String)>) -> AttributeMap {
    pairs
        .into_iter()
        .map(|(key, value)| (key.to_owned(), Attribute::new(value)))
        .collect()
}

/// `type` and `subtype` for a lane boundary carrying a given marking.
pub fn boundary_tags(marking: RoadMarking) -> (&'static str, Option<&'static str>) {
    match marking {
        // A boundary with no paint still exists geometrically; Lanelet2 calls that
        // `virtual`.
        RoadMarking::None => ("virtual", None),
        RoadMarking::Solid => ("line_thin", Some("solid")),
        RoadMarking::Broken => ("line_thin", Some("dashed")),
        RoadMarking::SolidSolid => ("line_thick", Some("solid_solid")),
        RoadMarking::SolidBroken => ("line_thin", Some("solid_dashed")),
        RoadMarking::BrokenSolid => ("line_thin", Some("dashed_solid")),
        RoadMarking::Curbstone => ("curbstone", Some("high")),
    }
}

/// The `subtype` of the lanelet a lane of this type becomes, if it becomes one.
pub fn lanelet_subtype(lane_type: LaneType) -> Option<&'static str> {
    match lane_type {
        LaneType::Driving => Some("road"),
        LaneType::Biking => Some("bicycle_lane"),
        LaneType::Sidewalk => Some("walkway"),
        // A shoulder, a border or a parking strip is part of the road surface but
        // carries no through traffic, so it is not a lanelet.
        _ => None,
    }
}

/// The participant tag a lanelet of this subtype carries.
pub fn participant(subtype: &str) -> &'static str {
    match subtype {
        "walkway" | "crosswalk" => "participant:pedestrian",
        "bicycle_lane" => "participant:bicycle",
        _ => "participant:vehicle",
    }
}

/// The `type` of the linestring a map object becomes.
pub fn object_type(kind: &MapObjectKind) -> &'static str {
    match kind {
        MapObjectKind::TrafficLight => "traffic_light",
        MapObjectKind::TrafficSign { .. } => "traffic_sign",
        MapObjectKind::StopLine => "stop_line",
        MapObjectKind::Crosswalk => "pedestrian_marking",
    }
}
