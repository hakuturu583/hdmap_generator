//! The Lanelet2 and Autoware tag vocabulary.
//!
//! Every key and value here comes from the Lanelet2 specification or from
//! `autoware_lanelet2_extension`. None of it is invented for this project: an
//! Autoware-specific tag that does not already exist upstream would be a tag
//! Autoware does not read.

use ll2_core::attribute::{Attribute, AttributeMap};

use roadgen_core::semantics::{FormOfWay, LaneType, MapObjectKind, RoadMarking};

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

/// The subtype a lanelet of a road with this form of way takes instead of
/// `lane_subtype`.
///
/// Lanelet2 has a `highway` subtype for a road only motor vehicles use and that is
/// entered and left only at its ramps — which is what the IR's controlled-access
/// forms say — and the traffic rules read it to keep pedestrians and bicycles off.
/// A motorway carriageway's driving lanes and a slip road's are `highway`; any
/// other lanelet keeps the subtype its lane type gives it, since a walkway beside a
/// slip road, if a map has one, is still a walkway.
pub fn road_subtype(lane_subtype: &'static str, form: FormOfWay) -> &'static str {
    if lane_subtype == "road" && form.is_controlled_access() {
        "highway"
    } else {
        lane_subtype
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

/// The `turn_direction` of a lanelet through a junction whose heading swings by
/// `swing` radians, counter-clockwise positive: `straight` within 45 degrees either
/// way, else `left` or `right`.
///
/// 45 degrees is half a right angle, so a turn at an ordinary crossroads (90) and
/// a straight on that bends to meet a skewed arm (CARLA's run to about 15) fall
/// clearly to either side.
pub fn turn_direction(swing: f64) -> &'static str {
    use std::f64::consts::{PI, TAU};
    let swing = (swing + PI).rem_euclid(TAU) - PI;
    if swing.abs() <= PI / 4.0 {
        "straight"
    } else if swing > 0.0 {
        "left"
    } else {
        "right"
    }
}

/// Is a traffic sign of this catalogue code a stop sign?
///
/// `stop` however it is spelt, `R1-1` (MUTCD) or `206` (the Vienna Convention, the
/// German catalogue OpenDRIVE and CARLA's towns write, with its country prefix or
/// without); a catalogue subtype after a slash says nothing more about which sign.
pub fn is_stop_sign(code: &str) -> bool {
    let code = code.split('/').next().unwrap_or_default();
    let code = code.trim().to_ascii_lowercase().replace([' ', '_'], "-");
    code.starts_with("stop") || code == "r1-1" || code == "206" || code == "de206"
}

/// The `subtype` of a traffic sign's way: `stop_sign` for a stop sign, which is how
/// `autoware_lanelet2_extension` names the sign a `traffic_sign` element's stop line
/// is for, and the caller's own catalogue code for any other.
pub fn sign_subtype(code: &str) -> String {
    if is_stop_sign(code) {
        "stop_sign".to_owned()
    } else {
        code.to_owned()
    }
}

/// The `type` of the linestring a map object becomes.
pub fn object_type(kind: &MapObjectKind) -> &'static str {
    match kind {
        MapObjectKind::TrafficLight { .. } => "traffic_light",
        MapObjectKind::TrafficSign { .. } => "traffic_sign",
        MapObjectKind::StopLine => "stop_line",
        MapObjectKind::Crosswalk => "pedestrian_marking",
        // Lanelet2 has no berm; what one is to a vehicle is the edge of the road it
        // may not cross.
        MapObjectKind::Barrier { .. } => "road_border",
    }
}

#[cfg(test)]
mod tests {
    use super::{sign_subtype, turn_direction};
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn a_stop_sign_is_named_as_autoware_names_it_whatever_catalogue_it_came_from() {
        for code in ["206", "de206", "206/-1", "stop", "Stop_Sign", "R1-1"] {
            assert_eq!(sign_subtype(code), "stop_sign", "{code}");
        }
        // Any other sign keeps the caller's code.
        for code in ["205", "de274-60", "R1-2", "1000001"] {
            assert_eq!(sign_subtype(code), code);
        }
    }

    #[test]
    fn a_swing_reads_as_the_turn_it_makes_whichever_way_round_it_is_measured() {
        assert_eq!(turn_direction(0.0), "straight");
        assert_eq!(turn_direction(0.25), "straight");
        assert_eq!(turn_direction(FRAC_PI_2), "left");
        assert_eq!(turn_direction(-FRAC_PI_2), "right");
        // Headings either side of +-pi: a quarter turn left measured across the seam.
        assert_eq!(turn_direction(-1.5 * PI), "left");
        assert_eq!(turn_direction(2.0 * PI - 0.1), "straight");
    }
}
