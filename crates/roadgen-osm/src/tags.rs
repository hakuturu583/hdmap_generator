//! What an OSM way says about a road.
//!
//! OSM describes a road as one line down the middle with tags on it. A lane is a
//! *number*, not a shape; the width, the boundaries and the markings the IR holds
//! have nowhere to go. So this module is the whole of the lowering: everything the
//! format can carry about a road is in the tags it produces.

use ll2_io::osm::Tags;

use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{LaneType, RoadType};
use roadgen_core::topology::Direction;

/// The `highway` value for a road.
///
/// OSM's values are a road's place in the network rather than its shape, so this is
/// a judgement rather than a translation: it puts each `RoadType` at the rung of the
/// hierarchy that carries the same traffic.
pub fn highway_value(road_type: RoadType) -> &'static str {
    match road_type {
        RoadType::Motorway => "motorway",
        RoadType::Rural => "tertiary",
        RoadType::Town => "residential",
        RoadType::LowSpeed => "living_street",
        RoadType::Pedestrian => "pedestrian",
    }
}

/// Every tag a road's way carries.
pub fn road_tags(map: &Map, road: &Road) -> Tags {
    let mut tags = Tags::new();
    tags.insert("highway".into(), highway_value(road.road_type).into());
    if let Some(name) = &road.name {
        tags.insert("name".into(), name.clone());
    }
    if let Some(limit) = road.speed_limit {
        // OSM's bare `maxspeed` is km/h, and a whole number of them is what a sign
        // says, so it is written as one.
        tags.insert("maxspeed".into(), format!("{}", limit.kph().round()));
    }

    // Lanes are counted over the road's *first* cross-section. A road whose lane
    // count changes along its length cannot be described by one way at all, which
    // `check` reports rather than papering over.
    let lanes = map.lanes_of_section(&road.id, 0);
    let forward = lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable() && lane.direction == Direction::Forward)
        .count();
    let backward = lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable() && lane.direction == Direction::Backward)
        .count();

    if forward + backward > 0 {
        tags.insert("lanes".into(), (forward + backward).to_string());
    }
    match (forward, backward) {
        // Every lane runs with the way's node order, or against it. `-1` is OSM's
        // way of saying the second, and it is why the node order has to follow the
        // reference line rather than being reversed for convenience.
        (_, 0) => {
            tags.insert("oneway".into(), "yes".into());
        }
        (0, _) => {
            tags.insert("oneway".into(), "-1".into());
        }
        _ => {
            tags.insert("oneway".into(), "no".into());
            tags.insert("lanes:forward".into(), forward.to_string());
            tags.insert("lanes:backward".into(), backward.to_string());
        }
    }

    if let Some(side) = sidewalk_value(map, road) {
        tags.insert("sidewalk".into(), side.into());
    }
    tags
}

/// `left`, `right` or `both`, from where the footways are in the cross-section.
fn sidewalk_value(map: &Map, road: &Road) -> Option<&'static str> {
    use roadgen_core::topology::LateralSide;
    let lanes = map.lanes_of_section(&road.id, 0);
    let footway = |side: LateralSide| {
        lanes
            .iter()
            .any(|lane| lane.lane_type == LaneType::Sidewalk && lane.side == side)
    };
    match (footway(LateralSide::Left), footway(LateralSide::Right)) {
        (true, true) => Some("both"),
        (true, false) => Some("left"),
        (false, true) => Some("right"),
        (false, false) => None,
    }
}

/// Which way a restriction turns, from the heading change through the junction.
///
/// Headings are radians measured counter-clockwise, so a positive change is a left
/// turn. A `None` is a u-turn, which is deliberately not written: the IR enumerates
/// movements between *different* arms, so it never says anything about turning back
/// the way you came, and an absent u-turn is silence rather than a prohibition.
pub fn restriction_value(change: f64) -> Option<&'static str> {
    let change = normalize(change);
    let degrees = change.to_degrees();
    match degrees {
        d if d.abs() <= 45.0 => Some("no_straight_on"),
        d if d > 45.0 && d < 135.0 => Some("no_left_turn"),
        d if d < -45.0 && d > -135.0 => Some("no_right_turn"),
        _ => None,
    }
}

/// An angle brought into `(-π, π]`.
fn normalize(angle: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut value = angle % two_pi;
    if value > std::f64::consts::PI {
        value -= two_pi;
    } else if value <= -std::f64::consts::PI {
        value += two_pi;
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_turn_is_named_by_how_far_the_heading_swings() {
        assert_eq!(restriction_value(0.0), Some("no_straight_on"));
        assert_eq!(
            restriction_value(std::f64::consts::FRAC_PI_2),
            Some("no_left_turn")
        );
        assert_eq!(
            restriction_value(-std::f64::consts::FRAC_PI_2),
            Some("no_right_turn")
        );
        assert_eq!(restriction_value(std::f64::consts::PI), None, "a u-turn");
        // And it does not matter which way round the angle was measured.
        assert_eq!(
            restriction_value(-3.0 * std::f64::consts::FRAC_PI_2),
            Some("no_left_turn")
        );
    }

    #[test]
    fn the_highway_hierarchy_has_a_rung_for_every_road_type() {
        for road_type in [
            RoadType::Motorway,
            RoadType::Rural,
            RoadType::Town,
            RoadType::LowSpeed,
            RoadType::Pedestrian,
        ] {
            assert!(!highway_value(road_type).is_empty());
        }
    }
}
