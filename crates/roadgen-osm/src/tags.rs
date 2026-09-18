//! What an OSM way says about a road.
//!
//! OSM describes a road as one line down the middle with tags on it. A lane is a
//! *number*, not a shape; the width, the boundaries and the markings the IR holds
//! have nowhere to go. So this module is the whole of the lowering: everything the
//! format can carry about a road is in the tags it produces.

use ll2_io::osm::Tags;

use roadgen_core::buildings::{Building, BuildingPart, RoofShape};
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

/// Every tag the way carrying a whole building's outline needs.
///
/// OSM's `building` key is open: the tens of values the wiki lists are a convention
/// that renderers and routers know, not a schema anything enforces. So the word the
/// generator used goes down as it stands — a grammar that emits `house` produces
/// `building=house`, which is the conventional value, and one that emits something of
/// its own produces that rather than losing it to `building=yes`.
///
/// A building of one part carries its part's heights and roof here as well, because
/// that is what a reader of a simple building expects to find on the building itself.
/// One of several parts carries only what it is; the heights are on the parts, which
/// is what Simple 3D Buildings asks for.
pub fn building_tags(building: &Building, single: Option<&BuildingPart>, ground: f64) -> Tags {
    let mut tags = Tags::new();
    tags.insert("building".into(), building_value(&building.kind));
    if let Some(part) = single {
        add_solid(&mut tags, part, ground);
    }
    tags
}

/// Every tag one part of a multi-part building carries.
///
/// `building:part` is how Simple 3D Buildings says "this is a piece of the building
/// whose outline surrounds it, not a building of its own" — which is exactly what the
/// IR means by a part, and why a renderer that knows the scheme draws the massing
/// rather than a pile of houses. Its value is `yes` for a part that is simply part of
/// its building, and the part's own kind for one that is something else: the offices
/// above a shopping podium are offices, and OSM has room to say so.
pub fn building_part_tags(part: &BuildingPart, ground: f64) -> Tags {
    let mut tags = Tags::new();
    tags.insert(
        "building:part".into(),
        part.kind
            .as_deref()
            .map(building_value)
            .unwrap_or_else(|| "yes".to_owned()),
    );
    add_solid(&mut tags, part, ground);
    tags
}

/// The heights and the roof, which mean the same thing on a building and on a part.
///
/// `height` is measured from the ground to the top of the roof and `min_height` from
/// the ground to the bottom of the walls, which is how OSM states a part that starts
/// partway up: the two together are the band of space the part occupies.
fn add_solid(tags: &mut Tags, part: &BuildingPart, ground: f64) {
    let solid = &part.solid;
    // Heights in OSM are above the ground the building stands on, not above the
    // datum, so `ground` is the zero for both of them. The IR holds absolute heights,
    // which is what makes that subtraction possible at all.
    tags.insert("height".into(), metres(solid.top_height() - ground));
    let foot = solid.base_height() - ground;
    // `min_height` is where the walls start, which is how OSM says that a part hangs
    // above the ground rather than standing on it. A part that does stand on it says
    // nothing, because zero is the default and writing it would be noise.
    if foot > 0.05 {
        tags.insert("min_height".into(), metres(foot));
    }
    tags.insert("building:levels".into(), part.levels.to_string());
    if solid.roof.shape != RoofShape::Flat {
        tags.insert("roof:shape".into(), solid.roof.shape.as_str().to_owned());
        tags.insert("roof:height".into(), metres(solid.roof.height));
        if solid.roof.shape.is_directed() {
            // OSM measures a roof's direction clockwise from north, as a compass
            // bearing; the IR measures it anticlockwise from east, as an angle. The
            // conversion is the whole of the difference.
            tags.insert(
                "roof:direction".into(),
                format!(
                    "{:.0}",
                    (90.0 - solid.roof.direction.to_degrees()).rem_euclid(360.0)
                ),
            );
        }
    }
}

fn metres(value: f64) -> String {
    format!("{value:.1}")
}

/// A building's kind as an OSM value: lower case, and word characters only.
///
/// `building=yes` for a kind that survives none of that, which is OSM's own way of
/// saying "a building, and nothing more is claimed".
fn building_value(kind: &str) -> String {
    let value: String = kind
        .trim()
        .to_ascii_lowercase()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    let value = value.trim_matches('_').to_owned();
    if value.is_empty() {
        "yes".to_owned()
    } else {
        value
    }
}

#[cfg(test)]
mod building_tests {
    use roadgen_core::buildings::{Footprint, Roof, Solid};
    use roadgen_core::geometry::Point3;
    use roadgen_core::id::{BuildingId, BuildingPartId};

    use super::*;

    fn part(roof: Roof, base: f64, wall_height: f64) -> BuildingPart {
        kinded(roof, base, wall_height, None)
    }

    fn kinded(roof: Roof, base: f64, wall_height: f64, kind: Option<&str>) -> BuildingPart {
        let id = BuildingId::new("a");
        BuildingPart {
            id: BuildingPartId::of_building(&id, 0),
            building: id,
            kind: kind.map(str::to_owned),
            solid: Solid {
                footprint: Footprint::new(vec![
                    Point3::new(0.0, 0.0, base),
                    Point3::new(10.0, 0.0, base),
                    Point3::new(10.0, 8.0, base),
                    Point3::new(0.0, 8.0, base),
                ])
                .unwrap(),
                wall_height,
                roof,
            },
            levels: 3,
        }
    }

    fn building(kind: &str) -> Building {
        Building {
            id: BuildingId::new("a"),
            parts: vec![BuildingPartId::of_building(&BuildingId::new("a"), 0)],
            kind: kind.to_owned(),
            frontage: None,
        }
    }

    #[test]
    fn a_buildings_kind_becomes_the_building_value() {
        let single = part(Roof::FLAT, 0.0, 9.5);
        let tags = building_tags(&building("apartments"), Some(&single), 0.0);
        assert_eq!(tags.get("building").map(String::as_str), Some("apartments"));
        assert_eq!(tags.get("building:levels").map(String::as_str), Some("3"));
        assert_eq!(tags.get("height").map(String::as_str), Some("9.5"));
        // A flat roof is what a building has when nothing says otherwise, so saying
        // so would be noise.
        assert!(!tags.contains_key("roof:shape"));
    }

    #[test]
    fn a_kind_osm_could_not_spell_becomes_one_it_can() {
        let single = part(Roof::FLAT, 0.0, 9.5);
        assert_eq!(
            building_tags(&building("Semi Detached"), Some(&single), 0.0)
                .get("building")
                .map(String::as_str),
            Some("semi_detached")
        );
        assert_eq!(
            building_tags(&building("  "), Some(&single), 0.0)
                .get("building")
                .map(String::as_str),
            Some("yes")
        );
    }

    #[test]
    fn a_pitched_roof_is_written_as_a_bearing_from_north() {
        // A ridge running east-west is an angle of zero in the IR, and a bearing of
        // ninety in OSM.
        let east = part(Roof::new(RoofShape::Gabled, 3.0, 0.0), 0.0, 6.0);
        let tags = building_part_tags(&east, 0.0);
        assert_eq!(tags.get("roof:shape").map(String::as_str), Some("gabled"));
        assert_eq!(tags.get("roof:height").map(String::as_str), Some("3.0"));
        assert_eq!(tags.get("roof:direction").map(String::as_str), Some("90"));
        // The building is as tall as its walls and its roof together.
        assert_eq!(tags.get("height").map(String::as_str), Some("9.0"));

        // And one running north-south is a bearing of zero.
        let north = part(
            Roof::new(RoofShape::Gabled, 3.0, std::f64::consts::FRAC_PI_2),
            0.0,
            6.0,
        );
        assert_eq!(
            building_part_tags(&north, 0.0)
                .get("roof:direction")
                .map(String::as_str),
            Some("0")
        );
    }

    #[test]
    fn a_part_says_it_is_a_part_and_where_it_starts() {
        // A part standing four metres up on a building whose ground is zero: eight
        // metres of wall, so it reaches twelve.
        let tags = building_part_tags(&part(Roof::FLAT, 4.0, 8.0), 0.0);
        assert_eq!(tags.get("building:part").map(String::as_str), Some("yes"));
        // A part that is something of its own says what.
        assert_eq!(
            building_part_tags(&kinded(Roof::FLAT, 4.0, 8.0, Some("office")), 0.0)
                .get("building:part")
                .map(String::as_str),
            Some("office")
        );
        assert!(!tags.contains_key("building"));
        assert_eq!(tags.get("min_height").map(String::as_str), Some("4.0"));
        assert_eq!(tags.get("height").map(String::as_str), Some("12.0"));

        // The same part on a building whose ground is at four: it stands on it, so
        // there is no `min_height` to write.
        let tags = building_part_tags(&part(Roof::FLAT, 4.0, 8.0), 4.0);
        assert!(!tags.contains_key("min_height"));
        assert_eq!(tags.get("height").map(String::as_str), Some("8.0"));
    }
}
