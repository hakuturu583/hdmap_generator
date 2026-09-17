//! What a SUMO edge and lane say about a road.
//!
//! SUMO describes a road by what may drive on it and how fast, not by what is
//! painted on it. So the lowering is three small tables: which vehicle classes a
//! lane type admits, how fast a road type is when the map gives no limit, and where
//! a road type sits in the priority order netconvert uses to work out who yields.

use roadgen_core::semantics::{LaneType, RoadType};

/// What a SUMO lane says about who may use it.
///
/// SUMO's two attributes are complementary — `allow` names the only classes
/// admitted, `disallow` names the ones kept out of an otherwise open lane — and
/// which of the two is right depends on the lane. A driving lane is open to
/// everything on wheels, so it is written as a lane pedestrians are kept out of; a
/// footway admits pedestrians and nothing else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Permission {
    /// `allow`: these classes and no others.
    Allow(&'static str),
    /// `disallow`: everything except these.
    Disallow(&'static str),
}

impl Permission {
    pub fn attribute(self) -> (&'static str, &'static str) {
        match self {
            Permission::Allow(classes) => ("allow", classes),
            Permission::Disallow(classes) => ("disallow", classes),
        }
    }
}

/// Who may use a lane of this type, or `None` for a lane SUMO has no place for.
///
/// A SUMO lane is a strip traffic of some class runs along. A border, a painted
/// island or a parking bay is none of those: it is part of the road surface with no
/// movement on it, and SUMO models it — when it models it at all — as a shape in a
/// separate additional file rather than as a lane. Such lanes are dropped, and
/// [`crate::check`] names them.
pub fn permission(lane_type: LaneType) -> Option<Permission> {
    Some(match lane_type {
        LaneType::Driving => Permission::Disallow("pedestrian"),
        LaneType::Biking => Permission::Allow("bicycle"),
        LaneType::Sidewalk => Permission::Allow("pedestrian"),
        // A hard shoulder is a lane in SUMO's sense — it is where a broken-down
        // vehicle stops and an ambulance passes — but not one open traffic uses.
        LaneType::Shoulder => Permission::Allow("emergency"),
        LaneType::Border | LaneType::Parking | LaneType::Restricted | LaneType::None => {
            return None
        }
    })
}

/// How fast a road of this type is when the map states no limit, metres per second.
///
/// SUMO requires a speed on every edge — it is what the car-following model works
/// from — so there is no writing "unknown". These are SUMO's own defaults for the
/// OSM `highway` values each road type maps to, which is what a network built from
/// OSM data would carry.
pub fn default_speed(road_type: RoadType) -> f64 {
    match road_type {
        RoadType::Motorway => 36.11,
        RoadType::Rural => 22.22,
        RoadType::Town => 13.89,
        RoadType::LowSpeed => 8.33,
        RoadType::Pedestrian => 2.78,
    }
}

/// Where a road of this type sits in SUMO's priority order.
///
/// netconvert reads this to decide who yields at an uncontrolled junction: the arm
/// with the higher priority keeps right of way and the lower one gets a give-way
/// line. The ladder is the same one the OpenStreetMap export climbs, so the two
/// formats rank a map's roads identically.
pub fn priority(road_type: RoadType) -> i32 {
    match road_type {
        RoadType::Motorway => 13,
        RoadType::Rural => 9,
        RoadType::Town => 4,
        RoadType::LowSpeed => 3,
        RoadType::Pedestrian => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_lanes_traffic_runs_along_become_sumo_lanes() {
        assert_eq!(
            permission(LaneType::Driving),
            Some(Permission::Disallow("pedestrian"))
        );
        assert_eq!(
            permission(LaneType::Sidewalk),
            Some(Permission::Allow("pedestrian"))
        );
        assert_eq!(permission(LaneType::Border), None);
        assert_eq!(permission(LaneType::None), None);
    }

    #[test]
    fn the_priority_ladder_puts_a_motorway_above_a_living_street() {
        assert!(priority(RoadType::Motorway) > priority(RoadType::Rural));
        assert!(priority(RoadType::Rural) > priority(RoadType::Town));
        assert!(priority(RoadType::Town) > priority(RoadType::LowSpeed));
        assert!(priority(RoadType::LowSpeed) > priority(RoadType::Pedestrian));
    }

    #[test]
    fn every_road_type_has_a_speed_to_fall_back_on() {
        for road_type in [
            RoadType::Motorway,
            RoadType::Rural,
            RoadType::Town,
            RoadType::LowSpeed,
            RoadType::Pedestrian,
        ] {
            assert!(default_speed(road_type) > 0.0);
        }
    }
}
