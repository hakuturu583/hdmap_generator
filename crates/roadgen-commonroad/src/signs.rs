//! Which of the IR's signs CommonRoad can hold, and as what.
//!
//! The IR keeps a sign's code in whatever catalogue the caller used. A CommonRoad
//! scenario written here is set in Zamunda — the fictional country CommonRoad keeps
//! for scenarios that are nowhere real — whose signs are Germany's, so a code is
//! written when it is one of those: a stop sign however it is spelt, as `206`, and
//! any other German sign by its own number, with or without a `DE` prefix. A speed
//! limit written as `274-50` is `274` with its limit as the additional value, in
//! metres per second, as CommonRoad keeps it. Anything else is dropped, and
//! [`crate::check`] says how many.

use roadgen_core::semantics::is_stop_sign;

use crate::model::{number, TrafficSignElement};

/// Maximum speed.
pub const MAX_SPEED: &str = "274";
/// Give way.
pub const YIELD: &str = "205";
/// Stop.
pub const STOP: &str = "206";

/// The sign ids both CommonRoad's 2020a schema and its reader's Zamunda catalogue
/// know, each with the name the 2024 protobuf format's `TrafficSignID` gives it:
/// the XML scenario writes the number, the protobuf map the name.
const CATALOGUE: &[(&str, &str)] = &[
    ("101", "WARNING_DANGER_SPOT"),
    ("102", "WARNING_RIGHT_BEFORE_LEFT"),
    ("103-10", "WARNING_LEFT_CURVE"),
    ("103-20", "WARNING_RIGHT_CURVE"),
    ("108", "WARNING_STEEP_HILL_DOWNWARDS"),
    ("114", "WARNING_SLIPPERY_ROAD"),
    ("123", "WARNING_CONSTRUCTION_SITE"),
    ("124", "WARNING_TRAFFIC_QUEUES_LIKELY"),
    ("125", "WARNING_ONCOMING_TRAFFIC"),
    ("131", "WARNING_TRAFFIC_LIGHTS_AHEAD"),
    ("133-10", "WARNING_PEDESTRIANS_RIGHT"),
    ("133-20", "WARNING_PEDESTRIANS_LEFT"),
    ("138", "WARNING_CROSSING_CYCLIST"),
    ("142-10", "WARNING_ANIMAL_CROSSING_RIGHT"),
    ("145-50", "WARNING_LOOSE_GRAVEL"),
    ("201", "RAILWAY"),
    ("205", "YIELD"),
    ("206", "STOP"),
    ("208", "PRIORITY_OPPOSITE_DIRECTION"),
    ("209-10", "TURN_RIGHT_AHEAD"),
    ("209-20", "TURN_LEFT_AHEAD"),
    ("211-20", "PRESCRIBED_DIRECTION_RIGHT"),
    ("215", "ROUNDABOUT"),
    ("220-10", "ONEWAY_RIGHT"),
    ("220-20", "ONEWAY_LEFT"),
    ("222-10", "PRESCRIBED_PASSING_LEFT"),
    ("222-20", "PRESCRIBED_PASSING_RIGHT"),
    ("223.2", "DO_NOT_DRIVE_ON_SHOULDER_LANE"),
    ("223.2-50", "DO_NOT_DRIVE_ON_SHOULDER_LANE_2_LANE"),
    ("223.2-51", "DO_NOT_DRIVE_ON_SHOULDER_LANE_3_LANE"),
    ("224-50", "BUS_STOP"),
    ("237", "BIKEWAY"),
    ("239", "PEDESTRIAN_SIDEWALK"),
    ("240", "PEDESTRIAN_AND_BICYCLE_ROAD"),
    ("242.1", "PEDESTRIAN_ZONE_START"),
    ("242.2", "PEDESTRIAN_ZONE_END"),
    ("244.1", "BICYCLE_ROAD_START"),
    ("244.2", "BICYCLE_ROAD_END"),
    ("245", "BUS_LANE"),
    ("250", "BAN_ALL_VEHICLES"),
    ("251", "BAN_CARS"),
    ("253", "BAN_TRUCKS"),
    ("254", "BAN_BICYCLE"),
    ("255", "BAN_MOTORCYCLE"),
    ("257-54", "BAN_BUS"),
    ("259", "BAN_PEDESTRIAN"),
    ("260", "BAN_CAR_TRUCK_BUS_MOTORCYCLE"),
    ("261", "BAN_VEHICLES_CARRYING_DANGEROUS_GOODS"),
    ("262", "MAX_WEIGHT"),
    ("264", "MAX_WIDTH"),
    ("265", "MAX_HEIGHT"),
    ("266", "MAX_LENGTH"),
    ("267", "NO_ENTRY"),
    ("270.1", "ENVIRONMENTAL_ZONE_START"),
    ("270.2", "ENVIRONMENTAL_ZONE_END"),
    ("272", "U_TURN"),
    ("274", "MAX_SPEED"),
    ("274.1", "MAX_SPEED_ZONE_START"),
    ("274.2", "MAX_SPEED_ZONE_END"),
    ("275", "MIN_SPEED"),
    ("276", "NO_OVERTAKING_START"),
    ("277", "NO_OVERTAKING_TRUCKS_START"),
    ("278", "MAX_SPEED_END"),
    ("280", "NO_OVERTAKING_END"),
    ("281", "NO_OVERTAKING_TRUCKS_END"),
    ("282", "ALL_MAX_SPEED_AND_OVERTAKING_END"),
    ("283-10", "NO_STOP_START_RIGHT"),
    ("283-30", "NO_STOP_CENTER_RIGHT"),
    ("286-30", "RESTRICTED_STOP_CENTER_RIGHT"),
    ("301", "RIGHT_OF_WAY"),
    ("306", "PRIORITY"),
    ("308", "PRIORITY_OVER_ONCOMING"),
    ("310", "TOWN_SIGN"),
    ("311", "TOWN_SIGN_BACK"),
    ("314", "PARKING_AREA"),
    ("314-10", "PARKING_AREA_LEFT"),
    ("314-20", "PARKING_AREA_RIGHT"),
    ("314-30", "PARKING_AREA_RIGHT_LEFT"),
    ("325.1", "TRAFFIC_CALMED_AREA_START"),
    ("325.2", "TRAFFIC_CALMED_AREA_END"),
    ("327", "TUNNEL"),
    ("328", "EMERGENCY_STOP"),
    ("330.1", "INTERSTATE_START"),
    ("330.2", "INTERSTATE_END"),
    ("331.1", "HIGHWAY_START"),
    ("331.2", "HIGHWAY_END"),
    ("332", "HIGHWAY_EXIT_WITH_PLACE_NAME"),
    ("332.1", "EXIT_ROUTE"),
    ("333", "HIGHWAY_EXIT"),
    ("333-21", "EXIT_BUILT_UP"),
    ("333-22", "EXIT_GENERAL"),
    ("350", "PEDESTRIANS_CROSSING"),
    ("354", "WATER_PROTECTION_ZONE"),
    ("356", "TRAFFIC_ASSISTANTS"),
    ("357", "DEAD_END"),
    ("363", "POLICE"),
    ("365-51", "EMERGENCY_CALL_STATION"),
    ("365-52", "GAS_STATION"),
    ("365-60", "CAMP_AND_CARAVAN_SITE"),
    ("386.1", "ATTRACTION_POINT"),
    ("386.2", "TOURISTIC_ROUTE"),
    ("386.3", "NEARBY_ATTRACTION_POINT"),
    ("406-50", "HIGHWAY_INTERSECTION"),
    ("418-20", "DIRECTION_ARROW_SIGN_MULTI"),
    ("419-20", "DIRECTION_ARROW_SIGN_SINGLE"),
    ("434-50", "DIRECTION_SIGN_CONSOLIDATED"),
    ("430-20", "EXPRESSWAY_ARROW_DIRECTION"),
    ("432-10", "ARROW_SIGN_POST_POINT_OF_INTEREST_LEFT"),
    ("432-20", "STATION"),
    ("434", "GUIDE_SIGN_TABLE"),
    ("438", "ADVANCE_DIRECTION"),
    ("439", "DIRECTIONS_SIGN"),
    ("440", "EXPRESSWAY_ENTRANCE_DIRECTIONS"),
    ("448", "INTERSTATE_ANNOUNCEMENT"),
    ("449", "INTERSTATE_ADVANCE_DIRECTION"),
    ("450-50", "HIGHWAY_EXIT_AHEAD_100_METER"),
    ("450-51", "HIGHWAY_EXIT_AHEAD_200_METER"),
    ("450-52", "HIGHWAY_EXIT_AHEAD_300_METER"),
    ("450-53", "EXPRESSWAY_EXIT_100_METRES"),
    ("450-54", "EXPRESSWAY_EXIT_200_METRES"),
    ("450-55", "EXPRESSWAY_EXIT_300_METRES"),
    ("453", "INTERSTATE_DISTANCE"),
    ("458", "DETOUR_SKETCH"),
    ("455.1-30", "DETOUR_STRAIGHT"),
    ("460-10", "DETOUR_ON_DEMAND_LEFT"),
    ("460-12", "DETOUR_ON_DEMAND_GET_IN_LEFT_LANE"),
    ("460-20", "DETOUR_ON_DEMAND_ANNOUNCEMENT_RIGHT"),
    ("460-21", "DETOUR_ON_DEMAND_RIGHT"),
    ("460-22", "DETOUR_ON_DEMAND_GET_IN_RIGHT_LANE"),
    ("460-30", "DETOUR_ON_DEMAND_STRAIGHTFORWARD"),
    ("501-15", "TRANSITION_3_LEFT_2_TRANSITIONED"),
    ("501-16", "TRANSITION_1_LEFT_1_STRAIGHT"),
    ("511-22", "TRANSITION_3_RIGHT"),
    ("521-30", "LANE_BOARD_NO_OPPOSITE_TWO_LANES"),
    ("521-31", "THREE_LANES_NO_ONCOMING_LANES"),
    ("521-32", "FOUR_LANES_NO_ONCOMING_LANES"),
    ("521-33", "FIVE_LANES_NO_ONCOMING_LANES"),
    ("531-10", "NARROWING_LANES_1_LANE_FROM_RIGHT"),
    ("531-20", "NARROWING_LANES_1_LANE_FROM_LEFT"),
    ("531-21", "NARROWING_LANES_2_LANES_PLUS_1_LEFT"),
    (
        "533-22",
        "FOUR_LANES_NO_ONCOMING_TRAFFIC_TWO_RIGHT_LANES_TURN_RIGHT",
    ),
    ("550-20", "MERGING_LANES_1_LANE_PLUS_1_LANE_RIGHT"),
    ("600-35", "BARRIER"),
    ("600-30", "BARRIER_GATE_100_800"),
    ("600-31", "BARRIER_GATE_100_1200"),
    ("600-32", "BARRIER_GATE_100_1600"),
    ("600-34", "BARRIER_GATE_250_1600"),
    ("600-38", "BARRIER_GATE"),
    ("605-10", "ROAD_WARNING_POST_SCRAPER_BEACON_RIGHT"),
    ("605-11", "ROAD_WARNING_POST_ARROW_BEACON_RIGHT"),
    ("605-20", "ROAD_WARNING_POST_SCRAPER_BEACON_LEFT"),
    ("605-21", "ROAD_WARNING_POST_SCRAPER_BEACON_ARROW_RIGHT"),
    ("605-31", "ROAD_WARNING_POST_GUIDE_UP_THREE_ARROWS"),
    ("625-10", "DIRECTION_SIGN_LEFT_SINGLE"),
    ("625-11", "DIRECTION_SIGN_LEFT_SMALL"),
    ("625-12", "DIRECTION_SIGN_LEFT_MEDIUM"),
    ("625-13", "DIRECTION_SIGN_LEFT_LARGE"),
    ("625-20", "DIRECTION_SIGN_RIGHT_SINGLE"),
    ("625-21", "DIRECTION_SIGN_RIGHT_SMALL"),
    ("625-22", "DIRECTION_SIGN_RIGHT_MEDIUM"),
    ("625-23", "DIRECTION_SIGN_RIGHT_LARGE"),
    ("626-10", "WARNING_PANEL_RIGHT"),
    ("626-20", "WARNING_PANEL_LEFT"),
    ("626-30", "WARNING_PANEL_STRAIGHT_BROAD"),
    ("626-31", "WARNING_PANEL_STRAIGHT_HIGH"),
    ("628-10", "GUIDE_SILL_WITH_GUIDE_BEACON_RIGHT"),
    ("629-10", "GUIDE_RAIL_WITH_GUIDE_BEACON_RIGHT"),
    ("629-20", "GUIDE_PANEL_WITH_GUIDE_BEACON_RIGHT"),
    ("720", "GREEN_ARROW"),
    ("1000", "ADDITION_LEFT_DIRECTION"),
    ("1000-10", "ADDITION_LEFT_DIRECTION_1"),
    ("1000-11", "ADDITION_LEFT_DIRECTION_DANGER_POINT"),
    ("1000-20", "ADDITION_RIGHT_DIRECTION_1"),
    ("1000-21", "ADDITION_RIGHT_DIRECTION_DANGER_POINT"),
    ("1000-30", "ADDITION_BOTH_DIRECTIONS_HORIZONTAL"),
    ("1000-31", "ADDITION_BOTH_DIRECTIONS_VERTICAL"),
    ("1001-30", "ADDITION_VALID_FOR_X_METERS"),
    ("1001-31", "ADDITION_VALID_FOR_X_KILOMETERS"),
    (
        "1002-10",
        "ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD",
    ),
    (
        "1002-11",
        "ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD",
    ),
    (
        "1002-12",
        "ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD",
    ),
    ("1002-13", "ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD"),
    (
        "1002-14",
        "ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD",
    ),
    (
        "1002-20",
        "ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD",
    ),
    (
        "1002-21",
        "ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD",
    ),
    (
        "1002-22",
        "ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD",
    ),
    ("1002-23", "ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD"),
    (
        "1002-24",
        "ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD",
    ),
    ("1004-30", "ADDITION_VALID_IN_X_METERS"),
    ("1004-31", "ADDITION_VALID_IN_X_KILOMETERS"),
    ("1004-32", "ADDITION_VALID_IN_200_KILOMETERS"),
    ("1004-33", "ADDITION_VALID_IN_400_METRES"),
    ("1004-34", "ADDITION_VALID_IN_600_METRES"),
    ("1004-35", "ADDITION_VALID_IN_2_KILOMETERS"),
    ("1006-30", "ADDITION_OIL_ON_ROAD"),
    ("1006-31", "ADDITION_SMOKE"),
    ("1006-32", "ADDITION_LOOSE_GRAVEL"),
    ("1006-33", "ADDITION_BUILDING_SITE_EXIT"),
    ("1006-34", "ADDITION_DAMAGED_ROAD"),
    ("1006-35", "ADDITION_DIRTY_ROAD"),
    ("1006-36", "ADDITION_DANGER_OF_COLLISION"),
    ("1006-37", "ADDITION_TOAD_MIGRATION"),
    ("1006-38", "ADDITION_DANGER_OF_CONGESTION"),
    ("1006-39", "ADDITION_RESTRICTED_VIEW_DUE_TO_TREES"),
    ("1007-31", "DANGER_INDICATION_SMOKE"),
    ("1010-10", "ADDITION_CHILDREN_PLAYING_ON_ROAD"),
    ("1010-11", "ADDITION_WINTER_SPORTS_ALLOWED"),
    (
        "1010-12",
        "ADDITION_TRAILERS_ALLOWED_TO_PARK_MORE_THAN_14_DAYS",
    ),
    (
        "1010-13",
        "ADDITION_CARAVANS_ALLOWED_TO_PARK_MORE_THAN_14_DAYS",
    ),
    ("1010-14", "ADDITION_ROLLING_HIGHWAY"),
    ("1012-30", "ADDITION_LOADING_AREA"),
    ("1012-31", "ADDITION_END"),
    ("1012-32", "ADDITION_GET_OFF_BICYCLES"),
    ("1012-33", "ADDITION_NO_MOPEDS"),
    ("1012-34", "ADDITION_GREEN_WAVE_AT_KM_H"),
    ("1012-35", "ADDITION_STOP_HERE_AT_RED"),
    ("1012-36", "ADDITION_NOISE_CONTROL"),
    ("1012-37", "ADDITION_INFLOW_REGULATION"),
    ("1012-38", "ADDITION_SECONDARY_LANE"),
    ("1020-30", "ADDITION_RESIDENTS_PERMITTED"),
    ("1022-10", "ADDITION_BICYCLES_PERMITTED"),
    ("1024-10", "ADDITION_CARS_PERMITTED"),
    ("1026-36", "ADDITION_AGRICULTURE_PERMITTED"),
    ("1026-37", "ADDITION_FOREST_PERMITTED"),
    ("1026-38", "ADDITION_AGRICULTURE_FOREST_PERMITTED"),
    ("1031-52", "ADDITION_GREEN_STICKER_PERMITTED"),
    ("1040-30", "ADDITION_TIME_PERIOD_PERMITTED"),
    ("1048-12", "ADDITION_MOTOR_VEHICLES_ALLOWED_MASS_3_5_TONS"),
    ("1049-13", "ADDITION_MIN_MASS_3_5_TONS"),
    ("1052-31", "ADDITION_NO_WATER_POLLUTANTS_LOADED"),
    ("1053-33", "ALLOWED_MASS_7_5_TONS"),
    ("1053-34", "ADDITION_VALID_ON_SHOULDER"),
    ("1053-35", "ADDITION_VALID_WHEN_WET"),
    ("2113", "LINE_MARKING_MISSING"),
];

/// The sign element a code is written as, or `None` when CommonRoad has no such
/// sign.
pub(crate) fn element(code: &str) -> Option<TrafficSignElement> {
    if is_stop_sign(code) || is_stop_sign(strip(code)) {
        return Some(TrafficSignElement {
            sign_id: STOP,
            additional_values: Vec::new(),
        });
    }
    let code = strip(code);
    if let Some(kph) = code
        .strip_prefix("274-")
        .and_then(|kph| kph.parse::<f64>().ok())
        .filter(|kph| kph.is_finite() && *kph > 0.0)
    {
        return Some(TrafficSignElement {
            sign_id: MAX_SPEED,
            additional_values: vec![number(kph / 3.6)],
        });
    }
    // A bare maximum-speed sign says no speed, which is no rule at all.
    if code == MAX_SPEED {
        return None;
    }
    CATALOGUE
        .iter()
        .find(|(number, _)| *number == code)
        .map(|(number, _)| TrafficSignElement {
            sign_id: number,
            additional_values: Vec::new(),
        })
}

/// The name the 2024 protobuf format gives a sign of the catalogue: `MAX_SPEED`
/// for `274`.
pub(crate) fn name(sign_id: &str) -> Option<&'static str> {
    CATALOGUE
        .iter()
        .find(|(number, _)| *number == sign_id)
        .map(|(_, name)| *name)
}

/// A code without the catalogue's country prefix or a subtype after a slash.
fn strip(code: &str) -> &str {
    let code = code.split('/').next().unwrap_or_default().trim();
    code.strip_prefix("DE")
        .or_else(|| code.strip_prefix("de"))
        .map(|rest| rest.trim_start_matches(['_', '-', ' ', '.']))
        .unwrap_or(code)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stop_sign_is_206_however_it_is_spelt() {
        for code in ["stop", "STOP_SIGN", "R1-1", "206", "DE206", "de_206"] {
            assert_eq!(element(code).unwrap().sign_id, STOP, "{code}");
        }
    }

    #[test]
    fn a_speed_limit_carries_its_limit_in_metres_per_second() {
        let element = element("274-36").unwrap();
        assert_eq!(element.sign_id, MAX_SPEED);
        assert_eq!(element.additional_values, vec!["10.0".to_owned()]);
        assert!(super::element("274").is_none());
    }

    #[test]
    fn german_signs_pass_and_others_do_not() {
        assert_eq!(element("DE205").unwrap().sign_id, "205");
        assert_eq!(element("306").unwrap().sign_id, "306");
        assert!(element("R2-1").is_none());
        assert!(element("no-such-sign").is_none());
    }
}
