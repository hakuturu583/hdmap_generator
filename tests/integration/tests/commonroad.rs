//! Exporting to CommonRoad.
//!
//! A CommonRoad scenario is one XML document whose elements refer to each other by
//! id, so these tests read the written document back the way a consumer does and
//! check that every reference lands, and then that the network in it is the one the
//! IR described: the lanelet graph and its neighbours, the signs, lights and stop
//! lines on it, its intersections, and the planning problem.

use std::collections::{HashMap, HashSet};

use quick_xml::events::Event;
use quick_xml::Reader;

use roadgen_commonroad::model::{Lanelet, Scenario};
use roadgen_commonroad::ScenarioConfig;
use roadgen_core::prelude::*;
use roadgen_core::{IrRef, LaneType, TrafficRule};
use roadgen_integration_tests::{highway, scenarios};

fn config(map: &ValidatedMap) -> ScenarioConfig {
    ScenarioConfig::for_map(map).with_date("2026-01-01")
}

fn scenario(map: &ValidatedMap) -> Scenario {
    roadgen_commonroad::to_scenario(map, &config(map)).unwrap()
}

fn lanelet(scenario: &Scenario, id: u64) -> &Lanelet {
    scenario
        .lanelets
        .iter()
        .find(|lanelet| lanelet.id == id)
        .unwrap_or_else(|| panic!("no lanelet {id}"))
}

/// The lanelet a lane became, by the trace.
fn lanelet_of(map: &ValidatedMap, lane: &LaneId) -> u64 {
    let (_, trace) = roadgen_commonroad::to_scenario_traced(map, &config(map)).unwrap();
    let local = trace
        .links_of(&IrRef::Lane(lane.clone()))
        .find(|link| link.local.starts_with("lanelet:"))
        .unwrap_or_else(|| panic!("{lane} became no lanelet"))
        .local
        .clone();
    local["lanelet:".len()..].parse().unwrap()
}

/// Every id the document declares and every reference it makes, read off the
/// written XML.
fn ids_and_refs(xml: &str) -> (Vec<u64>, Vec<u64>) {
    let mut reader = Reader::from_str(xml);
    let mut ids = Vec::new();
    let mut refs = Vec::new();
    loop {
        match reader.read_event().unwrap() {
            Event::Start(element) | Event::Empty(element) => {
                for attribute in element.attributes() {
                    let attribute = attribute.unwrap();
                    let value: u64 = match attribute.key.as_ref() {
                        b"id" | b"ref" => std::str::from_utf8(&attribute.value)
                            .unwrap()
                            .parse()
                            .unwrap(),
                        _ => continue,
                    };
                    if attribute.key.as_ref() == b"id" {
                        ids.push(value);
                    } else {
                        refs.push(value);
                    }
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    (ids, refs)
}

#[test]
fn every_reference_lands_on_one_element() {
    for map in [
        scenarios::controlled_crossroads(),
        scenarios::sidewalk_crossroads(TrafficHandedness::RightHand),
        scenarios::crosswalk_crossroads(TrafficHandedness::RightHand).0,
        scenarios::lane_drop(),
        highway::highway(),
    ] {
        let xml = roadgen_commonroad::to_xml(&map, &config(&map)).unwrap();
        let (ids, refs) = ids_and_refs(&xml);
        let declared: HashSet<u64> = ids.iter().copied().collect();
        assert_eq!(
            declared.len(),
            ids.len(),
            "{}: ids are unique",
            map.metadata.name.as_deref().unwrap_or_default()
        );
        assert!(ids.iter().all(|id| *id > 0), "ids are positive");
        for reference in refs {
            assert!(declared.contains(&reference), "{reference} names nothing");
        }
    }
}

#[test]
fn every_lanelet_has_bounds_of_one_length_that_meet_its_successors() {
    for map in [
        scenarios::controlled_crossroads(),
        scenarios::sidewalk_crossroads(TrafficHandedness::LeftHand),
        scenarios::widening_road(),
        highway::highway(),
    ] {
        let scenario = scenario(&map);
        for lanelet in &scenario.lanelets {
            assert_eq!(lanelet.left.points.len(), lanelet.right.points.len());
            assert!(lanelet.left.points.len() >= 2);
            assert!(!lanelet.lanelet_types.is_empty());
            for &next in &lanelet.successors {
                let next = self::lanelet(&scenario, next);
                assert!(next.predecessors.contains(&lanelet.id));
                let gap = lanelet
                    .left
                    .points
                    .last()
                    .unwrap()
                    .horizontal_distance_to(next.left.points[0]);
                assert!(gap < 1e-3, "{} → {}: {gap} m", lanelet.id, next.id);
            }
        }
    }
}

#[test]
fn the_lanelet_graph_is_the_maps() {
    let map = scenarios::controlled_crossroads();
    let scenario = scenario(&map);
    let ids: HashMap<&LaneId, u64> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable())
        .map(|lane| (&lane.id, lanelet_of(&map, &lane.id)))
        .collect();
    let mut connections = 0;
    for connection in map.connections.iter() {
        let (Some(from), Some(to)) = (ids.get(&connection.from.lane), ids.get(&connection.to.lane))
        else {
            continue;
        };
        assert!(lanelet(&scenario, *from).successors.contains(to));
        connections += 1;
    }
    assert!(connections > 0);
    let written: usize = scenario
        .lanelets
        .iter()
        .map(|lanelet| lanelet.successors.len())
        .sum();
    assert_eq!(
        written, connections,
        "and no successor the map does not have"
    );
}

#[test]
fn neighbours_are_lanes_of_one_kind_either_side_of_one_edge() {
    let map = scenarios::sidewalk_crossroads(TrafficHandedness::RightHand);
    let scenario = scenario(&map);
    let mut adjacencies = 0;
    for lanelet in &scenario.lanelets {
        let footway = lanelet.lanelet_types.contains(&"sidewalk");
        for (side, adjacent) in [
            ("left", lanelet.adjacent_left),
            ("right", lanelet.adjacent_right),
        ] {
            let Some(adjacent) = adjacent else { continue };
            adjacencies += 1;
            let other = self::lanelet(&scenario, adjacent.lanelet);
            assert_eq!(
                other.lanelet_types.contains(&"sidewalk"),
                footway,
                "a kerb is no lane change"
            );
            // The relation is the same seen from the other side: the same side when
            // they go the same way, the left of both when they meet head on.
            let back = match (side, adjacent.same_direction) {
                ("left", true) => other.adjacent_right,
                ("right", true) => other.adjacent_left,
                ("left", false) => other.adjacent_left,
                _ => other.adjacent_right,
            };
            assert_eq!(back.map(|back| back.lanelet), Some(lanelet.id));
            assert_eq!(back.unwrap().same_direction, adjacent.same_direction);
        }
        if footway {
            assert_eq!(lanelet.user_bidirectional, vec!["pedestrian"]);
            assert!(lanelet.user_one_way.is_empty());
        }
    }
    assert!(adjacencies > 0);
}

#[test]
fn a_traffic_light_and_a_give_way_stop_the_approach_at_its_line() {
    let map = scenarios::controlled_crossroads();
    let scenario = scenario(&map);
    let north = map
        .lanes
        .iter()
        .find(|lane| {
            map.road(&lane.road).and_then(|road| road.name.as_deref()) == Some("north")
                && lane.direction == Direction::Forward
        })
        .unwrap();
    let approach = lanelet(&scenario, lanelet_of(&map, &north.id));

    assert_eq!(scenario.traffic_lights.len(), 1);
    let light = &scenario.traffic_lights[0];
    assert_eq!(approach.traffic_lights, vec![light.id]);
    assert_eq!(light.cycle.len(), 1);
    assert_eq!(light.cycle[0].color, "inactive", "the IR holds no timing");

    let stop = approach.stop_line.as_ref().expect("the approach stops");
    assert_eq!(stop.traffic_lights, vec![light.id]);
    let [a, b] = stop.points.expect("at the line the map draws");
    let stop_line = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::StopLine)
        .unwrap();
    let ObjectGeometry::Line(line) = &stop_line.geometry else {
        panic!("a stop line is a line");
    };
    assert_eq!((a, b), (line.start_point(), line.end_point()));

    // The right-of-way rule: a virtual give-way sign on the lane that yields.
    let give_way = scenario
        .traffic_signs
        .iter()
        .find(|sign| sign.elements[0].sign_id == "205")
        .expect("a give-way sign");
    assert!(give_way.virtual_sign);
    assert!(approach.traffic_signs.contains(&give_way.id));
    assert!(stop.traffic_signs.contains(&give_way.id));
}

#[test]
fn signs_are_written_by_their_number_in_the_german_catalogue() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 2.0),
                Point3::new(80.0, 0.0, 2.0),
                scenarios::two_way(),
            )
            .unwrap(),
        )
        .unwrap();
    let forward = LaneRef::new(road.clone(), 0);
    let backward = LaneRef::new(road, 1);
    builder
        .add_traffic_sign(&forward, LaneEnd::End, "stop", 2.5)
        .unwrap();
    builder
        .add_traffic_sign(&forward, LaneEnd::Start, "274-36", 2.5)
        .unwrap();
    builder
        .add_traffic_sign(&forward, LaneEnd::Start, "speed_limit_50", 2.5)
        .unwrap();
    builder.add_speed_limit_rule(SpeedLimit::from_kph(30.0).unwrap(), vec![backward.clone()]);
    let map = builder.finish().unwrap().validate().unwrap();
    let scenario = scenario(&map);

    let codes: Vec<(&str, &[String], bool)> = scenario
        .traffic_signs
        .iter()
        .map(|sign| {
            (
                sign.elements[0].sign_id,
                sign.elements[0].additional_values.as_slice(),
                sign.virtual_sign,
            )
        })
        .collect();
    assert!(codes.contains(&("206", &[][..], false)));
    assert!(codes.contains(&("274", &["10.0".to_owned()][..], false)));
    assert!(!codes
        .iter()
        .any(|(_, values, _)| values.iter().any(|v| v.starts_with("13.8"))));

    let stop = scenario
        .traffic_signs
        .iter()
        .find(|sign| sign.elements[0].sign_id == "206")
        .unwrap();
    // The road is 2 m up and the sign hangs 2.5 m above it.
    assert!((stop.position.unwrap().x - 80.0).abs() < 1.0);
    let forward_lanelet = lanelet(&scenario, 1);
    assert!(forward_lanelet.traffic_signs.contains(&stop.id));
    assert!(forward_lanelet
        .stop_line
        .as_ref()
        .is_some_and(|line| line.traffic_signs == vec![stop.id]));

    // The rule's limit is a virtual sign on the lane it holds.
    let limit = scenario
        .traffic_signs
        .iter()
        .find(|sign| sign.virtual_sign && sign.elements[0].sign_id == "274")
        .expect("the rule's limit");
    let value: f64 = limit.elements[0].additional_values[0].parse().unwrap();
    assert!((value - 30.0 / 3.6).abs() < 1e-9);
    assert!(lanelet(&scenario, 2).traffic_signs.contains(&limit.id));

    let problems = roadgen_commonroad::check(&map, None);
    assert!(problems
        .iter()
        .any(|problem| problem.contains("1 traffic signs are not written")));
}

#[test]
fn a_junction_is_an_intersection_with_one_incoming_per_approach() {
    let map = scenarios::crosswalk_crossroads(TrafficHandedness::RightHand).0;
    let scenario = scenario(&map);
    assert_eq!(scenario.intersections.len(), 1);
    let intersection = &scenario.intersections[0];
    assert_eq!(
        intersection.incomings.len(),
        4,
        "four arms, each one way in"
    );
    let mut movements = 0;
    for incoming in &intersection.incomings {
        assert!(!incoming.incoming_lanelets.is_empty());
        for &id in &incoming.incoming_lanelets {
            assert!(!lanelet(&scenario, id)
                .lanelet_types
                .contains(&"intersection"));
        }
        for &id in incoming
            .successors_left
            .iter()
            .chain(&incoming.successors_straight)
            .chain(&incoming.successors_right)
        {
            let through = lanelet(&scenario, id);
            assert!(through.lanelet_types.contains(&"intersection"));
            assert!(incoming
                .incoming_lanelets
                .iter()
                .any(|from| through.predecessors.contains(from)));
            movements += 1;
        }
    }
    assert!(movements >= 4);
    assert!(
        !intersection.crossing.is_empty(),
        "the crossings at its mouths"
    );
    for &id in &intersection.crossing {
        assert_eq!(lanelet(&scenario, id).lanelet_types, vec!["crosswalk"]);
    }
}

#[test]
fn a_motorway_is_highway_and_its_slip_roads_ramps() {
    let map = highway::highway();
    let scenario = scenario(&map);
    let types: HashSet<&str> = scenario
        .lanelets
        .iter()
        .flat_map(|lanelet| lanelet.lanelet_types.iter().copied())
        .collect();
    assert!(types.contains("mainCarriageWay"), "{types:?}");
    assert!(
        types.contains("accessRamp") || types.contains("exitRamp"),
        "{types:?}"
    );
    assert!(scenario.tags.contains(&"highway"));
}

#[test]
fn the_planning_problem_drives_the_maps_own_route() {
    let map = scenarios::controlled_crossroads();
    let scenario = scenario(&map);
    let [problem] = &scenario.planning_problems[..] else {
        panic!("one planning problem");
    };
    let start = map.default_start().unwrap();
    let route = map.route_from(&start);
    let first = lanelet(&scenario, lanelet_of(&map, &start));
    // At the middle of the lane where it begins, at rest.
    let middle = first.left.points[0].lerp(first.right.points[0], 0.5);
    assert!(
        problem
            .initial_state
            .position
            .horizontal_distance_to(middle)
            < 1e-6
    );
    assert_eq!(problem.initial_state.velocity, 0.0);
    assert_eq!(
        problem.goal_states[0].lanelets,
        vec![lanelet_of(&map, route.last().unwrap())]
    );
    assert!(problem.goal_states[0].time.1 > 0);

    let none =
        roadgen_commonroad::to_scenario(&map, &config(&map).with_planning_problem(false)).unwrap();
    assert!(none.planning_problems.is_empty());
}

#[test]
fn the_trace_names_every_written_element() {
    let map = scenarios::controlled_crossroads();
    let (scenario, trace) = roadgen_commonroad::to_scenario_traced(&map, &config(&map)).unwrap();
    for lanelet in &scenario.lanelets {
        assert!(
            trace
                .links_to(&format!("lanelet:{}", lanelet.id))
                .next()
                .is_some(),
            "lanelet {} came from nothing",
            lanelet.id
        );
    }
    for sign in &scenario.traffic_signs {
        assert!(trace
            .links_to(&format!("traffic_sign:{}", sign.id))
            .next()
            .is_some());
    }
    for intersection in &scenario.intersections {
        assert!(trace
            .links_to(&format!("intersection:{}", intersection.id))
            .next()
            .is_some());
    }
    let light_rule = map
        .rules
        .iter()
        .position(|rule| matches!(rule, TrafficRule::TrafficLight { .. }))
        .unwrap();
    assert!(trace
        .links_of(&IrRef::Rule(light_rule))
        .any(|link| link.local.starts_with("traffic_light:")));
    let dropped = map
        .lanes
        .iter()
        .filter(|lane| {
            !matches!(
                lane.lane_type,
                LaneType::Driving | LaneType::Biking | LaneType::Sidewalk
            )
        })
        .count();
    assert_eq!(
        scenario.lanelets.len(),
        map.lanes.len() - dropped
            + map
                .objects
                .iter()
                .filter(|object| object.kind == MapObjectKind::Crosswalk)
                .count()
    );
}

#[test]
fn the_check_says_what_the_format_cannot_hold() {
    let map = scenarios::controlled_crossroads();
    let problems = roadgen_commonroad::check(&map, Some(&ScenarioConfig::default()));
    let said = |needle: &str| problems.iter().any(|problem| problem.contains(needle));
    assert!(said("no obstacles"));
    assert!(said("inactive phase"));
    assert!(said("give-way sign"));
    assert!(said("heights are not written"));
    assert!(!said("benchmark id"));

    let elevated = roadgen_commonroad::check(
        &map,
        Some(
            &ScenarioConfig::default()
                .with_benchmark_id("town")
                .with_elevation(true),
        ),
    );
    assert!(elevated
        .iter()
        .any(|problem| problem.contains("benchmark id")));
    assert!(!elevated
        .iter()
        .any(|problem| problem.contains("heights are not written")));
}

#[test]
fn a_corner_pavement_is_written_the_way_that_links_it() {
    for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
        let map = scenarios::sidewalk_crossroads(handedness);
        let scenario = scenario(&map);
        // A footway is walked both ways, and the IR joins the pavement round each
        // corner to the pavements either side whichever way they are drawn; every
        // corner is written the way that makes both of those links run end to start.
        let corners: Vec<&Lanelet> = scenario
            .lanelets
            .iter()
            .filter(|lanelet| {
                lanelet.lanelet_types.contains(&"sidewalk")
                    && lanelet.lanelet_types.contains(&"intersection")
            })
            .collect();
        assert_eq!(corners.len(), 4, "{handedness:?}");
        for corner in corners {
            assert_eq!(corner.predecessors.len(), 1, "{handedness:?} {}", corner.id);
            assert_eq!(corner.successors.len(), 1, "{handedness:?} {}", corner.id);
        }
        assert!(!roadgen_commonroad::check(&map, None)
            .iter()
            .any(|problem| problem.contains("links between footways")));
    }
}

/// What a light shows at `step`, the way commonroad-io reads a cycle: it starts
/// at the offset, and before that it is already running.
fn light_at(light: &roadgen_commonroad::model::TrafficLight, step: u64) -> &'static str {
    let total: u64 = light.cycle.iter().map(|phase| phase.duration).sum();
    let mut into = (step + total - light.time_offset % total) % total;
    for phase in &light.cycle {
        if into < phase.duration {
            return phase.color;
        }
        into -= phase.duration;
    }
    unreachable!("the steps add up to the cycle")
}

#[test]
fn lights_are_dark_unless_given_a_cycle_and_then_cross_traffic_takes_turns() {
    let (map, _) = scenarios::signalised_crosswalk_crossroads(TrafficHandedness::RightHand);
    let dark = scenario(&map);
    assert_eq!(dark.traffic_lights.len(), 4);
    for light in &dark.traffic_lights {
        assert_eq!(light.cycle.len(), 1);
        assert_eq!(light.cycle[0].color, "inactive");
        assert_eq!(light.time_offset, 0);
    }

    let lit =
        roadgen_commonroad::to_scenario(&map, &config(&map).with_default_light_cycle()).unwrap();
    // Which axis each light's approach arrives along, from the lanelet that
    // refers to it.
    let axis = |light: u64| {
        let lanelet = lit
            .lanelets
            .iter()
            .find(|lanelet| lanelet.traffic_lights.contains(&light))
            .expect("a lanelet stops for every light");
        let points = &lanelet.left.points;
        let (a, b) = (points[points.len() - 2], points[points.len() - 1]);
        if (b.x - a.x).abs() > (b.y - a.y).abs() {
            "east-west"
        } else {
            "north-south"
        }
    };
    let mut east_west = 0;
    for light in &lit.traffic_lights {
        let names: Vec<&str> = light.cycle.iter().map(|phase| phase.color).collect();
        assert_eq!(names, ["red", "redYellow", "green", "yellow"]);
        let steps: Vec<u64> = light.cycle.iter().map(|phase| phase.duration).collect();
        assert_eq!(steps, [60, 10, 60, 10], "seconds in steps of 0.1 s");
        east_west += usize::from(axis(light.id) == "east-west");
    }
    assert_eq!(east_west, 2);

    let mut green = [false, false];
    for step in 0..280 {
        let mut shown: HashMap<&str, Vec<&str>> = HashMap::new();
        for light in &lit.traffic_lights {
            shown
                .entry(axis(light.id))
                .or_default()
                .push(light_at(light, step));
        }
        let ew = shown["east-west"].contains(&"green");
        let ns = shown["north-south"].contains(&"green");
        assert!(!(ew && ns), "both ways green at step {step}");
        // The two lights on one axis show the same.
        for colours in shown.values() {
            assert!(colours.windows(2).all(|pair| pair[0] == pair[1]), "{step}");
        }
        green[0] |= ew;
        green[1] |= ns;
    }
    assert_eq!(green, [true, true], "each way gets its turn");
}

#[test]
fn a_light_cycle_the_check_can_see_through() {
    use roadgen_commonroad::LightPhase::{Green, Red, Yellow};
    let (map, _) = scenarios::signalised_crosswalk_crossroads(TrafficHandedness::RightHand);
    let lopsided = config(&map).with_light_cycle([(Red, 5.0), (Green, 20.0), (Yellow, 3.0)]);
    assert!(roadgen_commonroad::check(&map, Some(&lopsided))
        .iter()
        .any(|problem| problem.contains("green together")));
    let short = config(&map).with_light_cycle([(Red, 0.01), (Green, 5.0)]);
    assert!(roadgen_commonroad::to_scenario(&map, &short).is_err());
    let default = config(&map).with_default_light_cycle();
    assert!(!roadgen_commonroad::check(&map, Some(&default))
        .iter()
        .any(|problem| problem.contains("green together") || problem.contains("inactive")));
}

#[test]
fn one_rule_over_two_mouths_still_puts_their_lights_in_turn() {
    let mut builder = scenarios::crossroads_builder("one_rule", 70.0);
    // Every arm points at the centre, so each forward lane is an approach at its end.
    let north = LaneRef::new(RoadId::new("north"), 0);
    let east = LaneRef::new(RoadId::new("east"), 0);
    let north_light = builder
        .add_traffic_light(&north, LaneEnd::End, 5.0)
        .unwrap();
    let east_light = builder.add_traffic_light(&east, LaneEnd::End, 5.0).unwrap();
    builder.add_traffic_light_rule(vec![north_light, east_light], None, vec![north, east]);
    let map = builder.finish().unwrap().validate().unwrap();

    let lit =
        roadgen_commonroad::to_scenario(&map, &config(&map).with_default_light_cycle()).unwrap();
    let mut offsets: Vec<u64> = lit
        .traffic_lights
        .iter()
        .map(|light| light.time_offset)
        .collect();
    offsets.sort();
    assert_eq!(
        offsets,
        [0, 70],
        "a light at each mouth, half a cycle apart"
    );
}
