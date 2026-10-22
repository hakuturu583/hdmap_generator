//! Bicycles on the sidewalks, both ways, exported to SUMO.
//!
//! With `Options::cycle_on_sidewalks` bicycles may ride the sidewalks. A SUMO bicycle
//! is a vehicle: it follows an edge's direction and crosses a node along a
//! connection. So each sidewalk gets two edges laid along it, `<edge>.bike<index>` the
//! way its edge runs and `<edge>.bike<index>.rev` the other way — each cut in two
//! halfway, so that netconvert does not take it for the turnaround of a carriageway it
//! would share both its nodes with — and every corner the IR paves is a connection
//! each way between the ones that arrive at the junction and leave it.
//!
//! These tests take the sidewalk crossroads of `sumo_sidewalks` and the crosswalk
//! crossroads of `sumo_crossings`, signalised or not, one-way or not, right-hand and
//! left-hand, and check what SUMO made of each: that netconvert builds them without
//! complaint, that the pedestrians' walking areas and crossings are the ones they
//! have without bicycles, that a bicycle in the simulator rides round every corner
//! both ways on the sidewalks, and that the trace accounts for what was added.
//!
//! When SUMO is not installed the ones that need it skip, saying so; CI sets
//! `ROADGEN_REQUIRE_SUMO`, which turns the skip into a failure.

use std::collections::{BTreeMap, BTreeSet};

use roadgen_core::prelude::*;
use roadgen_core::semantics::MapObjectKind;
use roadgen_core::trace::Relation;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build::{self, SumoNetwork};
use roadgen_sumo::Options;
use roadgen_trace::{directory_sidecar, write_ir, write_trace, TraceIndex};

const HANDEDNESS: [TrafficHandedness; 2] =
    [TrafficHandedness::RightHand, TrafficHandedness::LeftHand];

fn cycling() -> Options {
    Options {
        cycle_on_sidewalks: true,
        ..Options::default()
    }
}

/// Every crossroads with sidewalks the scenarios have, under `handedness`.
fn maps(handedness: TrafficHandedness) -> Vec<ValidatedMap> {
    vec![
        scenarios::sidewalk_crossroads(handedness),
        scenarios::crosswalk_crossroads(handedness).0,
        scenarios::signalised_crosswalk_crossroads(handedness).0,
        scenarios::one_way_crosswalk_crossroads(handedness).0,
    ]
}

/// One pavement round one corner of the junction: the connector lane, and the two
/// sidewalks it joins as the SUMO network names them.
struct Corner {
    pavement: LaneId,
    from: String,
    to: String,
}

/// Every corner of the map the generator paved: each connector road with only
/// sidewalks on it, and the lanes its connections lead in from and out to.
fn corners(map: &ValidatedMap) -> Vec<Corner> {
    let lanes = roadgen_sumo::to_plain_xml_with(map, &cycling())
        .expect("the map should export as SUMO")
        .lanes;
    let mut corners = Vec::new();
    for road in map.roads.iter().filter(|road| road.is_connector()) {
        let on_it = map.lanes_of_section(&road.id, 0);
        if !on_it
            .iter()
            .all(|lane| lane.lane_type == LaneType::Sidewalk)
        {
            continue;
        }
        let pavement = on_it[0].id.clone();
        let into = map
            .connections
            .iter()
            .find(|connection| connection.to.lane == pavement)
            .expect("a pavement is connected at its start");
        let out = map
            .connections
            .iter()
            .find(|connection| connection.from.lane == pavement)
            .expect("a pavement is connected at its end");
        corners.push(Corner {
            from: lanes[&into.from.lane].clone(),
            to: lanes[&out.to.lane].clone(),
            pavement,
        });
    }
    corners
}

/// The lane a bicycle riding sidewalk `lane` rides into `node` on, and the lane it
/// rides out of it on: the second half of one of its two edges, and the first half of
/// the other.
fn ways_through(network: &SumoNetwork, lane: &str, node: &str) -> (String, String) {
    let (edge, index) = lane.rsplit_once('_').unwrap();
    let (along, against) = (
        format!("{edge}.bike{index}"),
        format!("{edge}.bike{index}.rev"),
    );
    if network.edge(edge).to.as_deref() == Some(node) {
        (format!("{along}.p1_0"), format!("{against}_0"))
    } else {
        assert_eq!(network.edge(edge).from.as_deref(), Some(node), "{lane}");
        (format!("{against}.p1_0"), format!("{along}_0"))
    }
}

/// The walking areas of junction `node`, each as the edges whose lanes lead onto it
/// or come off it.
fn walking_areas_at(network: &SumoNetwork, node: &str) -> BTreeSet<BTreeSet<String>> {
    let prefix = format!(":{node}_w");
    let mut touching: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for connection in &network.connections {
        if connection.to.starts_with(&prefix) && !connection.from.starts_with(':') {
            touching
                .entry(connection.to.clone())
                .or_default()
                .insert(connection.from.clone());
        }
        if connection.from.starts_with(&prefix) && !connection.to.starts_with(':') {
            touching
                .entry(connection.from.clone())
                .or_default()
                .insert(connection.to.clone());
        }
    }
    touching.into_values().collect()
}

/// The edges each crossing netconvert built crosses.
fn crossings(network: &SumoNetwork) -> BTreeSet<Vec<String>> {
    network
        .edges
        .iter()
        .filter(|edge| edge.function.as_deref() == Some("crossing"))
        .map(|edge| {
            let mut crossed = edge.crossing_edges.clone();
            crossed.sort();
            crossed
        })
        .collect()
}

/// Without the option nothing changes; with it every sidewalk has an edge for bicycles
/// each way along it, written in two pieces and admitting pedestrians too, and stays
/// the pedestrians' own.
#[test]
fn every_sidewalk_is_ridden_both_ways_and_only_when_asked() {
    let map = scenarios::sidewalk_crossroads(TrafficHandedness::RightHand);
    let plain = roadgen_sumo::to_plain_xml(&map).unwrap();
    assert!(!plain.edges.contains("bicycle\""), "{}", plain.edges);
    assert!(!plain.edges.contains(".bike"), "{}", plain.edges);
    assert_eq!(
        plain,
        roadgen_sumo::to_plain_xml_with(&map, &Options::default()).unwrap()
    );

    let network = roadgen_sumo::to_plain_xml_with(&map, &cycling()).unwrap();
    let sidewalks: Vec<&String> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type == LaneType::Sidewalk)
        .filter_map(|lane| network.lanes.get(&lane.id))
        .collect();
    assert_eq!(sidewalks.len(), 8);
    for sidewalk in sidewalks {
        let (edge, index) = sidewalk.rsplit_once('_').unwrap();
        for piece in [
            format!("{edge}.bike{index}"),
            format!("{edge}.bike{index}.p1"),
            format!("{edge}.bike{index}.rev"),
            format!("{edge}.bike{index}.rev.p1"),
        ] {
            assert!(
                network.edges.contains(&format!("<edge id=\"{piece}\"")),
                "{sidewalk} has no {piece}"
            );
        }
    }
    let allowing = |classes: &str| {
        network
            .edges
            .matches(&format!(r#" allow="{classes}""#))
            .count()
    };
    assert_eq!(allowing("pedestrian bicycle"), 8 * 4);
    assert_eq!(allowing("pedestrian"), 8);
}

/// Every crossroads builds and loads with the sidewalks ridden both ways, and keeps
/// the walking areas and crossings it has without bicycles: the edges bicycles ride a
/// sidewalk on join the walking area of that sidewalk, and no other.
#[test]
fn the_pedestrians_keep_their_walking_areas_and_crossings() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        for map in maps(handedness) {
            let name = roadgen_sumo::network_name(&map);
            let (_plain_directory, plain) = sumo_build::build(&map);
            let built = sumo_build::build_with(&map, &cycling());
            sumo_build::simulate(built.directory.path(), &built.prefix);

            assert_eq!(crossings(&built.network), crossings(&plain), "{name}");
            // A walking area of the cycling network, with the edges bicycles ride
            // the sidewalks on taken out, is one of the plain network's.
            let without_rides: BTreeSet<BTreeSet<String>> = walking_areas_at(&built.network, "j_x")
                .into_iter()
                .map(|area| {
                    area.into_iter()
                        .filter(|edge| !edge.contains(".bike"))
                        .collect()
                })
                .collect();
            assert_eq!(without_rides, walking_areas_at(&plain, "j_x"), "{name}");
            // And each takes in both of the edges along each sidewalk it joins.
            for area in walking_areas_at(&built.network, "j_x") {
                let sidewalks = area.iter().filter(|edge| !edge.contains(".bike")).count();
                let rides = area.iter().filter(|edge| edge.contains(".bike")).count();
                assert_eq!(2 * sidewalks, rides, "{name}: {area:?}");
            }
        }
    }
}

/// A bicycle rides round every corner the IR paves, both ways, on the sidewalks — a
/// few metres each, where any other way round is hundreds.
#[test]
fn a_bicycle_rides_round_every_corner_both_ways() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        for map in maps(handedness) {
            let name = roadgen_sumo::network_name(&map);
            let built = sumo_build::build_with(&map, &cycling());
            let network = &built.network;

            let mut rides = Vec::new();
            for corner in corners(&map) {
                let (into_from, out_of_from) = ways_through(network, &corner.from, "j_x");
                let (into_to, out_of_to) = ways_through(network, &corner.to, "j_x");
                rides.push((into_from, out_of_to));
                rides.push((into_to, out_of_from));
            }
            assert_eq!(
                rides.len(),
                8,
                "{name}: two rides round each of four corners"
            );

            let mut trips = String::new();
            for (index, (from, to)) in rides.iter().enumerate() {
                let (from_edge, from_lane) = from.rsplit_once('_').unwrap();
                let (to_edge, to_lane) = to.rsplit_once('_').unwrap();
                let depart = network.edge(from_edge).lanes[0].length - 1.0;
                trips.push_str(&format!(
                    r#"    <trip id="b{index}" type="bike" depart="0" from="{from_edge}" to="{to_edge}" departLane="{from_lane}" departPos="{depart:.2}" arrivalLane="{to_lane}" arrivalPos="1.00"/>
"#
                ));
            }
            let routes = format!(
                "<routes>\n    <vType id=\"bike\" vClass=\"bicycle\"/>\n{trips}</routes>\n"
            );
            std::fs::write(built.directory.path().join("rides.rou.xml"), routes).unwrap();

            let ridden = sumo_build::ride(built.directory.path(), &built.prefix);
            assert_eq!(
                ridden.len(),
                rides.len(),
                "{name}: every bicycle should arrive: {ridden:?}"
            );
            for (bicycle, length) in &ridden {
                let (from, to) = &rides[bicycle.trim_start_matches('b').parse::<usize>().unwrap()];
                assert!(
                    *length < 40.0,
                    "{name}: {from} to {to} took {length} m, so it did not go round the corner"
                );
            }
        }
    }
}

/// The trace accounts for every edge and connection the option adds — each piece of
/// an edge bicycles ride a sidewalk on is part of the sidewalk, each ride round a
/// corner is the pavement there — and the network netconvert built is still the
/// export's.
#[test]
fn the_trace_follows_every_ride_and_accepts_the_built_network() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let map = scenarios::signalised_crosswalk_crossroads(handedness).0;
        let directory = tempfile::tempdir().unwrap();
        write_ir(&map, directory.path().join("map.ir.json")).unwrap();
        let (prefix, trace) =
            roadgen_sumo::write_traced_with(&map, directory.path(), &cycling()).unwrap();
        let sidecar = directory_sidecar(directory.path(), &prefix, &trace.format);
        write_trace(&trace, &map, &sidecar).unwrap();

        for corner in corners(&map) {
            let ridden: Vec<String> = trace
                .links_of(&corner.pavement.clone().into())
                .filter(|link| link.role.as_deref() == Some("bicycle"))
                .inspect(|link| assert_eq!(link.relation, Relation::Collapsed))
                .map(|link| link.local.clone())
                .collect();
            assert_eq!(
                ridden.len(),
                2,
                "{handedness:?}: {}: {ridden:?}",
                corner.pavement
            );
        }
        let sidewalk = map
            .lanes
            .iter()
            .find(|lane| {
                lane.lane_type == LaneType::Sidewalk
                    && map
                        .road(&lane.road)
                        .is_some_and(|road| !road.is_connector())
            })
            .expect("a sidewalk along an arm");
        // Two pieces each way, and the turn back at the dead end it runs out to.
        let ridden: Vec<(&str, Relation)> = trace
            .links_of(&sidewalk.id.clone().into())
            .filter(|link| link.role.as_deref() == Some("bicycle"))
            .map(|link| (link.local.as_str(), link.relation))
            .collect();
        let pieces = ridden
            .iter()
            .filter(|(local, relation)| local.starts_with("lane:") && *relation == Relation::Part)
            .count();
        let turns = ridden
            .iter()
            .filter(|(local, relation)| {
                local.starts_with("connection:") && *relation == Relation::Collapsed
            })
            .count();
        assert_eq!((pieces, turns), (4, 1), "{handedness:?}: {ridden:?}");

        let net = sumo_build::netconvert(directory.path(), &prefix);
        let mut index = TraceIndex::new();
        index.load(directory.path().join("map.ir.json")).unwrap();
        index.load(&sidecar).unwrap();
        let report = index
            .load_sumo_net(&net)
            .expect("the built network, rides and all, is the export's");
        assert_eq!(report.untraced, 0, "{handedness:?}");
    }
}

/// What a bicycle does on one connection of the built network that the option adds,
/// read off the edges it joins: `<arm>.<fwd|bwd>` is a carriageway, and
/// `<arm>.<fwd|bwd>.bike<i>[.rev][.p1]` a piece of a cycleway along a sidewalk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Ride {
    /// From one sidewalk round a corner onto the next.
    Corner,
    /// Off a sidewalk onto a carriageway, or off a carriageway onto a sidewalk.
    OnOrOff,
    /// Across the road from the sidewalk on one kerb to the one on the other.
    Across,
    /// Back the other way along the same sidewalk, at a dead end.
    TurnBack,
}

fn ride(from: &str, to: &str) -> Option<Ride> {
    let sidewalk = |edge: &str| {
        edge.split_once(".bike")
            .map(|(carriageway, _)| carriageway.to_owned())
    };
    let arm = |edge: &str| edge.split('.').next().unwrap().to_owned();
    match (sidewalk(from), sidewalk(to)) {
        (None, None) => None,
        (Some(_), None) | (None, Some(_)) => Some(Ride::OnOrOff),
        // The joint between the two halves of one cycleway.
        _ if to == format!("{from}.p1") => None,
        (Some(one), Some(other)) if one == other => Some(Ride::TurnBack),
        _ if arm(from) == arm(to) => Some(Ride::Across),
        _ => Some(Ride::Corner),
    }
}

/// Every connection a bicycle can ride onto or off a sidewalk, across the road or
/// back at a dead end is one a bicycle in the simulator does ride: starting a metre
/// short of the end of the one edge, it arrives a metre into the other having gone
/// a few metres, where any other way round is hundreds — whether the junction is
/// signalised or not, and whichever side traffic keeps to.
///
/// One bicycle at a time, a full signal cycle and more apart. Released together, a
/// handful of them giving way to one another across an unsignalised junction can
/// each wait for the next, as cars on the minor arms of a priority junction can, and
/// that would say nothing about whether each way is there.
#[test]
fn a_bicycle_gets_on_and_off_the_road_crosses_it_and_turns_back() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        for map in maps(handedness) {
            let name = roadgen_sumo::network_name(&map);
            let built = sumo_build::build_with(&map, &cycling());
            let network = &built.network;
            let crosswalks = map
                .objects
                .iter()
                .any(|object| object.kind == MapObjectKind::Crosswalk);

            let mut rides: Vec<(Ride, &str, usize, &str)> = network
                .movements()
                .into_iter()
                .filter_map(|(from, from_lane, to, _)| Some((ride(from, to)?, from, from_lane, to)))
                .filter(|(kind, ..)| *kind != Ride::Corner)
                .collect();
            rides.sort();
            let kinds: BTreeSet<Ride> = rides.iter().map(|(kind, ..)| *kind).collect();
            let mut expected = BTreeSet::from([Ride::OnOrOff, Ride::TurnBack]);
            if crosswalks {
                expected.insert(Ride::Across);
            }
            assert_eq!(kinds, expected, "{name}: {rides:?}");
            // A turn back at the far end of every sidewalk of every arm.
            let sidewalks = map
                .lanes
                .iter()
                .filter(|lane| lane.lane_type == LaneType::Sidewalk)
                .filter(|lane| {
                    map.road(&lane.road)
                        .is_some_and(|road| !road.is_connector())
                })
                .count();
            let turns = rides
                .iter()
                .filter(|(kind, ..)| *kind == Ride::TurnBack)
                .count();
            assert_eq!(turns, sidewalks, "{name}: {rides:?}");

            let mut trips = String::new();
            for (index, (_, from, from_lane, to)) in rides.iter().enumerate() {
                let depart = network.edge(from).lanes[*from_lane].length - 1.0;
                let depart_time = index * 100;
                trips.push_str(&format!(
                    r#"    <trip id="b{index}" type="bike" depart="{depart_time}" from="{from}" to="{to}" departLane="{from_lane}" departPos="{depart:.2}" arrivalPos="1.00"/>
"#
                ));
            }
            let routes = format!(
                "<routes>\n    <vType id=\"bike\" vClass=\"bicycle\"/>\n{trips}</routes>\n"
            );
            std::fs::write(built.directory.path().join("rides.rou.xml"), routes).unwrap();

            let ridden = sumo_build::ride(built.directory.path(), &built.prefix);
            assert_eq!(
                ridden.len(),
                rides.len(),
                "{name}: every bicycle should arrive: {ridden:?} of {rides:?}"
            );
            for (bicycle, length) in &ridden {
                let ride = &rides[bicycle.trim_start_matches('b').parse::<usize>().unwrap()];
                assert!(
                    *length < 40.0,
                    "{name}: {ride:?} took {length} m, so it went some other way"
                );
            }
        }
    }
}

/// netconvert is told to leave out the junction logic a signal program releasing a
/// bicycle and a turning car together would conflict with — and only where a light
/// controls a bicycle, so a map without bicycles is configured as it always was.
#[test]
fn a_light_over_bicycles_is_built_without_the_internal_junction_jam() {
    let jam = "tls.ignore-internal-junction-jam";
    for handedness in HANDEDNESS {
        let signalised = scenarios::signalised_crosswalk_crossroads(handedness).0;
        let plain = roadgen_sumo::to_plain_xml(&signalised).unwrap();
        assert!(!plain.config.contains(jam), "{}", plain.config);
        let cycling_network = roadgen_sumo::to_plain_xml_with(&signalised, &cycling()).unwrap();
        assert!(
            cycling_network.config.contains(jam),
            "{}",
            cycling_network.config
        );

        let unsignalised = scenarios::crosswalk_crossroads(handedness).0;
        let cycling_network = roadgen_sumo::to_plain_xml_with(&unsignalised, &cycling()).unwrap();
        assert!(
            !cycling_network.config.contains(jam),
            "{}",
            cycling_network.config
        );
    }
}
