//! Footways through a junction, exported to SUMO.
//!
//! The generator lays a pavement round every corner of a junction: a connector road
//! from one arm's sidewalk to the next arm's, joined to each by a connection. SUMO
//! has no such thing. Its pedestrians cross a node on a *walking area* netconvert
//! builds there, joining every footway that meets at the node, walked either way —
//! so the export writes no connection between two footways and asks netconvert for
//! walking areas instead.
//!
//! It used to write them, and that went wrong: which sidewalk faces a corner depends
//! on which end of each arm meets the junction and on the side traffic keeps to, so
//! a pavement can run from a sidewalk *leaving* the junction to one *arriving* at it.
//! As a SUMO connection that is a connection from an edge that starts at the node,
//! and netconvert refused the whole network. These tests build both a right-hand and
//! a left-hand crossroads, whose corners join opposite sidewalks, and check what
//! SUMO made of each: that netconvert builds them without complaint, that every
//! corner the IR paves is one walking area, that a pedestrian in the simulator walks
//! round each corner both ways, and that the trace still accounts for every pavement.
//!
//! When SUMO is not installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`,
//! which turns the skip into a failure.

use std::collections::{BTreeMap, BTreeSet};

use roadgen_core::prelude::*;
use roadgen_core::trace::Relation;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build::{self, SumoNetwork};
use roadgen_trace::{directory_sidecar, write_ir, write_trace, TraceIndex};

const HANDEDNESS: [TrafficHandedness; 2] =
    [TrafficHandedness::RightHand, TrafficHandedness::LeftHand];

/// One pavement round one corner: the connector lane, and the two sidewalks it joins
/// as the SUMO network names them.
struct Corner {
    pavement: LaneId,
    from: String,
    to: String,
}

/// Every corner of the map the generator paved, read from the IR: each connector
/// road with only sidewalks on it, and the lanes its connections lead in from and
/// out to.
fn corners(map: &ValidatedMap) -> Vec<Corner> {
    let lanes = roadgen_sumo::to_plain_xml(map)
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

/// The walking areas of junction `node` that each of the network's lanes leads onto
/// or comes off, by lane.
fn walking_areas_at(network: &SumoNetwork, node: &str) -> BTreeMap<String, BTreeSet<String>> {
    let prefix = format!(":{node}_w");
    let mut touching: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for connection in &network.connections {
        if connection.to.starts_with(&prefix) && !connection.from.starts_with(':') {
            touching
                .entry(format!("{}_{}", connection.from, connection.from_lane))
                .or_default()
                .insert(connection.to.clone());
        }
        if connection.from.starts_with(&prefix) && !connection.to.starts_with(':') {
            touching
                .entry(format!("{}_{}", connection.to, connection.to_lane))
                .or_default()
                .insert(connection.from.clone());
        }
    }
    touching
}

/// The export says that the IR's footway connections are left to the walking areas,
/// and how many — and says nothing of the kind for a map with no footways.
#[test]
fn the_footway_connections_left_to_walking_areas_are_reported() {
    let report = roadgen_sumo::check(&scenarios::sidewalk_crossroads(
        TrafficHandedness::RightHand,
    ))
    .join("\n");
    // Four corners, each a connection into its pavement and one out of it.
    assert!(
        report.contains("the 8 connections between footways"),
        "{report}"
    );
    assert!(report.contains("walking area"), "{report}");

    let report = roadgen_sumo::check(&scenarios::crossroads()).join("\n");
    assert!(!report.contains("walking area"), "{report}");
}

/// Both crossroads build, load, and keep their footways out of the connections: a
/// footway is joined to the next one by a walking area and nothing else.
#[test]
fn a_crossroads_with_sidewalks_builds_and_loads_whichever_side_traffic_keeps_to() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let map = scenarios::sidewalk_crossroads(handedness);
        let prefix = roadgen_sumo::network_name(&map);
        let (directory, network) = sumo_build::build(&map);
        sumo_build::simulate(directory.path(), &prefix);

        let config =
            std::fs::read_to_string(directory.path().join(format!("{prefix}.netccfg"))).unwrap();
        assert!(
            config.contains(r#"<walkingareas value="true"/>"#),
            "{config}"
        );

        // Every lane the export wrote that only pedestrians may use.
        let footways: BTreeSet<String> = network
            .roads()
            .iter()
            .flat_map(|edge| edge.lanes.iter())
            .filter(|lane| lane.allow.as_deref() == Some("pedestrian"))
            .map(|lane| lane.id.clone())
            .collect();
        assert_eq!(
            footways.len(),
            8,
            "{handedness:?}: one sidewalk per side of each of four arms: {footways:?}"
        );
        // netconvert built the network for the side traffic keeps to, and each
        // sidewalk is its edge's kerb lane: lane 0, which SUMO counts from the
        // outside of the carriageway — the right where traffic keeps right, the left
        // where it keeps left.
        assert_eq!(
            network.lefthand,
            handedness == TrafficHandedness::LeftHand,
            "{handedness:?}"
        );
        for footway in &footways {
            assert!(
                footway.ends_with("_0"),
                "{handedness:?}: {footway} is not its edge's kerb lane"
            );
        }
        for (from, from_lane, to, to_lane) in network.movements() {
            let (from, to) = (format!("{from}_{from_lane}"), format!("{to}_{to_lane}"));
            assert!(
                !(footways.contains(&from) && footways.contains(&to)),
                "{handedness:?}: {from} > {to} joins two footways with a connection"
            );
        }
    }
}

/// Every corner the IR paves is a walking area at the junction joining the same two
/// sidewalks — and no walking area joins sidewalks the IR keeps apart, since the IR
/// has no crosswalk here and SUMO builds none.
#[test]
fn each_paved_corner_is_one_walking_area_joining_its_two_sidewalks() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let map = scenarios::sidewalk_crossroads(handedness);
        let (_directory, network) = sumo_build::build(&map);
        let touching = walking_areas_at(&network, "j_x");
        let corners = corners(&map);
        assert_eq!(corners.len(), 4, "{handedness:?}: one pavement per corner");

        let mut areas = BTreeSet::new();
        for corner in &corners {
            let shared: Vec<&String> = touching
                .get(&corner.from)
                .into_iter()
                .flatten()
                .filter(|area| {
                    touching
                        .get(&corner.to)
                        .is_some_and(|other| other.contains(*area))
                })
                .collect();
            assert_eq!(
                shared.len(),
                1,
                "{handedness:?}: {} paves the corner from {} to {}, but they share {shared:?} \
                 in {touching:?}",
                corner.pavement,
                corner.from,
                corner.to
            );
            areas.insert(shared[0].clone());
        }
        assert_eq!(
            areas.len(),
            4,
            "{handedness:?}: one walking area per corner"
        );
        for area in &areas {
            let joined = touching
                .values()
                .filter(|areas| areas.contains(area))
                .count();
            assert_eq!(joined, 2, "{handedness:?}: {area} joins {joined} sidewalks");
        }
        assert!(
            network
                .edges
                .iter()
                .all(|edge| edge.function.as_deref() != Some("crossing")),
            "{handedness:?}: the IR has no crosswalk, so SUMO should have no crossing"
        );
    }
}

/// A pedestrian walks round every corner in the simulator, both the way the IR's
/// pavement runs and back — the direction the pavement was drawn in is the IR's
/// bookkeeping, not a rule about which way people walk.
#[test]
fn a_pedestrian_walks_round_every_corner_both_ways() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let map = scenarios::sidewalk_crossroads(handedness);
        let prefix = roadgen_sumo::network_name(&map);
        let (directory, network) = sumo_build::build(&map);

        // Each walk starts a metre from the junction on one sidewalk and ends a
        // metre from it on the other, so the way round the corner is a few metres
        // and any other way — out to a dead end and back round the far side — is
        // hundreds.
        let near_junction = |lane: &str| {
            let (edge, _) = lane.rsplit_once('_').unwrap();
            let edge = network.edge(edge);
            let arriving = edge.to.as_deref() == Some("j_x");
            let position = if arriving {
                edge.lanes[0].length - 1.0
            } else {
                1.0
            };
            (edge.id.clone(), position)
        };
        let mut walks = Vec::new();
        for corner in corners(&map) {
            walks.push((corner.from.clone(), corner.to.clone()));
            walks.push((corner.to, corner.from));
        }
        let mut persons = String::new();
        for (index, (from, to)) in walks.iter().enumerate() {
            let (from_edge, depart) = near_junction(from);
            let (to_edge, arrive) = near_junction(to);
            persons.push_str(&format!(
                r#"    <person id="p{index}" depart="0" departPos="{depart:.2}">
        <walk from="{from_edge}" to="{to_edge}" arrivalPos="{arrive:.2}"/>
    </person>
"#
            ));
        }
        let routes = format!("<routes>\n{persons}</routes>\n");
        std::fs::write(directory.path().join("walks.rou.xml"), routes).unwrap();

        let walked = sumo_build::walk(directory.path(), &prefix);
        assert_eq!(
            walked.len(),
            walks.len(),
            "{handedness:?}: every pedestrian should arrive: {walked:?}"
        );
        for (person, length) in &walked {
            let (from, to) = &walks[person.trim_start_matches('p').parse::<usize>().unwrap()];
            assert!(
                *length < 40.0,
                "{handedness:?}: {from} to {to} took {length} m, so it did not go round \
                 the corner"
            );
        }
    }
}

/// Nothing of the IR's pavements is lost from the trace: each pavement lane, and the
/// connections into and out of it, became part of the junction node's walking area —
/// and the network netconvert built, walking areas and all, is still recognised as
/// the one built from the export.
#[test]
fn the_trace_puts_every_pavement_in_the_junction_and_accepts_the_built_network() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let map = scenarios::sidewalk_crossroads(handedness);
        let directory = tempfile::tempdir().unwrap();
        write_ir(&map, directory.path().join("map.ir.json")).unwrap();
        let (prefix, trace) = roadgen_sumo::write_traced(&map, directory.path()).unwrap();
        let sidecar = directory_sidecar(directory.path(), &prefix, &trace.format);
        write_trace(&trace, &map, &sidecar).unwrap();

        let in_walking_area: BTreeSet<String> = trace
            .links_to("node:j_x")
            .filter(|link| link.role.as_deref() == Some("walkingarea"))
            .inspect(|link| assert_eq!(link.relation, Relation::Collapsed))
            .map(|link| link.ir.to_string())
            .collect();
        for corner in corners(&map) {
            assert!(
                in_walking_area.contains(&corner.pavement.to_string()),
                "{handedness:?}: {} is not in the junction's walking area: \
                 {in_walking_area:?}",
                corner.pavement
            );
            for connection in map.connections.iter().filter(|connection| {
                connection.from.lane == corner.pavement || connection.to.lane == corner.pavement
            }) {
                assert!(
                    in_walking_area.contains(&connection.id.to_string()),
                    "{handedness:?}: {} is not in the junction's walking area",
                    connection.id
                );
            }
        }

        let net = sumo_build::netconvert(directory.path(), &prefix);
        let mut index = TraceIndex::new();
        index.load(directory.path().join("map.ir.json")).unwrap();
        index.load(&sidecar).unwrap();
        let report = index
            .load_sumo_net(&net)
            .expect("the built network, walking areas and all, is the export's");
        assert_eq!(report.untraced, 0, "{handedness:?}");
    }
}
