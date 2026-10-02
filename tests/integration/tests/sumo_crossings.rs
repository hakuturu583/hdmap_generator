//! Crosswalks, exported to SUMO as pedestrian crossings.
//!
//! A SUMO crossing is not a strip with a place along the road, as the IR's
//! crosswalk is. It belongs to a node: the export writes `<crossing node edges>` in
//! the connections file, and netconvert lays it across the mouth of the named edges,
//! between the walking areas either side of them. So a crosswalk near the end of a
//! road — where the IR puts one at a junction — becomes the crossing at that end's
//! node, across both carriageways of the road; one half way along a road has no node
//! to belong to, and the export reports it instead of writing it.
//!
//! These tests take a crossroads with a pavement down each side of every arm, a
//! crosswalk across every arm at the junction, and one more half way along the west
//! arm, in both a right-hand and a left-hand version, and check what SUMO made of
//! it: that netconvert builds it without complaint and with a crossing across each
//! arm at the junction and nowhere else, that a pedestrian in the simulator crosses
//! each arm there both ways, and that the trace follows each crosswalk to the
//! crossing netconvert built for it.
//!
//! When SUMO is not installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`,
//! which turns the skip into a failure.

use std::collections::BTreeSet;

use roadgen_core::prelude::*;
use roadgen_core::semantics::MapObject;
use roadgen_core::trace::{IrRef, Relation};
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build::{self, SumoNetwork};
use roadgen_trace::{directory_sidecar, write_ir, write_trace, TraceIndex};

const HANDEDNESS: [TrafficHandedness; 2] =
    [TrafficHandedness::RightHand, TrafficHandedness::LeftHand];

/// The arms of the scenario, as their edges are named.
const ARMS: [&str; 4] = ["west", "east", "north", "south"];

/// The edges a crossing across `arm` crosses: both of its carriageways.
fn both_ways(arm: &str) -> BTreeSet<String> {
    BTreeSet::from([format!("{arm}.fwd"), format!("{arm}.bwd")])
}

/// The crossings of the built network, as the node each is at and the edges it
/// crosses.
fn crossings(network: &SumoNetwork) -> Vec<(String, BTreeSet<String>)> {
    network
        .edges
        .iter()
        .filter(|edge| edge.function.as_deref() == Some("crossing"))
        .map(|edge| {
            let node = edge
                .id
                .strip_prefix(':')
                .and_then(|rest| rest.rsplit_once("_c"))
                .map(|(node, _)| node.to_owned())
                .unwrap_or_else(|| panic!("a crossing named {}", edge.id));
            (node, edge.crossing_edges.iter().cloned().collect())
        })
        .collect()
}

/// The export says the four crosswalks at the junction became crossings there, and
/// that the one half way along the west arm could not — naming it.
#[test]
fn the_export_says_which_crosswalks_became_crossings() {
    for handedness in HANDEDNESS {
        let (map, _) = scenarios::crosswalk_crossroads(handedness);
        let report = roadgen_sumo::check(&map).join("\n");
        assert!(
            report.contains("the 4 crosswalks near the end of their road are written as crossings"),
            "{handedness:?}: {report}"
        );
        assert!(
            report.contains("the 1 crosswalks more than 15 m from either end"),
            "{handedness:?}: {report}"
        );
        assert!(
            report.contains("object/crosswalk/west/0.5"),
            "{handedness:?}: {report}"
        );
        assert!(
            !report.contains("without a footway"),
            "{handedness:?}: {report}"
        );
    }
}

/// The plain connections file has one crossing per arm at the junction, across both
/// of the arm's edges; and netconvert builds exactly those, at the junction, without
/// a word of complaint, into a network the simulator loads.
#[test]
fn each_crosswalk_at_the_junction_is_a_crossing_of_its_arm_there() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let (map, _) = scenarios::crosswalk_crossroads(handedness);
        let prefix = roadgen_sumo::network_name(&map);
        let (directory, network) = sumo_build::build(&map);
        sumo_build::simulate(directory.path(), &prefix);

        let plain =
            std::fs::read_to_string(directory.path().join(format!("{prefix}.con.xml"))).unwrap();
        let written: Vec<&str> = plain
            .lines()
            .filter(|line| line.contains("<crossing "))
            .collect();
        assert_eq!(written.len(), 4, "{handedness:?}: {plain}");
        for arm in ARMS {
            let line = format!(
                r#"<crossing node="j_x" edges="{arm}.bwd {arm}.fwd" priority="1" width="4.000"/>"#
            );
            assert!(
                written.iter().any(|written| written.trim() == line),
                "{handedness:?}: no {line} in {plain}"
            );
        }

        let mut built = crossings(&network);
        built.sort();
        let mut expected: Vec<(String, BTreeSet<String>)> = ARMS
            .iter()
            .map(|arm| ("j_x".to_owned(), both_ways(arm)))
            .collect();
        expected.sort();
        assert_eq!(
            built, expected,
            "{handedness:?}: one crossing per arm, at the junction, and no other — the \
             crosswalk half way along the west arm is not one"
        );
    }
}

/// A pedestrian in the simulator crosses every arm at the junction, both ways: from
/// a metre short of the junction on one pavement to a metre short of it on the
/// pavement opposite. Across the crossing that is a walk of a few metres; with no
/// crossing there, the only way over would be round the three other arms' crossings
/// or out to the far end of the arm and back, which is many times further.
#[test]
fn a_pedestrian_crosses_every_arm_at_the_junction_both_ways() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let (map, _) = scenarios::crosswalk_crossroads(handedness);
        let prefix = roadgen_sumo::network_name(&map);
        let (directory, network) = sumo_build::build(&map);

        // The pavement of each carriageway is the lane only pedestrians may use.
        let near_junction = |edge: &str| {
            let edge = network.edge(edge);
            let pavement = edge
                .lanes
                .iter()
                .find(|lane| lane.allow.as_deref() == Some("pedestrian"))
                .unwrap_or_else(|| panic!("{} has no pavement", edge.id));
            let arriving = edge.to.as_deref() == Some("j_x");
            let position = if arriving { pavement.length - 1.0 } else { 1.0 };
            (edge.id.clone(), position)
        };
        let mut walks = Vec::new();
        for arm in ARMS {
            let (fwd, bwd) = (format!("{arm}.fwd"), format!("{arm}.bwd"));
            walks.push((fwd.clone(), bwd.clone()));
            walks.push((bwd, fwd));
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
                *length < 25.0,
                "{handedness:?}: {from} to {to} took {length} m, so it did not use the \
                 crossing"
            );
        }
    }
}

/// Each crosswalk at the junction is traced, exactly, to the crossing written for
/// it; the one half way along is traced to nothing, since nothing was written for
/// it; and once the built network is loaded, each crosswalk reaches the lane of the
/// crossing netconvert built, whatever number netconvert gave it.
#[test]
fn the_trace_follows_each_crosswalk_to_its_crossing() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in HANDEDNESS {
        let (map, arms) = scenarios::crosswalk_crossroads(handedness);
        let directory = tempfile::tempdir().unwrap();
        write_ir(&map, directory.path().join("map.ir.json")).unwrap();
        let (prefix, trace) = roadgen_sumo::write_traced(&map, directory.path()).unwrap();
        let sidecar = directory_sidecar(directory.path(), &prefix, &trace.format);
        write_trace(&trace, &map, &sidecar).unwrap();

        let crosswalks: Vec<&MapObject> = map
            .objects
            .iter()
            .filter(|object| object.kind == MapObjectKind::Crosswalk)
            .collect();
        assert_eq!(crosswalks.len(), 5);
        let mut at_junction: Vec<(String, &str)> = Vec::new();
        for crosswalk in &crosswalks {
            let links: Vec<_> = trace
                .links_of(&IrRef::Object(crosswalk.id.clone()))
                .collect();
            if crosswalk.id.as_str() == "object/crosswalk/west/0.5" {
                assert!(links.is_empty(), "{handedness:?}: {links:?}");
                continue;
            }
            let arm = map.lane(&crosswalk.lanes[0]).unwrap().road.clone();
            let name = ARMS[arms.iter().position(|road| *road == arm).unwrap()];
            assert_eq!(links.len(), 1, "{handedness:?}: {links:?}");
            assert_eq!(
                links[0].local,
                format!("crossing:j_x/{name}.bwd+{name}.fwd"),
                "{handedness:?}"
            );
            assert_eq!(links[0].relation, Relation::Exact, "{handedness:?}");
            at_junction.push((crosswalk.id.to_string(), name));
        }
        assert_eq!(at_junction.len(), 4);

        let net = sumo_build::netconvert(directory.path(), &prefix);
        let network = SumoNetwork::read(&net);
        let mut index = TraceIndex::new();
        index.load(directory.path().join("map.ir.json")).unwrap();
        index.load(&sidecar).unwrap();
        let report = index
            .load_sumo_net(&net)
            .expect("the built network, crossings and all, is the export's");
        assert_eq!(report.untraced, 0, "{handedness:?}");
        assert_eq!(report.crossings, 4, "{handedness:?}");

        for (crosswalk, name) in at_junction {
            let built = network
                .edges
                .iter()
                .find(|edge| {
                    edge.function.as_deref() == Some("crossing")
                        && edge.crossing_edges.iter().cloned().collect::<BTreeSet<_>>()
                            == both_ways(name)
                })
                .unwrap_or_else(|| panic!("{handedness:?}: no crossing of {name}"));
            let lanes: Vec<String> = index
                .from_ir(&crosswalk, "sumo")
                .unwrap()
                .into_iter()
                .filter(|link| link.role.as_deref() == Some("crossing"))
                .map(|link| link.local.clone())
                .collect();
            assert_eq!(
                lanes,
                [format!("lane:{}", built.lanes[0].id)],
                "{handedness:?}: {crosswalk}"
            );
        }
    }
}
