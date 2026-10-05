//! Curve speeds in the SUMO export: an edge cut where its curvature changes, each
//! piece held to the speed its bend allows at the lateral acceleration asked for.
//!
//! As with the rest of the SUMO tests, what is checked is what netconvert built and
//! what the simulator did with it, not what the export wrote. When SUMO is not
//! installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`, which turns the
//! skip into a failure.

use std::collections::BTreeSet;

use roadgen_core::prelude::*;
use roadgen_core::trace::IrRef;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build::{self, SumoNetwork};
use roadgen_sumo::Options;

/// 2 m/s² sideways: a comfortable bend for a passenger car.
const ACCELERATION: f64 = 2.0;

fn curved() -> Options {
    Options {
        curve_lateral_acceleration: Some(ACCELERATION),
    }
}

fn forward_edges(network: &SumoNetwork, road: &str) -> Vec<String> {
    let mut edges: Vec<String> = network
        .roads()
        .iter()
        .map(|edge| edge.id.clone())
        .filter(|id| id.starts_with(&format!("{road}.fwd")))
        .collect();
    edges.sort();
    edges
}

/// The spiral scenario's bend — 120 m radius between clothoids, on an 80 km/h road
/// — is cut out of both straights and held to √(2 · 120) ≈ 15.5 m/s; the straights
/// keep the limit.
#[test]
fn a_bend_is_cut_out_and_held_to_its_curve_speed() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::spiral_transition_road();
    let prefix = roadgen_sumo::network_name(&map);
    let (directory, network) = sumo_build::build_with(&map, &curved());
    sumo_build::simulate(directory.path(), &prefix);

    let edges = forward_edges(&network, "sweep");
    assert_eq!(edges, ["sweep.fwd", "sweep.fwd.p1", "sweep.fwd.p2"]);
    let limit = 80.0 / 3.6;
    // The forward carriageway's centre runs a lane's half-width outside the
    // reference line, on the outside of this left-hand bend.
    let bend = (ACCELERATION * 120.0).sqrt();
    let outside = (ACCELERATION * (120.0 + 3.5)).sqrt();
    let speed = |id: &str| network.edge(id).lanes[0].speed;
    assert!((speed("sweep.fwd") - limit).abs() < 0.01);
    assert!((speed("sweep.fwd.p2") - limit).abs() < 0.01);
    assert!(
        speed("sweep.fwd.p1") <= outside && speed("sweep.fwd.p1") > bend - 0.2,
        "the bend is written at {} m/s, its curve speed is {bend}–{outside} m/s",
        speed("sweep.fwd.p1")
    );
    // The bend proper is 140 m, and the cut falls partway into each clothoid.
    let bend_length = network.edge("sweep.fwd.p1").lanes[0].length;
    assert!((140.0..260.0).contains(&bend_length), "{bend_length}");

    // A car driven through it slows for the bend before it gets there, and gets
    // back up to the limit after.
    let route: Vec<&str> = edges.iter().map(String::as_str).collect();
    let driven = sumo_build::drive(directory.path(), &prefix, &route, limit);
    let fastest = |edge: &str| {
        driven
            .iter()
            .filter(|(lane, _)| lane.starts_with(&format!("{edge}_")))
            .map(|(_, speed)| *speed)
            .fold(0.0, f64::max)
    };
    assert!(fastest("sweep.fwd.p1") <= speed("sweep.fwd.p1") + 1e-6);
    assert!(fastest("sweep.fwd.p2") > bend + 1.0);
}

/// Every piece is part of the IR lane it was cut from, and so is each connection
/// straight on from one piece to the next — in the trace, and in what netconvert
/// built.
#[test]
fn the_pieces_of_a_lane_are_traced_to_it_and_joined_end_to_end() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::spiral_transition_road();
    let directory = tempfile::tempdir().unwrap();
    let (prefix, trace) =
        roadgen_sumo::write_traced_with(&map, directory.path(), &curved()).unwrap();
    let network = SumoNetwork::read(&sumo_build::netconvert(directory.path(), &prefix));

    for lane in map.lanes.iter() {
        let links: BTreeSet<String> = trace
            .links_of(&IrRef::Lane(lane.id.clone()))
            .map(|link| link.local.clone())
            .collect();
        let lanes = links
            .iter()
            .filter(|local| local.starts_with("lane:"))
            .count();
        let joints = links
            .iter()
            .filter(|local| local.starts_with("connection:"))
            .count();
        assert_eq!((lanes, joints), (3, 2), "{}: {links:?}", lane.id);
    }

    let movements: BTreeSet<(&str, &str)> = network
        .movements()
        .into_iter()
        .map(|(from, _, to, _)| (from, to))
        .collect();
    for (from, to) in [
        ("sweep.fwd", "sweep.fwd.p1"),
        ("sweep.fwd.p1", "sweep.fwd.p2"),
        ("sweep.bwd", "sweep.bwd.p1"),
        ("sweep.bwd.p1", "sweep.bwd.p2"),
    ] {
        assert!(
            movements.contains(&(from, to)),
            "no {from} > {to} in {movements:?}"
        );
    }
}

/// Junctions keep working: the signals, crossings and trip weights of a map with
/// them build and run, and netconvert is asked to hold its internal lanes to the same
/// lateral acceleration — which slows the turns across the junction.
#[test]
fn junctions_build_and_their_turns_slow_down() {
    if !sumo_build::sumo_available() {
        return;
    }
    let internal_speeds = |network: &SumoNetwork| -> f64 {
        network
            .edges
            .iter()
            .filter(|edge| edge.function.as_deref() == Some("internal"))
            .flat_map(|edge| edge.lanes.iter().map(|lane| lane.speed))
            .fold(f64::INFINITY, f64::min)
    };
    let (crosswalks, _) = scenarios::signalised_crosswalk_crossroads(TrafficHandedness::RightHand);
    for map in [scenarios::controlled_crossroads(), crosswalks] {
        let prefix = roadgen_sumo::network_name(&map);
        let (_, plain) = sumo_build::build(&map);
        let (directory, network) = sumo_build::build_with(&map, &curved());
        sumo_build::simulate(directory.path(), &prefix);

        let config =
            std::fs::read_to_string(directory.path().join(format!("{prefix}.netccfg"))).unwrap();
        assert!(
            config.contains(r#"<junctions.limit-turn-speed value="2.000"/>"#),
            "{config}"
        );
        assert!(internal_speeds(&network) < internal_speeds(&plain));

        // The weights name every edge netconvert built from the export.
        let weights =
            std::fs::read_to_string(directory.path().join(format!("{prefix}.safe.src.xml")))
                .unwrap();
        for edge in network.roads() {
            assert!(
                weights.contains(&format!(r#"id="{}""#, edge.id)),
                "{} is not weighted",
                edge.id
            );
        }
    }
}

/// Without the option the export is what it always was: no edge cut, and nothing
/// said to netconvert about turns.
#[test]
fn without_the_option_nothing_is_cut() {
    let map = scenarios::spiral_transition_road();
    let plain = roadgen_sumo::to_plain_xml(&map).unwrap();
    let default = roadgen_sumo::to_plain_xml_with(&map, &Options::default()).unwrap();
    assert_eq!(plain, default);
    assert!(!plain.edges.contains(".p1"));
    assert!(!plain.config.contains("limit-turn-speed"));

    let curved = roadgen_sumo::to_plain_xml_with(&map, &curved()).unwrap();
    assert!(curved.edges.contains(r#"id="sweep.fwd.p1""#));
    // The lane map points at the piece a lane is entered on.
    assert_eq!(plain.lanes, curved.lanes);
}

#[test]
fn a_lateral_acceleration_has_to_be_positive() {
    let map = scenarios::spiral_transition_road();
    for bad in [0.0, -1.0, f64::NAN] {
        let options = Options {
            curve_lateral_acceleration: Some(bad),
        };
        assert!(roadgen_sumo::to_plain_xml_with(&map, &options).is_err());
    }
}
