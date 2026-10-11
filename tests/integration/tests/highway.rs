//! A motorway generated from its rules: the structures it needs to cross the land,
//! its interchange onto a town, and what every format makes of it.

use roadgen_core::prelude::*;
use roadgen_core::structure::{StructureKind, Underneath};
use roadgen_core::FormOfWay;
use roadgen_integration_tests::highway::{build, highway};

#[test]
fn the_reference_motorway_validates() {
    let (map, report) = build(TrafficHandedness::RightHand);
    for note in &report.notes {
        eprintln!("note: {note}");
    }
    if let Err(error) = map.clone().validate() {
        panic!("{error}");
    }
}

#[test]
fn dump_structures() {
    let (map, report) = build(TrafficHandedness::RightHand);
    let map = map.into_map();
    for s in map.structures.iter() {
        let spans: Vec<String> = s
            .carries
            .iter()
            .map(|c| format!("{} {:.0}-{:.0}", c.road, c.from, c.to))
            .collect();
        eprintln!("S {} {} {:?}", s.id, s.kind.form(), spans);
    }
    for g in map.grade_separations.iter() {
        eprintln!("G {} via {}", g.id, g.via);
    }
    for i in map.interchanges.iter() {
        eprintln!(
            "I {} {} j={} r={} s={}",
            i.id,
            i.kind,
            i.junctions.len(),
            i.roads.len(),
            i.structures.len()
        );
    }
    eprintln!(
        "buildings {} notes {:?} sites {}",
        map.buildings.len(),
        report.notes,
        map.sites.len()
    );
    for c in map.land_covers.iter() {
        eprint!("{} ", c.kind);
    }
    eprintln!();
}
