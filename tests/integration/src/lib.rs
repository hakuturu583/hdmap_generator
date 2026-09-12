//! Shared fixtures for the roadgen scenario tests.
//!
//! Nothing in this crate is published; it exists so that the scenario maps and the
//! independent OpenDRIVE reader are written once and used by every test file.

pub mod clipgt_read;
pub mod opendrive_eval;
pub mod scenarios;

use std::sync::Arc;

use ll2_core::map::LaneletMap;
use ll2_routing::{default_costs, RoutingGraph};
use ll2_traffic_rules::{locations, participants, TrafficRules};
use opendrive::core::OpenDrive;

use roadgen_core::validation::ValidatedMap;

/// Exports to OpenDRIVE and reads the result back with the `opendrive` crate's own
/// parser, which is what a consumer of the file would do.
pub fn reparse_opendrive(map: &ValidatedMap) -> OpenDrive {
    let xml = roadgen_opendrive::to_xml(map).expect("the map should export as OpenDRIVE");
    OpenDrive::from_xml_str(&xml).expect("an OpenDRIVE parser should accept the export")
}

/// Exports to Lanelet2 and reads the result back with `simple_lanelet2`'s loader.
pub fn reload_lanelet2(map: &ValidatedMap) -> Arc<LaneletMap> {
    let xml = roadgen_lanelet2::to_osm_xml(map).expect("the map should export as Lanelet2");
    let projector = roadgen_lanelet2::projector_for(map).expect("a projector");
    ll2_io::load_str(&xml, projector.as_ref()).expect("a Lanelet2 loader should accept the export")
}

/// Exports a clip into a fresh directory and hands back both, so the directory lives
/// as long as the test does.
pub fn write_clip(
    map: &ValidatedMap,
    config: &roadgen_clipgt::ClipConfig,
) -> (tempfile::TempDir, String) {
    let directory = tempfile::tempdir().expect("a temporary directory");
    let clip = roadgen_clipgt::write(map, directory.path(), config).expect("the map should export");
    (directory, clip)
}

/// A routing graph over a loaded Lanelet2 map, built the way a consumer would.
pub fn routing_graph(map: &LaneletMap) -> RoutingGraph {
    let rules = TrafficRules::create(locations::GERMANY, participants::VEHICLE)
        .expect("the standard vehicle rules");
    RoutingGraph::build(map, Arc::new(rules), &default_costs())
}

/// Number of successor edges a Lanelet2 routing graph finds in the map.
pub fn routing_edges(map: &LaneletMap) -> usize {
    let graph = routing_graph(map);
    map.lanelets
        .all()
        .iter()
        .filter_map(ll2_core::map::as_lanelet)
        .map(|lanelet| graph.following(lanelet, false).len())
        .sum()
}
