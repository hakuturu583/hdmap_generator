//! Shared fixtures for the roadgen scenario tests.
//!
//! Nothing in this crate is published; it exists so that the scenario maps and the
//! independent OpenDRIVE reader are written once and used by every test file.

pub mod clipgt_read;
pub mod opendrive_eval;
pub mod scenarios;
pub mod sumo_build;

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

/// Exports a GPUDrive scene and reads it back through the data model, which is how a
/// consumer that knows the format reads it.
pub fn read_scene(
    map: &ValidatedMap,
    config: &roadgen_gpudrive::SceneConfig,
) -> roadgen_gpudrive::Scene {
    serde_json::from_str(&scene_text(map, config)).expect("the scene should read back")
}

/// The same export as raw JSON, for the checks that are about the shape of the
/// document rather than what is in it.
pub fn read_scene_json(
    map: &ValidatedMap,
    config: &roadgen_gpudrive::SceneConfig,
) -> serde_json::Value {
    serde_json::from_str(&scene_text(map, config)).expect("the scene should be JSON")
}

fn scene_text(map: &ValidatedMap, config: &roadgen_gpudrive::SceneConfig) -> String {
    roadgen_gpudrive::to_json(map, config).expect("the map should export as a scene")
}

/// Exports to plain OpenStreetMap and reads the result back with the same document
/// model a consumer would, along with a way of turning its nodes back into metres.
pub fn reload_osm(map: &ValidatedMap) -> OsmReading {
    let xml = roadgen_osm::to_xml(map).expect("the map should export as OSM");
    let (document, errors) = ll2_io::osm::parse(&xml).expect("an OSM parser should accept it");
    assert!(
        errors.is_empty(),
        "the file should parse cleanly: {errors:?}"
    );

    let projector = ll2_projection::LocalCartesian::new(ll2_projection::Origin::new(
        ll2_projection::GpsPoint::new(
            map.metadata.origin.latitude(),
            map.metadata.origin.longitude(),
            map.metadata.origin.altitude(),
        ),
    ));
    let metres = document
        .nodes
        .values()
        .map(|node| {
            let point = ll2_projection::Projector::forward(
                &projector,
                ll2_projection::GpsPoint::new(node.lat, node.lon, node.ele),
            )
            .expect("a node should project back");
            (
                node.id,
                roadgen_core::geometry::Point3::new(point[0], point[1], node.ele),
            )
        })
        .collect();
    OsmReading {
        xml,
        document,
        metres,
    }
}

/// An exported OSM file, parsed, with its nodes back in the map's own metres.
pub struct OsmReading {
    pub xml: String,
    pub document: ll2_io::osm::Document,
    pub metres: std::collections::HashMap<i64, roadgen_core::geometry::Point3>,
}

impl OsmReading {
    /// The way tagged with this `name`.
    pub fn way_named(&self, name: &str) -> &ll2_io::osm::Way {
        self.document
            .ways
            .values()
            .find(|way| way.tags.get("name").is_some_and(|value| value == name))
            .unwrap_or_else(|| panic!("no way called {name:?}"))
    }

    /// Every way carrying a `highway` tag with this value.
    pub fn ways_tagged(&self, key: &str, value: &str) -> Vec<&ll2_io::osm::Way> {
        self.document
            .ways
            .values()
            .filter(|way| way.tags.get(key).is_some_and(|found| found == value))
            .collect()
    }

    /// Every node carrying this tag.
    pub fn nodes_tagged(&self, key: &str, value: &str) -> Vec<&ll2_io::osm::Node> {
        self.document
            .nodes
            .values()
            .filter(|node| node.tags.get(key).is_some_and(|found| found == value))
            .collect()
    }

    /// Where a node sits in the map's own metres.
    pub fn at(&self, id: i64) -> roadgen_core::geometry::Point3 {
        self.metres[&id]
    }
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
