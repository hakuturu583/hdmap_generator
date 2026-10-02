//! One map, every format, and the trace files between them.
//!
//! Each exporter's own tests check that its trace names what it wrote. These ask
//! the question the traces exist for: whether, loaded together from disk, they take
//! an element of one format to the element of another that came from the same part
//! of the IR — and they check the answer against what each format says on its own,
//! not against the traces.

use std::path::Path;

use ll2_core::map::as_lanelet;
use roadgen_core::map::Lane;
use roadgen_core::prelude::*;
use roadgen_core::trace::Relation;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build::{netconvert, sumo_available, SumoNetwork};
use roadgen_trace::{directory_sidecar, sidecar_path, write_ir, write_trace, TraceIndex};

/// Every export the traces cover, written into `directory` with its trace beside it,
/// and the IR dump. Returns the prefix the SUMO export, in `sumo/`, named its files by.
fn export_everything(map: &ValidatedMap, directory: &Path) -> String {
    write_ir(map, directory.join("map.ir.json")).unwrap();

    let xodr = directory.join("map.xodr");
    let trace = roadgen_opendrive::write_traced(map, &xodr).unwrap();
    write_trace(&trace, map, sidecar_path(&xodr)).unwrap();

    let lanelet2 = directory.join("lanelet2_map.osm");
    let trace = roadgen_lanelet2::write_traced(map, &lanelet2).unwrap();
    write_trace(&trace, map, sidecar_path(&lanelet2)).unwrap();

    let osm = directory.join("openstreetmap.osm");
    let trace = roadgen_osm::write_traced(map, &osm).unwrap();
    write_trace(&trace, map, sidecar_path(&osm)).unwrap();

    let sumo = directory.join("sumo");
    let (prefix, trace) = roadgen_sumo::write_traced(map, &sumo).unwrap();
    write_trace(
        &trace,
        map,
        directory_sidecar(&sumo, &prefix, &trace.format),
    )
    .unwrap();

    let clip = directory.join("clip");
    let (id, trace) =
        roadgen_clipgt::write_traced(map, &clip, &roadgen_clipgt::ClipConfig::default()).unwrap();
    write_trace(&trace, map, directory_sidecar(&clip, &id, &trace.format)).unwrap();

    let scene = directory.join("scene.json");
    let trace =
        roadgen_gpudrive::write_traced(map, &scene, &roadgen_gpudrive::SceneConfig::default())
            .unwrap();
    write_trace(&trace, map, sidecar_path(&scene)).unwrap();
    prefix
}

fn load_everything(directory: &Path) -> TraceIndex {
    // A directory export names its trace after what it named its files, so those are
    // found by looking.
    let inside = |folder: &str| {
        std::fs::read_dir(directory.join(folder))
            .unwrap()
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| path.to_string_lossy().ends_with(".trace.json"))
            .collect::<Vec<_>>()
    };
    let (sumo, clip) = (inside("sumo"), inside("clip"));
    assert_eq!((sumo.len(), clip.len()), (1, 1), "{sumo:?} {clip:?}");
    let mut index = TraceIndex::new();
    for file in [
        "map.ir.json",
        "map.xodr.trace.json",
        "lanelet2_map.osm.trace.json",
        "openstreetmap.osm.trace.json",
        "scene.json.trace.json",
    ]
    .into_iter()
    .map(|name| directory.join(name))
    .chain(sumo)
    .chain(clip)
    {
        index
            .load(&file)
            .unwrap_or_else(|error| panic!("{}: {error}", file.display()));
    }
    index
}

/// The lanelet a lane became, read from the Lanelet2 trace.
fn lanelet_of(index: &TraceIndex, lane: &Lane) -> Option<String> {
    index
        .from_ir(lane.id.as_str(), "lanelet2")
        .unwrap()
        .into_iter()
        .find(|link| link.role.as_deref() == Some("lanelet"))
        .map(|link| link.local.clone())
}

#[test]
fn a_lanelet_translates_to_the_lane_each_format_wrote_for_it() {
    for map in [scenarios::crossroads(), scenarios::controlled_crossroads()] {
        lanelets_translate_to_their_lanes(&map);
    }
}

fn lanelets_translate_to_their_lanes(map: &ValidatedMap) {
    let map = map.clone();
    let directory = tempfile::tempdir().unwrap();
    export_everything(&map, directory.path());
    let index = load_everything(directory.path());
    let sumo_lanes = roadgen_sumo::to_plain_xml(&map).unwrap().lanes;

    let mut checked = 0;
    for lane in map.lanes.iter() {
        let Some(lanelet) = lanelet_of(&index, lane) else {
            continue;
        };

        // OpenDRIVE, against the exporter's own public numbering.
        let (road, number) = roadgen_opendrive::lane_id(&map, &lane.id).unwrap();
        let expected = format!("lane:{road}/{}/{number}", lane.section);
        let answers = index.translate("lanelet2", &lanelet, "opendrive").unwrap();
        let lanes: Vec<_> = answers
            .iter()
            .filter(|answer| answer.local.starts_with("lane:"))
            .collect();
        assert_eq!(lanes.len(), 1, "{lanelet}: {answers:?}");
        assert_eq!(lanes[0].local, expected);
        assert_eq!(lanes[0].ir, lane.id.as_str());

        // SUMO, against the lane table the exporter already published — or, for a
        // junction connector SUMO does not write, the connection it became.
        let answers = index.translate("lanelet2", &lanelet, "sumo").unwrap();
        match sumo_lanes.get(&lane.id) {
            Some(sumo) => {
                assert_eq!(answers.len(), 1, "{lanelet}: {answers:?}");
                assert_eq!(answers[0].local, format!("lane:{sumo}"));
                assert_eq!(answers[0].relation, Relation::Exact);
            }
            None if lane.lane_type.is_drivable() => {
                assert!(
                    answers
                        .iter()
                        .any(|answer| answer.local.starts_with("connection:")
                            && answer.relation == Relation::Collapsed),
                    "{lanelet} ({}) should have collapsed into a SUMO connection: {answers:?}",
                    lane.id
                );
            }
            None => {}
        }

        // And back again: the lanelet is the only lanelet of the lane.
        let back = index
            .translate("opendrive", &expected, "lanelet2")
            .unwrap()
            .into_iter()
            .filter(|answer| answer.role.as_deref() == Some("lanelet"))
            .map(|answer| answer.local)
            .collect::<Vec<_>>();
        assert_eq!(back, vec![lanelet]);
        checked += 1;
    }
    assert_eq!(
        checked,
        roadgen_lanelet2::to_lanelet_map_traced(&map)
            .unwrap()
            .1
            .links
            .iter()
            .filter(|link| link.role.as_deref() == Some("lanelet"))
            .count()
    );
    assert!(checked > 0);
}

#[test]
fn a_traced_lanelet_lies_on_the_lane_it_is_traced_to() {
    for map in [scenarios::crossroads(), scenarios::controlled_crossroads()] {
        traced_lanelets_lie_on_their_lanes(&map);
    }
}

fn traced_lanelets_lie_on_their_lanes(map: &ValidatedMap) {
    let map = map.clone();
    let directory = tempfile::tempdir().unwrap();
    export_everything(&map, directory.path());
    let index = load_everything(directory.path());

    let xml = std::fs::read_to_string(directory.path().join("lanelet2_map.osm")).unwrap();
    let projector = roadgen_lanelet2::projector_for(&map).unwrap();
    let loaded = ll2_io::load_str(&xml, projector.as_ref()).unwrap();

    let lanelets: Vec<_> = loaded
        .lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .collect();
    let mut checked = 0;
    for lanelet in &lanelets {
        let links = index.to_ir("lanelet2", &lanelet.id().to_string()).unwrap();
        let Some(link) = links
            .iter()
            .find(|link| link.role.as_deref() == Some("lanelet"))
        else {
            continue;
        };
        let lane = map
            .lane(&LaneId::from_raw(&link.ir))
            .expect("a trace names lanes of the map");

        // A lanelet runs the way traffic does, and a lane along its reference line,
        // so the two centrelines share their ends in one order or the other.
        let centerline = lanelet.centerline();
        let (front, back) = (centerline.front().unwrap(), centerline.back().unwrap());
        let (start, end) = (lane.centerline.start_point(), lane.centerline.end_point());
        let near = |x: f64, y: f64, point: Point3| (x - point.x).hypot(y - point.y) < 1e-3;
        let along = near(front.x(), front.y(), start) && near(back.x(), back.y(), end);
        let against = near(front.x(), front.y(), end) && near(back.x(), back.y(), start);
        assert!(
            along || against,
            "lanelet {} is not on {}",
            lanelet.id(),
            lane.id
        );
        checked += 1;
    }
    assert!(checked > 0, "no lanelet was traced");
}

#[test]
fn a_rewritten_export_is_not_joined_with_a_stale_trace() {
    let map = scenarios::controlled_crossroads();
    let directory = tempfile::tempdir().unwrap();
    export_everything(&map, directory.path());

    // Written again from another map, without its trace.
    roadgen_lanelet2::write(
        &scenarios::crossroads(),
        directory.path().join("lanelet2_map.osm"),
    )
    .unwrap();
    let error = TraceIndex::new()
        .load(directory.path().join("lanelet2_map.osm.trace.json"))
        .unwrap_err();
    assert!(
        matches!(error, roadgen_trace::TraceError::Stale { .. }),
        "{error}"
    );
}

#[test]
fn a_network_built_from_another_map_with_the_same_names_is_refused() {
    if !sumo_available() {
        return;
    }
    let ours = tempfile::tempdir().unwrap();
    export_everything(&scenarios::crossroads(), ours.path());

    // Every road, lane, junction and connection named as ours are, with arms 20 m
    // longer: a network netconvert builds from it shares every name ours has.
    let other = scenarios::crossroads_builder("crossroads", 90.0)
        .finish()
        .unwrap()
        .validate()
        .unwrap();
    let theirs = tempfile::tempdir().unwrap();
    let prefix = roadgen_sumo::write(&other, theirs.path()).unwrap();
    let net = netconvert(theirs.path(), &prefix);

    let mut index = load_everything(ours.path());
    let error = index.load_sumo_net(&net).unwrap_err();
    assert!(
        matches!(error, roadgen_trace::TraceError::Foreign { .. }),
        "{error}"
    );
    // Nothing was added from it.
    assert!(index.to_ir("sumo", ":j_x_0_0").unwrap().is_empty());
}

#[test]
fn an_internal_lane_netconvert_drew_translates_to_the_connector_it_carries() {
    if !sumo_available() {
        return;
    }
    let map = scenarios::crossroads();
    let directory = tempfile::tempdir().unwrap();
    let prefix = export_everything(&map, directory.path());
    let net = netconvert(&directory.path().join("sumo"), &prefix);

    let mut index = load_everything(directory.path());
    let report = index.load_sumo_net(&net).unwrap();
    // Twelve movements, two of them left turns that stop inside the junction and so
    // are drawn as two internal lanes each — and nothing netconvert made up, since
    // the export's configuration keeps it from adding turnarounds.
    assert_eq!(report.internal_lanes, 14);
    assert_eq!(report.untraced, 0);

    let connectors: Vec<&Lane> = map
        .lanes
        .iter()
        .filter(|lane| map.road(&lane.road).unwrap().is_connector())
        .collect();
    let mut reached = std::collections::BTreeSet::new();
    let network = SumoNetwork::read(&net);
    for internal in network
        .edges
        .iter()
        .filter(|edge| edge.function.as_deref() == Some("internal"))
        .flat_map(|edge| &edge.lanes)
        .map(|lane| lane.id.as_str())
        .filter(|id| id.starts_with(":j_"))
    {
        // One OpenDRIVE lane, reached through the connector lane itself and through
        // the connections into and out of it, which OpenDRIVE writes as that lane's
        // predecessor and successor.
        let answers = index.translate("sumo", internal, "opendrive").unwrap();
        let lanes: std::collections::BTreeSet<&str> = answers
            .iter()
            .filter(|answer| answer.local.starts_with("lane:"))
            .map(|answer| answer.local.as_str())
            .collect();
        assert_eq!(lanes.len(), 1, "{internal}: {answers:?}");
        let exact: Vec<_> = answers
            .iter()
            .filter(|answer| {
                answer.local.starts_with("lane:") && answer.relation == Relation::Exact
            })
            .collect();
        assert_eq!(exact.len(), 1, "{internal}: {answers:?}");
        let lane = connectors
            .iter()
            .find(|lane| lane.id.as_str() == exact[0].ir)
            .unwrap_or_else(|| panic!("{internal} traced to {}, not a connector", exact[0].ir));
        reached.insert(lane.id.clone());
    }
    assert_eq!(reached.len(), connectors.len());
}

#[test]
fn a_right_of_way_rule_translates_to_what_each_format_carries_it_by() {
    let map = scenarios::controlled_crossroads();
    let directory = tempfile::tempdir().unwrap();
    export_everything(&map, directory.path());
    let index = load_everything(directory.path());

    let rules: Vec<usize> = map
        .rules
        .iter()
        .enumerate()
        .filter(|(_, rule)| matches!(rule, TrafficRule::RightOfWay { .. }))
        .map(|(index, _)| index)
        .collect();
    assert!(!rules.is_empty());
    for rule in rules {
        let ir = format!("rule/{rule}");
        // Lanelet2 writes it as a regulatory element of its own.
        let element = index.from_ir(&ir, "lanelet2").unwrap();
        assert_eq!(element.len(), 1, "{ir}");
        let element = element[0].local.clone();

        // OpenDRIVE as `<priority>` entries of a junction.
        let answers = index.translate("lanelet2", &element, "opendrive").unwrap();
        assert!(
            answers
                .iter()
                .any(|answer| answer.local.starts_with("junction:")
                    && answer.role.as_deref() == Some("priority")
                    && answer.via.is_none()),
            "{ir}: {answers:?}"
        );
        // SUMO as the priority of the edges its lanes are on.
        let answers = index.translate("lanelet2", &element, "sumo").unwrap();
        assert!(!answers.is_empty(), "{ir}");
        assert!(
            answers
                .iter()
                .all(|answer| answer.local.starts_with("edge:")
                    && answer.role.as_deref() == Some("priority")
                    && answer.via.is_none()),
            "{ir}: {answers:?}"
        );
    }
}

/// A UTM map's network is offset by hundreds of kilometres — the origin's easting
/// and northing, which is its geo-reference — and still checks out against the export
/// node for node: that offset was declared by the node file and moved nothing, so it
/// is not mistaken for a shift netconvert applied.
#[test]
fn a_utm_network_is_recognised_as_built_from_its_export() {
    if !sumo_available() {
        return;
    }
    let mut builder = scenarios::crossroads_builder("crossroads", 70.0);
    builder.metadata_mut().projection = Projection::Utm;
    let map = builder.finish().unwrap().validate().unwrap();
    let directory = tempfile::tempdir().unwrap();
    let prefix = export_everything(&map, directory.path());
    let net = netconvert(&directory.path().join("sumo"), &prefix);
    let offset = SumoNetwork::read(&net).location.net_offset;
    assert!(
        offset.0 < -100_000.0 && offset.1 < -1_000_000.0,
        "{offset:?}"
    );

    let mut index = load_everything(directory.path());
    let report = index.load_sumo_net(&net).unwrap();
    assert_eq!(report.internal_lanes, 14);
    assert_eq!(report.untraced, 0);
}
