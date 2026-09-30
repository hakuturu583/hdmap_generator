//! The trace a clip is written with: every row it names is a row of the layer it
//! names, every map row is named, and the same map traces the same way twice.

use std::collections::{BTreeMap, HashSet};
use std::fs::File;
use std::path::PathBuf;

use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use roadgen_clipgt::ClipConfig;
use roadgen_core::prelude::*;
use roadgen_core::{IrRef, Relation, Trace};

/// The layers that describe the map, and so are the only ones a trace names.
const MAP_LAYERS: [&str; 8] = [
    "lane",
    "lane_line",
    "road_boundary",
    "crosswalk",
    "wait_line",
    "traffic_light",
    "traffic_sign",
    "intersection_area",
];

fn lane(width: f64, direction: Direction) -> LaneSpec {
    LaneSpec::new(PositiveWidth::new(width).unwrap(), direction)
}

/// Two arms meeting at a controlled junction, each with a pavement that is not
/// driven on, and one of everything a clip has a layer for.
fn crossroads() -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("traced".into()),
        ..MapMetadata::default()
    });
    let arms = [
        (
            "north",
            Point3::new(0.0, 70.0, 0.0),
            Point3::new(0.0, 14.0, 0.0),
        ),
        (
            "east",
            Point3::new(70.0, 0.0, 0.0),
            Point3::new(14.0, 0.0, 0.0),
        ),
    ];
    let mut roads = Vec::new();
    for (name, start, end) in arms {
        let lanes = vec![
            lane(3.5, Direction::Forward),
            lane(3.5, Direction::Backward),
            lane(2.0, Direction::Forward).with_type(LaneType::Sidewalk),
        ];
        roads.push(
            builder
                .add_road(RoadSpec::line(start, end, lanes).unwrap().with_name(name))
                .unwrap(),
        );
    }
    let junction = builder.add_junction(Some("x"));
    builder
        .connect_ends(
            &roads[0],
            RoadEnd::End,
            &roads[1],
            RoadEnd::End,
            Some(&junction),
        )
        .unwrap();
    let approach = LaneRef::new(roads[0].clone(), 0);
    builder.add_stop_line(&approach, LaneEnd::End).unwrap();
    builder
        .add_traffic_light(&approach, LaneEnd::End, 5.0)
        .unwrap();
    builder
        .add_traffic_sign(&approach, LaneEnd::End, "stop", 2.0)
        .unwrap();
    builder.add_crosswalk(&roads[0], 0.8, 4.0).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

/// A written element's layer and row.
fn split(local: &str) -> (&str, usize) {
    let (layer, row) = local.split_once(':').expect("<layer>:<row>");
    (layer, row.parse().expect("a row index"))
}

/// Every link names a row that exists, and every row of a map layer is named.
fn check_against(trace: &Trace, rows: &BTreeMap<String, usize>) {
    assert_eq!(trace.format, "clipgt");
    for link in &trace.links {
        let (layer, row) = split(&link.local);
        assert!(
            MAP_LAYERS.contains(&layer),
            "{} is not a map layer",
            link.local
        );
        assert!(
            row < rows[layer],
            "{} is past the end of {layer}",
            link.local
        );
    }
    let named: HashSet<&str> = trace.links.iter().map(|link| link.local.as_str()).collect();
    for layer in MAP_LAYERS {
        for row in 0..rows[layer] {
            assert!(
                named.contains(format!("{layer}:{row}").as_str()),
                "{layer}:{row}"
            );
        }
    }
}

fn temp_dir(name: &str) -> PathBuf {
    let directory =
        std::env::temp_dir().join(format!("roadgen-clipgt-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&directory);
    directory
}

#[test]
fn the_trace_names_the_rows_of_the_layers_it_was_built_with() {
    let map = crossroads();
    let (layers, trace) = roadgen_clipgt::to_layers_traced(&map, &ClipConfig::new("x")).unwrap();
    assert!(trace.files.is_empty());
    let rows: BTreeMap<String, usize> = layers
        .iter()
        .map(|layer| (layer.name.to_owned(), layer.batch.num_rows()))
        .collect();
    for layer in MAP_LAYERS {
        assert!(rows[layer] > 0, "the scenario should fill {layer}");
    }
    check_against(&trace, &rows);
}

#[test]
fn the_trace_names_the_rows_of_the_files_it_wrote() {
    let map = crossroads();
    let directory = temp_dir("written");
    let (clip, trace) =
        roadgen_clipgt::write_traced(&map, &directory, &ClipConfig::new("x")).unwrap();
    assert_eq!(clip, "x");
    assert_eq!(trace.files.len(), 10, "one Parquet file per layer");

    let mut rows = BTreeMap::new();
    for path in &trace.files {
        let name = path.file_name().unwrap().to_str().unwrap();
        let layer = name
            .strip_prefix("x.")
            .and_then(|rest| rest.strip_suffix(".parquet"))
            .expect("{clip}.{layer}.parquet");
        let reader = ParquetRecordBatchReaderBuilder::try_new(File::open(path).unwrap())
            .unwrap()
            .build()
            .unwrap();
        let count: usize = reader.map(|batch| batch.unwrap().num_rows()).sum();
        rows.insert(layer.to_owned(), count);
    }
    check_against(&trace, &rows);
    std::fs::remove_dir_all(&directory).unwrap();
}

#[test]
fn every_drivable_lane_is_exactly_one_lane_row_and_nothing_else_is() {
    let map = crossroads();
    let (_, trace) = roadgen_clipgt::to_layers_traced(&map, &ClipConfig::new("x")).unwrap();
    for lane in map.lanes.iter() {
        let ir = IrRef::Lane(lane.id.clone());
        let rows: Vec<_> = trace
            .links_of(&ir)
            .filter(|link| link.local.starts_with("lane:"))
            .collect();
        if lane.lane_type.is_drivable() {
            assert_eq!(rows.len(), 1, "{}", lane.id);
            assert_eq!(rows[0].relation, Relation::Exact);
        } else {
            assert!(rows.is_empty(), "{} is not driven on", lane.id);
        }
    }
}

#[test]
fn a_line_two_lanes_share_is_merged_from_both() {
    let map = crossroads();
    let (_, trace) = roadgen_clipgt::to_layers_traced(&map, &ClipConfig::new("x")).unwrap();
    let mut by_row: BTreeMap<&str, Vec<_>> = BTreeMap::new();
    for link in &trace.links {
        if link.local.starts_with("lane_line:") || link.local.starts_with("road_boundary:") {
            by_row.entry(link.local.as_str()).or_default().push(link);
        }
    }
    assert!(by_row
        .values()
        .any(|links| links.len() == 2 && links.iter().all(|l| l.relation == Relation::Merged)));
    for links in by_row.values() {
        let expected = if links.len() > 1 {
            Relation::Merged
        } else {
            Relation::Exact
        };
        for link in links {
            assert_eq!(link.relation, expected, "{}", link.local);
            let role = link.role.as_deref();
            assert!(matches!(role, Some("left_boundary" | "right_boundary")));
        }
    }

    // The objects and the junction are one row each.
    for object in map.objects.iter() {
        let ir = IrRef::Object(object.id.clone());
        let links: Vec<_> = trace.links_of(&ir).collect();
        assert_eq!(links.len(), 1, "{}", object.id);
        assert_eq!(links[0].relation, Relation::Exact);
    }
    for junction in map.junctions.iter() {
        let ir = IrRef::Junction(junction.id.clone());
        let links: Vec<_> = trace.links_of(&ir).collect();
        assert_eq!(links.len(), 1);
        assert!(links[0].local.starts_with("intersection_area:"));
    }
}

#[test]
fn the_same_map_traces_the_same_way() {
    let map = crossroads();
    let config = ClipConfig::new("x");
    let (_, first) = roadgen_clipgt::to_layers_traced(&map, &config).unwrap();
    let (_, second) = roadgen_clipgt::to_layers_traced(&map, &config).unwrap();
    assert_eq!(first, second);
}
