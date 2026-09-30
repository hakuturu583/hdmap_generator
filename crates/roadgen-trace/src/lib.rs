//! `roadgen-trace` — following one element of a map across every format it was
//! written in.
//!
//! Each exporter numbers its output its own way, and those numbers never flow back
//! into the IR. What an exporter can do is say, as it writes, which IR element each
//! written element came from: a [`Trace`](roadgen_core::trace::Trace). This crate
//! puts those on disk and reads them back.
//!
//! ```text
//! out/
//! ├── map.ir.json                    the IR dump          write_ir
//! ├── lanelet2_map.osm
//! ├── lanelet2_map.osm.trace.json    the Lanelet2 trace   write_trace
//! └── sumo/
//!     ├── town.nod.xml …
//!     └── sumo.trace.json            the SUMO trace
//! ```
//!
//! To go from lanelet `1000123` to SUMO, the Lanelet2 trace is read backwards to the
//! IR lane it came from, and the SUMO trace forwards from that lane: [`TraceIndex`]
//! does both. Formats are never mapped onto each other directly, so each one only
//! needs its own trace, and one written today joins one written tomorrow — provided
//! they came from the same map, which every file records as the IR's
//! [fingerprint](ir::IrCatalog::fingerprint) and every join checks.

pub mod error;
pub mod file;
pub mod index;
pub mod ir;

pub use error::TraceError;
pub use file::{read_ir, sidecar_path, write_ir, write_trace, TraceFile};
pub use index::{Link, SumoNetReport, TraceIndex, Translation};
pub use ir::{IrCatalog, IrDocument};

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;
    use roadgen_core::trace::{IrRef, Relation, Trace};

    fn two_roads() -> ValidatedMap {
        let mut builder = MapBuilder::default();
        let lanes = || {
            vec![
                LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
            ]
        };
        let a = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    lanes(),
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    lanes(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    /// A made-up format that numbers lanes by counting, as Lanelet2 does.
    fn counted(map: &ValidatedMap, format: &str, first: usize, file: &std::path::Path) -> Trace {
        let mut trace = Trace::new(format);
        for (index, lane) in map.lanes.iter().enumerate() {
            trace.link(
                lane.id.clone(),
                format!("lane:{}", first + index),
                Relation::Exact,
            );
        }
        std::fs::write(file, format!("{format} {first}")).unwrap();
        trace.files.push(file.to_path_buf());
        trace
    }

    #[test]
    fn two_traces_join_through_the_ir() {
        let map = two_roads();
        let dir = tempfile::tempdir().unwrap();
        let (a_file, b_file) = (dir.path().join("a.txt"), dir.path().join("b.txt"));
        write_ir(&map, dir.path().join("map.ir.json")).unwrap();
        write_trace(
            &counted(&map, "alpha", 100, &a_file),
            &map,
            sidecar_path(&a_file, "alpha"),
        )
        .unwrap();
        write_trace(
            &counted(&map, "beta", 7, &b_file),
            &map,
            sidecar_path(&b_file, "beta"),
        )
        .unwrap();

        let mut index = TraceIndex::new();
        index.load(dir.path().join("map.ir.json")).unwrap();
        index.load(dir.path().join("a.txt.trace.json")).unwrap();
        index.load(dir.path().join("b.txt.trace.json")).unwrap();

        let answers = index.translate("alpha", "lane:101", "beta").unwrap();
        assert_eq!(answers.len(), 1);
        assert_eq!(answers[0].local, "lane:8");
        assert_eq!(
            answers[0].ir,
            map.lanes.iter().nth(1).unwrap().id.to_string()
        );
        assert_eq!(answers[0].via, None);
    }

    /// A map whose numbers do not print short: a surveyed origin, a road whose
    /// length and stations are nothing round.
    fn awkward() -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("awkward".into()),
            origin: GeoOrigin::new(35.681236, 139.767125, 3.3).unwrap(),
            ..MapMetadata::default()
        });
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.1, 0.2, 0.3),
                    Point3::new(123.456789, 7.1, 0.7),
                    vec![LaneSpec::new(
                        PositiveWidth::new(3.3).unwrap(),
                        Direction::Forward,
                    )],
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn the_same_elements_in_another_place_are_another_map() {
        let moved = |end: f64| {
            let mut builder = MapBuilder::default();
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(end, 0.0, 0.0),
                        vec![LaneSpec::new(
                            PositiveWidth::new(3.5).unwrap(),
                            Direction::Forward,
                        )],
                    )
                    .unwrap()
                    .with_name("a"),
                )
                .unwrap();
            IrCatalog::of(&builder.finish().unwrap().validate().unwrap())
        };
        let (near, far) = (moved(100.0), moved(120.0));
        // Nothing a lookup reads tells them apart; the fingerprint still does.
        assert_eq!(near.roads.len(), far.roads.len());
        assert_eq!(near.lanes[0].id, far.lanes[0].id);
        assert_ne!(near.fingerprint(), far.fingerprint());
        assert_eq!(near.fingerprint(), moved(100.0).fingerprint());
    }

    #[test]
    fn a_dump_reads_back_to_the_fingerprint_it_was_written_with() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.ir.json");
        write_ir(&awkward(), &path).unwrap();
        let document = read_ir(&path).unwrap();
        assert_eq!(document, IrDocument::of(&awkward()));
    }

    #[test]
    fn a_dump_edited_under_its_fingerprint_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.ir.json");
        write_ir(&two_roads(), &path).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::write(&path, text.replacen("\"road/a\"", "\"road/z\"", 1)).unwrap();

        let error = read_ir(&path).unwrap_err();
        assert!(matches!(error, TraceError::Altered { .. }), "{error}");
        let error = TraceIndex::new().load(&path).unwrap_err();
        assert!(matches!(error, TraceError::Altered { .. }), "{error}");

        let mut document = IrDocument::of(&two_roads());
        document.body.lanes.pop();
        let error = TraceIndex::new().add_ir(document).unwrap_err();
        assert!(matches!(error, TraceError::Altered { .. }), "{error}");
    }

    #[test]
    fn a_rewritten_file_or_another_map_is_refused() {
        let map = two_roads();
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("a.txt");
        let trace_path = sidecar_path(&file, "alpha");
        write_trace(&counted(&map, "alpha", 1, &file), &map, &trace_path).unwrap();

        std::fs::write(&file, "rewritten").unwrap();
        let error = TraceIndex::new().load(&trace_path).unwrap_err();
        assert!(matches!(error, TraceError::Stale { .. }), "{error}");
        TraceIndex::new()
            .check_files(false)
            .load(&trace_path)
            .unwrap();

        let mut other = MapBuilder::default();
        other
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(50.0, 0.0, 0.0),
                    vec![LaneSpec::new(
                        PositiveWidth::new(3.0).unwrap(),
                        Direction::Forward,
                    )],
                )
                .unwrap()
                .with_name("other"),
            )
            .unwrap();
        let other = other.finish().unwrap().validate().unwrap();
        let other_path = dir.path().join("other.ir.json");
        write_ir(&other, &other_path).unwrap();
        let mut index = TraceIndex::new().check_files(false);
        index.load(&trace_path).unwrap();
        let error = index.load(&other_path).unwrap_err();
        assert!(matches!(error, TraceError::Mismatch { .. }), "{error}");
    }

    #[test]
    fn a_trace_whose_links_were_edited_is_refused() {
        let map = two_roads();
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("a.txt");
        let trace_path = sidecar_path(&file, "alpha");
        write_trace(&counted(&map, "alpha", 100, &file), &map, &trace_path).unwrap();
        TraceIndex::new().load(&trace_path).unwrap();

        // Another plausible value, with the file it describes left alone.
        let text = std::fs::read_to_string(&trace_path).unwrap();
        std::fs::write(&trace_path, text.replacen("lane:100", "lane:101", 1)).unwrap();
        let error = TraceIndex::new().load(&trace_path).unwrap_err();
        assert!(matches!(error, TraceError::Altered { .. }), "{error}");
    }

    #[test]
    fn a_missing_counterpart_steps_to_a_neighbour_in_the_dump() {
        let map = two_roads();
        let fingerprint = IrCatalog::of(&map).fingerprint();
        let lane = map.lanes.iter().next().unwrap();

        let mut lanes = Trace::new("lanes");
        lanes.link(lane.id.clone(), "lane:1", Relation::Exact);
        let mut roads = Trace::new("roads");
        for road in map.roads.iter() {
            roads.link(
                road.id.clone(),
                format!("road:{}", road.id.local_name()),
                Relation::Exact,
            );
        }

        let mut index = TraceIndex::new();
        index.add_trace(&lanes, &fingerprint).unwrap();
        index.add_trace(&roads, &fingerprint).unwrap();
        assert!(index
            .translate("lanes", "lane:1", "roads")
            .unwrap()
            .is_empty());

        index.add_ir(IrDocument::of(&map)).unwrap();
        let answers = index.translate("lanes", "lane:1", "roads").unwrap();
        assert_eq!(answers.len(), 1);
        assert_eq!(answers[0].ir, lane.road.to_string());
        assert_eq!(answers[0].via.as_deref(), Some(lane.id.as_str()));
    }

    #[test]
    fn bare_elements_take_the_formats_usual_kind() {
        let map = two_roads();
        let mut sumo = Trace::new("sumo");
        let lane = map.lanes.iter().next().unwrap();
        sumo.link(lane.id.clone(), "lane:a.fwd_0", Relation::Exact);
        sumo.link(lane.road.clone(), "edge:a.fwd", Relation::Part);
        let mut index = TraceIndex::new();
        index
            .add_trace(&sumo, &IrCatalog::of(&map).fingerprint())
            .unwrap();
        assert_eq!(index.qualify("sumo", "a.fwd_0"), "lane:a.fwd_0");
        assert_eq!(index.qualify("sumo", ":j_x_0_0"), "lane::j_x_0_0");
        assert_eq!(index.qualify("sumo", "edge:a.fwd"), "edge:a.fwd");
        assert_eq!(index.qualify("lanelet2", "1000123"), "lanelet:1000123");
        assert_eq!(
            index.to_ir("sumo", "a.fwd_0").unwrap()[0].ir,
            IrRef::Lane(lane.id.clone()).to_string()
        );
    }

    #[test]
    fn internal_lanes_of_a_built_network_are_traced_through_their_connections() {
        let map = two_roads();
        let lanes: Vec<_> = map.lanes.iter().map(|lane| lane.id.clone()).collect();
        let mut sumo = Trace::new("sumo");
        sumo.link(
            lanes[0].clone(),
            "connection:a.fwd_0>b.fwd_0",
            Relation::Merged,
        );
        let net = r#"<net>
            <connection from="a.fwd" to="b.fwd" fromLane="0" toLane="0" via=":j_0_0"/>
            <connection from=":j_0" to="b.fwd" fromLane="0" toLane="0" via=":j_2_0"/>
            <connection from=":j_2" to="b.fwd" fromLane="0" toLane="0"/>
            <connection from="b.bwd" to="b.fwd" fromLane="0" toLane="0" via=":t_0_0"/>
        </net>"#;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("x.net.xml");
        std::fs::write(&path, net).unwrap();

        let mut index = TraceIndex::new();
        index
            .add_trace(&sumo, &IrCatalog::of(&map).fingerprint())
            .unwrap();
        let report = index.load_sumo_net(&path).unwrap();
        assert_eq!(report.internal_lanes, 2);
        assert_eq!(report.untraced, 1);
        for internal in [":j_0_0", ":j_2_0"] {
            let links = index.to_ir("sumo", internal).unwrap();
            assert_eq!(links.len(), 1, "{internal}");
            assert_eq!(links[0].ir, lanes[0].to_string());
        }
    }
}
