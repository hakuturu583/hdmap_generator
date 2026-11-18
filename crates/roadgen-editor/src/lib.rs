//! `roadgen-editor` — the document a road-network editor keeps, and the step that
//! turns it into the canonical IR.
//!
//! ```text
//!   Document ──build()──▶ UnvalidatedMap ──validate()──▶ ValidatedMap ──▶ every format
//!      ▲                        │
//!      └──────from_map()────────┘ (an imported map, made editable)
//! ```
//!
//! The editor never edits the IR. It edits a [`Document`] — control points, lane
//! lists, which road ends meet — and rebuilds the IR from it after every change, so
//! that what is drawn, and what is written out, is always what the generator makes
//! of the document and never a hand-patched copy of it. `roadgen-irview` draws the
//! result; `roadgen-editor-wasm` puts both in a browser.
//!
//! # Example
//!
//! ```
//! use roadgen_editor::{build, Document};
//!
//! let built = build(&Document::example());
//! assert!(built.is_valid(), "{:?}", built.problems);
//! let map = built.validated()?;
//! assert!(map.junctions.len() == 1);
//! # Ok::<(), String>(())
//! ```

pub mod build;
pub mod document;
pub mod import;

pub use build::{build, Built, Problem, Severity};
pub use document::{
    Document, End, Handedness, LaneDoc, LinkDoc, Marking, RoadDoc, RutPath, Shape, Side,
    SurfaceDoc, Travel,
};
pub use import::from_map;

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::Curve3;

    #[test]
    fn the_example_builds_and_validates() {
        let built = build(&Document::example());
        assert!(built.is_valid(), "{:#?}", built.problems);
        assert!(built.problems.is_empty(), "{:#?}", built.problems);
        let map = built.validated().unwrap();
        // Five drawn roads, plus the connectors through the crossroads.
        let ordinary = map.roads.iter().filter(|road| !road.is_connector()).count();
        assert_eq!(ordinary, 5);
        assert!(map.roads.len() > 5);
        assert_eq!(map.junctions.len(), 1);
    }

    #[test]
    fn a_smooth_road_passes_through_its_points() {
        let road = RoadDoc::two_way(
            "s",
            vec![[0.0, 0.0, 0.0], [50.0, 20.0, 1.0], [100.0, 0.0, 2.0]],
        );
        let curve = build::reference_line(&road).unwrap();
        assert!(matches!(curve, Curve3::Composite(ref pieces) if pieces.len() == 2));
        assert!(
            curve
                .start_point()
                .distance_to(roadgen_core::Point3::new(0.0, 0.0, 0.0))
                < 1e-9
        );
        assert!(
            curve
                .end_point()
                .distance_to(roadgen_core::Point3::new(100.0, 0.0, 2.0))
                < 1e-9
        );
        // Tangent continuity at the middle point: the curve is smooth there.
        let Curve3::Composite(pieces) = curve else {
            unreachable!()
        };
        let out = pieces[0].end_tangent().unwrap().get();
        let into = pieces[1].start_tangent().unwrap().get();
        assert!(out.dot(into) > 0.999, "{out:?} vs {into:?}");
    }

    #[test]
    fn a_polyline_road_and_a_two_point_road() {
        let mut road = RoadDoc::two_way(
            "p",
            vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]],
        );
        road.shape = Shape::Polyline;
        assert!(matches!(
            build::reference_line(&road).unwrap(),
            Curve3::Polyline(_)
        ));
        road.points.truncate(2);
        assert!(matches!(
            build::reference_line(&road).unwrap(),
            Curve3::Line(_)
        ));
    }

    #[test]
    fn a_doubled_point_is_dropped_not_fatal() {
        let road = RoadDoc::two_way(
            "d",
            vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
        );
        assert!(matches!(
            build::reference_line(&road).unwrap(),
            Curve3::Line(_)
        ));
        let road = RoadDoc::two_way("e", vec![[0.0, 0.0, 0.0]]);
        assert!(build::reference_line(&road).is_err());
    }

    #[test]
    fn a_broken_road_is_left_out_and_named() {
        let mut document = Document::example();
        document.roads[0].lanes[0].width = -1.0;
        document.roads.push(RoadDoc::two_way(
            "lonely",
            vec![[500.0, 0.0, 0.0], [600.0, 0.0, 0.0]],
        ));
        let built = build(&document);
        let map = built.map.as_ref().expect("the rest still builds");
        assert!(map
            .as_map()
            .roads
            .iter()
            .any(|road| road.id.as_str() == "road/lonely"));
        assert!(!map
            .as_map()
            .roads
            .iter()
            .any(|road| road.id.as_str() == "road/west"));
        assert!(built
            .problems
            .iter()
            .any(|p| p.severity == Severity::Error && p.road.as_deref() == Some("west")));
        // The links to the missing road are refused, each by its index.
        assert!(built.problems.iter().any(|p| p.link == Some(0)));
    }

    #[test]
    fn a_link_whose_ends_are_apart_does_not_validate() {
        let mut document = Document::example();
        // Drag the avenue away from the end of `east` it is joined to.
        for point in &mut document.roads[4].points {
            point[1] += 30.0;
        }
        let built = build(&document);
        assert!(built.map.is_some());
        assert!(!built.is_valid());
        assert!(built.validated().is_err());
        // Either the generator refuses the joint, and the link is left out and
        // blamed, or it builds and validation finds the gap.
        assert!(built
            .problems
            .iter()
            .any(|p| p.link == Some(4) || p.severity == Severity::Invalid));
    }

    #[test]
    fn a_road_drawn_off_another_at_a_sharp_angle_is_bent_to_meet_it() {
        // The editor's own smoke test: a road drawn back over the map from the
        // end of the avenue, turning about 115° off it.
        let mut document = Document::example();
        let tip = *document.roads[4].points.last().unwrap();
        document.roads.push(RoadDoc::two_way(
            "back",
            vec![
                tip,
                [tip[0] - 10.0, tip[1] + 15.0, 0.0],
                [tip[0] - 25.0, tip[1] + 25.0, 0.0],
            ],
        ));
        document.links.push(LinkDoc {
            from: "avenue".into(),
            from_end: End::End,
            to: "back".into(),
            to_end: End::Start,
            junction: None,
        });
        let built = build(&document);
        assert!(built.is_valid(), "{:#?}", built.problems);
    }

    #[test]
    fn surface_features_go_on_their_roads() {
        let map = build(&Document::example()).validated().unwrap();
        let kinds: Vec<&str> = map
            .surface_features
            .iter()
            .map(|feature| feature.kind.as_str())
            .collect();
        for kind in ["rut", "pothole", "puddle", "bump"] {
            assert!(kinds.contains(&kind), "no {kind} in {kinds:?}");
        }
        assert!(map
            .surface_features
            .iter()
            .all(|feature| feature.road.as_str() == "road/west"));
    }

    #[test]
    fn a_rut_can_follow_a_path_through_a_lane() {
        let mut document = Document::example();
        document.roads[0].surface = vec![SurfaceDoc {
            depth: 0.025,
            path: Some(RutPath {
                lane: 0,
                knots: vec![[5.0, 0.0], [20.0, 0.6], [35.0, 0.0]],
            }),
            ..SurfaceDoc::new("rut", 0.0, 0.0, 0.0, 0.0)
        }];
        let built = build(&document);
        assert!(built.is_valid(), "{:#?}", built.problems);
        let map = built.validated().unwrap();
        let rut = map.surface_features.iter().next().unwrap();
        // The footprint was fitted to the path, not taken from the document.
        assert!((rut.station - 5.0).abs() < 1e-9 && (rut.length - 30.0).abs() < 1e-9);
        assert!(rut.width > 2.2);
    }

    #[test]
    fn a_bad_surface_feature_is_named_and_the_rest_builds() {
        let mut document = Document::example();
        document.roads[0].surface[1].kind = "crater".to_owned();
        document.roads[0].surface[2].bump = "wall".to_owned();
        document.roads[0].surface[2].kind = "bump".to_owned();
        let built = build(&document);
        let named: Vec<Option<usize>> = built
            .problems
            .iter()
            .filter(|p| p.road.as_deref() == Some("west"))
            .map(|p| p.surface)
            .collect();
        assert_eq!(named, vec![Some(1), Some(2)], "{:#?}", built.problems);
        let map = built.map.unwrap();
        assert_eq!(map.as_map().surface_features.len(), 2);
    }

    #[test]
    fn surface_features_come_back_from_the_ir() {
        let mut original = Document::example();
        original.roads[2].surface = vec![SurfaceDoc {
            class: "E".to_owned(),
            seed: 3,
            ..SurfaceDoc::new("roughness", 10.0, 40.0, 0.0, 7.0)
        }];
        let map = build(&original).validated().unwrap();
        let (document, notes) = from_map(&map);
        assert!(notes.is_empty(), "{notes:?}");
        for road in &original.roads {
            let back = &document.road(&road.id).unwrap().surface;
            assert_eq!(back.len(), road.surface.len(), "{}", road.id);
            for (a, b) in road.surface.iter().zip(back) {
                assert_eq!(a.kind, b.kind);
                assert!((a.station - b.station).abs() < 1e-9);
                assert!((a.offset - b.offset).abs() < 1e-9);
            }
        }
        let rebuilt = build(&document).validated().unwrap();
        assert_eq!(rebuilt.surface_features.len(), map.surface_features.len());
    }

    #[test]
    fn duplicate_and_unknown_names_are_problems() {
        let mut document = Document::example();
        document.roads.push(document.roads[0].clone());
        document.links.push(LinkDoc {
            from: "nowhere".into(),
            to: "west".into(),
            ..LinkDoc::default()
        });
        let built = build(&document);
        assert!(built
            .problems
            .iter()
            .any(|p| p.message.contains("two roads")));
        let last = document.links.len() - 1;
        assert!(built.problems.iter().any(|p| p.link == Some(last)));
        assert!(built.map.is_some());
    }

    #[test]
    fn importing_a_built_map_gives_back_the_same_network() {
        let original = Document::example();
        let map = build(&original).validated().unwrap();
        let (document, notes) = from_map(&map);
        assert!(notes.is_empty(), "{notes:?}");
        assert_eq!(document.roads.len(), original.roads.len());
        for road in &original.roads {
            let back = document.road(&road.id).expect("every road comes back");
            assert_eq!(back.lanes.len(), road.lanes.len());
            assert_eq!(back.points.first().unwrap(), road.points.first().unwrap());
            for (a, b) in back
                .points
                .last()
                .unwrap()
                .iter()
                .zip(road.points.last().unwrap())
            {
                assert!((a - b).abs() < 1e-6);
            }
        }
        // Every link the example made is there, in one direction or the other.
        assert_eq!(document.links.len(), original.links.len());
        for link in &original.links {
            assert!(
                document.links.iter().any(|back| {
                    let same = back.from == link.from && back.to == link.to;
                    let swapped = back.from == link.to && back.to == link.from;
                    (same || swapped) && back.junction == link.junction
                }),
                "{link:?} missing from {:?}",
                document.links
            );
        }

        // And it builds again into the same network.
        let rebuilt = build(&document).validated().unwrap();
        assert_eq!(rebuilt.roads.len(), map.roads.len());
        assert_eq!(rebuilt.lanes.len(), map.lanes.len());
        assert_eq!(rebuilt.connections.len(), map.connections.len());
    }

    #[test]
    fn simplifying_keeps_the_corners() {
        use roadgen_core::Point3;
        let line: Vec<Point3> = (0..=10)
            .map(|i| Point3::new(i as f64, 0.0, 0.0))
            .chain((1..=10).map(|i| Point3::new(10.0, i as f64, 0.0)))
            .collect();
        let simple = import::simplify(&line, 0.01);
        assert_eq!(simple.len(), 3);
        assert_eq!(simple[1], Point3::new(10.0, 0.0, 0.0));
    }

    #[test]
    fn left_hand_traffic_mirrors_the_lanes() {
        let mut document = Document::example();
        document.handedness = Handedness::Left;
        let map = build(&document).validated().unwrap();
        let lane = map
            .lanes
            .iter()
            .find(|lane| lane.road.as_str() == "road/west" && lane.index == 0)
            .unwrap();
        assert_eq!(lane.side, roadgen_core::LateralSide::Left);
    }
}
