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
    BuildingsDoc, Document, ElementDoc, End, Handedness, HeapDoc, HillDoc, LaneDoc, LinkDoc,
    Marking, PadDoc, RiverDoc, RoadDoc, RutPath, Shape, Side, StructureDoc, SurfaceDoc, TerrainDoc,
    Travel,
};
pub use import::{alignment_of, from_map};

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

    /// A road that leaves the origin eastwards, eases into a 60 m bend to the left,
    /// holds it, eases out and runs on straight.
    fn bend() -> RoadDoc {
        RoadDoc::aligned(
            "bend",
            [0.0, 0.0, 0.0],
            0.0,
            vec![
                ElementDoc::line(40.0),
                ElementDoc::spiral(20.0, Some(60.0)),
                ElementDoc::arc(30.0, 60.0),
                ElementDoc::spiral(20.0, None),
                ElementDoc::line(30.0),
            ],
        )
    }

    #[test]
    fn an_alignment_is_its_pieces_end_to_end() {
        let road = bend();
        let curve = build::reference_line(&road).unwrap();
        let expected = roadgen_core::Alignment::new(roadgen_core::Point3::ORIGIN, 0.0)
            .line(40.0, 0.0)
            .and_then(|a| a.spiral(20.0, 1.0 / 60.0, 0.0))
            .and_then(|a| a.arc(30.0, 1.0 / 60.0, 0.0))
            .and_then(|a| a.spiral(20.0, 0.0, 0.0))
            .and_then(|a| a.line(30.0, 0.0))
            .and_then(|a| a.finish())
            .unwrap();
        assert_eq!(curve, expected);
        assert!((curve.horizontal_length().unwrap() - 140.0).abs() < 1e-6);
        // Turned left by the arc and both spirals: 30/60 + 2 * 20/(2*60) radians.
        let turned = curve.end_tangent().unwrap().heading();
        assert!((turned - (0.5 + 1.0 / 3.0)).abs() < 1e-6, "{turned}");

        let document = Document {
            roads: vec![road],
            ..Document::default()
        };
        assert!(
            build(&document).is_valid(),
            "{:#?}",
            build(&document).problems
        );
    }

    #[test]
    fn a_bad_alignment_says_which_piece() {
        let mut road = bend();
        road.elements[2].radius = None;
        let error = build::reference_line(&road).unwrap_err();
        assert!(error.contains("piece 2"), "{error}");
        road.elements.clear();
        assert!(build::reference_line(&road).is_err());
    }

    #[test]
    fn a_smooth_road_meets_an_alignment_on_the_alignment_s_terms() {
        let road = bend();
        let curve = build::reference_line(&road).unwrap();
        let end = curve.end_point();
        let along = curve.end_tangent().unwrap().get();
        // Drawn off the end at an angle: the alignment cannot bend, so this does.
        let next = RoadDoc::two_way(
            "next",
            vec![
                end.as_array(),
                [end.x + 20.0, end.y + 40.0, 0.0],
                [end.x + 10.0, end.y + 80.0, 0.0],
            ],
        );
        let document = Document {
            roads: vec![road, next],
            links: vec![LinkDoc {
                from: "bend".into(),
                from_end: End::End,
                to: "next".into(),
                to_end: End::Start,
                junction: None,
            }],
            ..Document::default()
        };
        let built = build(&document);
        assert!(built.is_valid(), "{:#?}", built.problems);
        let map = built.validated().unwrap();
        let next = map.road(&roadgen_core::RoadId::new("next")).unwrap();
        let leaving = next.reference_line.start_tangent().unwrap().get();
        assert!(leaving.dot(along) > 1.0 - 1e-9, "{leaving:?} vs {along:?}");
    }

    #[test]
    fn a_handle_sets_the_direction_and_the_reach_at_its_point() {
        let mut road = RoadDoc::two_way(
            "h",
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 2.0], [100.0, 0.0, 4.0]],
        );
        let straight = build::reference_line(&road).unwrap();
        road.handles = vec![None, Some([12.0, 9.0])];
        let curve = build::reference_line(&road).unwrap();
        assert_ne!(curve, straight);
        let Curve3::Composite(pieces) = &curve else {
            panic!("{curve:?}")
        };
        // Through the point, along the handle, the same either side of it.
        assert!(
            pieces[0]
                .end_point()
                .distance_to(roadgen_core::Point3::new(50.0, 0.0, 2.0))
                < 1e-9
        );
        let out = pieces[0].end_tangent().unwrap().get();
        let into = pieces[1].start_tangent().unwrap().get();
        assert!(out.dot(into) > 1.0 - 1e-9);
        let plan = (out.y).atan2(out.x);
        assert!((plan - 9f64.atan2(12.0)).abs() < 1e-9, "{plan}");
        // A handle bends the road in plan, not in height: the grade is the chord's.
        let Curve3::Bezier(bezier) = &pieces[0] else {
            panic!("{:?}", pieces[0])
        };
        let [_, a, b, _] = *bezier.control();
        assert!(
            (b.x - 38.0).abs() < 1e-9 && (b.y + 9.0).abs() < 1e-9,
            "{b:?}"
        );
        assert!(
            (a.z - (50.0 / 3.0) * 0.04).abs() < 1e-9 && (b.z - (2.0 - 15.0 * 0.04)).abs() < 1e-9
        );

        let document = Document {
            roads: vec![road],
            ..Document::default()
        };
        assert!(
            build(&document).is_valid(),
            "{:#?}",
            build(&document).problems
        );
    }

    #[test]
    fn an_alignment_comes_back_from_the_ir_as_one() {
        let mut original = Document::example();
        let mut aligned = bend();
        aligned.points[0] = [0.0, -200.0, 0.0];
        original.roads.push(aligned.clone());
        let map = build(&original).validated().unwrap();
        let (document, _) = from_map(&map);
        let back = document.road("bend").unwrap();
        assert_eq!(back.shape, Shape::Alignment);
        assert!(back.heading.abs() < 1e-9);
        assert_eq!(back.elements.len(), aligned.elements.len());
        for (a, b) in aligned.elements.iter().zip(&back.elements) {
            assert_eq!(a.kind, b.kind);
            assert!((a.length - b.length).abs() < 1e-9);
            match (a.radius, b.radius) {
                (Some(x), Some(y)) => assert!((x - y).abs() < 1e-6, "{x} vs {y}"),
                (x, y) => assert_eq!(x, y),
            }
        }
        // Roads drawn through points stay drawn through points.
        assert_ne!(document.road("east").unwrap().shape, Shape::Alignment);
        assert_ne!(document.road("west").unwrap().shape, Shape::Alignment);
        let rebuilt = build(&document).validated().unwrap();
        let id = roadgen_core::RoadId::new("bend");
        let (was, now) = (map.road(&id).unwrap(), rebuilt.road(&id).unwrap());
        assert!(
            was.reference_line
                .end_point()
                .distance_to(now.reference_line.end_point())
                < 1e-6
        );
    }

    /// The example, with a bridge on the avenue as well as its tunnel.
    fn bridged() -> Document {
        let mut document = Document::example();
        let avenue = document
            .roads
            .iter_mut()
            .find(|r| r.id == "avenue")
            .unwrap();
        avenue.structures = vec![StructureDoc {
            name: "avenue bridge".to_owned(),
            ..StructureDoc::new("bridge", 12.0, 48.0)
        }];
        document
    }

    #[test]
    fn bridges_and_tunnels_go_on_their_roads() {
        let map = build(&bridged()).validated().unwrap();
        let kinds: Vec<(&str, &str)> = map
            .structures
            .iter()
            .map(|s| (s.kind.as_str(), s.carries[0].road.as_str()))
            .collect();
        assert!(kinds.contains(&("tunnel", "road/south")), "{kinds:?}");
        assert!(kinds.contains(&("bridge", "road/avenue")), "{kinds:?}");
        let south = map.road(&roadgen_core::RoadId::new("south")).unwrap();
        assert!(!south.structure.spans().is_empty());
    }

    #[test]
    fn a_bad_structure_is_named_and_the_rest_builds() {
        let mut document = bridged();
        document.roads[2].structures[0].to = 5.0; // ends before it starts
        let avenue = document
            .roads
            .iter_mut()
            .find(|r| r.id == "avenue")
            .unwrap();
        avenue.structures[0].form = "drawbridge".to_owned();
        let built = build(&document);
        let named: Vec<(Option<&str>, Option<usize>)> = built
            .problems
            .iter()
            .filter(|p| p.structure.is_some())
            .map(|p| (p.road.as_deref(), p.structure))
            .collect();
        assert_eq!(
            named,
            vec![(Some("south"), Some(0)), (Some("avenue"), Some(0))],
            "{:#?}",
            built.problems
        );
        assert!(built.map.unwrap().as_map().structures.is_empty());
    }

    #[test]
    fn bridges_and_tunnels_come_back_from_the_ir() {
        let mut original = bridged();
        original.roads[2].structures[0].form = "cut_and_cover".to_owned();
        original.roads[2].structures[0].clear_height = 4.8;
        let map = build(&original).validated().unwrap();
        let (document, notes) = from_map(&map);
        assert!(notes.is_empty(), "{notes:?}");
        for road in &original.roads {
            let back = &document.road(&road.id).unwrap().structures;
            assert_eq!(back.len(), road.structures.len(), "{}", road.id);
            for (a, b) in road.structures.iter().zip(back) {
                assert_eq!(
                    (a.kind.as_str(), a.name.as_str()),
                    (b.kind.as_str(), b.name.as_str())
                );
                assert!((a.from - b.from).abs() < 1e-9 && (a.to - b.to).abs() < 1e-9);
            }
        }
        let tunnel = &document.road("south").unwrap().structures[0];
        assert_eq!(tunnel.form, "cut_and_cover");
        assert!((tunnel.clear_height - 4.8).abs() < 1e-9);
        assert!(build(&document).is_valid());
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
