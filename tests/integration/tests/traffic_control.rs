//! Traffic control in both files.
//!
//! The IR holds a traffic light as a position in space and the lanes it governs.
//! OpenDRIVE wants it at `(s, t, zOffset)` in one road's own coordinates, with the
//! lanes named as a range of its lane ids; Lanelet2 wants a way and a regulatory
//! element. These check that the same object arrives, correctly placed, in both.

use ll2_core::map::as_lanelet;
use opendrive::object::orientation::{ObjectType, Orientation};
use roadgen_core::prelude::*;
use roadgen_integration_tests::opendrive_eval::{Position, RoadEvaluator};
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::{reload_lanelet2, reparse_opendrive};

#[test]
fn a_traffic_light_becomes_a_signal_where_the_ir_put_it() {
    let map = scenarios::controlled_crossroads();
    let document = reparse_opendrive(&map);

    let signals: Vec<_> = document
        .road
        .iter()
        .flat_map(|road| {
            road.signals
                .iter()
                .flat_map(|signals| signals.signal.iter())
        })
        .collect();
    assert_eq!(signals.len(), 1, "one light was added");
    let light = signals[0];
    assert!(light.dynamic, "a traffic light changes");
    assert_eq!(light.orientation, Orientation::Plus);

    // It governs the lane it was attached to, named by that lane's OpenDRIVE id.
    assert_eq!(light.validity.len(), 1);
    assert_eq!(light.validity[0].from_lane, -1);
    assert_eq!(light.validity[0].to_lane, -1);

    // And it is where the IR put it: at the far end of the north approach, over the
    // middle of its lane, five metres up.
    let ir_object = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::TrafficLight)
        .unwrap();
    let ObjectGeometry::Line(bar) = &ir_object.geometry else {
        panic!("a light is a bar across the lane")
    };
    let centre = bar.start_point().lerp(bar.end_point(), 0.5);

    let road = map.road(&RoadId::new("north")).unwrap();
    let length = road.horizontal_length().unwrap();
    assert!(
        (light.s.value - length).abs() < 1e-6,
        "the light sits at the end of the approach, at s={}",
        light.s.value
    );
    assert!((light.t.value + 1.75).abs() < 1e-6, "t={}", light.t.value);
    assert!(
        (light.z_offset.value - 5.0).abs() < 1e-6,
        "z={}",
        light.z_offset.value
    );

    // The whole point of `(s, t, zOffset)`: rebuilding the position from the document
    // lands back on the position the IR held.
    let evaluator = RoadEvaluator::find(&document, "0").unwrap();
    let rebuilt = evaluator.lane_center(-1, light.s.value).unwrap();
    assert!(
        Position {
            x: centre.x,
            y: centre.y,
            z: centre.z - light.z_offset.value,
        }
        .distance_to(rebuilt)
            < 1e-6
    );
}

#[test]
fn a_traffic_sign_carries_the_callers_own_catalogue_code() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    let lane = LaneRef::new(road.clone(), 0);
    builder
        .add_traffic_sign(&lane, LaneEnd::End, "de274-60", 2.5)
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let document = reparse_opendrive(&map);
    let signal = &document.road[0].signals.as_ref().unwrap().signal[0];
    // OpenDRIVE identifies a sign by a code from a country's catalogue, and the IR
    // already holds the caller's, so it passes straight through.
    assert_eq!(signal.r#type, "de274-60");
    assert!(!signal.dynamic);
    assert!((signal.z_offset.value - 2.5).abs() < 1e-6);
}

#[test]
fn a_stop_line_becomes_a_road_mark_object_across_the_lane() {
    let map = scenarios::controlled_crossroads();
    let document = reparse_opendrive(&map);

    let objects: Vec<_> = document
        .road
        .iter()
        .flat_map(|road| {
            road.objects
                .iter()
                .flat_map(|objects| objects.object.iter())
        })
        .collect();

    let stop_line = objects
        .iter()
        .find(|object| object.r#type == Some(ObjectType::RoadMark))
        .expect("the stop line is a road-mark object");
    assert_eq!(stop_line.name.as_deref(), Some("stopLine"));
    assert_eq!(stop_line.subtype.as_deref(), Some("stopLine"));
    // It reaches right across the lane it stops, and only a little along the road.
    assert!((stop_line.width.unwrap().value - 3.5).abs() < 1e-6);
    assert!(stop_line.length.unwrap().value < 1.0);
    assert!(
        stop_line.z_offset.value.abs() < 1e-6,
        "paint is on the road"
    );
    assert_eq!(stop_line.validity[0].from_lane, -1);
}

#[test]
fn a_crosswalk_becomes_an_object_with_its_four_corners() {
    let map = scenarios::controlled_crossroads();
    let document = reparse_opendrive(&map);

    let crosswalk = document
        .road
        .iter()
        .flat_map(|road| {
            road.objects
                .iter()
                .flat_map(|objects| objects.object.iter())
        })
        .find(|object| object.r#type == Some(ObjectType::Crosswalk))
        .expect("the crosswalk is a crosswalk object");

    let outline = crosswalk.outline.as_ref().expect("with an outline");
    assert_eq!(outline.closed, Some(true));
    assert_eq!(outline.choice.len(), 4);

    // The corners are in the road's own coordinates, spanning its full width and the
    // crosswalk's own depth along it.
    let corners: Vec<(f64, f64)> = outline
        .choice
        .iter()
        .map(|corner| match corner {
            opendrive::object::corner::Corner::Road(road) => (road.s.value, road.t.value),
            _ => panic!("corners are given in road coordinates"),
        })
        .collect();
    let across = corners
        .iter()
        .map(|(_, t)| *t)
        .fold(f64::NEG_INFINITY, f64::max)
        - corners
            .iter()
            .map(|(_, t)| *t)
            .fold(f64::INFINITY, f64::min);
    assert!((across - 7.0).abs() < 1e-3, "across {across} m");
    let along = corners
        .iter()
        .map(|(s, _)| *s)
        .fold(f64::NEG_INFINITY, f64::max)
        - corners
            .iter()
            .map(|(s, _)| *s)
            .fold(f64::INFINITY, f64::min);
    assert!((along - 4.0).abs() < 1e-3, "along {along} m");
}

#[test]
fn right_of_way_becomes_junction_priority() {
    let map = scenarios::controlled_crossroads();
    let document = reparse_opendrive(&map);

    // OpenDRIVE says which connecting road has priority over which, so the rule —
    // written over lanes — arrives as the pairs of connectors those lanes feed.
    let junction = &document.junction[0];
    assert!(
        !junction.priority.is_empty(),
        "the right-of-way rule should reach the junction"
    );
    for priority in &junction.priority {
        let (high, low) = (
            priority.high.as_deref().unwrap(),
            priority.low.as_deref().unwrap(),
        );
        assert_ne!(high, low);
        // Both are connecting roads of this junction.
        for id in [high, low] {
            let road = document.road.iter().find(|road| road.id == id).unwrap();
            assert_eq!(road.junction, junction.id);
        }
    }
}

#[test]
fn the_same_control_reaches_both_formats() {
    let map = scenarios::controlled_crossroads();

    // OpenDRIVE: one signal, two objects, and a junction priority.
    let document = reparse_opendrive(&map);
    let signals: usize = document
        .road
        .iter()
        .filter_map(|road| road.signals.as_ref())
        .map(|signals| signals.signal.len())
        .sum();
    let objects: usize = document
        .road
        .iter()
        .filter_map(|road| road.objects.as_ref())
        .map(|objects| objects.object.len())
        .sum();
    assert_eq!(signals, 1);
    assert_eq!(objects, 2);

    // Lanelet2: the light and the stop line are ways, the crosswalk is a lanelet, and
    // the rules are regulatory elements.
    let loaded = reload_lanelet2(&map);
    let line_types: Vec<String> = loaded
        .line_strings
        .all()
        .iter()
        .filter_map(ll2_core::map::as_linestring)
        .filter_map(|line| {
            line.attributes()
                .read()
                .get("type")
                .map(|value| value.value().to_owned())
        })
        .collect();
    assert!(line_types.contains(&"traffic_light".to_owned()));
    assert!(line_types.contains(&"stop_line".to_owned()));
    assert_eq!(
        loaded
            .lanelets
            .all()
            .iter()
            .filter_map(as_lanelet)
            .filter(|lanelet| lanelet.attributes().read()["subtype"].value() == "crosswalk")
            .count(),
        1
    );
    assert_eq!(loaded.regulatory_elements.len(), 2);
}

#[test]
fn an_object_on_a_curved_road_still_lands_in_the_right_place() {
    // The projection is a nearest-point search along the reference line, so a road
    // that bends is the case worth checking: `s` has to follow the curve rather than
    // the straight line to it.
    let alignment = Alignment::new(Point3::new(0.0, 0.0, 0.0), 0.0)
        .line(40.0, 0.0)
        .unwrap()
        .arc(120.0, 1.0 / 90.0, 3.0)
        .unwrap()
        .finish()
        .unwrap();
    let mut builder = MapBuilder::new(scenarios::metadata("bent"));
    let road = builder
        .add_road(RoadSpec::new(alignment, scenarios::two_way()).with_name("bend"))
        .unwrap();
    let lane = LaneRef::new(road.clone(), 0);
    builder.add_stop_line(&lane, LaneEnd::End).unwrap();
    builder.add_traffic_light(&lane, LaneEnd::End, 5.5).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let document = reparse_opendrive(&map);
    let length = map.road(&road).unwrap().horizontal_length().unwrap();
    let signal = &document.road[0].signals.as_ref().unwrap().signal[0];

    // At the end of the road, over the middle of the right-hand lane, 5.5 m up —
    // measured along the curve, not across the chord.
    assert!(
        (signal.s.value - length).abs() < 1e-4,
        "s={}",
        signal.s.value
    );
    assert!((signal.t.value + 1.75).abs() < 1e-4, "t={}", signal.t.value);
    assert!((signal.z_offset.value - 5.5).abs() < 1e-4);
}
