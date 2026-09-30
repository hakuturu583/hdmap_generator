//! A traffic light's housing and lamps, as Autoware draws them, kept through the IR:
//! read from Lanelet2, written back to Lanelet2, and written to and read from
//! OpenDRIVE.

use roadgen_core::semantics::{LightArrow, LightColor, LightHead, MapObjectKind};
use roadgen_core::validation::ValidatedMap;
use roadgen_integration_tests::osm::Osm;
use roadgen_lanelet2::ReadOptions;

/// One lane 40 m long with a traffic light over its end: a housing 0.45 m tall
/// whose bottom edge is 5 m up, and four lamps — red, yellow, green, and a green
/// arrow to the right.
fn lit_lane() -> String {
    let mut osm = Osm::new();
    let left = osm.line(None, &[(0.0, 3.5), (20.0, 3.5), (40.0, 3.5)]);
    let right = osm.line(None, &[(0.0, 0.0), (20.0, 0.0), (40.0, 0.0)]);
    let lanelet = osm.lanelet(left, right);

    let housing = vec![osm.node(37.0, 0.5, 5.0), osm.node(37.0, 3.0, 5.0)];
    let light = osm.tagged_way(
        housing,
        vec![
            ("type", "traffic_light".into()),
            ("subtype", "red_yellow_green".into()),
            ("height", "0.450000".into()),
        ],
    );
    let lamps = [
        (1.0, "red", None),
        (1.6, "yellow", None),
        (2.2, "green", None),
        (2.8, "green", Some("right")),
    ]
    .into_iter()
    .map(|(y, color, arrow)| {
        let mut tags = vec![("color", color.to_owned())];
        if let Some(arrow) = arrow {
            tags.push(("arrow", arrow.to_owned()));
        }
        osm.tagged_node(37.0, y, 5.225, tags)
    })
    .collect();
    let bulbs = osm.tagged_way(
        lamps,
        vec![
            ("type", "light_bulbs".into()),
            ("traffic_light_id", light.to_string()),
        ],
    );
    let stop = vec![osm.node(35.0, 0.0, 0.0), osm.node(35.0, 3.5, 0.0)];
    let stop_line = osm.tagged_way(stop, vec![("type", "stop_line".into())]);
    let element = osm.regulatory_element(
        "traffic_light",
        vec![
            ("way", light, "refers"),
            ("way", stop_line, "ref_line"),
            ("way", bulbs, "light_bulbs"),
        ],
    );
    osm.refer(lanelet, element);
    osm.xml()
}

fn read(xml: &str) -> ValidatedMap {
    roadgen_lanelet2::from_osm_str(xml, &ReadOptions::default())
        .unwrap()
        .map
        .validate()
        .unwrap()
}

/// Reads a map written from `map` again, about the same origin.
fn read_back(xml: &str, map: &ValidatedMap) -> ValidatedMap {
    roadgen_lanelet2::from_osm_str(
        xml,
        &ReadOptions {
            origin: Some(map.metadata.origin),
            ..ReadOptions::default()
        },
    )
    .unwrap()
    .map
    .validate()
    .unwrap()
}

fn head(map: &ValidatedMap) -> LightHead {
    let lights: Vec<&LightHead> = map
        .objects
        .iter()
        .filter_map(|object| match &object.kind {
            MapObjectKind::TrafficLight { head } => Some(head),
            _ => None,
        })
        .collect();
    assert_eq!(lights.len(), 1, "one traffic light");
    lights[0].clone()
}

/// Whether two heads are the same, the lamps within `tolerance` metres.
fn same(a: &LightHead, b: &LightHead, tolerance: f64) -> bool {
    a.height
        .zip(b.height)
        .is_some_and(|(x, y)| (x - y).abs() < 1e-9)
        && a.bulbs.len() == b.bulbs.len()
        && a.bulbs.iter().zip(&b.bulbs).all(|(p, q)| {
            p.color == q.color
                && p.arrow == q.arrow
                && p.position.distance_to(q.position) < tolerance
        })
}

#[test]
fn a_traffic_light_keeps_its_housing_and_lamps() {
    let map = read(&lit_lane());
    let head = head(&map);
    assert_eq!(head.height, Some(0.45));
    let shown: Vec<(LightColor, Option<LightArrow>)> = head
        .bulbs
        .iter()
        .map(|bulb| (bulb.color, bulb.arrow))
        .collect();
    assert_eq!(
        shown,
        vec![
            (LightColor::Red, None),
            (LightColor::Yellow, None),
            (LightColor::Green, None),
            (LightColor::Green, Some(LightArrow::Right)),
        ]
    );
    // The lamps are where the file puts them: 0.225 m above the housing's bottom
    // edge, and 0.6 m apart — as near as the file's rounded degrees say.
    let bottom = map
        .objects
        .iter()
        .find(|object| object.kind.is_traffic_light())
        .map(|object| match &object.geometry {
            roadgen_core::semantics::ObjectGeometry::Line(line) => line.start_point().z,
            _ => panic!("a traffic light is a line"),
        })
        .unwrap();
    for bulb in &head.bulbs {
        assert!((bulb.position.z - bottom - 0.225).abs() < 1e-6);
    }
    assert!((head.bulbs[0].position.distance_to(head.bulbs[1].position) - 0.6).abs() < 0.01);
}

#[test]
fn the_lamps_come_back_from_lanelet2() {
    let map = read(&lit_lane());
    let back = read_back(&roadgen_lanelet2::to_osm_xml(&map).unwrap(), &map);
    assert!(same(&head(&map), &head(&back), 1e-4));
}

#[test]
fn opendrive_carries_the_housing_height_and_the_lamps() {
    let map = read(&lit_lane());
    let xml = roadgen_opendrive::to_xml(&map).unwrap();
    assert!(
        xml.contains("height=\"4.5"),
        "the signal has its housing's height"
    );
    assert_eq!(
        xml.matches(&format!("code=\"{}\"", roadgen_opendrive::BULB_CODE))
            .count(),
        4
    );
    let back = roadgen_opendrive::from_xml(&xml)
        .unwrap()
        .map
        .validate()
        .unwrap();
    assert!(same(&head(&map), &head(&back), 1e-4));
}

#[test]
fn lamps_naming_no_light_go_with_the_light_listed_in_the_same_place() {
    // Two lights over one lane whose lamps name ids that are not theirs, as two
    // of Nishi-Shinjuku's do: 413 for 3002413.
    let mut osm = Osm::new();
    let left = osm.line(None, &[(0.0, 3.5), (40.0, 3.5)]);
    let right = osm.line(None, &[(0.0, 0.0), (40.0, 0.0)]);
    let lanelet = osm.lanelet(left, right);
    let mut members = Vec::new();
    let mut lamps_of = Vec::new();
    for (x, color) in [(37.0, "red"), (38.0, "green")] {
        let housing = vec![osm.node(x, 0.5, 5.0), osm.node(x, 3.0, 5.0)];
        let light = osm.tagged_way(housing, vec![("type", "traffic_light".into())]);
        let lamp = osm.tagged_node(x, 1.0, 5.2, vec![("color", color.to_owned())]);
        let bulbs = osm.tagged_way(
            vec![lamp],
            vec![
                ("type", "light_bulbs".into()),
                ("traffic_light_id", "7".into()),
            ],
        );
        members.push(("way", light, "refers"));
        lamps_of.push(("way", bulbs, "light_bulbs"));
    }
    members.extend(lamps_of);
    let element = osm.regulatory_element("traffic_light", members);
    osm.refer(lanelet, element);

    let imported = roadgen_lanelet2::from_osm_str(&osm.xml(), &ReadOptions::default()).unwrap();
    assert!(imported
        .approximations
        .iter()
        .any(|note| note.contains("light_bulbs")));
    let map = imported.map.validate().unwrap();
    let mut shown: Vec<(f64, Vec<LightColor>)> = map
        .objects
        .iter()
        .filter_map(|object| match &object.kind {
            MapObjectKind::TrafficLight { head } => Some((
                head.bulbs[0].position.x,
                head.bulbs.iter().map(|bulb| bulb.color).collect(),
            )),
            _ => None,
        })
        .collect();
    shown.sort_by(|a, b| a.0.total_cmp(&b.0));
    let colors: Vec<Vec<LightColor>> = shown.into_iter().map(|(_, colors)| colors).collect();
    assert_eq!(colors, vec![vec![LightColor::Red], vec![LightColor::Green]]);
}
