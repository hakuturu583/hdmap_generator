//! A traffic light's housing and lamps, kept through the IR: written to Lanelet2 as
//! Autoware draws them, and written to and read back from OpenDRIVE.

use roadgen_core::geometry::Point3;
use roadgen_core::semantics::{
    LightArrow, LightBulb, LightColor, LightHead, MapObjectKind, ObjectGeometry,
};
use roadgen_core::validation::{UnvalidatedMap, ValidatedMap};
use roadgen_integration_tests::{reload_lanelet2, scenarios};

/// The controlled crossroads, each light given a housing 0.45 m tall and four
/// lamps along its bar 0.225 m up: red, yellow, green, and a green arrow right.
fn lit_crossroads() -> ValidatedMap {
    let mut map = scenarios::controlled_crossroads().into_map();
    for object in map.objects.iter_mut() {
        let (MapObjectKind::TrafficLight { head }, ObjectGeometry::Line(bar)) =
            (&mut object.kind, &object.geometry)
        else {
            continue;
        };
        let (from, to) = (bar.start_point(), bar.end_point());
        let lamp = |at: f64, color, arrow| LightBulb {
            position: {
                let p = from.lerp(to, at);
                Point3::new(p.x, p.y, p.z + 0.225)
            },
            color,
            arrow,
        };
        *head = LightHead {
            height: Some(0.45),
            bulbs: vec![
                lamp(0.2, LightColor::Red, None),
                lamp(0.4, LightColor::Yellow, None),
                lamp(0.6, LightColor::Green, None),
                lamp(0.8, LightColor::Green, Some(LightArrow::Right)),
            ],
        };
    }
    UnvalidatedMap::from_map(map).validate().unwrap()
}

fn heads(map: &ValidatedMap) -> Vec<LightHead> {
    let mut heads: Vec<LightHead> = map
        .objects
        .iter()
        .filter_map(|object| match &object.kind {
            MapObjectKind::TrafficLight { head } => Some(head.clone()),
            _ => None,
        })
        .collect();
    heads.sort_by(|a, b| {
        let (p, q) = (a.bulbs[0].position, b.bulbs[0].position);
        p.x.total_cmp(&q.x).then(p.y.total_cmp(&q.y))
    });
    heads
}

#[test]
fn lanelet2_draws_the_housing_and_lamps_as_autoware_does() {
    let map = lit_crossroads();
    let lights = heads(&map).len();
    assert!(lights > 0);
    let xml = roadgen_lanelet2::to_osm_xml(&map).unwrap();
    // The housing's height on each light's way, and the light_bulbs way listed in
    // the light's regulatory element.
    assert_eq!(xml.matches("k=\"height\" v=\"0.450000\"").count(), lights);
    assert!(xml.matches("role=\"light_bulbs\"").count() >= lights);

    // As Lanelet2's own loader reads it: a light_bulbs way per light, naming its
    // light, with a point per lamp tagged with what it shows.
    let loaded = reload_lanelet2(&map);
    let lines: Vec<_> = loaded
        .line_strings
        .all()
        .iter()
        .filter_map(ll2_core::map::as_linestring)
        .cloned()
        .collect();
    let tag = |attributes: &ll2_core::refs::Attrs, key: &str| {
        attributes
            .read()
            .get(key)
            .map(|value| value.value().to_owned())
    };
    let light_ids: Vec<String> = lines
        .iter()
        .filter(|line| tag(line.attributes(), "type").as_deref() == Some("traffic_light"))
        .map(|line| line.id().to_string())
        .collect();
    let bulbs: Vec<_> = lines
        .iter()
        .filter(|line| tag(line.attributes(), "type").as_deref() == Some("light_bulbs"))
        .collect();
    assert_eq!(bulbs.len(), lights);
    for way in bulbs {
        let named = tag(way.attributes(), "traffic_light_id").unwrap();
        assert!(light_ids.contains(&named), "{named} is not a traffic light");
        let shown: Vec<(Option<String>, Option<String>)> = way
            .points()
            .iter()
            .map(|point| {
                (
                    tag(point.attributes(), "color"),
                    tag(point.attributes(), "arrow"),
                )
            })
            .collect();
        let text = |value: &str| Some(value.to_owned());
        assert_eq!(
            shown,
            vec![
                (text("red"), None),
                (text("yellow"), None),
                (text("green"), None),
                (text("green"), text("right")),
            ]
        );
    }
}

#[test]
fn opendrive_carries_the_housing_height_and_the_lamps() {
    let map = lit_crossroads();
    let lights = heads(&map).len();
    let xml = roadgen_opendrive::to_xml(&map).unwrap();
    assert_eq!(
        xml.matches(&format!("code=\"{}\"", roadgen_opendrive::BULB_CODE))
            .count(),
        4 * lights
    );
    let back = roadgen_opendrive::from_xml(&xml)
        .unwrap()
        .map
        .validate()
        .unwrap();
    let (written, read) = (heads(&map), heads(&back));
    assert_eq!(written.len(), read.len());
    for (a, b) in written.iter().zip(&read) {
        assert!(a
            .height
            .zip(b.height)
            .is_some_and(|(x, y)| (x - y).abs() < 1e-9));
        assert_eq!(a.bulbs.len(), b.bulbs.len());
        for (p, q) in a.bulbs.iter().zip(&b.bulbs) {
            assert_eq!((p.color, p.arrow), (q.color, q.arrow));
            assert!(p.position.distance_to(q.position) < 1e-4);
        }
    }
}

#[test]
fn a_lights_lamp_way_traces_back_to_the_light() {
    use roadgen_core::trace::{IrRef, Relation};

    let map = lit_crossroads();
    let (written, trace) = roadgen_lanelet2::to_lanelet_map_traced(&map).unwrap();
    let lines: Vec<_> = written
        .line_strings
        .all()
        .iter()
        .filter_map(ll2_core::map::as_linestring)
        .cloned()
        .collect();
    let named = |local: &str| {
        lines
            .iter()
            .find(|line| format!("linestring:{}", line.id()) == local)
            .unwrap_or_else(|| panic!("{local} is not in the export"))
    };
    let tag = |line: &ll2_core::linestring::LineString, key: &str| {
        line.attributes()
            .read()
            .get(key)
            .map(|value| value.value().to_owned())
    };

    let mut lights = 0;
    for object in map.objects.iter().filter(|o| o.kind.is_traffic_light()) {
        lights += 1;
        let links: Vec<_> = trace.links_of(&IrRef::Object(object.id.clone())).collect();
        let housing = links.iter().find(|link| link.role.is_none()).unwrap();
        let bulbs: Vec<_> = links
            .iter()
            .filter(|link| link.role.as_deref() == Some("light_bulbs"))
            .collect();
        assert_eq!(bulbs.len(), 1, "{}", object.id);
        assert_eq!(bulbs[0].relation, Relation::Exact);
        // The traced way is the lamps of the traced housing, by Autoware's own tag.
        let way = named(&bulbs[0].local);
        assert_eq!(tag(way, "type").as_deref(), Some("light_bulbs"));
        assert_eq!(
            format!("linestring:{}", tag(way, "traffic_light_id").unwrap()),
            housing.local
        );
    }
    assert!(lights > 0);
}
