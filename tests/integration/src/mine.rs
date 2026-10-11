//! An open-pit mine, put together from the parts a caller has: a pit, the ramp
//! down it and the haul roads out of it as ordinary roads, the heaps they lead to,
//! a pad for the workshops, and a berm along the ramp.
//!
//! It is what a caller writes with `roadgen_carla::environment`, written once so
//! that every test of what a mine comes to starts from the same one.

use roadgen_carla::environment::{HeapSpec, PadSpec, PitSpec, Places, RampSpec};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{
    BoundaryMarking, LaneType, MarkingColor, Pavement, PavementProfile, RoadMarking, RoadType,
    UnpavedSurface,
};
use roadgen_core::units::SpeedLimit;

/// The haul road's cross-section: a lane and a shoulder each way, no paint.
pub fn haul_lanes() -> Vec<LaneSpec> {
    let bare = BoundaryMarking::new(RoadMarking::None, MarkingColor::White);
    let lane = |width: f64, direction, lane_type| {
        LaneSpec::new(PositiveWidth::new(width).unwrap(), direction)
            .with_type(lane_type)
            .with_markings(bare, bare)
            .with_speed_limit(SpeedLimit::from_kph(40.0).unwrap())
    };
    vec![
        lane(7.5, Direction::Forward, LaneType::Driving),
        lane(3.0, Direction::Forward, LaneType::Shoulder),
        lane(7.5, Direction::Backward, LaneType::Driving),
        lane(3.0, Direction::Backward, LaneType::Shoulder),
    ]
}

fn haul_road(line: Curve3, name: &str) -> RoadSpec {
    RoadSpec::new(line, haul_lanes())
        .with_name(name)
        .with_type(RoadType::LowSpeed)
        .with_pavement(PavementProfile::constant(Pavement::Unpaved(
            UnpavedSurface::Gravel,
        )))
}

/// Puts a mine named `name` on `builder`, its pit centred at `centre`, and says
/// what to generate onto the map once it is built.
pub fn open_pit_mine(builder: &mut MapBuilder, name: &str, centre: Point3) -> Places {
    let pit = PitSpec {
        name: name.into(),
        centre,
        ..PitSpec::default()
    };
    let ramp = RampSpec::default();
    let points = pit.ramp(&ramp).expect("the ramp reaches the floor");
    let key = roadgen_carla::environment::slug(name);
    let ramp_id = builder
        .add_road(haul_road(
            Curve3::polyline(points.iter().copied()).unwrap(),
            &format!("{key}_ramp"),
        ))
        .unwrap();
    // The berm on the pit side: right of a ramp winding down clockwise, on the
    // forward shoulder.
    builder
        .add_barrier(&LaneRef::new(ramp_id.clone(), 1), "berm", 1.5)
        .unwrap();

    // The junction, back along the ramp's first direction from where it starts.
    let (p0, p1) = (points[0], points[1]);
    let length = p0.horizontal_distance_to(p1);
    let back = [(p0.x - p1.x) / length, (p0.y - p1.y) / length];
    let hub = [p0.x + back[0] * 30.0, p0.y + back[1] * 30.0];
    let toward = |bearing: f64, distance: f64| {
        let (sin, cos) = bearing.to_radians().sin_cos();
        Point3::new(hub[0] + cos * distance, hub[1] + sin * distance, centre.z)
    };

    let mut heaps = Vec::new();
    let mut roads = vec![ramp_id];
    let mut pads = Vec::new();
    for (kind, bearing, distance, radius, height) in [
        ("waste_dump", -60.0, 260.0, 110.0, 30.0),
        ("stockpile", 20.0, 210.0, 38.0, 14.0),
    ] {
        heaps.push(HeapSpec {
            name: format!("{key}_{kind}"),
            kind: kind.into(),
            centre: toward(bearing, distance),
            radius,
            height,
            site: Some(name.into()),
            ..HeapSpec::default()
        });
        // The road stops short of the toe, where a truck turns to tip.
        let (start, end) = (
            toward(bearing, 30.0),
            toward(bearing, distance - radius - 15.0),
        );
        roads.push(
            builder
                .add_road(haul_road(
                    Curve3::line(start, end).unwrap(),
                    &format!("{key}_haul_{kind}"),
                ))
                .unwrap(),
        );
        if kind == "waste_dump" {
            // The yard: halfway along, on the side away from the pit.
            let along = [end.x - start.x, end.y - start.y];
            let span = along[0].hypot(along[1]);
            let along = [along[0] / span, along[1] / span];
            let middle = [(start.x + end.x) / 2.0, (start.y + end.y) / 2.0];
            let mut across = [-along[1], along[0]];
            if (middle[0] - centre.x) * across[0] + (middle[1] - centre.y) * across[1] < 0.0 {
                across = [-across[0], -across[1]];
            }
            let corner = |u: f64, v: f64| {
                Point3::new(
                    middle[0] + along[0] * u + across[0] * v,
                    middle[1] + along[1] * u + across[1] * v,
                    centre.z,
                )
            };
            let mut outline = vec![
                corner(-40.0, 12.5),
                corner(40.0, 12.5),
                corner(40.0, 57.5),
                corner(-40.0, 57.5),
            ];
            // Anticlockwise, whichever side it fell.
            if across[0] * along[1] - across[1] * along[0] > 0.0 {
                outline.reverse();
            }
            pads.push(PadSpec {
                name: format!("{key}_pad"),
                outline,
                buildings: roadgen_buildings::Rules::preset("industrial"),
                site: Some(name.into()),
            });
        }
    }
    let junction = builder.add_junction(Some(&format!("{key}_hub")));
    for (index, from) in roads.iter().enumerate() {
        for to in &roads[index + 1..] {
            builder
                .connect_ends(from, RoadEnd::Start, to, RoadEnd::Start, Some(&junction))
                .unwrap();
        }
    }
    Places {
        pits: vec![pit],
        heaps,
        pads,
        site_roads: roads
            .into_iter()
            .map(|road| (road, name.to_owned()))
            .collect(),
        ..Places::default()
    }
}

/// The default mine, generated and validated.
pub fn mine() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let places = open_pit_mine(&mut builder, "mine", Point3::ORIGIN);
    let mut map = builder.finish().expect("builds");
    roadgen_carla::environment::generate(&mut map, &places).expect("generates");
    map.validate().expect("validates")
}
