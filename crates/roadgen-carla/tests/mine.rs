use roadgen_carla::environment::mine::{generate, lay_out, MineSpec};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{MapObjectKind, Pavement, UnpavedSurface};

fn mine(spec: &MineSpec) -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let plan = lay_out(&mut builder, spec).expect("the roads lay out");
    let mut map = builder.finish().expect("the roads build");
    generate(&mut map, &plan).expect("the land generates");
    match map.validate() {
        Ok(map) => map,
        Err(error) => panic!("{error}"),
    }
}

#[test]
fn the_default_mine_validates() {
    let map = mine(&MineSpec::default());
    assert_eq!(map.sites.len(), 1);
    assert_eq!(map.tin_surfaces.len(), 1);
    let site = map.sites.iter().next().unwrap();
    assert_eq!(site.kind, "open_pit_mine");
    let kinds: Vec<&str> = site
        .land_covers
        .iter()
        .map(|id| map.land_covers.get(id).unwrap().kind.as_str())
        .collect();
    assert_eq!(kinds, ["pit", "waste_dump", "stockpile", "pad"]);
    // The ramp, two haul roads and the movements through the junction between them.
    assert!(site.roads.len() > 3);
    for road in &site.roads {
        assert_eq!(
            map.road(road).unwrap().pavement.at(0.0),
            Some(Pavement::Unpaved(UnpavedSurface::Gravel)),
            "{road}"
        );
    }
    let berms = map
        .objects
        .iter()
        .filter(|object| matches!(object.kind, MapObjectKind::Barrier { .. }))
        .count();
    assert_eq!(berms, 1);
}

#[test]
fn the_land_meets_the_roads_and_steps_down_to_the_floor() {
    let spec = MineSpec::default();
    let map = mine(&spec);
    let surface = map.tin_surfaces.iter().next().unwrap();
    let locator = surface.tin.locator();

    // Every road's lanes lie on the land: the road edges are breaklines of it, and
    // the land between them is the road's own plane.
    let sampling = map.metadata.sampling;
    for road in map.roads.iter().filter(|road| !road.is_connector()) {
        for lane in map.lanes_of_section(&road.id, 0) {
            for point in lane.centerline.to_polyline(sampling).unwrap().points() {
                let ground = locator.height_at(point.x, point.y).expect("on the land");
                assert!(
                    (ground - point.z).abs() < 0.3,
                    "{}: the road is at {:.2} and the land at {:.2}",
                    lane.id,
                    point.z,
                    ground
                );
            }
        }
    }

    // The floor is as deep as the benches add up to.
    let depth = spec.pit.benches as f64 * spec.pit.bench_height;
    let floor = locator.height_at(spec.centre.x, spec.centre.y).unwrap();
    assert!((floor - (spec.centre.z - depth)).abs() < 1e-6, "{floor}");

    // Every bench has a crest and a toe, at the bench's own height.
    for bench in 0..spec.pit.benches {
        for (kind, rise) in [("toe", 0.0), ("crest", 1.0)] {
            let expected = spec.centre.z - depth + (bench as f64 + rise) * spec.pit.bench_height;
            let lines: Vec<_> = surface
                .breaklines
                .iter()
                .filter(|id| {
                    let id = id.as_str();
                    id.contains(&format!("/bench/{bench}/{kind}"))
                })
                .collect();
            assert!(!lines.is_empty(), "bench {bench} has no {kind}");
            for id in lines {
                let line = &map.tin_breaklines.get(id).unwrap().line;
                assert!(line.points().iter().all(|p| (p.z - expected).abs() < 1e-9));
            }
        }
    }
}

#[test]
fn one_spec_is_one_mine() {
    let spec = MineSpec {
        seed: 7,
        ..MineSpec::default()
    };
    assert_eq!(mine(&spec).as_map(), mine(&spec).as_map());
    let other = MineSpec {
        seed: 8,
        ..MineSpec::default()
    };
    assert_ne!(
        mine(&spec).tin_surfaces.iter().next().unwrap().tin,
        mine(&other).tin_surfaces.iter().next().unwrap().tin
    );
}

#[test]
fn a_turned_anticlockwise_pit_validates_too() {
    let spec = MineSpec {
        name: "East Pit".into(),
        centre: Point3::new(500.0, -200.0, 120.0),
        heading: 0.6,
        ramp: roadgen_carla::environment::mine::RampSpec {
            clockwise: false,
            start: 45.0,
            grade: 0.1,
        },
        ..MineSpec::default()
    };
    let map = mine(&spec);
    assert!(map.sites.contains(&SiteId::new("east_pit")));
}

#[test]
fn a_spec_no_pit_could_be_is_refused() {
    let mut builder = MapBuilder::default();
    let spec = MineSpec {
        pit: roadgen_carla::environment::mine::PitSpec {
            benches: 0,
            ..Default::default()
        },
        ..MineSpec::default()
    };
    assert!(lay_out(&mut builder, &spec).is_err());
}

#[test]
fn a_ramp_too_gentle_to_reach_the_floor_is_refused() {
    let mut builder = MapBuilder::default();
    let spec = MineSpec {
        ramp: roadgen_carla::environment::mine::RampSpec {
            grade: 0.01,
            ..Default::default()
        },
        ..MineSpec::default()
    };
    let error = lay_out(&mut builder, &spec).unwrap_err();
    assert!(error.to_string().contains("cannot reach"), "{error}");
}

#[test]
fn a_map_states_its_land_once() {
    let mut builder = MapBuilder::default();
    let first = lay_out(&mut builder, &MineSpec::default()).unwrap();
    let second = lay_out(
        &mut builder,
        &MineSpec {
            name: "South Pit".into(),
            centre: Point3::new(0.0, -2000.0, 0.0),
            ..MineSpec::default()
        },
    )
    .unwrap();
    let mut map = builder.finish().unwrap();
    generate(&mut map, &first).unwrap();
    let error = generate(&mut map, &second).unwrap_err();
    assert!(error.to_string().contains("states it once"), "{error}");
}
