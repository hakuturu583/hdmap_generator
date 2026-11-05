use roadgen_carla::environment::{generate, PitSpec, Places, RampSpec};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{Pavement, PavementProfile, UnpavedSurface};

/// A pit, and the ramp into it as a road of the caller's.
fn pit(spec: &PitSpec, ramp: &RampSpec, seed: u64) -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let points = spec.ramp(ramp).expect("the ramp reaches the floor");
    let lanes = vec![
        LaneSpec::new(PositiveWidth::new(10.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(10.5).unwrap(), Direction::Backward),
    ];
    let road = builder
        .add_road(
            RoadSpec::new(Curve3::polyline(points).unwrap(), lanes)
                .with_name("ramp")
                .with_pavement(PavementProfile::constant(Pavement::Unpaved(
                    UnpavedSurface::Gravel,
                ))),
        )
        .unwrap();
    let places = Places {
        pits: vec![spec.clone()],
        site_roads: vec![(road, spec.name.clone())],
        seed,
        ..Places::default()
    };
    let mut map = builder.finish().expect("the roads build");
    generate(&mut map, &places).expect("the land generates");
    match map.validate() {
        Ok(map) => map,
        Err(error) => panic!("{error}"),
    }
}

#[test]
fn a_pit_and_its_ramp_validate_as_a_site() {
    let map = pit(&PitSpec::default(), &RampSpec::default(), 0);
    assert_eq!(map.sites.len(), 1);
    assert_eq!(map.tin_surfaces.len(), 1);
    let site = map.sites.iter().next().unwrap();
    assert_eq!(site.kind, "open_pit_mine");
    assert_eq!(site.roads, [RoadId::new("ramp")]);
    let kinds: Vec<&str> = site
        .land_covers
        .iter()
        .map(|id| map.land_covers.get(id).unwrap().kind.as_str())
        .collect();
    assert_eq!(kinds, ["pit"]);
}

#[test]
fn the_land_meets_the_ramp_and_steps_down_to_the_floor() {
    let spec = PitSpec::default();
    let map = pit(&spec, &RampSpec::default(), 0);
    let surface = map.tin_surfaces.iter().next().unwrap();
    let locator = surface.tin.locator();

    // The ramp's lanes lie on the land: its edges are breaklines of it, and the
    // land between them is the road's own plane.
    let sampling = map.metadata.sampling;
    for lane in map.lanes_of_section(&RoadId::new("ramp"), 0) {
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

    // The floor is as deep as the benches add up to.
    let floor = locator.height_at(spec.centre.x, spec.centre.y).unwrap();
    assert!((floor - spec.floor_height()).abs() < 1e-6, "{floor}");

    // Every bench has a crest and a toe, at the bench's own height.
    for bench in 0..spec.benches {
        for (kind, rise) in [("toe", 0.0), ("crest", 1.0)] {
            let expected = spec.floor_height() + (bench as f64 + rise) * spec.bench_height;
            let lines: Vec<_> = surface
                .breaklines
                .iter()
                .filter(|id| id.as_str().contains(&format!("/bench/{bench}/{kind}")))
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
fn one_spec_is_one_pit() {
    let (spec, ramp) = (PitSpec::default(), RampSpec::default());
    assert_eq!(pit(&spec, &ramp, 7).as_map(), pit(&spec, &ramp, 7).as_map());
    assert_ne!(
        pit(&spec, &ramp, 7).tin_surfaces.iter().next().unwrap().tin,
        pit(&spec, &ramp, 8).tin_surfaces.iter().next().unwrap().tin
    );
}

#[test]
fn a_turned_pit_with_an_anticlockwise_ramp_validates_too() {
    let spec = PitSpec {
        name: "East Pit".into(),
        centre: Point3::new(500.0, -200.0, 120.0),
        heading: 0.6,
        ..PitSpec::default()
    };
    let ramp = RampSpec {
        clockwise: false,
        start: 45.0,
        approach: 0.0,
        ..RampSpec::default()
    };
    let map = pit(&spec, &ramp, 0);
    assert!(map.sites.contains(&SiteId::new("east_pit")));
}

#[test]
fn a_pit_no_pit_could_be_is_refused() {
    let spec = PitSpec {
        benches: 0,
        ..PitSpec::default()
    };
    assert!(spec.check().is_err());
    let mut map = MapBuilder::default().finish().unwrap();
    let places = Places {
        pits: vec![spec],
        ..Places::default()
    };
    assert!(generate(&mut map, &places).is_err());
}

#[test]
fn a_map_states_its_land_once() {
    let mut map = MapBuilder::default().finish().unwrap();
    let places = Places {
        pits: vec![PitSpec::default()],
        ..Places::default()
    };
    generate(&mut map, &places).unwrap();
    let error = generate(&mut map, &places).unwrap_err();
    assert!(error.to_string().contains("states it once"), "{error}");
}
