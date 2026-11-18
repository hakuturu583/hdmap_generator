//! Builds a few maps with surface features on them and dumps the CARLA meshes
//! round a point of interest as JSON, with a camera, for rendering.
//!
//! ```sh
//! cargo run -p roadgen-carla --example surface_scenes -- <out dir>
//! ```

use std::fmt::Write as _;

use roadgen_carla::environment::{generate, HeapSpec, PadSpec, PitSpec, Places, RampSpec};
use roadgen_carla::materials::MATERIALS;
use roadgen_carla::{to_meshes, Mesh, PackageConfig};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{PavedSurface, Pavement, PavementProfile, UnpavedSurface};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

struct Shot {
    name: &'static str,
    eye: Spot,
    target: Spot,
    radius: f64,
    fov: f64,
}

/// Where a camera's eye or target is.
#[derive(Clone, Copy)]
enum Spot {
    /// (station, lateral, height above the road) on the scene's road.
    Road(f64, f64, f64),
    /// The map's own coordinates.
    World(f64, f64, f64),
}

impl From<(f64, f64, f64)> for Spot {
    fn from((s, t, h): (f64, f64, f64)) -> Self {
        Spot::Road(s, t, h)
    }
}

fn lanes(width: f64) -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(width).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(width).unwrap(), Direction::Backward),
    ]
}

fn dump(map: &ValidatedMap, road: &RoadId, shots: &[Shot], out: &str) -> Result<()> {
    dump_with(
        map,
        &PackageConfig::new("scene"),
        &[(road.clone(), shots)],
        out,
    )
}

/// Every shot of every road, from one set of meshes.
fn dump_with(
    map: &ValidatedMap,
    config: &PackageConfig,
    shots: &[(RoadId, &[Shot])],
    out: &str,
) -> Result<()> {
    let meshes = to_meshes(map, config);
    let sampling = map.metadata.sampling;
    for (road, shots) in shots {
        let road = map.road(road).unwrap();
        let at = |spot: Spot| -> Result<Point3> {
            Ok(match spot {
                Spot::Road(s, t, h) => {
                    let frame = road.frame_at(s, sampling)?;
                    let p = frame.to_global([0.0, t, 0.0]);
                    Point3::new(p.x, p.y, p.z + h)
                }
                Spot::World(x, y, z) => Point3::new(x, y, z),
            })
        };
        for shot in shots.iter() {
            let eye = at(shot.eye)?;
            let origin = at(shot.target)?;
            let mut json = String::from("{\"meshes\":[");
            let mut first = true;
            for mesh in &meshes {
                if let Some(text) = mesh_json(mesh, origin, shot.radius) {
                    if !first {
                        json.push(',');
                    }
                    first = false;
                    json.push_str(&text);
                }
            }
            let [ex, ey, ez] = [eye.x - origin.x, eye.y - origin.y, eye.z - origin.z];
            write!(
                json,
                "],\"eye\":[{ex:.3},{ey:.3},{ez:.3}],\"target\":[0,0,0],\"fov\":{}}}",
                shot.fov
            )?;
            std::fs::write(format!("{out}/{}.json", shot.name), json)?;
            eprintln!("{}: written", shot.name);
        }
    }
    Ok(())
}

fn mesh_json(mesh: &Mesh, origin: Point3, radius: f64) -> Option<String> {
    let near = |p: &Point3| (p.x - origin.x).hypot(p.y - origin.y) <= radius;
    let mut by_slot: Vec<Vec<[u32; 3]>> = vec![Vec::new(); mesh.materials.len()];
    for (triangle, &slot) in mesh.triangles.iter().zip(&mesh.slots) {
        if triangle.iter().any(|&i| near(&mesh.positions[i as usize])) {
            by_slot[slot].push(*triangle);
        }
    }
    if by_slot.iter().all(Vec::is_empty) {
        return None;
    }
    let mut out = String::new();
    write!(out, "{{\"role\":\"{:?}\",\"positions\":[", mesh.role).ok()?;
    for (i, p) in mesh.positions.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write!(
            out,
            "{:.4},{:.4},{:.4}",
            p.x - origin.x,
            p.y - origin.y,
            p.z - origin.z
        )
        .ok()?;
    }
    out.push_str("],\"groups\":[");
    let mut first = true;
    for (slot, triangles) in by_slot.iter().enumerate() {
        if triangles.is_empty() {
            continue;
        }
        if !first {
            out.push(',');
        }
        first = false;
        let material = &MATERIALS[mesh.materials[slot]];
        let [r, g, b] = material.color;
        write!(
            out,
            "{{\"material\":\"{}\",\"color\":[{r:.3},{g:.3},{b:.3}],\"triangles\":[",
            material.name
        )
        .ok()?;
        for (i, t) in triangles.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            write!(out, "{},{},{}", t[0], t[1], t[2]).ok()?;
        }
        out.push_str("]}");
    }
    out.push_str("]}");
    Some(out)
}

/// An open-pit mine worn by its haul trucks: the pit with its ramp, a haul road
/// from the rim out to a tipping pad at the foot of the waste dump, and on both
/// roads the ruts the trucks leave — down the middle of the loaded lane, wandering
/// in the empty one, cutting the inside of the bends — with water standing in
/// them and potholes where the trucks turn.
fn mine(out: &str) -> Result<()> {
    let pit = PitSpec {
        name: "North Pit".into(),
        ..PitSpec::default()
    };
    let ramp_spec = RampSpec {
        width: 24.0,
        ..RampSpec::default()
    };
    // Two 10.5 m lanes and a 3 m shoulder outside the forward one, which the
    // safety berm stands on.
    let haul_lanes = || {
        let mut spec = lanes(10.5);
        spec.push(
            LaneSpec::new(PositiveWidth::new(3.0).unwrap(), Direction::Forward)
                .with_type(LaneType::Shoulder),
        );
        spec
    };
    let gravel = PavementProfile::constant(Pavement::Unpaved(UnpavedSurface::Gravel));
    let ramp_points = pit.ramp(&ramp_spec)?;
    // The haul road comes in from the tipping pad and meets the ramp where it
    // leaves the rim, heading the way the ramp sets off.
    let rim = ramp_points[0];
    let heading = {
        let next = ramp_points[1];
        let (dx, dy) = (next.x - rim.x, next.y - rim.y);
        let length = dx.hypot(dy);
        (dx / length, dy / length)
    };
    let back = |d: f64| Point3::new(rim.x - heading.0 * d, rim.y - heading.1 * d, rim.z);
    let haul_points = vec![
        Point3::new(330.0, -300.0, 0.0),
        Point3::new(250.0, -300.0, 0.0),
        Point3::new(185.0, -275.0, 0.0),
        back(70.0),
        back(0.0),
    ];
    let spec = |points: Vec<Point3>, name: &str| -> Result<RoadSpec> {
        Ok(RoadSpec::new(Curve3::polyline(points)?, haul_lanes())
            .with_name(name)
            .with_pavement(gravel.clone()))
    };
    // The roads' lengths, from a map with nothing else on it, to place the
    // features by.
    let (ramp_length, haul_length) = {
        let mut probe = MapBuilder::default();
        let haul = probe.add_road(spec(haul_points.clone(), "haul")?)?;
        let ramp = probe.add_road(spec(ramp_points.clone(), "ramp")?)?;
        probe.connect(&haul, &ramp)?;
        let map = probe.finish()?;
        let map = map.as_map();
        (
            map.road(&ramp).unwrap().horizontal_length()?,
            map.road(&haul).unwrap().horizontal_length()?,
        )
    };
    eprintln!("ramp {ramp_length:.1} m, haul road {haul_length:.1} m");

    let mut builder = MapBuilder::default();
    let haul = builder.add_road(spec(haul_points, "haul")?)?;
    let ramp = builder.add_road(spec(ramp_points, "ramp")?)?;
    builder.connect(&haul, &ramp)?;
    for road in [&haul, &ramp] {
        builder.add_barrier(&LaneRef::new(road.clone(), 2), "berm", 1.5)?;
    }

    // A large haul truck: wheels 5.5 m apart, each trough as wide as a dual tyre
    // set, and 15 cm deep where it has worn worst.
    let (track, trough) = (5.5, 1.5);
    // A path that wanders by up to `swing` metres about the lane's middle, a knot
    // every `every` metres, the same wandering for the same `seed`.
    let wander = |from: f64, to: f64, every: f64, swing: f64, seed: u64| {
        let mut knots = Vec::new();
        let mut state = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1;
        let mut station = from;
        while station < to - 1e-6 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let unit = (state >> 11) as f64 / (1u64 << 53) as f64;
            let offset = if station == from {
                0.0
            } else {
                swing * (2.0 * unit - 1.0)
            };
            knots.push((station, offset));
            station += every;
        }
        knots.push((to, 0.0));
        knots
    };

    // The ramp. Loaded trucks climb in lane 1 and keep to it, so its ruts are
    // deep and straight; empty ones come down lane 0 and wander.
    let ramp_ruts = (25.0, ramp_length - 25.0);
    builder.add_rut_along(
        &LaneRef::new(ramp.clone(), 1),
        vec![(ramp_ruts.0, 0.0), (ramp_ruts.1, 0.0)],
        0.15,
        track,
        trough,
    )?;
    builder.add_rut_along(
        &LaneRef::new(ramp.clone(), 0),
        wander(ramp_ruts.0, ramp_ruts.1, 45.0, 2.2, 7),
        0.10,
        track,
        trough,
    )?;
    // Water in the climbing lane's troughs, where the ramp levels at its foot
    // and here and there on the way up.
    let ramp_lane_centre = 5.25;
    for (k, station) in [120.0, 260.0, 410.0, 560.0, 700.0, ramp_length - 60.0]
        .into_iter()
        .enumerate()
    {
        let side = if k % 2 == 0 { 2.75 } else { -2.75 };
        builder.add_surface_feature(
            &ramp,
            station,
            8.0 + (k % 3) as f64 * 3.0,
            ramp_lane_centre + side,
            1.3,
            SurfaceFeatureKind::Puddle { depth: 0.08 },
        )?;
    }

    // The haul road: both lanes wander, and cut the inside of its bends.
    let haul_ruts = (15.0, haul_length - 5.0);
    builder.add_rut_along(
        &LaneRef::new(haul.clone(), 0),
        wander(haul_ruts.0, haul_ruts.1, 35.0, 2.5, 3),
        0.12,
        track,
        trough,
    )?;
    builder.add_rut_along(
        &LaneRef::new(haul.clone(), 1),
        wander(haul_ruts.0, haul_ruts.1, 35.0, 2.5, 11),
        0.12,
        track,
        trough,
    )?;
    for (k, station) in [40.0, 95.0, 150.0].into_iter().enumerate() {
        builder.add_surface_feature(
            &haul,
            station,
            12.0,
            if k % 2 == 0 { -3.0 } else { 4.5 },
            3.0,
            SurfaceFeatureKind::Puddle { depth: 0.07 },
        )?;
    }
    // Where the trucks turn onto the pad, the surface has broken up.
    for (k, (station, lateral)) in [
        (4.0, -3.0),
        (6.5, 2.0),
        (9.0, -6.5),
        (3.0, 7.0),
        (11.0, 4.0),
    ]
    .into_iter()
    .enumerate()
    {
        builder.add_surface_feature(
            &haul,
            station,
            1.6,
            lateral,
            1.3,
            SurfaceFeatureKind::Pothole {
                depth: 0.15,
                seed: k as u64,
            },
        )?;
    }
    builder.add_surface_feature(
        &haul,
        0.0,
        14.0,
        0.0,
        20.0,
        SurfaceFeatureKind::Roughness {
            class: RoughnessClass::F,
            seed: 5,
        },
    )?;

    let places = Places {
        heaps: vec![HeapSpec {
            name: "dump".into(),
            centre: Point3::new(470.0, -300.0, 0.0),
            radius: 100.0,
            height: 30.0,
            site: Some(pit.name.clone()),
            ..HeapSpec::default()
        }],
        pads: vec![PadSpec {
            name: "tipping".into(),
            outline: vec![
                Point3::new(330.0, -340.0, 0.0),
                Point3::new(370.0, -340.0, 0.0),
                Point3::new(370.0, -260.0, 0.0),
                Point3::new(330.0, -260.0, 0.0),
            ],
            buildings: None,
            site: Some(pit.name.clone()),
        }],
        site_roads: vec![
            (ramp.clone(), pit.name.clone()),
            (haul.clone(), pit.name.clone()),
        ],
        pits: vec![pit],
        seed: 0,
        ..Places::default()
    };
    let mut map = builder.finish()?;
    generate(&mut map, &places)?;
    let map = map.validate()?;
    eprintln!("{} surface features", map.surface_features.len());

    let mut config = PackageConfig::new("scene");
    // A coarser grid than the default, over a kilometre and more of ruts.
    config.surfaces.relief_cell = 0.3;
    let ramp_shots = [
        Shot {
            name: "mine_ramp",
            eye: (300.0, -1.0, 5.5).into(),
            target: (345.0, 2.0, 0.0).into(),
            radius: 130.0,
            fov: 55.0,
        },
        Shot {
            name: "mine_overview",
            eye: Spot::World(40.0, -560.0, 220.0),
            target: Spot::World(200.0, -150.0, -40.0),
            radius: 1100.0,
            fov: 52.0,
        },
        Shot {
            name: "mine_pit_top",
            eye: Spot::World(10.0, -330.0, 330.0),
            target: Spot::World(0.0, -40.0, -40.0),
            radius: 400.0,
            fov: 55.0,
        },
    ];
    let haul_shots = [Shot {
        name: "mine_haul",
        eye: (2.0, -1.0, 6.0).into(),
        target: (60.0, 0.0, 0.0).into(),
        radius: 160.0,
        fov: 55.0,
    }];
    dump_with(
        &map,
        &config,
        &[(ramp, &ramp_shots[..]), (haul, &haul_shots[..])],
        out,
    )
}

fn street(paved: bool, with_sidewalks: bool, length: f64) -> (MapBuilder, RoadId) {
    let mut builder = MapBuilder::default();
    let mut spec = lanes(3.5);
    if with_sidewalks {
        spec.push(
            LaneSpec::new(PositiveWidth::new(2.5).unwrap(), Direction::Forward)
                .with_type(LaneType::Sidewalk),
        );
        spec.push(
            LaneSpec::new(PositiveWidth::new(2.5).unwrap(), Direction::Backward)
                .with_type(LaneType::Sidewalk),
        );
    }
    let pavement = if paved {
        Pavement::Paved(PavedSurface::Asphalt)
    } else {
        Pavement::Unpaved(UnpavedSurface::Dirt)
    };
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(length, 0.0, 0.0),
                spec,
            )
            .unwrap()
            .with_name("street")
            .with_pavement(PavementProfile::constant(pavement)),
        )
        .unwrap();
    (builder, road)
}

/// A worn town street: potholes, a puddle and wheel ruts in the asphalt.
fn potholes(out: &str) -> Result<()> {
    let (mut builder, road) = street(true, true, 80.0);
    for (station, lateral, size, depth, seed) in [
        (30.0, -1.2, 0.9, 0.08, 1),
        (31.4, -2.4, 0.6, 0.06, 2),
        (35.0, 1.6, 1.3, 0.1, 3),
        (41.0, -0.6, 0.5, 0.05, 4),
        (44.0, 2.4, 0.8, 0.07, 5),
    ] {
        builder.add_surface_feature(
            &road,
            station,
            size,
            lateral,
            size * 0.8,
            SurfaceFeatureKind::Pothole { depth, seed },
        )?;
    }
    builder.add_surface_feature(
        &road,
        37.0,
        3.0,
        -2.0,
        1.8,
        SurfaceFeatureKind::Puddle { depth: 0.03 },
    )?;
    builder.add_surface_feature(
        &road,
        48.0,
        25.0,
        -1.75,
        2.4,
        SurfaceFeatureKind::Rut {
            depth: 0.035,
            track: 1.8,
            trough: 0.6,
            path: None,
        },
    )?;
    let map = builder.finish()?.validate()?;
    dump(
        &map,
        &road,
        &[Shot {
            name: "street_potholes",
            eye: (24.0, -3.0, 2.2).into(),
            target: (38.0, 0.0, 0.0).into(),
            radius: 45.0,
            fov: 55.0,
        }],
        out,
    )
}

/// Traffic calming down a residential street: a hump, a table and cushions.
fn calming(out: &str) -> Result<()> {
    let (mut builder, road) = street(true, true, 100.0);
    builder.add_surface_feature(
        &road,
        25.0,
        3.7,
        0.0,
        7.0,
        SurfaceFeatureKind::Bump {
            kind: BumpKind::SpeedHump,
            height: 0.1,
            profile: BumpProfile::Circular,
        },
    )?;
    builder.add_surface_feature(
        &road,
        45.0,
        7.0,
        0.0,
        7.0,
        SurfaceFeatureKind::Bump {
            kind: BumpKind::SpeedTable,
            height: 0.1,
            profile: BumpProfile::Flat { ramp: 1.5 },
        },
    )?;
    for lateral in [-1.75, 1.75] {
        builder.add_surface_feature(
            &road,
            68.0,
            3.0,
            lateral,
            1.8,
            SurfaceFeatureKind::Bump {
                kind: BumpKind::SpeedCushion,
                height: 0.07,
                profile: BumpProfile::Sinusoidal,
            },
        )?;
    }
    builder.add_surface_feature(
        &road,
        85.0,
        0.5,
        0.0,
        7.0,
        SurfaceFeatureKind::Bump {
            kind: BumpKind::SpeedBump,
            height: 0.08,
            profile: BumpProfile::Parabolic,
        },
    )?;
    let map = builder.finish()?.validate()?;
    dump(
        &map,
        &road,
        &[
            Shot {
                name: "street_calming",
                eye: (18.0, -12.0, 7.0).into(),
                target: (55.0, 0.0, 0.0).into(),
                radius: 70.0,
                fov: 50.0,
            },
            Shot {
                name: "street_calming_close",
                eye: (38.5, -5.2, 1.3).into(),
                target: (49.0, 0.5, 0.0).into(),
                radius: 50.0,
                fov: 50.0,
            },
        ],
        out,
    )
}

/// A country road gone to pieces: ISO 8608 class F roughness and frost heaves.
fn rural(out: &str) -> Result<()> {
    let (mut builder, road) = street(true, false, 140.0);
    builder.add_surface_feature(
        &road,
        20.0,
        100.0,
        0.0,
        7.0,
        SurfaceFeatureKind::Roughness {
            class: RoughnessClass::F,
            seed: 11,
        },
    )?;
    for (station, lateral, height) in [(45.0, 0.0, 0.07), (70.0, -1.0, 0.09)] {
        builder.add_surface_feature(
            &road,
            station,
            6.0,
            lateral,
            5.0,
            SurfaceFeatureKind::Bump {
                kind: BumpKind::Heave,
                height,
                profile: BumpProfile::Sinusoidal,
            },
        )?;
    }
    let map = builder.finish()?.validate()?;
    dump(
        &map,
        &road,
        &[Shot {
            name: "rural_rough",
            eye: (25.0, -1.75, 1.6).into(),
            target: (60.0, -1.0, 0.0).into(),
            radius: 70.0,
            fov: 55.0,
        }],
        out,
    )
}

fn main() -> Result<()> {
    let out = std::env::args().nth(1).unwrap_or_else(|| ".".into());
    std::fs::create_dir_all(&out)?;
    // A second argument picks one scene: `mine`, `potholes`, `calming`, `rural`.
    let only = std::env::args().nth(2);
    let wanted = |name: &str| only.as_deref().is_none_or(|only| only == name);
    if wanted("mine") {
        mine(&out)?;
    }
    if wanted("potholes") {
        potholes(&out)?;
    }
    if wanted("calming") {
        calming(&out)?;
    }
    if wanted("rural") {
        rural(&out)?;
    }
    Ok(())
}
