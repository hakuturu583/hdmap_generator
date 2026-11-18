//! Builds a few maps with surface features on them and dumps the CARLA meshes
//! round a point of interest as JSON, with a camera, for rendering.
//!
//! ```sh
//! cargo run -p roadgen-carla --example surface_scenes -- <out dir>
//! ```

use std::fmt::Write as _;

use roadgen_carla::environment::{generate, HeapSpec, PitSpec, Places, RampSpec};
use roadgen_carla::materials::MATERIALS;
use roadgen_carla::{to_meshes, Mesh, PackageConfig};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{PavedSurface, Pavement, PavementProfile, UnpavedSurface};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

struct Shot {
    name: &'static str,
    /// (station, lateral, height) of the eye and of the target, on `road`.
    eye: (f64, f64, f64),
    target: (f64, f64, f64),
    radius: f64,
    fov: f64,
}

fn lanes(width: f64) -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(width).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(width).unwrap(), Direction::Backward),
    ]
}

fn dump(map: &ValidatedMap, road: &RoadId, shots: &[Shot], out: &str) -> Result<()> {
    let meshes = to_meshes(map, &PackageConfig::new("scene"));
    let road = map.road(road).unwrap();
    let sampling = map.metadata.sampling;
    let at = |(s, t, h): (f64, f64, f64)| -> Result<Point3> {
        let frame = road.frame_at(s, sampling)?;
        let p = frame.to_global([0.0, t, 0.0]);
        Ok(Point3::new(p.x, p.y, p.z + h))
    };
    for shot in shots {
        let eye = at(shot.eye)?;
        let target = at(shot.target)?;
        let origin = target;
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
        let rel = |p: Point3| [p.x - origin.x, p.y - origin.y, p.z - origin.z];
        let [ex, ey, ez] = rel(eye);
        write!(
            json,
            "],\"eye\":[{ex:.3},{ey:.3},{ez:.3}],\"target\":[0,0,0],\"fov\":{}}}",
            shot.fov
        )?;
        std::fs::write(format!("{out}/{}.json", shot.name), json)?;
        eprintln!("{}: written", shot.name);
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

/// A haul ramp into an open pit, worn into ruts by the trucks, with water in them.
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
    let mut builder = MapBuilder::default();
    let ramp = builder.add_road(
        RoadSpec::new(Curve3::polyline(pit.ramp(&ramp_spec)?)?, haul_lanes())
            .with_name("ramp")
            .with_pavement(PavementProfile::constant(Pavement::Unpaved(
                UnpavedSurface::Gravel,
            ))),
    )?;
    builder.add_barrier(&LaneRef::new(ramp.clone(), 2), "berm", 1.5)?;
    // Plan the features on a copy first, to know the road's length.
    let length = {
        let mut probe = MapBuilder::default();
        let id = probe.add_road(RoadSpec::new(
            Curve3::polyline(pit.ramp(&ramp_spec)?)?,
            haul_lanes(),
        ))?;
        let map = probe.finish()?;
        map.as_map().road(&id).unwrap().horizontal_length()?
    };
    eprintln!("ramp length {length:.1}");
    let (from, to) = (150.0, 260.0);
    // A large haul truck's wheels are about 5.5 m apart; each trough as wide as a
    // dual tyre set, and deep: 15 cm.
    // Downhill, the trucks keep to their lane: straight ruts down its middle.
    builder.add_surface_feature(
        &ramp,
        from,
        to - from,
        -5.25,
        7.0,
        SurfaceFeatureKind::Rut {
            depth: 0.15,
            track: 5.5,
            trough: 1.5,
            path: None,
        },
    )?;
    // Uphill, they wander: one line drifts to the berm side and back, cuts in
    // towards the downhill lane and settles, in the lane's own coordinates.
    builder.add_rut_along(
        &LaneRef::new(ramp.clone(), 1),
        vec![
            (from, 0.0),
            (180.0, 1.8),
            (205.0, -2.0),
            (225.0, -3.6),
            (245.0, -0.5),
            (to, 0.5),
        ],
        0.15,
        5.5,
        1.5,
    )?;
    for (lateral, seed) in [(-5.25, 1), (5.25, 2)] {
        // Water standing in the troughs, here and there.
        for (k, station) in [175.0, 205.0, 232.0].into_iter().enumerate() {
            let side = if (k + seed) % 2 == 0 { 2.75 } else { -2.75 };
            builder.add_surface_feature(
                &ramp,
                station + 3.0 * seed as f64,
                7.0 + 2.0 * k as f64,
                lateral + side,
                1.3,
                SurfaceFeatureKind::Puddle { depth: 0.06 },
            )?;
        }
    }
    builder.add_surface_feature(
        &ramp,
        from - 10.0,
        to - from + 20.0,
        0.0,
        20.5,
        SurfaceFeatureKind::Roughness {
            class: RoughnessClass::E,
            seed: 5,
        },
    )?;
    builder.add_surface_feature(
        &ramp,
        240.0,
        1.6,
        1.0,
        1.2,
        SurfaceFeatureKind::Pothole {
            depth: 0.18,
            seed: 7,
        },
    )?;
    let places = Places {
        heaps: vec![HeapSpec {
            name: "dump".into(),
            centre: Point3::new(300.0, -250.0, 0.0),
            radius: 90.0,
            height: 25.0,
            site: Some(pit.name.clone()),
            ..HeapSpec::default()
        }],
        site_roads: vec![(ramp.clone(), pit.name.clone())],
        pits: vec![pit],
        seed: 0,
        ..Places::default()
    };
    let mut map = builder.finish()?;
    generate(&mut map, &places)?;
    let map = map.validate()?;
    dump(
        &map,
        &ramp,
        &[
            Shot {
                name: "mine_ruts_close",
                eye: (166.0, 1.5, 4.0),
                target: (192.0, -2.0, 0.0),
                radius: 70.0,
                fov: 55.0,
            },
            Shot {
                name: "mine_ruts_top",
                eye: (196.0, 0.0, 95.0),
                target: (206.0, 0.0, 0.0),
                radius: 90.0,
                fov: 42.0,
            },
            Shot {
                name: "mine_ruts_wide",
                eye: (120.0, 0.0, 40.0),
                target: (210.0, 0.0, 0.0),
                radius: 420.0,
                fov: 60.0,
            },
        ],
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
            eye: (24.0, -3.0, 2.2),
            target: (38.0, 0.0, 0.0),
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
                eye: (18.0, -12.0, 7.0),
                target: (55.0, 0.0, 0.0),
                radius: 70.0,
                fov: 50.0,
            },
            Shot {
                name: "street_calming_close",
                eye: (38.5, -5.2, 1.3),
                target: (49.0, 0.5, 0.0),
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
            eye: (25.0, -1.75, 1.6),
            target: (60.0, -1.0, 0.0),
            radius: 70.0,
            fov: 55.0,
        }],
        out,
    )
}

fn main() -> Result<()> {
    let out = std::env::args().nth(1).unwrap_or_else(|| ".".into());
    std::fs::create_dir_all(&out)?;
    mine(&out)?;
    potholes(&out)?;
    calming(&out)?;
    rural(&out)?;
    Ok(())
}
