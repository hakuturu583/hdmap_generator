use roadgen_carla::{materials, PackageConfig, Role};
use roadgen_core::prelude::*;
use roadgen_integration_tests::highway::build;

fn thin(p: [[f64; 3]; 3]) -> (f64, f64) {
    let d = |a: [f64; 3], b: [f64; 3]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let e = [d(p[0], p[1]), d(p[1], p[2]), d(p[2], p[0])];
    let long = e.iter().cloned().fold(0.0, f64::max);
    let area = ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
        - (p[2][0] - p[0][0]) * (p[1][1] - p[0][1]))
        .abs()
        / 2.0;
    (
        long,
        if area > 1e-9 {
            long * long / (2.0 * area)
        } else {
            1e9
        },
    )
}

#[test]
#[ignore]
fn samples() {
    for urban in [false, true] {
        let map = if urban {
            let mut builder = MapBuilder::default();
            for (name, x) in [("first", -100.0), ("second", 200.0)] {
                builder
                    .add_road(
                        RoadSpec::line(
                            Point3::new(x, -300.0, 0.0),
                            Point3::new(x, 300.0, 0.0),
                            roadgen_integration_tests::highway::street_lanes(),
                        )
                        .unwrap()
                        .with_name(name),
                    )
                    .unwrap();
            }
            let mut spec = roadgen_carla::environment::CorridorSpec::new(
                "Shuto",
                vec![Point3::new(-800.0, 0.0, 0.0), Point3::new(900.0, 0.0, 0.0)],
                60.0,
            )
            .unwrap();
            spec.elevated = vec![(200.0, 1400.0)];
            let mut places = roadgen_carla::environment::Places {
                corridors: vec![spec.clone()],
                town: Some(roadgen_buildings::Rules::preset("town").unwrap()),
                ..Default::default()
            };
            spec.plan(TrafficHandedness::RightHand)
                .unwrap()
                .apply(&mut builder, &mut places)
                .unwrap();
            let mut map = builder.finish().unwrap();
            roadgen_carla::environment::generate(&mut map, &places).unwrap();
            map.validate().unwrap()
        } else {
            build(TrafficHandedness::LeftHand).0.validate().unwrap()
        };
        println!("== urban {urban}");
        for s in map.tin_surfaces.iter() {
            let nodes = s.tin.nodes();
            let mut bad = 0;
            let mut long = 0;
            for t in s.tin.triangles() {
                let p = t.map(|i| {
                    let n = nodes[i as usize];
                    [n.x, n.y, n.z]
                });
                let (l, r) = thin(p);
                if r > 6.0 && l > 10.0 {
                    bad += 1;
                    if bad < 6 {
                        println!("tin thin at {:?} long {l:.0} ratio {r:.0}", p[0]);
                    }
                }
                if l > 60.0 {
                    long += 1;
                }
            }
            println!(
                "TIN {} tris, thin {bad}, long>60 {long}",
                s.tin.triangles().len()
            );
        }
        let meshes = roadgen_carla::to_meshes(&map, &PackageConfig::new("E"));
        let mut out = String::new();
        let (mut bad, mut total) = (0, 0);
        for mesh in &meshes {
            if mesh.role != Role::Terrain {
                continue;
            }
            for (t, slot) in mesh.triangles.iter().zip(&mesh.slots) {
                if mesh.materials[*slot] != materials::GRASS {
                    continue;
                }
                total += 1;
                let p = t.map(|i| {
                    let v = mesh.positions[i as usize];
                    [v.x, v.y, v.z]
                });
                let (l, r) = thin(p);
                let flag = r > 6.0 && l > 10.0;
                if flag {
                    bad += 1;
                }
                out.push_str(&format!(
                    "{} {} {} {} {} {} {}\n",
                    flag as u8, p[0][0], p[0][1], p[1][0], p[1][1], p[2][0], p[2][1]
                ));
            }
        }
        println!("CARLA land {total} tris, thin {bad}");
        std::fs::write(format!("/tmp/claude-0/-home-user-hdmap-generator/a5095769-8d3d-54e8-8498-0614ebeab469/scratchpad/land{}.txt", urban as u8), out).unwrap();
    }
}
