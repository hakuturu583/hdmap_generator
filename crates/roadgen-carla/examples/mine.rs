//! Puts a pit, the ramp into it and a waste dump on a map, and prints the terrain,
//! breaklines, roads and land covers as JSON, for looking at.
//!
//! ```sh
//! cargo run -p roadgen-carla --example mine > mine.json
//! ```

use roadgen_carla::environment::{generate, HeapSpec, PitSpec, Places, RampSpec};
use roadgen_core::prelude::*;

fn points(points: &[Point3]) -> String {
    let inner: Vec<String> = points
        .iter()
        .map(|p| format!("[{:.3},{:.3},{:.3}]", p.x, p.y, p.z))
        .collect();
    format!("[{}]", inner.join(","))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let seed = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let pit = PitSpec {
        name: "North Pit".into(),
        ..PitSpec::default()
    };
    let mut builder = MapBuilder::default();
    let lanes = vec![
        LaneSpec::new(PositiveWidth::new(10.5)?, Direction::Forward),
        LaneSpec::new(PositiveWidth::new(10.5)?, Direction::Backward),
    ];
    let ramp = builder.add_road(
        RoadSpec::new(Curve3::polyline(pit.ramp(&RampSpec::default())?)?, lanes).with_name("ramp"),
    )?;
    builder.add_barrier(&LaneRef::new(ramp.clone(), 0), "berm", 1.5)?;
    let places = Places {
        heaps: vec![HeapSpec {
            name: "dump".into(),
            centre: Point3::new(300.0, -250.0, 0.0),
            radius: 90.0,
            height: 25.0,
            site: Some(pit.name.clone()),
            ..HeapSpec::default()
        }],
        site_roads: vec![(ramp, pit.name.clone())],
        pits: vec![pit],
        seed,
        ..Places::default()
    };
    let mut map = builder.finish()?;
    let report = generate(&mut map, &places)?;
    eprintln!("{report:?}");
    let map = map.validate()?;
    let sampling = map.metadata.sampling;

    let mut out = String::from("{");
    for surface in map.tin_surfaces.iter() {
        out.push_str(&format!("\"nodes\":{},", points(surface.tin.nodes())));
        let triangles: Vec<String> = surface
            .tin
            .triangles()
            .iter()
            .map(|t| format!("[{},{},{}]", t[0], t[1], t[2]))
            .collect();
        out.push_str(&format!("\"triangles\":[{}],", triangles.join(",")));
    }
    let breaklines: Vec<String> = map
        .tin_breaklines
        .iter()
        .map(|line| {
            format!(
                "{{\"kind\":\"{}\",\"points\":{}}}",
                line.kind,
                points(line.line.points())
            )
        })
        .collect();
    out.push_str(&format!("\"breaklines\":[{}],", breaklines.join(",")));
    let mut lanes = Vec::new();
    for lane in map.lanes.iter() {
        for boundary in [&lane.left_boundary, &lane.right_boundary] {
            lanes.push(format!(
                "{{\"type\":\"{}\",\"points\":{}}}",
                lane.lane_type.as_str(),
                points(boundary.to_polyline(sampling)?.points())
            ));
        }
    }
    out.push_str(&format!("\"lanes\":[{}],", lanes.join(",")));
    let covers: Vec<String> = map
        .land_covers
        .iter()
        .map(|cover| {
            format!(
                "{{\"kind\":\"{}\",\"points\":{}}}",
                cover.kind,
                points(cover.area.points())
            )
        })
        .collect();
    out.push_str(&format!("\"land_covers\":[{}]", covers.join(",")));
    out.push('}');
    println!("{out}");
    Ok(())
}
