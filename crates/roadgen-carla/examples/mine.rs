//! Generates the default mine and prints its terrain, breaklines, roads and land
//! covers as JSON, for looking at.
//!
//! ```sh
//! cargo run -p roadgen-carla --example mine > mine.json
//! ```

use roadgen_carla::environment::mine::{generate, lay_out, MineSpec};
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
    let spec = MineSpec {
        name: "North Pit".into(),
        seed,
        ..MineSpec::default()
    };
    let mut builder = MapBuilder::default();
    let plan = lay_out(&mut builder, &spec)?;
    let mut map = builder.finish()?;
    let report = generate(&mut map, &plan)?;
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
