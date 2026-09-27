//! How far the lanes of a Lanelet2 map move on the way through OpenDRIVE.
//!
//! ```text
//! cargo run --release -p roadgen-integration-tests --example lanelet2_opendrive_drift -- map.osm [--lht] [--csv drift.csv]
//! ```
//!
//! Reads the Lanelet2 map into the IR, writes it as OpenDRIVE, reads that back, and
//! measures, in 3D, how far every lane boundary in the document is from the one in
//! the file: every vertex of the file's against the document's boundary, every
//! vertex of the document's against the file's, and every end against the
//! document's end — which is what a consumer of the OpenDRIVE sees. `--csv`
//! writes every vertex as `x,y,z,drift` for plotting.

use std::collections::BTreeMap;

use roadgen_core::geometry::Point3;
use roadgen_core::id::RoadId;
use roadgen_core::map::{Lane, TrafficHandedness};
use roadgen_lanelet2::ReadOptions;

/// Beyond this a road is listed by name, metres.
const REPORTED: f64 = 0.1;

fn distance_to(polyline: &[Point3], point: Point3) -> f64 {
    polyline
        .windows(2)
        .map(|pair| {
            let (a, b) = (pair[0], pair[1]);
            let (dx, dy, dz) = (b.x - a.x, b.y - a.y, b.z - a.z);
            let length_squared = dx * dx + dy * dy + dz * dz;
            let u = if length_squared > 0.0 {
                (((point.x - a.x) * dx + (point.y - a.y) * dy + (point.z - a.z) * dz)
                    / length_squared)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            a.lerp(b, u).distance_to(point)
        })
        .fold(f64::INFINITY, f64::min)
}

fn summary(name: &str, values: &mut [f64]) {
    values.sort_by(f64::total_cmp);
    let at = |fraction: f64| values[((values.len() - 1) as f64 * fraction) as usize];
    println!(
        "{name:10} n={:6}  p50 {:.4}  p90 {:.4}  p99 {:.4}  max {:.4}  >1cm {}  >10cm {}",
        values.len(),
        at(0.5),
        at(0.9),
        at(0.99),
        at(1.0),
        values.iter().filter(|d| **d > 0.01).count(),
        values.iter().filter(|d| **d > 0.1).count()
    );
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .ok_or("usage: lanelet2_opendrive_drift <map.osm> [--lht] [--csv drift.csv]")?;
    let mut options = ReadOptions::default();
    if args.iter().any(|arg| arg == "--lht") {
        options.handedness = TrafficHandedness::LeftHand;
    }
    let map = roadgen_lanelet2::read_with(path, &options)?
        .map
        .validate()?;
    let config = map.metadata.sampling;
    let back = roadgen_opendrive::from_xml(&roadgen_opendrive::to_xml(&map)?)?.map;
    let back = back.as_map();

    let csv = args
        .iter()
        .position(|arg| arg == "--csv")
        .and_then(|index| args.get(index + 1));
    let mut rows = String::from("x,y,z,drift\n");
    let mut vertices = Vec::new();
    let mut strays = Vec::new();
    let mut ends = Vec::new();
    let mut by_road: BTreeMap<String, f64> = BTreeMap::new();
    // The exporter numbers roads in the map's order, and the reader names them so.
    for (index, road) in map.roads.iter().enumerate() {
        let twins: Vec<&Lane> = back.lanes_of(&RoadId::new(index.to_string()));
        for lane in map.lanes_of(&road.id) {
            let Some(twin) = twins
                .iter()
                .find(|other| other.side == lane.side && other.ordinal == lane.ordinal)
            else {
                println!("{} has no lane {} in the document", road.id, lane.ordinal);
                continue;
            };
            for (mine, theirs) in [
                (&lane.left_boundary, &twin.left_boundary),
                (&lane.right_boundary, &twin.right_boundary),
            ] {
                let mine = mine.to_polyline(config)?;
                let theirs = theirs.to_polyline(config)?;
                let mut worst: f64 = 0.0;
                for (a, b) in [(mine.first(), theirs.first()), (mine.last(), theirs.last())] {
                    let d = a.distance_to(b);
                    ends.push(d);
                    worst = worst.max(d);
                }
                // And the document's vertices against the file's boundary, which is
                // what shows a boundary that loops out between two of the file's.
                for point in theirs.points() {
                    let d = distance_to(mine.points(), *point);
                    strays.push(d);
                    worst = worst.max(d);
                }
                for point in mine.points() {
                    let d = distance_to(theirs.points(), *point);
                    vertices.push(d);
                    rows.push_str(&format!("{},{},{},{d}\n", point.x, point.y, point.z));
                    worst = worst.max(d);
                }
                let entry = by_road.entry(road.id.to_string()).or_insert(0.0);
                *entry = entry.max(worst);
            }
        }
    }

    if let Some(csv) = csv {
        std::fs::write(csv, rows)?;
    }
    summary("ends", &mut ends);
    summary("vertices", &mut vertices);
    summary("document", &mut strays);
    let mut far: Vec<(&String, &f64)> = by_road.iter().filter(|(_, d)| **d > REPORTED).collect();
    far.sort_by(|a, b| b.1.total_cmp(a.1));
    println!(
        "roads with an edge further than {REPORTED} m: {} of {}",
        far.len(),
        by_road.len()
    );
    for (road, d) in far {
        let name = map
            .road(&RoadId::from_raw(road))
            .and_then(|road| road.name.clone())
            .unwrap_or_default();
        println!("  {d:7.3} m  {road}  ({name})");
    }
    Ok(())
}
