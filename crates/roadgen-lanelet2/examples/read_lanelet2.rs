//! Reads a Lanelet2 map into the IR, validates it and writes it out again.
//!
//! ```text
//! cargo run --release -p roadgen-lanelet2 --example read_lanelet2 -- map.osm [--lht] [out.osm]
//! ```

use std::collections::BTreeMap;

use roadgen_core::map::TrafficHandedness;
use roadgen_lanelet2::ReadOptions;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .ok_or("usage: read_lanelet2 <map.osm> [--lht] [out.osm]")?;
    let mut options = ReadOptions::default();
    if args.iter().any(|arg| arg == "--lht") {
        options.handedness = TrafficHandedness::LeftHand;
    }
    let output = args.iter().skip(1).find(|arg| !arg.starts_with("--"));

    let started = std::time::Instant::now();
    let imported = roadgen_lanelet2::read_with(path, &options)?;
    println!("read in {:.2?}", started.elapsed());
    for note in &imported.approximations {
        println!("  note: {note}");
    }
    let map = imported.map.as_map();
    println!(
        "roads {} (connectors {}), lanes {}, junctions {}, connections {}, objects {}, rules {}",
        map.roads.len(),
        map.roads.iter().filter(|road| road.is_connector()).count(),
        map.lanes.len(),
        map.junctions.len(),
        map.connections.len(),
        map.objects.len(),
        map.rules.len()
    );

    match imported.map.validate() {
        Ok(map) => {
            println!("validation: passed");
            for problem in roadgen_lanelet2::check(&map) {
                println!("  lanelet2 check: {problem}");
            }
            if let Some(output) = output {
                roadgen_lanelet2::write(&map, output)?;
                println!("wrote {output}");
            }
        }
        Err(error) => {
            println!("validation: {} issues", error.issues.len());
            let mut kinds: BTreeMap<String, Vec<String>> = BTreeMap::new();
            for issue in &error.issues {
                let debug = format!("{issue:?}");
                let kind = debug
                    .split(|c: char| !c.is_alphanumeric())
                    .next()
                    .unwrap_or("")
                    .to_owned();
                kinds.entry(kind).or_default().push(issue.to_string());
            }
            for (kind, issues) in kinds {
                println!("  {kind}: {}", issues.len());
                for issue in issues.iter().take(20) {
                    println!("    {issue}");
                }
            }
        }
    }
    Ok(())
}
