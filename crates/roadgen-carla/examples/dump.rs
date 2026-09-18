//! Writes a small CARLA package into a directory, so that one can be looked at.
//!
//! ```sh
//! cargo run -p roadgen-carla --example dump -- /tmp/Import
//! ```

use roadgen_core::prelude::*;
use roadgen_core::semantics::LaneType;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out = std::env::args().nth(1).unwrap_or_else(|| "Import".into());

    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("Town01".into()),
        ..MapMetadata::default()
    });
    let width = |metres: f64| PositiveWidth::new(metres).unwrap();
    let lanes = || {
        vec![
            // Outwards from the reference line on each side, which is the order the
            // builder counts ordinals in: carriageway first, then the pavement
            // beyond it.
            LaneSpec::new(width(3.5), Direction::Backward),
            LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
            LaneSpec::new(width(3.5), Direction::Forward),
            LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
        ]
    };
    let a = builder.add_road(
        RoadSpec::line(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(120.0, 0.0, 2.0),
            lanes(),
        )?
        .with_name("first"),
    )?;
    let b = builder.add_road(
        RoadSpec::line(
            Point3::new(120.0, 0.0, 2.0),
            Point3::new(220.0, 60.0, 4.0),
            lanes(),
        )?
        .with_name("second"),
    )?;
    builder.connect(&a, &b)?;

    let mut unvalidated = builder.finish()?;
    roadgen_buildings::generate(&mut unvalidated, &roadgen_buildings::Rules::default())?;
    let map = unvalidated.validate()?;
    let config = roadgen_carla::PackageConfig::for_map(&map);
    let package = roadgen_carla::write(&map, &out, &config)?;
    println!("{} meshes, {} triangles", package.meshes, package.triangles);
    println!("{:?}", package.labels);
    for warning in roadgen_carla::check(&map, &config) {
        println!("- {warning}");
    }
    Ok(())
}
