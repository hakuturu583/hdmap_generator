//! Prints the OpenDRIVE document for the two-road example, for eyeballing.

use roadgen_core::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let lanes = || {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    };
    let mut builder = MapBuilder::default();
    let a = builder.add_road(
        RoadSpec::line(
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(100.0, 0.0, 12.0),
            lanes(),
        )?
        .with_name("a"),
    )?;
    let b = builder.add_road(
        RoadSpec::line(
            Point3::new(100.0, 0.0, 12.0),
            Point3::new(200.0, 50.0, 15.0),
            lanes(),
        )?
        .with_name("b"),
    )?;
    builder.connect(&a, &b)?;
    let map = builder.finish()?.validate()?;
    println!("{}", roadgen_opendrive::to_xml(&map)?);
    Ok(())
}
