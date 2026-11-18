//! A road with every kind of surface feature on it, written as OpenDRIVE with its
//! OpenCRG files: each road with features gets a `<surface>` whose `<CRG>`s are in
//! the spelling the schema has for a road, the grids hold the IR's displacement
//! cell for cell, a puddle lowers the friction, the files land beside the
//! document, and the document still reads back.

use roadgen_core::prelude::*;
use roadgen_core::surface::Relief;
use roadgen_opendrive::{
    check, crg, from_xml, to_xml, to_xml_with_crg, write_with_traced, CrgOptions, CrgPurpose,
    Options,
};

fn rough_road() -> (ValidatedMap, RoadId) {
    let mut builder = MapBuilder::default();
    let lanes = || {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    };
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(120.0, 0.0, 2.0),
                lanes(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    // A plain road beside it, which gets no <surface>.
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 50.0, 0.0),
                Point3::new(60.0, 50.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("plain"),
        )
        .unwrap();
    // Right-hand traffic: the forward lane is to the right, its middle at t = -1.75.
    builder
        .add_surface_feature(
            &road,
            10.0,
            40.0,
            -1.75,
            2.4,
            SurfaceFeatureKind::Rut {
                depth: 0.03,
                track: 1.8,
                trough: 0.6,
                path: None,
            },
        )
        .unwrap();
    builder
        .add_surface_feature(
            &road,
            60.0,
            0.9,
            1.5,
            0.7,
            SurfaceFeatureKind::Pothole {
                depth: 0.07,
                seed: 1,
            },
        )
        .unwrap();
    builder
        .add_surface_feature(
            &road,
            70.0,
            2.5,
            -2.0,
            1.6,
            SurfaceFeatureKind::Puddle { depth: 0.025 },
        )
        .unwrap();
    builder
        .add_surface_feature(
            &road,
            90.0,
            3.7,
            0.0,
            7.0,
            SurfaceFeatureKind::Bump {
                kind: BumpKind::SpeedHump,
                height: 0.08,
                profile: BumpProfile::Circular,
            },
        )
        .unwrap();
    builder
        .add_surface_feature(
            &road,
            100.0,
            15.0,
            0.0,
            7.0,
            SurfaceFeatureKind::Roughness {
                class: RoughnessClass::C,
                seed: 9,
            },
        )
        .unwrap();
    (builder.finish().unwrap().validate().unwrap(), road)
}

fn with_crg() -> Options {
    Options {
        crg: Some(CrgOptions {
            increment_u: 0.1,
            increment_v: 0.1,
            ..CrgOptions::default()
        }),
        ..Options::default()
    }
}

#[test]
fn a_feature_outside_its_road_does_not_validate() {
    let mut builder = MapBuilder::default();
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(20.0, 0.0, 0.0),
                vec![LaneSpec::new(
                    PositiveWidth::new(3.5).unwrap(),
                    Direction::Forward,
                )],
            )
            .unwrap(),
        )
        .unwrap();
    builder
        .add_surface_feature(
            &road,
            19.0,
            3.0,
            0.0,
            1.0,
            SurfaceFeatureKind::Puddle { depth: 0.02 },
        )
        .unwrap();
    assert!(builder.finish().unwrap().validate().is_err());
}

#[test]
fn without_the_option_nothing_changes_but_the_warning() {
    let (map, _) = rough_road();
    let xml = to_xml(&map).unwrap();
    assert!(!xml.contains("<surface"));
    assert!(check(&map)
        .iter()
        .any(|problem| problem.contains("5 surface features")));
    // Asked for the grids, the export loses nothing, and says nothing.
    assert!(!roadgen_opendrive::check_with(&map, &with_crg())
        .iter()
        .any(|problem| problem.contains("surface features")));
}

#[test]
fn each_road_with_features_refers_to_its_files_as_a_road_crg() {
    let (map, road) = rough_road();
    let (xml, trace, documents) = to_xml_with_crg(&map, &with_crg()).unwrap();
    // An elevation file, and a friction file because a puddle holds water.
    assert_eq!(documents.len(), 2);
    assert!(documents.iter().all(|document| document.road == road));
    assert_eq!(documents[0].purpose, CrgPurpose::Elevation);
    assert_eq!(documents[0].file_name, "main.elevation.crg");
    assert_eq!(documents[1].file_name, "main.friction.crg");
    assert_eq!(xml.matches("<surface>").count(), 1);
    assert_eq!(xml.matches("<CRG ").count(), 2);
    assert!(xml.contains(r#"file="main.elevation.crg""#));
    assert!(xml.contains(r#"mode="attached""#));
    assert!(xml.contains(r#"orientation="same""#));
    assert!(xml.contains(r#"purpose="friction""#));
    assert!(!xml.contains(r#"mode="global""#));
    // Every feature is traced to the file that holds it.
    for feature in map.surface_features.iter() {
        let links: Vec<_> = trace
            .links_of(&feature.id.clone().into())
            .map(|link| link.local.clone())
            .collect();
        assert!(links.contains(&"crg:main.elevation.crg".to_owned()));
    }
    // And a document roadgen wrote reads back, the grids passed over.
    let imported = from_xml(&xml).unwrap();
    assert!(imported
        .approximations
        .iter()
        .any(|note| note.contains("<CRG>")));
    assert_eq!(imported.map.as_map().roads.len(), 2);
}

#[test]
fn the_grid_holds_the_irs_displacement_cell_for_cell() {
    let (map, road) = rough_road();
    let options = CrgOptions {
        increment_u: 0.1,
        increment_v: 0.1,
        ..CrgOptions::default()
    };
    let documents = roadgen_opendrive::crg_documents(&map, &options).unwrap();
    let elevation = &documents[0];
    let text = &elevation.text(map.as_map()).unwrap();
    assert!(text.starts_with("$CT\n"));
    assert!(text.contains("#:LRFI"));
    assert!(text.contains("REFERENCE_LINE_INCREMENT = 0.100000"));
    // The grid's u is the road's own station, so no consumer has to offset it.
    assert!(text.contains(&format!(
        "REFERENCE_LINE_START_U   = {:.6}",
        elevation.s_start
    )));
    assert!(text.contains(&format!(
        "REFERENCE_LINE_END_U     = {:.6}",
        elevation.s_end
    )));
    for line in text.split("$$$$").next().unwrap().lines() {
        assert!(
            line.len() <= 72,
            "a header record is at most 72 bytes: {line:?}"
        );
    }
    let rows = crg::parse_values(text, elevation.columns).unwrap();
    assert_eq!(rows.len(), elevation.rows);
    // Margins either side of 10..115.
    assert!(elevation.s_start <= 9.5 + 1e-9 && elevation.s_end >= 115.0);
    assert!((elevation.s_end - elevation.s_start - 0.1 * (elevation.rows - 1) as f64).abs() < 1e-9);
    let relief = Relief::of(map.as_map(), &road);
    let mut deepest: f64 = 0.0;
    let mut highest: f64 = 0.0;
    for (i, row) in rows.iter().enumerate() {
        let s = elevation.s_start + 0.1 * i as f64;
        for (j, value) in row.iter().enumerate() {
            let t = elevation.v_right + 0.1 * j as f64;
            assert!(
                (value - relief.displacement(s, t)).abs() < 1e-6,
                "at s={s}, t={t}: {value} in the file"
            );
            deepest = deepest.min(*value);
            highest = highest.max(*value);
        }
    }
    // The pothole's floor and the hump's crest are both on the grid.
    assert!(deepest <= -0.07 + 1e-6, "deepest {deepest}");
    assert!(highest >= 0.08 - 1e-3, "highest {highest}");

    // The friction file is the same grid: dry asphalt's, halved under the puddle.
    let friction = &documents[1];
    assert_eq!(
        (friction.rows, friction.columns),
        (elevation.rows, elevation.columns)
    );
    let values =
        crg::parse_values(&friction.text(map.as_map()).unwrap(), friction.columns).unwrap();
    let all: Vec<f64> = values.into_iter().flatten().collect();
    let max = all.iter().copied().fold(f64::MIN, f64::max);
    let min = all.iter().copied().fold(f64::MAX, f64::min);
    assert!((max - 0.8).abs() < 1e-6);
    assert!((min - 0.4).abs() < 1e-6);
}

#[test]
fn the_files_are_written_beside_the_document() {
    let (map, _) = rough_road();
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("map.xodr");
    let trace = write_with_traced(&map, &path, &with_crg()).unwrap();
    for name in ["map.xodr", "main.elevation.crg", "main.friction.crg"] {
        let file = directory.path().join(name);
        assert!(file.exists(), "{name}");
        assert!(trace.files.contains(&file));
    }
}
