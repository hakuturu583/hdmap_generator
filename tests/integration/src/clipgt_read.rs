//! An independent reader for the ClipGT files, for the tests to check against.
//!
//! The exporter builds Arrow arrays; this pulls values back out of a written Parquet
//! file by name, the way a consumer indexing `row["lane"]["left_rail"]` does. Going
//! through the file rather than through the exporter's own structs is the point: a
//! test that never opens the file cannot tell whether the file says what it should.

use std::collections::HashMap;
use std::fs::File;
use std::path::Path;

use arrow::array::{Array, Float64Array, Int64Array, ListArray, StringArray, StructArray};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

use roadgen_core::geometry::Point3;

/// One row of a layer, flattened to the fields the tests ask about.
#[derive(Debug, Clone, Default)]
pub struct Element {
    pub point_lists: HashMap<String, Vec<Point3>>,
    pub points: HashMap<String, Point3>,
    pub quaternions: HashMap<String, [f64; 4]>,
    pub strings: HashMap<String, String>,
    pub string_lists: HashMap<String, Vec<String>>,
    pub integers: HashMap<String, i64>,
}

impl Element {
    pub fn list(&self, field: &str) -> &[Point3] {
        self.point_lists
            .get(field)
            .unwrap_or_else(|| panic!("no point list called {field:?}; this row has {self:?}"))
    }

    pub fn point(&self, field: &str) -> Point3 {
        self.points[field]
    }
}

/// Reads one layer of a clip: `{clip_id}.{layer}.parquet`, flattened per row.
///
/// Every top-level column is a struct, so the rows come back with the fields of all
/// of them together — which is what the tests want, since a layer with two columns
/// (`key` and `egomotion_estimate`) is describing one thing.
pub fn layer(directory: &Path, clip_id: &str, name: &str) -> Vec<Element> {
    let path = directory.join(format!("{clip_id}.{name}.parquet"));
    let file = File::open(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    let reader = ParquetRecordBatchReaderBuilder::try_new(file)
        .expect("a parquet reader should accept the export")
        .build()
        .expect("a record batch reader");

    let mut rows: Vec<Element> = Vec::new();
    for batch in reader {
        let batch = batch.expect("a readable batch");
        let start = rows.len();
        rows.resize(start + batch.num_rows(), Element::default());
        for column in batch.columns() {
            let structs = column
                .as_any()
                .downcast_ref::<StructArray>()
                .expect("every ClipGT column is a struct");
            read_struct(structs, &mut rows[start..]);
        }
    }
    rows
}

/// Whether a layer's file is there at all.
pub fn exists(directory: &Path, clip_id: &str, name: &str) -> bool {
    directory
        .join(format!("{clip_id}.{name}.parquet"))
        .is_file()
}

fn read_struct(structs: &StructArray, rows: &mut [Element]) {
    for (field, array) in structs.fields().iter().zip(structs.columns()) {
        let name = field.name().to_owned();
        if let Some(lists) = array.as_any().downcast_ref::<ListArray>() {
            for (index, row) in rows.iter_mut().enumerate() {
                let values = lists.value(index);
                if let Some(points) = values.as_any().downcast_ref::<StructArray>() {
                    row.point_lists.insert(name.clone(), read_points(points));
                } else if let Some(text) = values.as_any().downcast_ref::<StringArray>() {
                    row.string_lists.insert(
                        name.clone(),
                        (0..text.len()).map(|i| text.value(i).to_owned()).collect(),
                    );
                }
            }
        } else if let Some(nested) = array.as_any().downcast_ref::<StructArray>() {
            let names: Vec<&str> = nested.fields().iter().map(|f| f.name().as_str()).collect();
            if names == ["x", "y", "z", "w"] {
                for (index, quaternion) in read_quaternions(nested).into_iter().enumerate() {
                    rows[index].quaternions.insert(name.clone(), quaternion);
                }
            } else {
                for (index, point) in read_points(nested).into_iter().enumerate() {
                    rows[index].points.insert(name.clone(), point);
                }
            }
        } else if let Some(text) = array.as_any().downcast_ref::<StringArray>() {
            for (index, row) in rows.iter_mut().enumerate() {
                row.strings
                    .insert(name.clone(), text.value(index).to_owned());
            }
        } else if let Some(numbers) = array.as_any().downcast_ref::<Int64Array>() {
            for (index, row) in rows.iter_mut().enumerate() {
                row.integers.insert(name.clone(), numbers.value(index));
            }
        }
    }
}

fn floats<'a>(structs: &'a StructArray, field: &str) -> &'a Float64Array {
    let index = structs
        .fields()
        .iter()
        .position(|candidate| candidate.name() == field)
        .unwrap_or_else(|| panic!("a point should have a {field} field"));
    structs.column(index).as_any().downcast_ref().expect("f64")
}

fn read_points(structs: &StructArray) -> Vec<Point3> {
    let (x, y, z) = (
        floats(structs, "x"),
        floats(structs, "y"),
        floats(structs, "z"),
    );
    (0..structs.len())
        .map(|index| Point3::new(x.value(index), y.value(index), z.value(index)))
        .collect()
}

fn read_quaternions(structs: &StructArray) -> Vec<[f64; 4]> {
    let (x, y, z, w) = (
        floats(structs, "x"),
        floats(structs, "y"),
        floats(structs, "z"),
        floats(structs, "w"),
    );
    (0..structs.len())
        .map(|index| {
            [
                x.value(index),
                y.value(index),
                z.value(index),
                w.value(index),
            ]
        })
        .collect()
}
