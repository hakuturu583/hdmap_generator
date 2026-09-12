//! Building the nested Arrow columns ClipGT's tables are made of.
//!
//! Every layer is a one-column table whose column is a struct named after the layer,
//! and almost every field of that struct is either a `{x, y, z}` point or a list of
//! them. These helpers build those two shapes so the layer code can stay a
//! description of what goes where.
//!
//! Points are written as `f64`. The reader casts to `float32` on the way in, so the
//! extra precision is lost there — but losing it *here* would mean the file no longer
//! says what the IR computed, and a height a few millimetres out is exactly the kind
//! of error nobody notices until a vehicle is placed on the surface.

use std::sync::Arc;

use arrow::array::{ArrayRef, Float64Array, Int64Array, ListArray, StringArray, StructArray};
use arrow::buffer::OffsetBuffer;
use arrow::datatypes::{DataType, Field, Fields};

use roadgen_core::geometry::Point3;

/// The `{x, y, z}` a ClipGT point is written as.
pub fn point_fields() -> Fields {
    Fields::from(vec![
        Field::new("x", DataType::Float64, false),
        Field::new("y", DataType::Float64, false),
        Field::new("z", DataType::Float64, false),
    ])
}

/// The `{x, y, z, w}` an orientation is written as.
pub fn quaternion_fields() -> Fields {
    Fields::from(vec![
        Field::new("x", DataType::Float64, false),
        Field::new("y", DataType::Float64, false),
        Field::new("z", DataType::Float64, false),
        Field::new("w", DataType::Float64, false),
    ])
}

pub fn point_type() -> DataType {
    DataType::Struct(point_fields())
}

pub fn quaternion_type() -> DataType {
    DataType::Struct(quaternion_fields())
}

/// The list-of-points type a rail, a boundary or a polygon is written as.
pub fn point_list_type() -> DataType {
    DataType::List(Arc::new(Field::new("item", point_type(), false)))
}

pub fn string_list_type() -> DataType {
    DataType::List(Arc::new(Field::new("item", DataType::Utf8, false)))
}

/// One point per row.
pub fn points(values: &[Point3]) -> StructArray {
    let columns: Vec<ArrayRef> = vec![
        Arc::new(Float64Array::from_iter_values(
            values.iter().map(|point| point.x),
        )),
        Arc::new(Float64Array::from_iter_values(
            values.iter().map(|point| point.y),
        )),
        Arc::new(Float64Array::from_iter_values(
            values.iter().map(|point| point.z),
        )),
    ];
    StructArray::new(point_fields(), columns, None)
}

/// One orientation per row, as `(x, y, z, w)`.
pub fn quaternions(values: &[[f64; 4]]) -> StructArray {
    let component = |index: usize| -> ArrayRef {
        Arc::new(Float64Array::from_iter_values(
            values.iter().map(|quaternion| quaternion[index]),
        ))
    };
    StructArray::new(
        quaternion_fields(),
        vec![component(0), component(1), component(2), component(3)],
        None,
    )
}

/// One list of points per row.
pub fn point_lists(rows: &[Vec<Point3>]) -> ListArray {
    let flat: Vec<Point3> = rows.iter().flatten().copied().collect();
    let offsets = OffsetBuffer::from_lengths(rows.iter().map(|row| row.len()));
    ListArray::new(
        Arc::new(Field::new("item", point_type(), false)),
        offsets,
        Arc::new(points(&flat)),
        None,
    )
}

/// One list of strings per row.
pub fn string_lists(rows: &[Vec<String>]) -> ListArray {
    let flat: Vec<&str> = rows.iter().flatten().map(String::as_str).collect();
    let offsets = OffsetBuffer::from_lengths(rows.iter().map(|row| row.len()));
    ListArray::new(
        Arc::new(Field::new("item", DataType::Utf8, false)),
        offsets,
        Arc::new(StringArray::from(flat)),
        None,
    )
}

pub fn strings(values: &[String]) -> StringArray {
    StringArray::from_iter_values(values)
}

pub fn integers(values: &[i64]) -> Int64Array {
    Int64Array::from_iter_values(values.iter().copied())
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::Array;

    #[test]
    fn a_list_of_points_keeps_each_rows_length() {
        let rows = vec![
            vec![Point3::new(0.0, 0.0, 1.0), Point3::new(1.0, 0.0, 2.0)],
            vec![Point3::new(0.0, 5.0, 3.0)],
        ];
        let array = point_lists(&rows);
        assert_eq!(array.len(), 2);
        assert_eq!(array.value_length(0), 2);
        assert_eq!(array.value_length(1), 1);
    }

    #[test]
    fn a_points_z_survives_the_column() {
        let array = points(&[Point3::new(1.0, 2.0, 3.5)]);
        let heights = array
            .column(2)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap();
        assert_eq!(heights.value(0), 3.5);
    }

    #[test]
    fn an_empty_layer_is_a_column_with_no_rows() {
        let array = point_lists(&[]);
        assert_eq!(array.len(), 0);
    }
}
