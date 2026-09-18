//! Reading a ClipGT clip back.
//!
//! A clip is a directory of Parquet files named `{clip_id}.{layer}.parquet`, each one
//! a single column named after its layer whose rows are nested structs. So reading it
//! is two steps: Parquet to Arrow, which the `parquet` crate does, and then Arrow's
//! type system to points, which is what most of this module is.
//!
//! A layer that is not there is not an error. ClipGT has no required set — a map with
//! no crossings is written with an empty `crosswalk` table, and a caller who only
//! kept some of the files should still see what is in the ones it kept.
//!
//! # Lanes have no centreline here
//!
//! ClipGT states a lane as its two rails and nothing between them, so the centre
//! drawn here is the midpoint of the two, computed. It is drawn because a lane you
//! cannot see the middle of is hard to read, and it is computed rather than read
//! because the format does not carry it.

use std::collections::BTreeMap;

use arrow::array::{Array, AsArray, RecordBatch, StructArray};
use arrow::datatypes::Float64Type;
use bytes::Bytes;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

use crate::drawing::{Drawing, Kind, Mark, MarkColor, Point, Shape};
use crate::error::ViewError;

/// A clip, as the layer tables that were kept. The key is the layer name — `lane`,
/// `lane_line` — without the clip id or the extension.
pub type Clip = BTreeMap<String, Vec<u8>>;

/// The layers this draws, in the order it draws them. A file outside this list is
/// ignored rather than guessed at: the calibration table is a rig, not a place.
pub const DRAWN: [&str; 8] = [
    "intersection_area",
    "lane",
    "lane_line",
    "road_boundary",
    "crosswalk",
    "wait_line",
    "traffic_light",
    "traffic_sign",
];

/// Draws a clip.
pub fn draw(clip: &Clip) -> Result<Drawing, ViewError> {
    let mut drawing = Drawing::new("ClipGT");
    let mut counts: Vec<String> = Vec::new();

    for layer in DRAWN {
        let Some(bytes) = clip.get(layer) else {
            continue;
        };
        let rows = read(bytes, layer)?;
        let mut drawn = 0usize;
        for row in &rows {
            drawn += 1;
            match layer {
                "intersection_area" => drawing.area(Kind::Junction, points_of(row, "location")?),
                "lane" => {
                    let left = points_of(row, "left_rail")?;
                    let right = points_of(row, "right_rail")?;
                    // The surface is the ring the two rails bound: down the left and
                    // back up the right.
                    let mut ring = left.clone();
                    ring.extend(right.iter().rev());
                    drawing.area(Kind::Surface, ring);
                    drawing.path(Kind::Center, midline(&left, &right));
                }
                "lane_line" => drawing.path(
                    Kind::Marking(mark(
                        first_string(row, "colors").as_deref(),
                        first_string(row, "styles").as_deref(),
                    )),
                    points_of(row, "line_rail")?,
                ),
                "road_boundary" => drawing.path(Kind::Boundary, points_of(row, "location")?),
                "crosswalk" => drawing.area(Kind::Crosswalk, points_of(row, "location")?),
                "wait_line" => drawing.path(Kind::StopLine, points_of(row, "location")?),
                "traffic_light" | "traffic_sign" => {
                    let kind = if layer == "traffic_light" {
                        Kind::TrafficLight
                    } else {
                        Kind::TrafficSign
                    };
                    if let Some(at) = point_of(row, "center") {
                        drawing.push(Shape::Dot { kind, at });
                    }
                }
                _ => drawn -= 1,
            }
        }
        if drawn > 0 {
            counts.push(format!("{drawn} {layer}"));
        }
    }

    // The ego track is a layer like any other to read and unlike any other to draw:
    // it is not the map, it is the drive through it, which is what makes the
    // directory a clip rather than a map.
    if let Some(bytes) = clip.get("egomotion_estimate") {
        let rows = read(bytes, "egomotion_estimate")?;
        let track: Vec<Point> = rows
            .iter()
            .filter_map(|row| point_of(row, "location"))
            .collect();
        if track.len() >= 2 {
            counts.push(format!("{} ego poses", track.len()));
        }
        drawing.path(Kind::Track, track);
    }

    drawing.note(if counts.is_empty() {
        "no layer of this clip holds anything to draw".to_owned()
    } else {
        counts.join(", ")
    });
    drawing.note(
        "a lane is its two rails; the centre line drawn between them is computed, not \
         read"
            .to_owned(),
    );
    Ok(drawing)
}

/// Every row of a layer, as the struct the layer's column holds.
///
/// A ClipGT table is one column named after the layer — except `egomotion_estimate`,
/// which has a `key` column beside it — so the column wanted is the one whose name
/// matches, and the whole file is read at once because a layer is a handful of rows.
fn read(bytes: &[u8], layer: &str) -> Result<Vec<Row>, ViewError> {
    let reader = ParquetRecordBatchReaderBuilder::try_new(Bytes::from(bytes.to_vec()))
        .map_err(|error| ViewError::Parse(format!("{layer}: {error}")))?
        .build()
        .map_err(|error| ViewError::Parse(format!("{layer}: {error}")))?;

    let mut rows = Vec::new();
    for batch in reader {
        let batch: RecordBatch =
            batch.map_err(|error| ViewError::Parse(format!("{layer}: {error}")))?;
        let column = batch
            .column_by_name(layer)
            .ok_or_else(|| ViewError::Shape(format!("{layer}.parquet has no {layer} column")))?;
        let structs = column.as_struct_opt().ok_or_else(|| {
            ViewError::Shape(format!("the {layer} column is not a column of structs"))
        })?;
        for index in 0..structs.len() {
            rows.push(Row {
                structs: structs.clone(),
                index,
            });
        }
    }
    Ok(rows)
}

/// One row of a layer: the column, and which row of it.
struct Row {
    structs: StructArray,
    index: usize,
}

impl Row {
    fn field(&self, name: &str) -> Option<&dyn Array> {
        self.structs
            .column_by_name(name)
            .map(|array| array.as_ref())
    }
}

/// A `{x, y, z}` at one row of a struct column, z dropped.
fn read_point(structs: &StructArray, index: usize) -> Option<Point> {
    if structs.is_null(index) {
        return None;
    }
    let x = structs
        .column_by_name("x")?
        .as_primitive_opt::<Float64Type>()?;
    let y = structs
        .column_by_name("y")?
        .as_primitive_opt::<Float64Type>()?;
    Some(Point::new(x.value(index), y.value(index)))
}

/// The single point a field holds at this row.
fn point_of(row: &Row, field: &str) -> Option<Point> {
    read_point(row.field(field)?.as_struct_opt()?, row.index)
}

/// The list of points a field holds at this row.
fn points_of(row: &Row, field: &str) -> Result<Vec<Point>, ViewError> {
    let Some(array) = row.field(field) else {
        return Ok(Vec::new());
    };
    let lists = array
        .as_list_opt::<i32>()
        .ok_or_else(|| ViewError::Shape(format!("{field} is not a list")))?;
    if lists.is_null(row.index) {
        return Ok(Vec::new());
    }
    let values = lists.value(row.index);
    let structs = values
        .as_struct_opt()
        .ok_or_else(|| ViewError::Shape(format!("{field} is not a list of points")))?;
    Ok((0..structs.len())
        .filter_map(|index| read_point(structs, index))
        .collect())
}

/// The first of the strings a list field holds at this row. ClipGT gives a line one
/// colour per row in a list of one, and the first is the line's.
fn first_string(row: &Row, field: &str) -> Option<String> {
    let lists = row.field(field)?.as_list_opt::<i32>()?;
    if lists.is_null(row.index) {
        return None;
    }
    let values = lists.value(row.index);
    let strings = values.as_string_opt::<i32>()?;
    (strings.len() > 0).then(|| strings.value(0).to_owned())
}

/// ClipGT's colour and style words, as the two things a picture can show.
///
/// A double line is drawn as a single one, for the same reason as everywhere else: a
/// row carries one polyline whatever its style says, and drawing two would be
/// inventing the gap between them.
fn mark(color: Option<&str>, style: Option<&str>) -> Mark {
    let color = match color {
        Some("WHITE") => MarkColor::White,
        Some("YELLOW") => MarkColor::Yellow,
        _ => MarkColor::Other,
    };
    let broken = matches!(style, Some(style) if style.starts_with("DASHED"));
    Mark::new(color, broken)
}

/// The line between two rails, vertex by vertex.
///
/// The rails are sampled from the same stations by the exporter, so they are the same
/// length and pairing them up is pairing up stations. Where they are not — a clip
/// written by something else — the shorter one ends the line rather than the longer
/// one being paired against nothing.
fn midline(left: &[Point], right: &[Point]) -> Vec<Point> {
    left.iter()
        .zip(right.iter())
        .map(|(left, right)| Point::new((left.x + right.x) / 2.0, (left.y + right.y) / 2.0))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_centre_of_a_lane_is_between_its_rails() {
        let left = [Point::new(0.0, 1.75), Point::new(10.0, 1.75)];
        let right = [Point::new(0.0, -1.75), Point::new(10.0, -1.75)];
        assert_eq!(
            midline(&left, &right),
            vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)]
        );
    }

    #[test]
    fn rails_of_different_lengths_end_with_the_shorter_one() {
        let left = [
            Point::new(0.0, 1.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 1.0),
        ];
        let right = [Point::new(0.0, -1.0)];
        assert_eq!(midline(&left, &right).len(), 1);
    }

    #[test]
    fn clipgts_own_words_decide_how_a_line_is_painted() {
        assert_eq!(
            mark(Some("YELLOW"), Some("DASHED_SINGLE")),
            Mark::new(MarkColor::Yellow, true)
        );
        assert_eq!(
            mark(Some("WHITE"), Some("SOLID_DOUBLE")),
            Mark::new(MarkColor::White, false)
        );
        assert_eq!(mark(None, None), Mark::new(MarkColor::Other, false));
    }

    #[test]
    fn a_clip_with_no_files_draws_nothing_rather_than_failing() {
        let drawing = draw(&Clip::new()).unwrap();
        assert!(drawing.shapes.is_empty());
        assert!(drawing.notes[0].contains("no layer"), "{:?}", drawing.notes);
    }

    #[test]
    fn bytes_that_are_not_parquet_are_an_error() {
        let mut clip = Clip::new();
        clip.insert("lane".into(), b"not a parquet file".to_vec());
        assert!(matches!(draw(&clip), Err(ViewError::Parse(_))));
    }
}
