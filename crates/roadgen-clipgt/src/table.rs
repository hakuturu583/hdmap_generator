//! Writing one layer as a Parquet file.

use std::fs::File;
use std::path::Path;

use arrow::array::RecordBatch;
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;

use crate::error::ExportError;

pub fn write(path: &Path, batch: &RecordBatch) -> Result<(), ExportError> {
    let file = File::create(path)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    // Snappy because every reader of these files has it, and because leaving the
    // pages uncompressed makes a map of any size wasteful to move around.
    let properties = WriterProperties::builder()
        .set_compression(Compression::SNAPPY)
        .build();
    let mut writer = ArrowWriter::try_new(file, batch.schema(), Some(properties))
        .map_err(|error| ExportError::Schema(error.to_string()))?;
    writer
        .write(batch)
        .map_err(|error| ExportError::Schema(error.to_string()))?;
    writer
        .close()
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    Ok(())
}
