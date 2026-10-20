//! The container: one-sample WebDataset shards, and the `.npy` arrays inside them.
//!
//! Every file of an RDS-HQ clip is a tar archive holding a single WebDataset sample.
//! A member is named `{key}.{entry}`, and a reader groups members by everything
//! before the first dot and decodes each by what comes after the last one —
//! `.json` as JSON, `.npy` as a NumPy array, `.txt` as text. That is the whole of
//! the convention; nothing else about the archive is read.

use std::path::Path;

use crate::error::ExportError;

/// One WebDataset sample: a key, and the entries filed under it.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// Everything before the first dot of each member's name — the clip id.
    pub key: String,
    /// `(entry, contents)`, in the order they are archived. An entry's name is the
    /// member's name after the key, `000012.pose.camera_front_wide_120fov.npy`.
    pub entries: Vec<(String, Vec<u8>)>,
}

impl Sample {
    pub fn new(key: impl Into<String>) -> Self {
        Sample {
            key: key.into(),
            entries: Vec::new(),
        }
    }

    pub fn push(&mut self, entry: impl Into<String>, contents: Vec<u8>) {
        self.entries.push((entry.into(), contents));
    }

    pub fn push_json(&mut self, entry: impl Into<String>, value: &serde_json::Value) {
        self.push(entry, value.to_string().into_bytes());
    }

    /// The entry's contents, if the sample has it.
    pub fn get(&self, entry: &str) -> Option<&[u8]> {
        self.entries
            .iter()
            .find(|(name, _)| name == entry)
            .map(|(_, contents)| contents.as_slice())
    }

    /// The sample as a tar archive.
    ///
    /// Every header has the same mode, owner and a zero timestamp, so writing the same
    /// clip twice writes the same bytes.
    pub fn to_tar(&self) -> Result<Vec<u8>, ExportError> {
        let mut builder = tar::Builder::new(Vec::new());
        for (entry, contents) in &self.entries {
            let mut header = tar::Header::new_gnu();
            header.set_size(contents.len() as u64);
            header.set_mode(0o644);
            header.set_mtime(0);
            header.set_entry_type(tar::EntryType::Regular);
            builder
                .append_data(
                    &mut header,
                    format!("{}.{entry}", self.key),
                    contents.as_slice(),
                )
                .map_err(|error| ExportError::Io(format!("{}.{entry}: {error}", self.key)))?;
        }
        builder
            .into_inner()
            .map_err(|error| ExportError::Io(format!("{}: {error}", self.key)))
    }

    pub fn write(&self, path: &Path) -> Result<(), ExportError> {
        let bytes = self.to_tar()?;
        std::fs::write(path, bytes)
            .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))
    }
}

/// A row-major array of `f64` as a version 1.0 `.npy` file.
///
/// The format is a magic string, a Python dict literal describing the array, padded
/// so the data starts on a 64-byte boundary, and the data little-endian. That is
/// small enough that writing it beats depending on a crate for it.
pub fn npy(shape: &[usize], values: &[f64]) -> Vec<u8> {
    debug_assert_eq!(shape.iter().product::<usize>(), values.len());
    let shape_text = match shape {
        [single] => format!("({single},)"),
        _ => format!(
            "({})",
            shape
                .iter()
                .map(usize::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };
    let mut header = format!("{{'descr': '<f8', 'fortran_order': False, 'shape': {shape_text}, }}");
    // Magic (6) + version (2) + header length (2), then the header, ending in a
    // newline, padded with spaces to the boundary.
    let unpadded = 10 + header.len() + 1;
    header.push_str(&" ".repeat((64 - unpadded % 64) % 64));
    header.push('\n');

    let mut bytes = Vec::with_capacity(10 + header.len() + values.len() * 8);
    bytes.extend_from_slice(b"\x93NUMPY\x01\x00");
    bytes.extend_from_slice(&(header.len() as u16).to_le_bytes());
    bytes.extend_from_slice(header.as_bytes());
    for value in values {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    bytes
}

/// A 4 × 4 matrix, rows first, as `.npy`.
pub fn npy_matrix(matrix: &[[f64; 4]; 4]) -> Vec<u8> {
    let values: Vec<f64> = matrix.iter().flatten().copied().collect();
    npy(&[4, 4], &values)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_npy_header_puts_the_data_on_a_64_byte_boundary() {
        for shape in [vec![4, 4], vec![14], vec![6]] {
            let count = shape.iter().product();
            let bytes = npy(&shape, &vec![1.5; count]);
            assert_eq!(&bytes[..8], b"\x93NUMPY\x01\x00");
            let header = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
            assert_eq!((10 + header) % 64, 0, "{shape:?}");
            assert_eq!(bytes[10 + header - 1], b'\n');
            assert_eq!(bytes.len(), 10 + header + count * 8);
            let text = std::str::from_utf8(&bytes[10..10 + header]).unwrap();
            assert!(text.contains("'descr': '<f8'"), "{text}");
        }
        let vector = npy(&[14], &[0.0; 14]);
        assert!(std::str::from_utf8(&vector[10..74])
            .unwrap()
            .contains("(14,)"));
    }

    #[test]
    fn a_sample_archives_every_entry_under_its_key() {
        let mut sample = Sample::new("clip_01");
        sample.push("lanes.json", b"{}".to_vec());
        sample.push(
            "000000.pose.a_camera_name_long_enough_to_need_more_than_a_ustar_header_can_hold.npy",
            npy(&[1], &[2.0]),
        );
        let bytes = sample.to_tar().unwrap();
        let mut archive = tar::Archive::new(bytes.as_slice());
        let names: Vec<String> = archive
            .entries()
            .unwrap()
            .map(|entry| entry.unwrap().path().unwrap().display().to_string())
            .collect();
        assert_eq!(
            names,
            vec![
                "clip_01.lanes.json".to_owned(),
                "clip_01.000000.pose.a_camera_name_long_enough_to_need_more_than_a_ustar_header_can_hold.npy".to_owned(),
            ]
        );
        assert_eq!(
            sample.to_tar().unwrap(),
            bytes,
            "the same sample, the same bytes"
        );
    }
}
