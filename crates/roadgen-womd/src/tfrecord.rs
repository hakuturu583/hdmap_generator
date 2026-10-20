//! The TFRecord container the Waymo Open Motion Dataset is distributed in.
//!
//! A TFRecord file is a run of records, each framed as
//!
//! ```text
//! u64  length              little-endian
//! u32  masked CRC-32C of the eight length bytes
//! [u8] data                `length` bytes
//! u32  masked CRC-32C of the data
//! ```
//!
//! and nothing else: no header, no index, no compression unless the reader is told
//! to expect it. WOMD's files hold one serialized `Scenario` per record, which is what
//! `tf.data.TFRecordDataset` and Waymax read. The container is small enough that it is
//! written here rather than pulled in with a TensorFlow dependency.

use crate::error::ExportError;

/// Frames `data` as one record.
pub fn record(data: &[u8]) -> Vec<u8> {
    let length = (data.len() as u64).to_le_bytes();
    let mut out = Vec::with_capacity(data.len() + 16);
    out.extend_from_slice(&length);
    out.extend_from_slice(&masked_crc(&length).to_le_bytes());
    out.extend_from_slice(data);
    out.extend_from_slice(&masked_crc(data).to_le_bytes());
    out
}

/// The records of a TFRecord file, each checked against both of its checksums.
pub fn records(mut bytes: &[u8]) -> Result<Vec<Vec<u8>>, ExportError> {
    let mut out = Vec::new();
    while !bytes.is_empty() {
        let index = out.len();
        let malformed = |what: &str| ExportError::TfRecord(format!("record {index}: {what}"));
        let (header, rest) = split(bytes, 12).ok_or_else(|| malformed("truncated header"))?;
        let length: [u8; 8] = header[..8].try_into().expect("eight bytes");
        if masked_crc(&length) != u32_at(&header[8..]) {
            return Err(malformed("the length's checksum does not match"));
        }
        let size = usize::try_from(u64::from_le_bytes(length))
            .map_err(|_| malformed("longer than this machine can address"))?;
        let (data, rest) = split(rest, size).ok_or_else(|| malformed("truncated data"))?;
        let (crc, rest) = split(rest, 4).ok_or_else(|| malformed("truncated checksum"))?;
        if masked_crc(data) != u32_at(crc) {
            return Err(malformed("the data's checksum does not match"));
        }
        out.push(data.to_vec());
        bytes = rest;
    }
    Ok(out)
}

fn split(bytes: &[u8], at: usize) -> Option<(&[u8], &[u8])> {
    (bytes.len() >= at).then(|| bytes.split_at(at))
}

fn u32_at(bytes: &[u8]) -> u32 {
    u32::from_le_bytes(bytes[..4].try_into().expect("four bytes"))
}

/// The checksum TFRecord stores: CRC-32C, rotated and offset so that the CRC of a
/// buffer that itself holds CRCs is not degenerate.
fn masked_crc(data: &[u8]) -> u32 {
    let crc = crc32c(data);
    crc.rotate_right(15).wrapping_add(0xa282_ead8)
}

/// CRC-32C (Castagnoli), the reflected polynomial `0x82F63B78`, a table at a time.
fn crc32c(data: &[u8]) -> u32 {
    const TABLE: [u32; 256] = {
        let mut table = [0u32; 256];
        let mut index = 0;
        while index < 256 {
            let mut crc = index as u32;
            let mut bit = 0;
            while bit < 8 {
                crc = if crc & 1 == 1 {
                    (crc >> 1) ^ 0x82f6_3b78
                } else {
                    crc >> 1
                };
                bit += 1;
            }
            table[index] = crc;
            index += 1;
        }
        table
    };
    !data.iter().fold(!0u32, |crc, &byte| {
        TABLE[((crc ^ byte as u32) & 0xff) as usize] ^ (crc >> 8)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_checksum_is_castagnolis() {
        // The check value every CRC-32C implementation is held to.
        assert_eq!(crc32c(b"123456789"), 0xe306_9283);
        assert_eq!(crc32c(b""), 0);
    }

    #[test]
    fn an_empty_record_is_framed_the_way_tensorflow_frames_it() {
        // `tf.io.TFRecordWriter` writing b"" produces exactly these sixteen bytes.
        assert_eq!(
            record(b""),
            [0, 0, 0, 0, 0, 0, 0, 0, 0x29, 0x03, 0x98, 0x07, 0xd8, 0xea, 0x82, 0xa2]
        );
    }

    #[test]
    fn records_read_back_and_a_flipped_byte_is_refused() {
        let mut file = record(b"first");
        file.extend(record(b"second"));
        assert_eq!(
            records(&file).unwrap(),
            vec![b"first".to_vec(), b"second".to_vec()]
        );
        let last = file.len() - 6;
        file[last] ^= 1;
        assert!(records(&file).is_err());
        assert!(records(&file[..10]).is_err());
    }
}
