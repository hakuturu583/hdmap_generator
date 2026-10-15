//! Compiles the schema in `proto/` into Rust types.
//!
//! The `.proto` files are compiled by protox, a protobuf compiler written in Rust,
//! so the build needs no `protoc` on the machine. The generated code lands in
//! `OUT_DIR` and is included by `src/lib.rs`, with the descriptor set and the
//! table of sources behind `SCHEMA`; none of it is committed, so none of it can
//! drift from the schema.

use std::path::PathBuf;

const FILES: &[&str] = &[
    "roadgen_ir/v1/geometry.proto",
    "roadgen_ir/v1/topology.proto",
    "roadgen_ir/v1/semantics.proto",
    "roadgen_ir/v1/structure.proto",
    "roadgen_ir/v1/environment.proto",
    "roadgen_ir/v1/map.proto",
];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=proto");
    let descriptors = protox::compile(FILES, ["proto"])?;
    let out = PathBuf::from(std::env::var("OUT_DIR")?);
    std::fs::write(
        out.join("roadgen_ir_v1_descriptor.bin"),
        prost::Message::encode_to_vec(&descriptors),
    )?;
    // `SCHEMA`, the same files as source, from the same list.
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?).join("proto");
    let entries: Vec<String> = FILES
        .iter()
        .map(|file| format!("({file:?}, include_str!({:?})),", root.join(file)))
        .collect();
    std::fs::write(out.join("schema.rs"), format!("&[{}]", entries.join("\n")))?;
    prost_build::Config::new().compile_fds(descriptors)?;
    Ok(())
}
