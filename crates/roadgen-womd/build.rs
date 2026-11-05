//! Compiles the Waymo Open Dataset schema in `proto/` into Rust types.
//!
//! The same way `roadgen-proto` compiles its own: protox reads the `.proto` files, so
//! the build needs no `protoc`, and the generated code lands in `OUT_DIR` where
//! nothing can drift from the schema.

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=proto");
    let descriptors = protox::compile(["waymo_open_dataset/protos/scenario.proto"], ["proto"])?;
    prost_build::Config::new().compile_fds(descriptors)?;
    Ok(())
}
