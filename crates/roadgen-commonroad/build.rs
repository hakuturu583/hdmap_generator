//! Compiles CommonRoad's 2024 map schema in `proto/` into Rust types.
//!
//! The same way `roadgen-womd` compiles Waymo's: protox reads the `.proto` files, so
//! the build needs no `protoc`, and the generated code lands in `OUT_DIR` where
//! nothing can drift from the schema.

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=proto");
    let descriptors = protox::compile(["map/commonroad_map.proto"], ["proto"])?;
    prost_build::Config::new().compile_fds(descriptors)?;
    Ok(())
}
