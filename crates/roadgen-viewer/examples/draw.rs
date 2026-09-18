//! Draws an exported file or directory, and writes the SVG to stdout.
//!
//! ```text
//! cargo run -p roadgen-viewer --example draw -- map.xodr > map.svg
//! ```
//!
//! What it is for is looking at a real export without a browser in the loop: the
//! format is chosen by what the path is, the same way the demo page chooses it.

use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: draw <map.xodr | scene.json | map.fbx | sumo/ | clip/>");
        std::process::exit(2);
    };
    let path = Path::new(&path);
    let drawing = if path.is_dir() {
        // A clip is Parquet and a network is XML, so which directory this is can be
        // read off the files rather than asked for.
        match roadgen_viewer::read::clipgt(path) {
            Ok(clip) => roadgen_viewer::clipgt::draw(&clip)?,
            Err(_) => roadgen_viewer::sumo_directory(path)?,
        }
    } else if path
        .extension()
        .is_some_and(|extension| extension == "json")
    {
        roadgen_viewer::gpudrive_file(path)?
    } else if path.extension().is_some_and(|extension| extension == "fbx") {
        roadgen_viewer::fbx_file(path)?
    } else {
        roadgen_viewer::opendrive_file(path)?
    };
    eprintln!("{}: {}", drawing.title, drawing.notes.join(" · "));
    println!("{}", drawing.to_svg());
    Ok(())
}
