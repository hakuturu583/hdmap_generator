//! The build script that takes a written package the rest of the way.
//!
//! A package is four things CARLA's importer reads and two it does not: the
//! descriptor, the `.fbx`, the `.xodr`, the texture manifest — and then the sky,
//! which `Import.py` does not give a map and `roadgen.carla_sky()` does, and the
//! furniture, which `Import.py` imports as props and leaves in the content browser
//! and `roadgen.carla_furniture()` stands in the level. Getting from the written
//! folder to a level that runs is half a dozen steps in the right order with the
//! right environment, and the second time through anyone would have scripted them. So the exporter writes the script, next to the descriptor, with
//! everything it knows baked in: the package and map names, the sun, whether the
//! package wears its own textures, and — when the caller said — where CARLA and its
//! engine are.
//!
//! It is Python rather than shell because the steps are Python's already
//! (`roadgen.fetch_textures`, `roadgen.carla_sky`, CARLA's own `Import.py`), and
//! because a package has to build on Windows too. It does what a careful hand
//! would: it copies the package into CARLA's `Import/` unless it is already there;
//! it moves every other `.json` under `Import/` out of the way while `Import.py`
//! runs, because that script imports every descriptor it finds, and puts them back
//! afterwards, whatever happened; and it says how to start the server at the end
//! rather than starting it, unless asked.

use crate::PackageConfig;

/// The script, as text. `config.carla_root` and `config.engine` become its
/// defaults; without them it takes both from its arguments or the environment.
/// `furniture` is whether the package has any lights or signs to place.
pub fn script(config: &PackageConfig, furniture: bool) -> String {
    let python_string = |value: Option<&str>| match value {
        Some(text) => format!("{text:?}"),
        None => "None".to_owned(),
    };
    format!(
        r#"#!/usr/bin/env python3
"""Written by roadgen beside the package for {map}. Imports it into a CARLA UE5
checkout and gives the level a sky.

    python {package}.py [--carla ROOT] [--engine DIR] [--launch]

`--carla` is the CARLA checkout (the one with Unreal/CarlaUnreal in it) and
`--engine` the Unreal Engine it was built against. Each is taken from the option,
else from the environment (CARLA_ROOT, CARLA_UNREAL_ENGINE_PATH), else from what
the package was exported with. Run it with the interpreter that has `roadgen` and
CARLA's own `carla` module: CARLA's Import.py needs the second, and is run with
the same one.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

PACKAGE = {package:?}
MAP = {map:?}
CARLA_ROOT = {carla_root}
ENGINE = {engine}
SUN_ALTITUDE = {sun_altitude}
SUN_AZIMUTH = {sun_azimuth}
#: Whether the package wears its own textures rather than CARLA's materials, and
#: so needs them fetched.
OWN_TEXTURES = {own_textures}
#: Whether the package has traffic lights, signs or a town of props to stand in
#: the level.
FURNITURE = {furniture}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--carla", default=os.environ.get("CARLA_ROOT", CARLA_ROOT), help="the CARLA checkout")
    parser.add_argument("--engine", default=os.environ.get("CARLA_UNREAL_ENGINE_PATH", ENGINE), help="the Unreal Engine directory")
    parser.add_argument("--no-textures", action="store_true", help="do not fetch the package's textures")
    parser.add_argument("--no-sky", action="store_true", help="leave the level without a sky")
    parser.add_argument("--no-furniture", action="store_true", help="leave the lights and signs unplaced")
    parser.add_argument("--launch", action="store_true", help="start the CARLA server on the map afterwards")
    args = parser.parse_args(argv)
    if not args.carla or not args.engine:
        parser.error("say where CARLA and its engine are: --carla and --engine, or CARLA_ROOT and CARLA_UNREAL_ENGINE_PATH")
    carla = Path(args.carla).resolve()
    engine = Path(args.engine).resolve()
    uproject = carla / "Unreal" / "CarlaUnreal" / "CarlaUnreal.uproject"
    importer = carla / "Util" / "Tools" / "Import.py"
    if not uproject.exists():
        sys.exit("no CarlaUnreal.uproject under %s" % carla)
    if not importer.exists():
        sys.exit("no Util/Tools/Import.py under %s: is this the UE5 branch?" % carla)

    try:
        import roadgen
    except ImportError:
        sys.exit("this needs the `roadgen` package in the interpreter it is run with")

    here = Path(__file__).resolve().parent
    if OWN_TEXTURES and not args.no_textures:
        # Not fatal: a package without them imports with flat colours in their place.
        try:
            fetched = roadgen.fetch_textures(str(here / MAP))
            print("%d textures fetched" % len(fetched))
        except roadgen.TextureError as error:
            print("warning: %s" % error, file=sys.stderr)

    # Where Import.py looks. A package exported straight into Import/ is there already.
    import_dir = carla / "Import"
    if here != import_dir:
        print("copying the package into %s" % import_dir)
        shutil.rmtree(import_dir / MAP, ignore_errors=True)
        shutil.copytree(here / MAP, import_dir / MAP)
        shutil.copy2(here / (PACKAGE + ".json"), import_dir / (PACKAGE + ".json"))

    # What an earlier import of this package left in CARLA's content tree. It has
    # to go: Unreal will not create the map's mesh assets in a folder that already
    # holds a level of the map's name, so a second Import.py over the first fails
    # the whole map group — and logs it, in the middle of a few thousand lines,
    # while the props, the .xodr and the level are all replaced. The result is a
    # level whose road is the old one and whose everything else is new.
    content = carla / "Unreal" / "CarlaUnreal" / "Content" / PACKAGE
    if content.exists():
        print("removing the previous import at %s" % content)
        shutil.rmtree(content)

    # Pedestrians. Import.py builds their navigation mesh with RecastBuilder, from
    # an .obj of the map, in Util/DockerUtils/dist — where the UE5 branch's build
    # puts neither the tool nor the converter that made the .obj from the FBX. The
    # package brings its own .obj; the tool is found where CMake built it.
    dist = carla / "Util" / "DockerUtils" / "dist"
    recast = dist / "RecastBuilder"
    if not recast.exists():
        built = sorted(carla.glob("Build/*/_deps/recastnavigation-build/RecastBuilder/RecastBuilder*"))
        built = [path for path in built if path.is_file() and os.access(str(path), os.X_OK)]
        if built:
            shutil.copy2(built[0], recast)
            print("staged %s" % built[0])
        else:
            print("warning: no RecastBuilder under %s/Build, so no pedestrian navigation will be built" % carla, file=sys.stderr)
    shutil.copy2(import_dir / MAP / (MAP + ".obj"), dist / (MAP + ".obj"))

    # Import.py imports every .json it finds under Import/, other packages'
    # descriptors included. Everything but this one steps aside until it is done.
    descriptor = import_dir / (PACKAGE + ".json")
    aside = [path for path in import_dir.rglob("*.json") if path != descriptor]
    for path in aside:
        path.rename(path.with_name(path.name + ".roadgen-aside"))
    try:
        print("importing %s (%s) into %s" % (PACKAGE, MAP, carla))
        environment = dict(os.environ, CARLA_UNREAL_ENGINE_PATH=str(engine))
        subprocess.run([sys.executable, str(importer), "--package", PACKAGE], cwd=str(carla), env=environment, check=True)
    finally:
        for path in aside:
            path.with_name(path.name + ".roadgen-aside").rename(path)

    nav = carla / "Unreal" / "CarlaUnreal" / "Content" / PACKAGE / "Maps" / MAP / "Nav" / (MAP + ".bin")
    if nav.exists():
        print("pedestrian navigation built: %s" % nav)
    else:
        print("warning: no pedestrian navigation was built (%s is missing)" % nav, file=sys.stderr)

    if not args.no_sky:
        print("adding a sky to %s" % MAP)
        roadgen.carla_sky(str(carla), PACKAGE, MAP, engine=str(engine), sun_altitude=SUN_ALTITUDE, sun_azimuth=SUN_AZIMUTH)

    # The lights and signs: props Import.py brought in and did not place, and the
    # map_logic.json beside the .xodr that makes CARLA adopt them as its own.
    if FURNITURE and not args.no_furniture:
        print("standing the lights and signs in %s" % MAP)
        lights, signs, buildings = roadgen.carla_furniture(str(carla), PACKAGE, MAP, engine=str(engine))
        print("%d lights, %d signs and %d buildings placed" % (lights, signs, buildings))

    editor = engine / "Engine" / "Binaries" / ("Win64" if os.name == "nt" else "Linux") / "UnrealEditor"
    level = "/Game/%s/Maps/%s/%s" % (PACKAGE, MAP, MAP)
    command = [str(editor), str(uproject), level, "-game", "-vulkan", "-carla-rpc-port=2000", "-RenderOffScreen"]
    print()
    print("%s is imported. To run it:" % MAP)
    print("  " + " ".join(command))
    print('and from a client, client.load_world("%s").' % MAP)
    if args.launch:
        subprocess.run(command, cwd=str(uproject.parent), check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
"#,
        package = config.package,
        map = config.map,
        carla_root = python_string(config.carla_root.as_deref()),
        engine = python_string(config.engine.as_deref()),
        sun_altitude = config.sun_altitude,
        sun_azimuth = config.sun_azimuth,
        own_textures = if config.use_carla_materials {
            "False"
        } else {
            "True"
        },
        furniture = if furniture { "True" } else { "False" },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_script_carries_what_the_package_was_exported_with() {
        let config = PackageConfig::new("Town01")
            .with_package("Pkg")
            .with_carla_materials(false)
            .with_carla("/opt/carla", "/opt/ue5");
        let text = script(&config, true);
        assert!(text.starts_with("#!/usr/bin/env python3\n"));
        assert!(text.contains("PACKAGE = \"Pkg\"") && text.contains("MAP = \"Town01\""));
        assert!(
            text.contains("CARLA_ROOT = \"/opt/carla\"") && text.contains("ENGINE = \"/opt/ue5\"")
        );
        assert!(text.contains("OWN_TEXTURES = True"));
        assert!(text.contains("SUN_ALTITUDE = 45") && text.contains("roadgen.carla_sky("));
        assert!(text.contains("FURNITURE = True") && text.contains("roadgen.carla_furniture("));
    }

    #[test]
    fn a_package_in_carlas_materials_fetches_nothing_and_asks_where_carla_is() {
        let text = script(&PackageConfig::new("Town01"), false);
        assert!(text.contains("OWN_TEXTURES = False"));
        assert!(text.contains("FURNITURE = False"));
        assert!(text.contains("CARLA_ROOT = None") && text.contains("ENGINE = None"));
    }

    #[test]
    fn a_windows_path_survives_as_a_python_string() {
        let config = PackageConfig::new("Town01").with_carla("C:\\carla", "C:\\UE_5.5");
        let text = script(&config, false);
        assert!(text.contains("CARLA_ROOT = \"C:\\\\carla\""), "{text}");
    }
}
