"""Standing an imported CARLA map's traffic lights and signs in its level.

`Import.py` places a map's own meshes — the road, the paint, the pavements, the
land — and nothing else: a prop is imported into the content browser and left
there. So the lights and signs `export_carla()` writes, which are props, are
placed by this: one more run of the editor after `Import.py`'s four, reading the
`furniture.manifest` the package carries, spawning each prop at the foot of its
post facing its traffic, and saving the level.

It does three more things while it is there, each of which is what makes CARLA
adopt the furniture as its own rather than spawn a second set:

- it copies the package's `map_logic.carla` beside the `.xodr` CARLA reads, as the
  `map_logic.json` CARLA looks for — the file whose presence stops
  `ATrafficLightManager` spawning its blueprints and makes it turn the actor at
  each signal into a working light instead (the package cannot carry it under
  that name: `Import.py` takes every `.json` under `Import/` for a package);
- it gives each light's three lamp slots an instance of CARLA's own
  `M_TrafficLights`, whose `Emissive Intensity` is what that light drives;
- it stands a bare `ATrafficSignBase` in the matching state at each stop, yield
  and speed-limit sign, which is what `SpawnSignals` looks for before spawning a
  plate of its own;
- and, when the town was exported as props (`buildings="props"`), it stands every
  building in the level too — tagged `Buildings`, which is the one thing a
  building inside the map's own FBX can never be.

    roadgen.carla_furniture("/opt/carla", "Town01Package", "Town01")
    python -m roadgen furniture /opt/carla Town01Package Town01   # from a shell

Running it twice places once: what an earlier run put in the level is removed
before the manifest is read again.
"""

from __future__ import annotations

import os
import shutil

from ._editor import editor_paths, run_editor_script

__all__ = ["carla_furniture", "CarlaFurnitureError"]

#: The editor-side script, run inside Unreal's Python.
EDITOR_SCRIPT = os.path.join(os.path.dirname(__file__), "_unreal_furniture.py")


class CarlaFurnitureError(RuntimeError):
    """The editor could not be run, or it ran and the level was not changed."""


def carla_furniture(
    carla_root,
    package,
    map_name,
    *,
    engine=None,
    manifest=None,
    log=None,
):
    """Places the furniture of `map_name` in `package` in the level `Import.py` built.

    `carla_root` is the CARLA checkout, the one with `Unreal/CarlaUnreal` in it.
    `engine` is the Unreal Engine directory; when it is not given,
    `CARLA_UNREAL_ENGINE_PATH` is read, which is what CARLA's own scripts read.
    `manifest` is the package's `furniture.manifest`; by default the one under the
    checkout's `Import/`, which is where the package was imported from.

    Returns `(lights, signs, buildings)`: how many of each were placed. `log`, if
    given, receives the editor's output line by line.
    """
    editor, uproject = editor_paths(carla_root, engine, CarlaFurnitureError)
    if manifest is None:
        manifest = os.path.join(carla_root, "Import", map_name, "furniture.manifest")
    manifest = os.path.abspath(manifest)
    if not os.path.exists(manifest):
        raise CarlaFurnitureError("no furniture manifest at %s" % manifest)

    # The file CARLA's `InitializeTrafficLights` looks for beside the .xodr —
    # which Import.py copied and this did not, since Import.py knows nothing of it.
    logic = os.path.join(os.path.dirname(manifest), "map_logic.carla")
    if os.path.exists(logic):
        opendrive = os.path.join(
            carla_root, "Unreal", "CarlaUnreal", "Content", package, "Maps", map_name, "OpenDrive"
        )
        if not os.path.isdir(opendrive):
            raise CarlaFurnitureError(
                "no %s: has Import.py imported %s?" % (opendrive, package)
            )
        shutil.copy2(logic, os.path.join(opendrive, "map_logic.json"))

    level = "/Game/%s/Maps/%s/%s" % (package, map_name, map_name)
    lines, returncode, succeeded = run_editor_script(
        editor,
        uproject,
        EDITOR_SCRIPT,
        {
            "ROADGEN_LEVEL": level,
            "ROADGEN_PACKAGE": package,
            "ROADGEN_MAP": map_name,
            "ROADGEN_FURNITURE": manifest,
        },
        log=log,
    )
    placed = None
    saved = None
    for line in lines:
        if "roadgen-furniture: placed" in line:
            words = line.split("roadgen-furniture: placed", 1)[1].split()
            try:
                placed = (int(words[0]), int(words[3]), int(words[-2]))
            except (IndexError, ValueError):
                placed = None
        if "roadgen-furniture: saved" in line:
            saved = "True" in line
    if not succeeded:
        raise CarlaFurnitureError(
            "the editor did not run the furniture script to completion (exit %s); "
            "run it with log=print to see why" % returncode
        )
    if saved is False:
        raise CarlaFurnitureError("the editor ran but could not save %s" % level)
    return placed or (0, 0, 0)


def main(argv=None):
    """The command: `python -m roadgen furniture <carla_root> <package> <map>`."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m roadgen furniture",
        description="Stand an imported CARLA map's traffic lights and signs in its level.",
    )
    parser.add_argument("carla_root", help="the CARLA checkout")
    parser.add_argument("package", help="the package name, as given to Import.py")
    parser.add_argument("map_name", help="the map inside it")
    parser.add_argument("--engine", help="Unreal Engine directory (default: $CARLA_UNREAL_ENGINE_PATH)")
    parser.add_argument("--manifest", help="the package's furniture.manifest (default: the one under Import/)")
    parser.add_argument("--verbose", action="store_true", help="show the editor's output")
    args = parser.parse_args(argv)
    lights, signs, buildings = carla_furniture(
        args.carla_root,
        args.package,
        args.map_name,
        engine=args.engine,
        manifest=args.manifest,
        log=print if args.verbose else None,
    )
    print("%d lights, %d signs and %d buildings placed in %s/%s" % (lights, signs, buildings, args.package, args.map_name))
    return 0
