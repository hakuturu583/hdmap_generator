"""Giving an imported CARLA map's water the Unreal materials its package names.

`Import.py` imports the map's water as it imports everything else of the land:
meshes in the flat colours the FBX gives them. A flat colour is no water. So the
package's descriptor names, for each map, an Unreal material for running water
and one for standing water, beside the material slot each goes in — and this is
one more run of the editor after `Import.py`'s, putting each material in every
slot of its name on the package's meshes, and on the actors of the map's level,
whose components CARLA's cooking gave the land's material.

The two are apart because the meshes are: a river's texture coordinates run down
it from its source (`u`) and across it (`v`), so a material that pans its normal
map along `u` flows downstream; a lake's or a pond's lie in plan. By default they
are the Unreal Engine Water plugin's river and lake materials; any other — one of
the project's own under `/Game/` — is named by editing the descriptor, or here:

    roadgen.carla_water("/opt/carla", "Town01Package", "Town01")
    roadgen.carla_water("/opt/carla", "Town01Package", "Town01",
                        flowing="/Game/Water/M_River_Panning")
    python -m roadgen water /opt/carla Town01Package Town01   # from a shell

A material that is not in the project is reported and its slots keep their
colour. Running it twice dresses once.
"""

from __future__ import annotations

import json
import os

from ._editor import editor_paths, run_editor_script

__all__ = ["carla_water", "CarlaWaterError"]

#: The editor-side script, run inside Unreal's Python.
EDITOR_SCRIPT = os.path.join(os.path.dirname(__file__), "_unreal_water.py")


class CarlaWaterError(RuntimeError):
    """The editor could not be run, or the descriptor names no water for the map."""


def water_slots(descriptor, map_name, *, flowing=None, still=None):
    """The material each water slot of `map_name` takes, `{slot: material}`, read
    from the package descriptor at `descriptor` and overridden by `flowing` and
    `still` where given."""
    with open(descriptor) as handle:
        maps = json.load(handle).get("maps", [])
    entry = next((entry for entry in maps if entry.get("name") == map_name), None)
    if entry is None or "water" not in entry:
        raise CarlaWaterError("%s names no water for %s" % (descriptor, map_name))
    water = entry["water"]
    slots = {}
    for kind, override in (("flowing", flowing), ("still", still)):
        slot = water.get(kind)
        if slot:
            slots[slot["slot"]] = override or slot["material"]
    return slots


def carla_water(
    carla_root,
    package,
    map_name,
    *,
    engine=None,
    descriptor=None,
    flowing=None,
    still=None,
    log=None,
):
    """Puts the descriptor's water materials on `map_name`'s water in `package`.

    `carla_root` is the CARLA checkout, the one with `Unreal/CarlaUnreal` in it.
    `engine` is the Unreal Engine directory; when it is not given,
    `CARLA_UNREAL_ENGINE_PATH` is read. `descriptor` is the package's `.json`; by
    default the one under the checkout's `Import/`. `flowing` and `still` are
    Unreal material paths that override the descriptor's for running and
    standing water.

    Returns how many meshes were dressed. `log`, if given, receives the editor's
    output line by line.
    """
    editor, uproject = editor_paths(carla_root, engine, CarlaWaterError)
    if descriptor is None:
        descriptor = os.path.join(carla_root, "Import", package + ".json")
    descriptor = os.path.abspath(descriptor)
    if not os.path.exists(descriptor):
        raise CarlaWaterError("no package descriptor at %s" % descriptor)
    slots = water_slots(descriptor, map_name, flowing=flowing, still=still)
    lines, returncode, succeeded = run_editor_script(
        editor,
        uproject,
        EDITOR_SCRIPT,
        {
            "ROADGEN_LEVEL": "/Game/%s/Maps/%s/%s" % (package, map_name, map_name),
            "ROADGEN_PACKAGE": package,
            "ROADGEN_MAP": map_name,
            "ROADGEN_WATER": json.dumps(slots),
        },
        log=log,
    )
    dressed = 0
    for line in lines:
        if "roadgen-water: dressed" in line:
            words = line.split("roadgen-water: dressed", 1)[1].split()
            try:
                dressed = int(words[0])
            except (IndexError, ValueError):
                dressed = 0
    if not succeeded:
        raise CarlaWaterError(
            "the editor did not run the water script to completion (exit %s); "
            "run it with log=print to see why" % returncode
        )
    return dressed


def main(argv=None):
    """The command: `python -m roadgen water <carla_root> <package> <map>`."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m roadgen water",
        description="Give an imported CARLA map's water the Unreal materials its package names.",
    )
    parser.add_argument("carla_root", help="the CARLA checkout")
    parser.add_argument("package", help="the package name, as given to Import.py")
    parser.add_argument("map_name", help="the map inside it")
    parser.add_argument("--engine", help="Unreal Engine directory (default: $CARLA_UNREAL_ENGINE_PATH)")
    parser.add_argument("--descriptor", help="the package's .json (default: the one under Import/)")
    parser.add_argument("--flowing", help="the material for rivers, overriding the descriptor's")
    parser.add_argument("--still", help="the material for lakes and ponds, overriding the descriptor's")
    parser.add_argument("--verbose", action="store_true", help="show the editor's output")
    args = parser.parse_args(argv)
    dressed = carla_water(
        args.carla_root,
        args.package,
        args.map_name,
        engine=args.engine,
        descriptor=args.descriptor,
        flowing=args.flowing,
        still=args.still,
        log=print if args.verbose else None,
    )
    print("%d water meshes dressed in %s/%s" % (dressed, args.package, args.map_name))
    return 0
