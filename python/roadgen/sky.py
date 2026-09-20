"""Giving an imported CARLA map a sky.

CARLA's importer builds every imported map on its `BaseMap`, and in the UE5 branch
that level has a sun and a sky light and nothing for either to light up: no sky
atmosphere, no fog, no exposure. A map imported through `Import.py` renders its
surfaces and, above them, black. CARLA's own towns get their sky from a
`BP_Carla_Sky` actor that its weather system drives, but that actor is not in the
`BaseMap`, and dropping one into an imported level does not give it the daylight
the towns have — those are lit by per-map weather sublevels with baked lighting
(`Town10HD_Opt/Weathers/T10HD_Day`), which an imported map has not got.

So this adds a plain Unreal daylight instead, with the editor's own scripting: a
sky atmosphere and height fog, a physical sun (75 000 lux, which is what the
atmosphere is calibrated for) in place of `BaseMap`'s dim one, a sky light that
captures the sky it now has, and an unbound post-process volume that turns auto
exposure on — the project leaves it off, and without it a physical sun is a white
frame. It is one more run of the editor after `Import.py`'s four, on the same
level, and running it twice changes nothing.

    roadgen.carla_sky("/opt/carla", "Town01Package", "Town01")
    python -m roadgen sky /opt/carla Town01Package Town01     # the same, from a shell

What it does not do is hook the sky into CARLA's weather API: `set_weather()` will
store its parameters and move nothing, because the thing it knows how to move is
`BP_Carla_Sky`. The sun's position is an argument here instead.
"""

from __future__ import annotations

import os

from ._editor import editor_paths, run_editor_script

__all__ = ["carla_sky", "CarlaSkyError"]

#: The editor-side script, run inside Unreal's Python.
EDITOR_SCRIPT = os.path.join(os.path.dirname(__file__), "_unreal_sky.py")


class CarlaSkyError(RuntimeError):
    """The editor could not be run, or it ran and the level was not changed."""


def carla_sky(
    carla_root,
    package,
    map_name,
    *,
    engine=None,
    sun_altitude=45.0,
    sun_azimuth=-50.0,
    log=None,
):
    """Adds a daylight sky to the level `Import.py` built for `map_name` in `package`.

    `carla_root` is the CARLA checkout, the one with `Unreal/CarlaUnreal` in it.
    `engine` is the Unreal Engine directory; when it is not given,
    `CARLA_UNREAL_ENGINE_PATH` is read, which is what CARLA's own scripts read.

    `sun_altitude` and `sun_azimuth` are degrees, the way `carla.WeatherParameters`
    spells them. `log`, if given, receives the editor's output line by line.
    """
    editor, uproject = editor_paths(carla_root, engine, CarlaSkyError)
    level = "/Game/%s/Maps/%s/%s" % (package, map_name, map_name)
    lines, returncode, succeeded = run_editor_script(
        editor,
        uproject,
        EDITOR_SCRIPT,
        {
            "ROADGEN_LEVEL": level,
            "ROADGEN_SUN_ALTITUDE": repr(float(sun_altitude)),
            "ROADGEN_SUN_AZIMUTH": repr(float(sun_azimuth)),
        },
        log=log,
    )
    changed = None
    for line in lines:
        if "roadgen-sky: saved" in line:
            changed = "True" in line
    if not succeeded:
        raise CarlaSkyError(
            "the editor did not run the sky script to completion (exit %s); "
            "run it with log=print to see why" % returncode
        )
    if changed is False:
        raise CarlaSkyError("the editor ran but could not save %s" % level)


def main(argv=None):
    """The command: `python -m roadgen sky <carla_root> <package> <map>`."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m roadgen sky",
        description="Give an imported CARLA map a daylight sky.",
    )
    parser.add_argument("carla_root", help="the CARLA checkout")
    parser.add_argument("package", help="the package name, as given to Import.py")
    parser.add_argument("map_name", help="the map inside it")
    parser.add_argument("--engine", help="Unreal Engine directory (default: $CARLA_UNREAL_ENGINE_PATH)")
    parser.add_argument("--sun-altitude", type=float, default=45.0, help="degrees above the horizon")
    parser.add_argument("--sun-azimuth", type=float, default=-50.0, help="degrees, as CARLA's weather spells it")
    parser.add_argument("--verbose", action="store_true", help="show the editor's output")
    args = parser.parse_args(argv)
    carla_sky(
        args.carla_root,
        args.package,
        args.map_name,
        engine=args.engine,
        sun_altitude=args.sun_altitude,
        sun_azimuth=args.sun_azimuth,
        log=print if args.verbose else None,
    )
    print("sky added to %s/%s" % (args.package, args.map_name))
    return 0
