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

What it does not do is hook the sky into CARLA's weather API: `set_weather()` will
store its parameters and move nothing, because the thing it knows how to move is
`BP_Carla_Sky`. The sun's position is an argument here instead.
"""

from __future__ import annotations

import os
import subprocess
import sys

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
    sun_lux=75000.0,
    log=None,
):
    """Adds a daylight sky to the level `Import.py` built for `map_name` in `package`.

    `carla_root` is the CARLA checkout, the one with `Unreal/CarlaUnreal` in it.
    `engine` is the Unreal Engine directory; when it is not given,
    `CARLA_UNREAL_ENGINE_PATH` is read, which is what CARLA's own scripts read.

    `sun_altitude` and `sun_azimuth` are degrees, the way `carla.WeatherParameters`
    spells them. `log`, if given, receives the editor's output line by line.

    Returns the path of the editor log the run wrote.
    """
    engine = engine or os.environ.get("CARLA_UNREAL_ENGINE_PATH")
    if not engine:
        raise CarlaSkyError(
            "no Unreal Engine given and CARLA_UNREAL_ENGINE_PATH is not set"
        )
    editor = os.path.join(
        engine,
        "Engine",
        "Binaries",
        "Win64" if os.name == "nt" else "Linux",
        "UnrealEditor",
    )
    if not os.path.exists(editor) and not os.path.exists(editor + ".exe"):
        raise CarlaSkyError("no UnrealEditor at %s" % editor)
    uproject = os.path.join(carla_root, "Unreal", "CarlaUnreal", "CarlaUnreal.uproject")
    if not os.path.exists(uproject):
        raise CarlaSkyError("no CarlaUnreal.uproject under %s" % carla_root)
    level = "/Game/%s/Maps/%s/%s" % (package, map_name, map_name)

    environment = dict(os.environ)
    environment.update(
        {
            "ROADGEN_LEVEL": level,
            "ROADGEN_SUN_ALTITUDE": repr(float(sun_altitude)),
            "ROADGEN_SUN_AZIMUTH": repr(float(sun_azimuth)),
            "ROADGEN_SUN_LUX": repr(float(sun_lux)),
        }
    )
    command = [
        editor,
        uproject,
        "-run=pythonscript",
        "-script=%s" % EDITOR_SCRIPT,
        "-RenderOffScreen",
        "-unattended",
        "-nosourcecontrol",
        "-nopause",
    ]
    process = subprocess.Popen(
        command,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )
    succeeded = False
    changed = None
    log_path = None
    for line in process.stdout:
        if log is not None:
            log(line.rstrip("\n"))
        if "Python script executed successfully" in line:
            succeeded = True
        if "roadgen-sky: saved" in line:
            changed = "True" in line
        if "LogInit: Display: Log file: " in line or "Log file open, " in line:
            log_path = line.strip()
    process.wait()
    if not succeeded:
        raise CarlaSkyError(
            "the editor did not run the sky script to completion (exit %s); "
            "run it with log=print to see why" % process.returncode
        )
    if changed is False:
        raise CarlaSkyError("the editor ran but could not save %s" % level)
    return log_path


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m roadgen.sky",
        description="Give an imported CARLA map a daylight sky.",
    )
    parser.add_argument("carla_root", help="the CARLA checkout")
    parser.add_argument("package", help="the package name, as given to Import.py")
    parser.add_argument("map_name", help="the map inside it")
    parser.add_argument("--engine", help="Unreal Engine directory (default: $CARLA_UNREAL_ENGINE_PATH)")
    parser.add_argument("--sun-altitude", type=float, default=45.0, help="degrees above the horizon")
    parser.add_argument("--sun-azimuth", type=float, default=-50.0, help="degrees, as CARLA's weather spells it")
    parser.add_argument("--sun-lux", type=float, default=75000.0)
    parser.add_argument("--verbose", action="store_true", help="show the editor's output")
    args = parser.parse_args(argv)
    carla_sky(
        args.carla_root,
        args.package,
        args.map_name,
        engine=args.engine,
        sun_altitude=args.sun_altitude,
        sun_azimuth=args.sun_azimuth,
        sun_lux=args.sun_lux,
        log=print if args.verbose else None,
    )
    print("sky added to %s/%s" % (args.package, args.map_name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
