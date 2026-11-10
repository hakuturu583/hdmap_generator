"""Putting an imported map's level into a CARLA content pack, where CARLA serves it.

A content pack keeps a map's OpenDRIVE, pedestrian navigation and Traffic Manager
data as loose files in its own `Content/Maps/{OpenDrive,Nav,TM}/`, and the server
hands them to a client by looking beside the level: `get_required_files` searches
the folder the level is in. The two only meet when the level sits straight in the
pack's `Maps/` folder, at `/<Pack>/Maps/<Map>`:

| Where the level is | What the client gets |
| --- | --- |
| `/<Pack>/Maps/<Map>` | the xodr, the navigation, the Traffic Manager cache |
| `/<Pack>/Maps/<Map>/<Map>` (`carla-pack add --import`) | nothing |
| `/Game/<Package>/Maps/<Map>/<Map>` (`carla-pack add --map` in place) | nothing |

Given nothing, `get_random_location_from_navigation()` returns `None` every time, so
no pedestrian can walk the map, and the Traffic Manager rebuilds its map on every
load. Registering a level in place is meant for CARLA's own towns, whose files the
base package carries anyway; a roadgen map is never one of them.

So this saves the level `Import.py` built — in `/Game/<Package>/Maps/<Map>/` — under
the pack's mount point at `/<Pack>/Maps/<Map>`, with the editor's Save As, and
registers it with CARLA's `carla-pack`, sidecars and all:

    roadgen.carla_pack_level("/opt/carla", "HighwayPackage", "Highway", "HighwayPack")
    python -m roadgen pack-level /opt/carla HighwayPackage Highway HighwayPack
    # then: carla-pack build HighwayPack

The pack is made with `carla-pack init` when there is none yet. The level keeps
referring to its meshes and materials under `/Game/<Package>/`, which the cook
takes into the pack along with it; the level `Import.py` built stays where it is,
so the map still opens from the editor as before.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

from ._editor import editor_paths, run_editor_script

__all__ = ["carla_pack_level", "CarlaPackLevelError"]

#: The editor-side script, run inside Unreal's Python.
EDITOR_SCRIPT = os.path.join(os.path.dirname(__file__), "_unreal_pack_level.py")


class CarlaPackLevelError(RuntimeError):
    """The level could not be saved into the pack, or `carla-pack` refused it."""


def carla_pack_level(carla_root, package, map_name, pack, *, engine=None, log=None):
    """Saves the level `Import.py` built for `map_name` in `package` into the content
    pack `pack`, at `/<pack>/Maps/<map_name>`, and registers it there.

    `carla_root` is the CARLA checkout, the one with `Unreal/CarlaUnreal` and
    `Util/ContentPacks/carla_pack.py` in it; the pack is its
    `Unreal/CarlaUnreal/Plugins/Packs/<pack>`, made with `carla-pack init` if it is
    not there. `engine` is the Unreal Engine directory; when it is not given,
    `CARLA_UNREAL_ENGINE_PATH` is read. `log`, if given, receives the editor's and
    `carla-pack`'s output line by line.

    The map's OpenDRIVE (with its `map_logic.json`), navigation and Traffic Manager
    data are taken from beside the imported level and copied into the pack by
    `carla-pack add`. Returns the level's package path in the pack.
    """
    editor, uproject = editor_paths(carla_root, engine, CarlaPackLevelError)
    engine = engine or os.environ["CARLA_UNREAL_ENGINE_PATH"]
    tool = os.path.join(carla_root, "Util", "ContentPacks", "carla_pack.py")
    if not os.path.isfile(tool):
        raise CarlaPackLevelError("no Util/ContentPacks/carla_pack.py under %s: content packs need CARLA 1.0" % carla_root)

    content = os.path.join(carla_root, "Unreal", "CarlaUnreal", "Content")
    imported = os.path.join(content, package, "Maps", map_name)
    if not os.path.isfile(os.path.join(imported, map_name + ".umap")):
        raise CarlaPackLevelError("no level %s/%s.umap: import the package first" % (imported, map_name))
    xodr = os.path.join(imported, "OpenDrive", map_name + ".xodr")
    if not os.path.isfile(xodr):
        raise CarlaPackLevelError("no OpenDRIVE at %s" % xodr)

    def carla_pack(*arguments):
        environment = dict(os.environ, CARLA_UNREAL_ENGINE_PATH=engine)
        process = subprocess.run(
            [sys.executable, tool] + list(arguments),
            cwd=carla_root,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            check=False,
        )
        for line in process.stdout.splitlines():
            if log is not None:
                log(line)
        if process.returncode != 0:
            raise CarlaPackLevelError("carla-pack %s failed:\n%s" % (" ".join(arguments), process.stdout))

    pack_dir = os.path.join(carla_root, "Unreal", "CarlaUnreal", "Plugins", "Packs", pack)
    if not os.path.isfile(os.path.join(pack_dir, "carla-pack.json")):
        carla_pack("init", pack)

    # A level of the same name an earlier run saved: Save As will not write over it.
    target = os.path.join(pack_dir, "Content", "Maps", map_name + ".umap")
    for path in (target, os.path.join(pack_dir, "Content", "Maps", map_name + "_BuiltData.uasset")):
        if os.path.exists(path):
            os.remove(path)

    level = "/Game/%s/Maps/%s/%s" % (package, map_name, map_name)
    pack_level = "/%s/Maps/%s" % (pack, map_name)
    source = os.path.join(imported, map_name + ".umap")
    with open(source, "rb") as handle:
        original = handle.read()
    with _mountable(os.path.join(pack_dir, pack + ".uplugin")):
        try:
            lines, returncode, succeeded = run_editor_script(
                editor,
                uproject,
                EDITOR_SCRIPT,
                {"ROADGEN_LEVEL": level, "ROADGEN_PACK_LEVEL": pack_level, "ROADGEN_PACK": pack},
                log=log,
                arguments=["-EnablePlugins=%s" % pack],
            )
        finally:
            # Save As saves the level it started from too; Import.py's is put back.
            with open(source, "rb") as handle:
                if handle.read() != original:
                    with open(source, "wb") as rewrite:
                        rewrite.write(original)
    saved = any("roadgen-pack-level: saved True" in line for line in lines)
    if not succeeded or not saved or not os.path.isfile(target):
        raise CarlaPackLevelError(
            "the editor did not save %s as %s (exit %s); run it with log=print to see why"
            % (level, pack_level, returncode)
        )

    arguments = ["add", pack, "--map", target, "--xodr", xodr]
    nav = os.path.join(imported, "Nav", map_name + ".bin")
    if os.path.isfile(nav):
        arguments += ["--nav", nav]
    tm = os.path.join(imported, "TM")
    if os.path.isdir(tm) and os.listdir(tm):
        arguments += ["--tm", tm]
    carla_pack(*arguments)
    return pack_level


class _mountable:
    """The pack's descriptor with `ExplicitlyLoaded` off while the editor runs.

    A pack is authored explicitly loaded, which is what lets a packaged server
    mount it on demand, and which the editor's plugin manager skips — so
    `-EnablePlugins` would not mount it. `carla-pack` does the same around its own
    editor runs; the descriptor is put back as it was, whatever happens.
    """

    def __init__(self, path):
        self.path = path
        self.original = None

    def __enter__(self):
        if not os.path.isfile(self.path):
            raise CarlaPackLevelError("no %s: is this a content pack?" % self.path)
        with open(self.path) as handle:
            self.original = handle.read()
        descriptor = json.loads(self.original)
        if descriptor.get("ExplicitlyLoaded"):
            descriptor["ExplicitlyLoaded"] = False
            with open(self.path, "w") as handle:
                json.dump(descriptor, handle, indent="\t")
        return self

    def __exit__(self, *exc):
        with open(self.path, "w") as handle:
            handle.write(self.original)
        return False


def main(argv=None):
    """The command: `python -m roadgen pack-level <carla_root> <package> <map> <pack>`."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m roadgen pack-level",
        description="Save an imported CARLA map's level into a content pack, at /<Pack>/Maps/<Map>.",
    )
    parser.add_argument("carla_root", help="the CARLA checkout")
    parser.add_argument("package", help="the package name, as given to Import.py")
    parser.add_argument("map_name", help="the map inside it")
    parser.add_argument("pack", help="the content pack, made with carla-pack init if there is none")
    parser.add_argument("--engine", help="Unreal Engine directory (default: $CARLA_UNREAL_ENGINE_PATH)")
    parser.add_argument("--verbose", action="store_true", help="show the editor's and carla-pack's output")
    args = parser.parse_args(argv)
    level = carla_pack_level(
        args.carla_root,
        args.package,
        args.map_name,
        args.pack,
        engine=args.engine,
        log=print if args.verbose else None,
    )
    print("%s is in %s; build it with: carla-pack build %s" % (level, args.pack, args.pack))
    return 0
