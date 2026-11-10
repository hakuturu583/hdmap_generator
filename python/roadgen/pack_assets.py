"""Putting a map in every format roadgen writes into a CARLA content pack.

A CARLA content pack carries a map for the simulator: the cooked level, and beside
it the files the server reads loose — the OpenDRIVE, the pedestrians' navigation
mesh, the Traffic Manager's cache. A pack built from a roadgen map can carry the
rest of the map too, so that whoever installs it has the same road network for
Autoware (Lanelet2, the point cloud map and its projector), for SUMO, CommonRoad,
ClipGT, RDS-HQ, GPUDrive and WOMD, all from the one map and in one download:

    m = ...                                            # a roadgen.Map
    m.export_carla("Import/", name="Highway")          # the level, imported as usual
    # python Import/HighwayPackage.py --pack HighwayPack  (the level into the pack), then:
    roadgen.carla_pack_assets(m, "Unreal/CarlaUnreal/Plugins/Packs/HighwayPack",
                              "Highway")
    # carla-pack build HighwayPack

What it writes, inside the pack folder:

    Content/HDMaps/<map>/<format>/...       every format, one folder each
    Content/Config/roadgen_asset.json       what is where, for tools to look up
    Config/PluginSettings.ini               `Content/HDMaps` staged as loose files

`Content/Config` is staged by CARLA's own settings already; `Content/HDMaps` is
added to them here, so the cook copies both into the built pack as they are and
`carla-pack build` lists every file, with its SHA-256, in the pack's
`carla-pack.json`. The OpenDRIVE is not written twice: the asset file points at the
one `carla-pack add` put beside the level, when there is one.

The level has to be in the pack at `/<Pack>/Maps/<Map>`, where
`roadgen.carla_pack_level()` (which `--pack` runs) puts it: CARLA hands a client
the map's navigation and Traffic Manager files from beside the level, and from
anywhere else it hands none. This warns when the pack registers the level elsewhere.

`roadgen_asset.json` is optional to the pack and to anything that reads one: a pack
without it is an ordinary CARLA pack, with the formats its `carla-pack.json` lists.
Its paths are relative to the pack folder, the one holding `carla-pack.json`:

    {
      "schema": 1,
      "generator": {"name": "roadgen", "version": "0.43.0"},
      "maps": {
        "Highway": {
          "projection": "local_cartesian",
          "formats": {
            "lanelet2": {"type": "file",
                         "path": "Content/HDMaps/Highway/autoware/lanelet2_map.osm"},
            "autoware": {"type": "directory",
                         "path": "Content/HDMaps/Highway/autoware",
                         "files": {"lanelet2_map": "...", "pointcloud_map": "...",
                                   "map_projector_info": "..."}},
            ...
          }
        }
      }
    }

Running it again for the same map replaces that map's entry and its folder; other
maps of the pack keep theirs.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
import warnings

__all__ = [
    "ASSET_FILE",
    "ASSET_PATH",
    "FORMATS",
    "HDMAPS_DIR",
    "CarlaPackAssetsError",
    "carla_pack_assets",
    "merge_pcd",
]

#: The asset file's name, and where it is in a pack folder.
ASSET_FILE = "roadgen_asset.json"
ASSET_PATH = "Content/Config/" + ASSET_FILE
#: Where the formats go in a pack folder; staged as loose files.
HDMAPS_DIR = "Content/HDMaps"
#: The asset file's schema version.
SCHEMA = 1
#: Every format, in the order they are written and listed.
FORMATS = (
    "opendrive",
    "lanelet2",
    "pointcloud",
    "autoware",
    "osm",
    "sumo",
    "commonroad",
    "commonroad_map",
    "clipgt",
    "rdshq",
    "gpudrive",
    "womd",
    "roadgen_proto",
    "roadgen_ir",
)

_STAGE_SECTION = "[StageSettings]"
_STAGE_LINE = "+AdditionalNonUSFDirectories=" + HDMAPS_DIR


class CarlaPackAssetsError(RuntimeError):
    """The pack folder or the map could not be given its formats."""


#: How far from the nearest road the point cloud map reaches by default, metres:
#: past what a LiDAR on the road sees. The land of a map with a stated terrain
#: runs to the edge of the map, kilometres from the roads, and a cloud of all of
#: it is hundreds of megabytes of points no vehicle can see.
POINTCLOUD_REACH = 150.0


def carla_pack_assets(
    m,
    pack_dir,
    map_name=None,
    *,
    formats=None,
    xodr=None,
    pointcloud_reach=POINTCLOUD_REACH,
    pointcloud_spacing=None,
):
    """Writes `m` in every format into the content pack at `pack_dir`, lists them in
    the pack's `roadgen_asset.json`, and stages them, and returns the map's entry.

    `pack_dir` is the pack's source folder, the one `carla-pack init` made and that
    holds its `carla-pack.json`. `map_name` is the map's name in the pack, the
    level's; by default the map's own. `formats` limits what is written to those
    of [`FORMATS`] named; by default all of them. `xodr` is the OpenDRIVE the entry
    points at, relative to the pack folder; by default the one `carla-pack add`
    registered for the map, and a fresh one when it registered none.

    The point cloud map is the one large file: `pointcloud_reach` keeps only its
    cells within that many metres of a road (`None` keeps all of them), and
    `pointcloud_spacing` is its points' spacing, `export_pointcloud_map`'s.
    """
    pack_dir = os.path.abspath(pack_dir)
    manifest_path = os.path.join(pack_dir, "carla-pack.json")
    if not os.path.isfile(manifest_path):
        raise CarlaPackAssetsError("%s is not a content pack folder: it has no carla-pack.json" % pack_dir)
    wanted = list(FORMATS) if formats is None else list(formats)
    unknown = [name for name in wanted if name not in FORMATS]
    if unknown:
        raise CarlaPackAssetsError("unknown formats %s; roadgen writes %s" % (unknown, ", ".join(FORMATS)))

    with tempfile.TemporaryDirectory() as scratch:
        # The IR first: it says the map's name and projection, and is a format too.
        ir_path = os.path.join(scratch, "map.ir.json")
        m.export_ir(ir_path)
        with open(ir_path) as handle:
            metadata = json.load(handle).get("metadata", {})
        name = map_name or metadata.get("name") or ""
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
            raise CarlaPackAssetsError("map name %r is not one a pack can hold; pass map_name" % name)
        misplaced = _misplaced_level(manifest_path, name)
        if misplaced:
            warnings.warn(misplaced, stacklevel=2)
        root_rel = "%s/%s" % (HDMAPS_DIR, name)
        root = os.path.join(pack_dir, *root_rel.split("/"))
        if os.path.isdir(root):
            shutil.rmtree(root)
        os.makedirs(root)

        written = {}

        def rel(*parts):
            return "/".join((root_rel,) + parts)

        def path_of(*parts):
            target = os.path.join(root, *parts)
            os.makedirs(os.path.dirname(target), exist_ok=True)
            return target

        def file_entry(*parts):
            return {"type": "file", "path": rel(*parts)}

        def dir_entry(folder, files):
            return {
                "type": "directory",
                "path": rel(folder),
                "files": {role: rel(folder, file) for role, file in sorted(files.items())},
            }

        def listing(folder):
            base = os.path.join(root, folder)
            return {
                os.path.relpath(os.path.join(here, file), base).replace(os.sep, "/")
                for here, _, files in os.walk(base)
                for file in files
            }

        if "opendrive" in wanted:
            source = xodr or _registered_xodr(manifest_path, name)
            if source is None:
                m.export_opendrive(path_of("opendrive", name + ".xodr"), trace=False)
                written["opendrive"] = file_entry("opendrive", name + ".xodr")
            else:
                written["opendrive"] = {"type": "file", "path": source}
        if wanted_any(wanted, "lanelet2", "pointcloud", "autoware"):
            m.export_lanelet2(path_of("autoware", "lanelet2_map.osm"), trace=False)
            tiles = os.path.join(scratch, "pcd")
            m.export_pointcloud_map(tiles, name=name, spacing=pointcloud_spacing)
            keep = None
            if pointcloud_reach is not None:
                keep = _cells_near_roads(
                    os.path.join(tiles, "pointcloud_map_metadata.yaml"),
                    path_of("autoware", "lanelet2_map.osm"),
                    pointcloud_reach,
                )
            merge_pcd(os.path.join(tiles, "pointcloud_map"), path_of("autoware", "pointcloud_map.pcd"), keep)
            with open(path_of("autoware", "map_projector_info.yaml"), "w") as handle:
                handle.write(_projector_info(m, metadata))
            if "lanelet2" in wanted:
                written["lanelet2"] = file_entry("autoware", "lanelet2_map.osm")
            if "pointcloud" in wanted:
                written["pointcloud"] = file_entry("autoware", "pointcloud_map.pcd")
            if "autoware" in wanted:
                written["autoware"] = dir_entry(
                    "autoware",
                    {
                        "lanelet2_map": "lanelet2_map.osm",
                        "pointcloud_map": "pointcloud_map.pcd",
                        "map_projector_info": "map_projector_info.yaml",
                    },
                )
        if "osm" in wanted:
            m.export_osm(path_of("osm", name + ".osm"), trace=False)
            written["osm"] = file_entry("osm", name + ".osm")
        if "sumo" in wanted:
            prefix = m.export_sumo(os.path.join(root, "sumo"), trace=False)
            files = {file.split(".", 1)[1].replace(".", "_"): file for file in listing("sumo")
                     if file.startswith(prefix + ".")}
            written["sumo"] = dir_entry("sumo", files)
        if "commonroad" in wanted:
            m.export_commonroad(path_of("commonroad", name + ".xml"), trace=False)
            written["commonroad"] = file_entry("commonroad", name + ".xml")
        if "commonroad_map" in wanted:
            m.export_commonroad_map(path_of("commonroad_map", name + ".pb"), trace=False)
            written["commonroad_map"] = file_entry("commonroad_map", name + ".pb")
        for kind, export in (("clipgt", m.export_clipgt), ("rdshq", m.export_rdshq)):
            if kind in wanted:
                os.makedirs(os.path.join(root, kind))
                export(os.path.join(root, kind), trace=False)
                written[kind] = dir_entry(kind, {file: file for file in listing(kind)})
        if "gpudrive" in wanted:
            m.export_gpudrive(path_of("gpudrive", name + ".json"), trace=False)
            written["gpudrive"] = file_entry("gpudrive", name + ".json")
        if "womd" in wanted:
            m.export_womd(path_of("womd", name + ".tfrecord"), trace=False)
            written["womd"] = file_entry("womd", name + ".tfrecord")
        if "roadgen_proto" in wanted:
            m.export_proto(path_of("roadgen_proto", name + ".pb"))
            written["roadgen_proto"] = file_entry("roadgen_proto", name + ".pb")
        if "roadgen_ir" in wanted:
            shutil.copyfile(ir_path, path_of("roadgen_ir", name + ".ir.json"))
            written["roadgen_ir"] = file_entry("roadgen_ir", name + ".ir.json")

    entry = {
        "projection": metadata.get("projection", "local_cartesian"),
        "formats": {kind: written[kind] for kind in FORMATS if kind in written},
    }
    _write_assets(pack_dir, name, entry)
    _stage(pack_dir)
    return entry


def wanted_any(wanted, *kinds):
    return any(kind in wanted for kind in kinds)


def merge_pcd(tiles_dir, target, keep=None):
    """Joins a divided point cloud map — binary PCD tiles of the same fields, as
    `export_pointcloud_map` writes — into the one file Autoware's
    `pointcloud_map.pcd` is, and returns how many points it holds. `keep`, if
    given, is the tiles' file names to keep; the rest are left out."""
    header = None
    count = 0
    names = sorted(
        name for name in os.listdir(tiles_dir) if name.endswith(".pcd") and (keep is None or name in keep)
    )
    if not names:
        raise CarlaPackAssetsError("no .pcd tiles in %s" % tiles_dir)
    body = target + ".body"
    with open(body, "wb") as out:
        for name in names:
            with open(os.path.join(tiles_dir, name), "rb") as handle:
                raw = handle.read()
            marker = raw.find(b"DATA binary\n")
            if marker < 0:
                raise CarlaPackAssetsError("%s is not a binary PCD" % name)
            fields = [line for line in raw[:marker].decode("ascii").splitlines()
                      if not line.startswith(("WIDTH", "POINTS", "#"))]
            if header is None:
                header = fields
            elif fields != header:
                raise CarlaPackAssetsError("%s has other fields than the first tile" % name)
            points = re.search(rb"\nPOINTS (\d+)", raw[:marker])
            count += int(points.group(1))
            out.write(raw[marker + len(b"DATA binary\n"):])
    lines = []
    for line in header:
        lines.append(line)
        if line.startswith("HEIGHT"):
            lines.insert(len(lines) - 1, "WIDTH %d" % count)
        if line.startswith("VIEWPOINT"):
            lines.append("POINTS %d" % count)
    text = "# .PCD v0.7 - Point Cloud Data file format\n" + "\n".join(lines) + "\nDATA binary\n"
    with open(target, "wb") as out, open(body, "rb") as data:
        out.write(text.encode("ascii"))
        shutil.copyfileobj(data, out)
    os.remove(body)
    return count


def _cells_near_roads(metadata_path, lanelet2_path, reach):
    """The point cloud tiles within `reach` metres of a road: of every cell in
    `metadata_path` (Autoware's `pointcloud_map_metadata.yaml`), those any node of
    the Lanelet2 map is within `reach` of, by the nodes' `local_x`/`local_y`."""
    cells = {}
    size = 20.0
    with open(metadata_path) as handle:
        for line in handle:
            key, _, value = line.partition(":")
            key = key.strip()
            if key == "x_resolution":
                size = float(value)
            elif key.endswith(".pcd"):
                x, y = (float(v) for v in value.strip().strip("[]").split(","))
                cells[(int(round(x / size)), int(round(y / size)))] = key
    near = set()
    span = int(reach // size) + 1
    pattern = re.compile(r'<tag k="local_x" v="([-0-9.eE]+)"\s*/>\s*<tag k="local_y" v="([-0-9.eE]+)"')
    with open(lanelet2_path) as handle:
        text = handle.read()
    for match in pattern.finditer(text):
        x, y = float(match.group(1)), float(match.group(2))
        cx, cy = int(x // size), int(y // size)
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                # The nearest point of the cell to the node, within reach.
                left, bottom = (cx + dx) * size, (cy + dy) * size
                nx = min(max(x, left), left + size)
                ny = min(max(y, bottom), bottom + size)
                if (nx - x) ** 2 + (ny - y) ** 2 <= reach * reach:
                    near.add((cx + dx, cy + dy))
    return {name for cell, name in cells.items() if cell in near}


def _misplaced_level(manifest_path, name):
    """Why the pack's level for `name` will reach a client without its files, or None.

    CARLA hands a client a map's OpenDRIVE, navigation and Traffic Manager files
    from beside its level, and a pack keeps them in its `Content/Maps/`: only a level
    at `/<Pack>/Maps/<Map>` has them beside it.
    """
    with open(manifest_path) as handle:
        manifest = json.load(handle)
    expected = "/%s/Maps/%s" % (manifest.get("name", ""), name)
    for entry in manifest.get("maps", []):
        if entry.get("name") == name and entry.get("package") and entry["package"] != expected:
            return (
                "the pack registers %s's level at %s, not at %s: a CARLA client gets none of its "
                "navigation or Traffic Manager files from there, so no pedestrian can walk it. "
                "Put the level in the pack with roadgen.carla_pack_level() (or `python -m roadgen "
                "pack-level`) first." % (name, entry["package"], expected)
            )
    return None


def _registered_xodr(manifest_path, name):
    """The OpenDRIVE `carla-pack add` put beside the level, relative to the pack folder."""
    with open(manifest_path) as handle:
        manifest = json.load(handle)
    for entry in manifest.get("maps", []):
        if entry.get("name") == name and entry.get("xodr"):
            return "Content/" + entry["xodr"]
    return None


def _projector_info(m, metadata):
    """Autoware's `map_projector_info.yaml` for the map's projection."""
    projection = metadata.get("projection", "local_cartesian")
    if projection == "mgrs":
        return "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: %s\n" % m.mgrs_grid()
    if projection == "utm":
        origin = metadata.get("origin", {})
        return (
            "projector_type: LocalCartesianUTM\nvertical_datum: WGS84\nmap_origin:\n"
            "  latitude: %r\n  longitude: %r\n  altitude: %r\n"
            % (origin.get("latitude", 0.0), origin.get("longitude", 0.0), origin.get("altitude", 0.0))
        )
    return "projector_type: Local\n"


def _write_assets(pack_dir, name, entry):
    from . import __version__

    path = os.path.join(pack_dir, *ASSET_PATH.split("/"))
    assets = {"schema": SCHEMA, "maps": {}}
    if os.path.isfile(path):
        with open(path) as handle:
            assets = json.load(handle)
        if assets.get("schema") != SCHEMA:
            raise CarlaPackAssetsError("%s has schema %r; this roadgen writes %d" % (path, assets.get("schema"), SCHEMA))
    maps = dict(assets.get("maps", {}))
    maps[name] = entry
    assets = {
        "schema": SCHEMA,
        "generator": {"name": "roadgen", "version": __version__},
        "maps": dict(sorted(maps.items())),
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as handle:
        json.dump(assets, handle, indent=2)
        handle.write("\n")


def _stage(pack_dir):
    """Adds `Content/HDMaps` to the pack's staged loose folders, keeping the rest of
    `Config/PluginSettings.ini` as it is."""
    path = os.path.join(pack_dir, "Config", "PluginSettings.ini")
    text = ""
    if os.path.isfile(path):
        with open(path) as handle:
            text = handle.read()
    if any(line.strip().lower() == _STAGE_LINE.lower() for line in text.splitlines()):
        return
    lines = text.splitlines()
    if not any(line.strip() == _STAGE_SECTION for line in lines):
        lines += ([""] if lines else []) + [_STAGE_SECTION]
    index = next(i for i, line in enumerate(lines) if line.strip() == _STAGE_SECTION)
    end = index + 1
    while end < len(lines) and not lines[end].strip().startswith("["):
        end += 1
    while end > index + 1 and not lines[end - 1].strip():
        end -= 1
    lines.insert(end, _STAGE_LINE)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as handle:
        handle.write("\n".join(lines) + "\n")
