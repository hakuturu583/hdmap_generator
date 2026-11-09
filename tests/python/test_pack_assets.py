"""A map in every format, put into a CARLA content pack beside its level."""

import json
import os
import struct

import pytest

import roadgen
from roadgen.pack_assets import ASSET_PATH, FORMATS, merge_pcd


def two_way():
    return [
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ]


def town():
    m = roadgen.Map(name="Packed")
    a = m.add_road(start=(0.0, 0.0, 0.0), end=(200.0, 0.0, 0.0), lanes=two_way(), name="a")
    b = m.add_road(start=(200.0, 0.0, 0.0), end=(300.0, 100.0, 0.0), lanes=two_way(), name="b")
    m.connect(a, b)
    return m


def pack(tmp_path, xodr=True):
    """A pack folder as `carla-pack init` and `add --map` leave it."""
    folder = tmp_path / "PackedPack"
    (folder / "Config").mkdir(parents=True)
    maps = []
    if xodr:
        maps.append({"name": "Packed", "package": "/Game/P/Maps/Packed/Packed",
                     "xodr": "Maps/OpenDrive/Packed/Packed.xodr"})
    (folder / "carla-pack.json").write_text(json.dumps({
        "name": "PackedPack", "version": "1.0.0", "carla_version": "1.0", "base_release": "",
        "engine": {"version": "", "commit": ""}, "platform": "Linux",
        "maps": maps, "catalogs": [], "files": {},
    }))
    (folder / "Config" / "PluginSettings.ini").write_text(
        "[StageSettings]\n+AdditionalNonUSFDirectories=Content/Config\n"
    )
    return folder


def test_every_format_is_written_and_listed(tmp_path):
    folder = pack(tmp_path)
    entry = roadgen.carla_pack_assets(town(), str(folder))
    assert list(entry["formats"]) == list(FORMATS)
    for kind, item in entry["formats"].items():
        if kind == "opendrive":
            continue
        target = folder / item["path"]
        assert target.exists(), (kind, item["path"])
        assert (item["type"] == "directory") == target.is_dir(), kind
        for member in item.get("files", {}).values():
            assert (folder / member).is_file(), (kind, member)
    # The OpenDRIVE is the one beside the level, not a second copy.
    assert entry["formats"]["opendrive"]["path"] == "Content/Maps/OpenDrive/Packed/Packed.xodr"
    # Autoware's map directory, whole.
    autoware = entry["formats"]["autoware"]["files"]
    assert set(autoware) == {"lanelet2_map", "pointcloud_map", "map_projector_info"}
    assert (folder / autoware["map_projector_info"]).read_text() == "projector_type: Local\n"
    # The asset file is where CARLA stages loose files already, and says what made it.
    assets = json.loads((folder / ASSET_PATH).read_text())
    assert assets["schema"] == 1
    assert assets["generator"] == {"name": "roadgen", "version": roadgen.__version__}
    assert assets["maps"]["Packed"] == entry
    # The formats' folder is staged too, once.
    ini = (folder / "Config" / "PluginSettings.ini").read_text()
    assert ini.count("+AdditionalNonUSFDirectories=Content/HDMaps") == 1
    assert "+AdditionalNonUSFDirectories=Content/Config" in ini


def test_a_map_with_no_level_xodr_gets_its_own_and_a_rerun_replaces_it(tmp_path):
    folder = pack(tmp_path, xodr=False)
    first = roadgen.carla_pack_assets(town(), str(folder), formats=["opendrive", "osm"])
    assert first["formats"]["opendrive"]["path"] == "Content/HDMaps/Packed/opendrive/Packed.xodr"
    assert (folder / first["formats"]["opendrive"]["path"]).is_file()
    second = roadgen.carla_pack_assets(town(), str(folder), formats=["sumo"])
    assert list(second["formats"]) == ["sumo"]
    assert not (folder / "Content/HDMaps/Packed/osm").exists()
    ini = (folder / "Config" / "PluginSettings.ini").read_text()
    assert ini.count("Content/HDMaps") == 1
    # Another map of the pack keeps its entry.
    roadgen.carla_pack_assets(town(), str(folder), "Other", formats=["osm"])
    assets = json.loads((folder / ASSET_PATH).read_text())
    assert sorted(assets["maps"]) == ["Other", "Packed"]


def test_the_point_cloud_reaches_only_so_far_from_the_roads(tmp_path):
    near = roadgen.carla_pack_assets(town(), str(pack(tmp_path / "near")), formats=["pointcloud"],
                                     pointcloud_reach=10.0)
    far = roadgen.carla_pack_assets(town(), str(pack(tmp_path / "far")), formats=["pointcloud"],
                                    pointcloud_reach=None)
    count = lambda root, entry: int(  # noqa: E731
        open(root / entry["formats"]["pointcloud"]["path"], "rb").read(400).split(b"POINTS ")[1].split(b"\n")[0]
    )
    assert 0 < count(tmp_path / "near" / "PackedPack", near) < count(tmp_path / "far" / "PackedPack", far)


def test_refuses_what_is_not_a_pack_or_not_a_format(tmp_path):
    with pytest.raises(roadgen.CarlaPackAssetsError, match="no carla-pack.json"):
        roadgen.carla_pack_assets(town(), str(tmp_path))
    with pytest.raises(roadgen.CarlaPackAssetsError, match="unknown formats"):
        roadgen.carla_pack_assets(town(), str(pack(tmp_path)), formats=["shapefile"])


def write_tile(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        "TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        % (len(points), len(points))
    )
    path.write_bytes(header.encode() + b"".join(struct.pack("<fff", *p) for p in points))


def test_tiles_join_into_one_cloud(tmp_path):
    tiles = tmp_path / "tiles"
    tiles.mkdir()
    write_tile(tiles / "0_0.pcd", [(1.0, 2.0, 3.0)])
    write_tile(tiles / "0_1.pcd", [(4.0, 5.0, 6.0), (7.0, 8.0, 9.0)])
    target = tmp_path / "pointcloud_map.pcd"
    assert merge_pcd(str(tiles), str(target)) == 3
    raw = target.read_bytes()
    head, body = raw.split(b"DATA binary\n")
    assert b"WIDTH 3\n" in head and b"POINTS 3\n" in head
    assert list(struct.iter_unpack("<fff", body)) == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
    assert merge_pcd(str(tiles), str(target), keep={"0_1.pcd"}) == 2
