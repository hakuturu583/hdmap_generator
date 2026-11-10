"""An imported map's level, put into a CARLA content pack at /<Pack>/Maps/<Map>."""

import json
import subprocess

import pytest

import roadgen
import roadgen.pack_level as pack_level

UPLUGIN = {"FileVersion": 3, "FriendlyName": "HighwayPack", "ExplicitlyLoaded": True, "CanContainContent": True}


def checkout(tmp_path, with_pack=True, tm=True, nav=True):
    """A CARLA checkout after `Import.py` has imported HighwayPackage."""
    carla = tmp_path / "carla"
    tools = carla / "Util" / "ContentPacks"
    tools.mkdir(parents=True)
    (tools / "carla_pack.py").write_text("")
    imported = carla / "Unreal" / "CarlaUnreal" / "Content" / "HighwayPackage" / "Maps" / "Highway"
    (imported / "OpenDrive").mkdir(parents=True)
    (imported / "Highway.umap").write_bytes(b"imported level")
    (imported / "OpenDrive" / "Highway.xodr").write_text("<OpenDRIVE/>")
    if nav:
        (imported / "Nav").mkdir()
        (imported / "Nav" / "Highway.bin").write_bytes(b"nav")
    if tm:
        (imported / "TM").mkdir()
        (imported / "TM" / "Highway.bin").write_bytes(b"tm")
    if with_pack:
        make_pack(carla)
    return carla


def make_pack(carla):
    folder = carla / "Unreal" / "CarlaUnreal" / "Plugins" / "Packs" / "HighwayPack"
    (folder / "Content" / "Maps").mkdir(parents=True)
    (folder / "carla-pack.json").write_text(json.dumps({"name": "HighwayPack", "maps": []}))
    (folder / "HighwayPack.uplugin").write_text(json.dumps(UPLUGIN, indent="\t"))
    return folder


@pytest.fixture
def editor(monkeypatch):
    """The editor, faked: it saves the level where it is told to, and records how it was run."""
    runs = []

    def run(editor_binary, uproject, script, environment, log=None, arguments=()):
        pack = environment["ROADGEN_PACK"]
        carla = uproject.rsplit("/Unreal/", 1)[0]
        folder = "%s/Unreal/CarlaUnreal/Plugins/Packs/%s" % (carla, pack)
        with open(folder + "/HighwayPack.uplugin") as handle:
            explicitly_loaded = json.load(handle)["ExplicitlyLoaded"]
        runs.append({"environment": environment, "arguments": list(arguments), "explicitly_loaded": explicitly_loaded})
        name = environment["ROADGEN_PACK_LEVEL"].rsplit("/", 1)[1]
        with open("%s/Content/Maps/%s.umap" % (folder, name), "wb") as handle:
            handle.write(b"saved level")
        # Save As also saves the level it started from.
        source = environment["ROADGEN_LEVEL"].split("/")
        with open("%s/Unreal/CarlaUnreal/Content/%s/Maps/%s/%s.umap" % (carla, source[2], source[4], source[5]), "wb") as handle:
            handle.write(b"resaved by the editor")
        return ["roadgen-pack-level: saved True", "Python script executed successfully"], 0, True

    monkeypatch.setattr(pack_level, "editor_paths", lambda carla, engine, error: ("UnrealEditor", carla + "/Unreal/CarlaUnreal/CarlaUnreal.uproject"))
    monkeypatch.setattr(pack_level, "run_editor_script", run)
    monkeypatch.setenv("CARLA_UNREAL_ENGINE_PATH", "/opt/ue5")
    return runs


@pytest.fixture
def carla_pack(monkeypatch):
    """CARLA's `carla-pack`, faked: `init` makes the pack folder, every call is recorded."""
    calls = []

    def run(command, cwd=None, env=None, **kwargs):
        arguments = command[2:]
        calls.append({"arguments": arguments, "engine": env["CARLA_UNREAL_ENGINE_PATH"]})
        if arguments[0] == "init":
            make_pack(__import__("pathlib").Path(cwd))
        return subprocess.CompletedProcess(command, 0, stdout="carla-pack: ok\n")

    monkeypatch.setattr(pack_level.subprocess, "run", run)
    return calls


def test_the_level_lands_at_the_packs_maps_root_with_its_files(tmp_path, editor, carla_pack):
    carla = checkout(tmp_path)
    level = roadgen.carla_pack_level(str(carla), "HighwayPackage", "Highway", "HighwayPack")
    assert level == "/HighwayPack/Maps/Highway"
    assert editor[0]["environment"]["ROADGEN_LEVEL"] == "/Game/HighwayPackage/Maps/Highway/Highway"
    assert editor[0]["environment"]["ROADGEN_PACK_LEVEL"] == "/HighwayPack/Maps/Highway"
    assert editor[0]["arguments"] == ["-EnablePlugins=HighwayPack"]

    imported = carla / "Unreal" / "CarlaUnreal" / "Content" / "HighwayPackage" / "Maps" / "Highway"
    pack = carla / "Unreal" / "CarlaUnreal" / "Plugins" / "Packs" / "HighwayPack"
    [add] = carla_pack
    assert add["arguments"] == [
        "add", "HighwayPack",
        "--map", str(pack / "Content" / "Maps" / "Highway.umap"),
        "--xodr", str(imported / "OpenDrive" / "Highway.xodr"),
        "--nav", str(imported / "Nav" / "Highway.bin"),
        "--tm", str(imported / "TM"),
    ]
    assert add["engine"] == "/opt/ue5"


def test_the_editor_mounts_the_pack_and_both_files_are_put_back(tmp_path, editor, carla_pack):
    carla = checkout(tmp_path)
    roadgen.carla_pack_level(str(carla), "HighwayPackage", "Highway", "HighwayPack")
    # Explicitly loaded, the editor would not mount it for -EnablePlugins.
    assert editor[0]["explicitly_loaded"] is False
    pack = carla / "Unreal" / "CarlaUnreal" / "Plugins" / "Packs" / "HighwayPack"
    assert json.loads((pack / "HighwayPack.uplugin").read_text()) == UPLUGIN
    imported = carla / "Unreal" / "CarlaUnreal" / "Content" / "HighwayPackage" / "Maps" / "Highway"
    assert (imported / "Highway.umap").read_bytes() == b"imported level"


def test_a_missing_pack_is_made_first_and_missing_sidecars_are_not_passed(tmp_path, editor, carla_pack):
    carla = checkout(tmp_path, with_pack=False, tm=False, nav=False)
    roadgen.carla_pack_level(str(carla), "HighwayPackage", "Highway", "HighwayPack")
    assert carla_pack[0]["arguments"] == ["init", "HighwayPack"]
    assert "--nav" not in carla_pack[1]["arguments"] and "--tm" not in carla_pack[1]["arguments"]


def test_a_level_saved_by_an_earlier_run_is_replaced(tmp_path, editor, carla_pack):
    carla = checkout(tmp_path)
    maps = carla / "Unreal" / "CarlaUnreal" / "Plugins" / "Packs" / "HighwayPack" / "Content" / "Maps"
    (maps / "Highway.umap").write_bytes(b"old")
    (maps / "Highway_BuiltData.uasset").write_bytes(b"old")
    roadgen.carla_pack_level(str(carla), "HighwayPackage", "Highway", "HighwayPack")
    assert (maps / "Highway.umap").read_bytes() == b"saved level"
    assert not (maps / "Highway_BuiltData.uasset").exists()


def test_nothing_runs_before_the_package_is_imported(tmp_path, editor, carla_pack):
    carla = checkout(tmp_path)
    (carla / "Unreal" / "CarlaUnreal" / "Content" / "HighwayPackage" / "Maps" / "Highway" / "Highway.umap").unlink()
    with pytest.raises(roadgen.CarlaPackLevelError, match="import the package first"):
        roadgen.carla_pack_level(str(carla), "HighwayPackage", "Highway", "HighwayPack")
    assert not editor and not carla_pack


def test_a_failed_save_is_reported_and_carla_pack_is_not_run(tmp_path, editor, carla_pack, monkeypatch):
    carla = checkout(tmp_path)
    monkeypatch.setattr(pack_level, "run_editor_script", lambda *a, **k: (["roadgen-pack-level: saved False"], 1, True))
    with pytest.raises(roadgen.CarlaPackLevelError, match="did not save"):
        roadgen.carla_pack_level(str(carla), "HighwayPackage", "Highway", "HighwayPack")
    assert not carla_pack


def test_carla_pack_level_refuses_to_run_without_an_engine(tmp_path, monkeypatch):
    monkeypatch.delenv("CARLA_UNREAL_ENGINE_PATH", raising=False)
    with pytest.raises(roadgen.CarlaPackLevelError, match="CARLA_UNREAL_ENGINE_PATH"):
        roadgen.carla_pack_level(str(tmp_path), "Pkg", "Town", "Pack")
