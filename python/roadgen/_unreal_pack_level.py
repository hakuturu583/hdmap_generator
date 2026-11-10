"""Runs inside Unreal's Python, not yours: `roadgen.carla_pack_level()` hands it to
the editor with `-run=pythonscript`. It saves one imported level, as it is, under a
content pack's mount point. Everything it reads comes in through the environment.
"""

import os

import unreal  # only exists inside the editor

SOURCE = os.environ["ROADGEN_LEVEL"]
TARGET = os.environ["ROADGEN_PACK_LEVEL"]
PACK = os.environ["ROADGEN_PACK"]


def say(message):
    unreal.log_warning("roadgen-pack-level: %s" % message)


# A commandlet starts before the asset registry has scanned the pack's mount point.
unreal.AssetRegistryHelpers.get_asset_registry().scan_paths_synchronous(["/" + PACK], force_rescan=True)
if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(SOURCE):
    raise RuntimeError("could not load %s" % SOURCE)
world = unreal.EditorLevelLibrary.get_editor_world()
saved = bool(unreal.EditorLoadingAndSavingUtils.save_map(world, TARGET))
say("saved %s" % (saved and unreal.EditorAssetLibrary.does_asset_exist(TARGET)))
