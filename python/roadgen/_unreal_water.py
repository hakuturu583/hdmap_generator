"""Runs inside Unreal's Python, not yours: `roadgen.carla_water()` hands it to the
editor with `-run=pythonscript`. It puts the package's water materials in the
water slots of every mesh `Import.py` brought in for the package, by the slots'
names, and saves the meshes — and then in the same slots of every actor in the
map's level, whose components CARLA's cooking gave its own material for the
mesh's kind (the water is land to it), and saves the level. Everything it reads
comes in through the environment.
"""

import json
import os

import unreal  # only exists inside the editor

LEVEL = os.environ["ROADGEN_LEVEL"]
PACKAGE = os.environ["ROADGEN_PACKAGE"]
MAP = os.environ["ROADGEN_MAP"]
#: `{slot name: material path}`.
WATER = json.loads(os.environ["ROADGEN_WATER"])


def say(message):
    unreal.log_warning("roadgen-water: %s" % message)


assets = unreal.EditorAssetLibrary

materials = {}
for slot_name, path in WATER.items():
    if assets.does_asset_exist(path):
        materials[slot_name] = assets.load_asset(path)
    else:
        say("no %s in this project; %s keeps its flat colour" % (path, slot_name))

dressed = 0
if materials:
    for path in assets.list_assets("/Game/%s" % PACKAGE, recursive=True, include_folder=False):
        mesh = assets.load_asset(path)
        if not isinstance(mesh, unreal.StaticMesh):
            continue
        changed = False
        for index, slot in enumerate(mesh.get_editor_property("static_materials")):
            names = {
                str(slot.get_editor_property("material_slot_name")),
                str(slot.get_editor_property("imported_material_slot_name")),
            }
            for slot_name, material in materials.items():
                if slot_name in names:
                    mesh.set_material(index, material)
                    changed = True
        if changed:
            assets.save_loaded_asset(mesh)
            dressed += 1

# The level's actors: with CARLA's materials, its cooking set each mesh's
# component to the material for its kind, and an override on the component
# wins over the mesh's own. Water is land to CARLA, so its components carry the
# land's; they are given the water's instead, slot for slot.
overridden = 0
if materials:
    unreal.EditorLoadingAndSavingUtils.load_map(LEVEL)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    for actor in actors.get_all_level_actors():
        for component in actor.get_components_by_class(unreal.StaticMeshComponent):
            mesh = component.get_editor_property("static_mesh")
            if mesh is None:
                continue
            for index, slot in enumerate(mesh.get_editor_property("static_materials")):
                names = {
                    str(slot.get_editor_property("material_slot_name")),
                    str(slot.get_editor_property("imported_material_slot_name")),
                }
                for slot_name, material in materials.items():
                    if slot_name in names:
                        component.set_material(index, material)
                        overridden += 1
    if overridden:
        say("saved %s" % unreal.EditorLoadingAndSavingUtils.save_current_level())

say("dressed %d meshes of %s" % (dressed, MAP))
say("set %d slots in the level" % overridden)
