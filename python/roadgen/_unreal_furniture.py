"""Runs inside Unreal's Python, not yours: `roadgen.carla_furniture()` hands it to
the editor with `-run=pythonscript`. It places one imported map's traffic lights
and signs in its level and saves it. Everything it reads comes in through the
environment; what it places comes from the package's `furniture.manifest`.
"""

import json
import math
import os

import unreal  # only exists inside the editor

LEVEL = os.environ["ROADGEN_LEVEL"]
PACKAGE = os.environ["ROADGEN_PACKAGE"]
MAP = os.environ["ROADGEN_MAP"]
MANIFEST = os.environ["ROADGEN_FURNITURE"]

#: CARLA's own traffic light material: the one with the `Emissive Color` and
#: `Emissive Intensity` parameters `ADigitalTwinsTrafficLight` drives, and a name
#: holding `TrafficLight`, which is how it finds them.
LAMP_PARENT = "/Game/Carla/Static/TrafficLight/00_GenericComponents/M_TrafficLights"
#: What each lamp's instance is set to. The light sorts its three by green: the
#: greenest is green, the least green is red, and the third is amber.
LAMP_COLOURS = {
    "red": (1.0, 0.05, 0.05),
    "amber": (1.0, 0.5, 0.0),
    "green": (0.1, 1.0, 0.3),
}
#: The actor tag everything placed here carries, so a second run can find the
#: first run's work and take it down before placing again.
TAG = "roadgen.furniture"


def say(message):
    unreal.log_warning("roadgen-furniture: %s" % message)


with open(MANIFEST) as handle:
    manifest = json.load(handle)

unreal.EditorLoadingAndSavingUtils.load_map(LEVEL)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assets = unreal.EditorAssetLibrary

# Take down what an earlier run placed.
removed = 0
for actor in actors.get_all_level_actors():
    if any(str(tag) == TAG for tag in actor.tags):
        actors.destroy_actor(actor)
        removed += 1
if removed:
    say("removed %d actors from an earlier run" % removed)


def to_unreal(position, heading):
    """roadgen's metres, right-handed and z up, to Unreal's centimetres, left-handed.

    The one flip: Unreal negates y, so a heading anticlockwise from east becomes a
    yaw clockwise from it.
    """
    x, y, z = position
    location = unreal.Vector(x * 100.0, -y * 100.0, z * 100.0)
    rotation = unreal.Rotator(roll=0.0, pitch=0.0, yaw=-math.degrees(heading))
    return location, rotation


def lamp_materials():
    """Three instances of CARLA's light material, one per lamp, made once and kept
    in the package beside the meshes they go on."""
    if not assets.does_asset_exist(LAMP_PARENT):
        say("no %s in this CARLA; the lamps keep their flat colours and will not switch" % LAMP_PARENT)
        return {}
    parent = assets.load_asset(LAMP_PARENT)
    folder = "/Game/%s/Static/TrafficLight/%s_Lamps" % (PACKAGE, MAP)
    library = unreal.MaterialEditingLibrary
    instances = {}
    for lamp, (red, green, blue) in LAMP_COLOURS.items():
        name = "MI_TrafficLight_%s" % lamp.capitalize()
        path = "%s/%s" % (folder, name)
        if assets.does_asset_exist(path):
            instance = assets.load_asset(path)
        else:
            instance = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
                name, folder, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew()
            )
            if instance is None:
                say("could not create %s" % path)
                continue
            library.set_material_instance_parent(instance, parent)
        library.set_material_instance_vector_parameter_value(
            instance, "Emissive Color", unreal.LinearColor(red, green, blue, 1.0)
        )
        library.set_material_instance_scalar_parameter_value(instance, "Emissive Intensity", 0.0)
        library.update_material_instance(instance)
        assets.save_loaded_asset(instance)
        instances[lamp] = instance
    return instances


def dress(mesh, lamps):
    """Puts the lamp materials on the mesh's lamp slots, found by the names the
    package gave them."""
    wanted = {slot_name: lamp for lamp, slot_name in manifest["lamps"].items()}
    changed = False
    for index, slot in enumerate(mesh.get_editor_property("static_materials")):
        names = {
            str(slot.get_editor_property("material_slot_name")),
            str(slot.get_editor_property("imported_material_slot_name")),
        }
        for slot_name, lamp in wanted.items():
            if slot_name in names and lamp in lamps:
                mesh.set_material(index, lamps[lamp])
                changed = True
    return changed


def collide_by_triangles(mesh):
    """A post collides as the shape it is, not as its convex hull.

    The importer's auto-generated collision is a convex hull, and the hull of a
    pole with a mast arm is a wedge filling the whole space under the arm — a
    wall across the lane for any vehicle to hit. CARLA's own importer sets
    complex-as-simple on a map's meshes for the same reason; a prop has to be
    told here.
    """
    try:
        body = mesh.get_editor_property("body_setup")
        body.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
        return True
    except Exception as error:  # noqa: BLE001 - whatever the editor raises
        say("could not set complex collision on %s (%s); removing its simple collision instead" % (mesh.get_name(), error))
        try:
            unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem).remove_collisions(mesh)
            return True
        except Exception as fallback:  # noqa: BLE001
            say("could not remove collision from %s either (%s)" % (mesh.get_name(), fallback))
            return False


def prepare(asset, kind, lamps):
    """Loads a prop's mesh and readies it: triangle collision, and lamp materials
    on a light's lamps. Returns the mesh or None."""
    mesh = assets.load_asset(asset) if assets.does_asset_exist(asset) else None
    if mesh is None:
        say("no mesh at %s" % asset)
        return None
    prepared = collide_by_triangles(mesh)
    if kind == "lamps":
        prepared = dress(mesh, lamps) or prepared
    if prepared:
        assets.save_loaded_asset(mesh)
    return mesh


def add_component(actor, cls):
    """Adds a component of `cls` to a placed actor, the way the editor's own "Add
    Component" does it, and returns it. The first one added becomes the root."""
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    library = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_instance(actor)
    params = unreal.AddNewSubobjectParams(
        parent_handle=handles[0],
        new_class=cls,
        blueprint_context=None,
        skip_mark_blueprint_modified=False,
        conform_transform_to_parent=True,
    )
    handle, reason = subsystem.add_new_subobject(params)
    if not library.is_handle_valid(handle):
        raise RuntimeError("could not add %s to %s: %s" % (cls.get_name(), actor.get_actor_label(), reason))
    return library.get_object(library.get_data(handle))


def stand(mesh, label, position, heading, lift=0.0):
    """Spawns an actor holding `mesh` with its foot at `position`, facing `heading`.

    Not a StaticMeshActor. CARLA's `ApplyLaneIdsFromMapLogic` rebuilds the actor it
    adopts by copying each mesh component's *relative* transform onto a new actor
    that already has the old one's location and rotation — right for a mesh that
    is a child of a scene root, and a double placement for a StaticMeshActor,
    whose mesh is its root and whose relative transform is therefore the world
    one. So the actor is a bare one with a scene root at `position` and the mesh
    a child of it at the origin.

    `lift` moves the *actor* down by that much and the mesh back up, leaving the
    geometry where it was: a post lifted a metre is outside the fifty centimetres
    CARLA searches around a signal, so the lamps beside it are what it finds.
    """
    location, rotation = to_unreal(position, heading)
    location.z -= lift * 100.0
    actor = actors.spawn_actor_from_class(unreal.Actor, location, rotation)
    if actor is None:
        say("could not spawn %s" % label)
        return None
    actor.set_actor_label(label, False)
    root = add_component(actor, unreal.SceneComponent)
    root.set_mobility(unreal.ComponentMobility.STATIC)
    actor.set_actor_location(location, False, False)
    actor.set_actor_rotation(rotation, False)
    component = add_component(actor, unreal.StaticMeshComponent)
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    component.set_static_mesh(mesh)
    component.set_relative_location(unreal.Vector(0.0, 0.0, lift * 100.0), False, False)
    component.set_relative_rotation(unreal.Rotator(0.0, 0.0, 0.0), False, False)
    component.set_mobility(unreal.ComponentMobility.STATIC)
    actor.set_editor_property("tags", [unreal.Name(TAG)])
    return actor


def place(entry, kind, lamps):
    """Spawns one prop where its entry says, and returns the actor or None."""
    mesh = prepare(entry["asset"], kind, lamps)
    if mesh is None:
        return None
    if kind == "light":
        lamp_mesh = prepare(entry.get("lamps_asset", ""), "lamps", lamps)
        if lamp_mesh is None:
            return None
        if stand(lamp_mesh, entry["name"] + "_Lamps", entry["position"], entry["heading"]) is None:
            return None
        return stand(mesh, entry["name"], entry["position"], entry["heading"], lift=1.0)
    return stand(mesh, entry["name"], entry["position"], entry["heading"])


def mark(entry):
    """Stands CARLA's own sign actor, in the sign's state, at the sign.

    `ATrafficLightManager::SpawnSignals` looks for an `ATrafficSignBase` within
    five metres of each signal whose state matches the signal's type, and hangs its
    sign component on the one it finds instead of spawning a blueprint. A bare one
    at the post is enough.
    """
    state = entry.get("carla_state")
    if not state:
        return False
    try:
        cls = unreal.TrafficSignBase
        value = getattr(unreal.TrafficSignState, state)
    except AttributeError as error:
        say("this CARLA has no %s; %s keeps the plate CARLA spawns beside it (%s)" % (state, entry["name"], error))
        return False
    location, rotation = to_unreal(entry["position"], entry["heading"])
    marker = actors.spawn_actor_from_class(cls, location, rotation)
    if marker is None:
        say("could not spawn a sign marker for %s" % entry["name"])
        return False
    marker.set_traffic_sign_state(value)
    marker.set_actor_label(entry["name"] + "_Marker", False)
    marker.set_editor_property("tags", [unreal.Name(TAG)])
    return True


lamps = lamp_materials() if manifest.get("lights") else {}
lights = 0
for entry in manifest.get("lights", []):
    if place(entry, "light", lamps) is not None:
        lights += 1
signs = 0
marked = 0
for entry in manifest.get("signs", []):
    actor = place(entry, "sign", lamps)
    if actor is not None:
        signs += 1
        if mark(entry):
            marked += 1

# The town, when it came as props: its meshes are in the map's own coordinates,
# so each stands at the origin. Tagged Buildings by the folder it was imported to.
buildings = 0
for entry in manifest.get("buildings", []):
    mesh = prepare(entry["asset"], "building", lamps)
    if mesh is not None and stand(mesh, entry["name"], [0.0, 0.0, 0.0], 0.0) is not None:
        buildings += 1

say("placed %d lights and %d signs (%d signs with a CARLA marker) and %d buildings" % (lights, signs, marked, buildings))
say("saved %s" % unreal.EditorLoadingAndSavingUtils.save_current_level())
