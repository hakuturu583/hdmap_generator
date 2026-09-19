"""Runs inside Unreal's Python, not yours: `roadgen.carla_sky()` hands it to the
editor with `-run=pythonscript`. It adds a daylight sky to one imported level and
saves it. Everything it reads comes in through the environment.
"""

import os

import unreal  # only exists inside the editor

LEVEL = os.environ["ROADGEN_LEVEL"]
SUN_ALTITUDE = float(os.environ.get("ROADGEN_SUN_ALTITUDE", "45"))
SUN_AZIMUTH = float(os.environ.get("ROADGEN_SUN_AZIMUTH", "-50"))
SUN_LUX = float(os.environ.get("ROADGEN_SUN_LUX", "75000"))


def say(message):
    unreal.log_warning("roadgen-sky: %s" % message)


unreal.EditorLoadingAndSavingUtils.load_map(LEVEL)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
present = actors.get_all_level_actors()
by_class = {}
for actor in present:
    by_class.setdefault(actor.get_class().get_name(), []).append(actor)


def one(cls, name):
    """The level's actor of this class, made if there is none."""
    if name in by_class:
        return by_class[name][0]
    actor = actors.spawn_actor_from_class(cls, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
    say("added %s" % name)
    return actor


# The sky itself: an atmosphere for the sun to light, and fog for depth.
one(unreal.SkyAtmosphere, "SkyAtmosphere")
one(unreal.ExponentialHeightFog, "ExponentialHeightFog")

# BaseMap's sun, turned into a physical one the atmosphere is calibrated for.
# CARLA's weather gives altitude and azimuth in degrees; Unreal's light points the
# way it shines, so the altitude goes in as a negative pitch.
sun = one(unreal.DirectionalLight, "DirectionalLight")
light = sun.get_component_by_class(unreal.DirectionalLightComponent)
light.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
light.set_editor_property("intensity", SUN_LUX)
light.set_editor_property("atmosphere_sun_light", True)
light.set_editor_property("cast_shadows", True)
sun.set_actor_rotation(unreal.Rotator(roll=0.0, pitch=-SUN_ALTITUDE, yaw=SUN_AZIMUTH), False)

# BaseMap's sky light, pointed at the sky it now has rather than at nothing.
sky_light = one(unreal.SkyLight, "SkyLight")
capture = sky_light.get_component_by_class(unreal.SkyLightComponent)
capture.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
capture.set_editor_property("intensity", 1.0)
capture.set_editor_property("source_type", unreal.SkyLightSourceType.SLS_CAPTURED_SCENE)
capture.set_editor_property("real_time_capture", True)

# Exposure. The project turns auto exposure off, and a physical sun with a fixed
# exposure is a white frame; this puts it back for the whole level, within the
# range daylight needs.
volume = one(unreal.PostProcessVolume, "PostProcessVolume")
volume.set_editor_property("unbound", True)
settings = volume.get_editor_property("settings")
settings.set_editor_property("override_auto_exposure_method", True)
settings.set_editor_property("auto_exposure_method", unreal.AutoExposureMethod.AEM_HISTOGRAM)
settings.set_editor_property("override_auto_exposure_min_brightness", True)
settings.set_editor_property("auto_exposure_min_brightness", 8.0)
settings.set_editor_property("override_auto_exposure_max_brightness", True)
settings.set_editor_property("auto_exposure_max_brightness", 16.0)
settings.set_editor_property("override_auto_exposure_bias", True)
settings.set_editor_property("auto_exposure_bias", 0.0)
volume.set_editor_property("settings", settings)

say("saved %s" % unreal.EditorLoadingAndSavingUtils.save_current_level())
