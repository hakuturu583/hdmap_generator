"""Running a script inside Unreal's own Python, headless.

Two things this package does to an imported CARLA map — giving it a sky, placing
its furniture — are done with the editor's scripting, because the level is an
editor asset and nothing outside the editor can open it. This is the one way both
are run: `UnrealEditor <project> -run=pythonscript -script=<file>`, unattended and
off screen, with what the script needs handed to it through the environment.
"""

from __future__ import annotations

import os
import subprocess
import tempfile


def editor_paths(carla_root, engine, error):
    """The editor binary and the CARLA project, checked to exist.

    `error` is the exception class to raise when they do not, so that each caller
    reports in its own terms.
    """
    engine = engine or os.environ.get("CARLA_UNREAL_ENGINE_PATH")
    if not engine:
        raise error("no Unreal Engine given and CARLA_UNREAL_ENGINE_PATH is not set")
    editor = os.path.join(
        engine,
        "Engine",
        "Binaries",
        "Win64" if os.name == "nt" else "Linux",
        "UnrealEditor",
    )
    if not os.path.exists(editor) and not os.path.exists(editor + ".exe"):
        raise error("no UnrealEditor at %s" % editor)
    uproject = os.path.join(carla_root, "Unreal", "CarlaUnreal", "CarlaUnreal.uproject")
    if not os.path.exists(uproject):
        raise error("no CarlaUnreal.uproject under %s" % carla_root)
    return editor, uproject


def run_editor_script(editor, uproject, script, environment, log=None):
    """Runs `script` inside the editor and returns its output, line by line.

    `environment` is added to the process's own. Returns `(lines, returncode,
    succeeded)`, where `succeeded` is whether the editor reported the script
    running to completion — an exception inside it does not fail the process.
    """
    full_environment = dict(os.environ)
    full_environment.update(environment)
    command = [
        editor,
        uproject,
        "-run=pythonscript",
        "-script=%s" % script,
        "-RenderOffScreen",
        "-unattended",
        "-nosourcecontrol",
        "-nopause",
    ]
    # The editor's output goes to a file rather than a pipe: the editor leaves a
    # trace daemon behind that inherits its stdout, and a pipe read to its end
    # would wait for that daemon to close it, long after the editor has gone.
    with tempfile.TemporaryFile(mode="w+", errors="replace") as output:
        process = subprocess.run(
            command,
            env=full_environment,
            stdout=output,
            stderr=subprocess.STDOUT,
            check=False,
        )
        output.seek(0)
        lines = output.read().splitlines()
    succeeded = False
    for line in lines:
        if log is not None:
            log(line)
        if "Python script executed successfully" in line:
            succeeded = True
    return lines, process.returncode, succeeded
