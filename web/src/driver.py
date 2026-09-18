"""What the page runs around your code.

The script you write is ordinary roadgen: it builds a map and calls `export_*`. This
runs it in a directory of its own, and then goes and looks at what turned up there —
which is why the page shows whatever your script wrote rather than a fixed set of
panels. Write one export and you get one picture.

Nothing here touches the map. Every picture is made by reading a file back with
`roadgen.render_*`, so a panel that is wrong means the file is wrong, not the page.
"""

import io
import json
import os
import shutil
import sys
import time
import traceback

import roadgen

WORK = "/work"

#: The order the panels come out in: the map formats first, the scene formats after,
#: because a scene is a map plus something driving through it.
ORDER = ["OpenDRIVE", "Lanelet2", "OpenStreetMap", "SUMO", "CARLA", "ClipGT", "GPUDrive"]


def run(code):
    """Runs `code`, then draws everything it wrote. Returns JSON."""
    _reset()
    output = io.StringIO()
    error = None

    started = time.monotonic()
    stdout, stderr = sys.stdout, sys.stderr
    try:
        sys.stdout = sys.stderr = output
        exec(compile(code, "<your script>", "exec"), {"__name__": "__main__"})
    except BaseException:
        # The traceback is the answer to "why is there nothing to look at", so it is
        # collected rather than raised: whatever the script did manage to write is
        # still worth showing beside it.
        error = _traceback()
    finally:
        sys.stdout, sys.stderr = stdout, stderr
    elapsed = time.monotonic() - started

    views = _views()
    return json.dumps(
        {
            "stdout": output.getvalue(),
            "error": error,
            "seconds": round(elapsed, 3),
            # Where the files are, so that the page can read one back without
            # spelling this directory a second time on its own side.
            "work": WORK,
            "files": _files(),
            "views": sorted(
                views,
                key=lambda view: (
                    ORDER.index(view["format"]) if view["format"] in ORDER else len(ORDER),
                    view["title"],
                ),
            ),
        }
    )


def _reset():
    shutil.rmtree(WORK, ignore_errors=True)
    os.makedirs(WORK, exist_ok=True)
    os.chdir(WORK)


def _traceback():
    """The traceback, with this file's own frames left out.

    A user looking at `File "driver.py", line 41, in run` learns nothing: the frames
    worth showing start at their own script.
    """
    lines = traceback.format_exc().splitlines(keepends=True)
    kept = [line for line in lines if "driver.py" not in line]
    return "".join(kept) if len(kept) > 1 else "".join(lines)


def _files():
    found = [
        _entry(os.path.join(root, name))
        for root, _, names in os.walk(WORK)
        for name in names
    ]
    return sorted(found, key=lambda entry: entry["path"])


def _entry(path):
    """One written file, as the page lists it.

    Whether it can be shown as text is decided here rather than in the page: a NUL
    byte in the first kilobyte is crude, and right about every file roadgen writes —
    the text formats have none, and Parquet is full of them.
    """
    with open(path, "rb") as file:
        head = file.read(1024)
    return {
        "path": os.path.relpath(path, WORK),
        "size": os.path.getsize(path),
        "text": b"\0" not in head,
    }


def _views():
    """One panel per thing that was written, whatever it turned out to be."""
    # `run` sorts what comes back, so nothing here has to be in any order.
    views = []
    for root, directories, names in os.walk(WORK):
        for directory in directories:
            views.extend(_directory(os.path.join(root, directory)))
        for name in names:
            views.extend(_file(os.path.join(root, name)))
    return views


def _directory(path):
    """A SUMO network and a ClipGT clip are both directories, told apart by what is
    in them: one is XML the other has never heard of, the other is Parquet."""
    names = os.listdir(path)
    if any(name.endswith(".nod.xml") for name in names):
        return [_drawn("SUMO", path, roadgen.render_sumo)]
    if any(name.endswith(".parquet") for name in names):
        return [_drawn("ClipGT", path, roadgen.render_clipgt)]
    return []


def _file(path):
    if path.endswith(".xodr"):
        # A CARLA package holds one of these too, beside its .fbx. It is the same
        # OpenDRIVE and the same picture, so the package's panel is the FBX and this
        # one is left to the export a script wrote on its own.
        if _in_carla_package(path):
            return []
        return [_drawn("OpenDRIVE", path, roadgen.render_opendrive)]
    if path.endswith(".fbx"):
        return [_drawn("CARLA", path, roadgen.render_carla, files=_carla_package(path))]
    if path.endswith(".osm"):
        return [_osm(path)]
    if path.endswith(".json"):
        # A scene is JSON and so is anything else a script chose to write, so the
        # only honest test is whether it reads back as a scene.
        try:
            svg = roadgen.render_gpudrive(path)
        except Exception:
            return []
        return [_view("GPUDrive", path, svg=svg)]
    return []


def _carla_package(path):
    """The files of the CARLA package an .fbx belongs to, as the panel lists them.

    A CARLA map is not one file: the mesh is only half of it, and a reader who cannot
    see the .xodr and the descriptor beside it cannot see whether the export is a map
    or a model. Listed rather than walked, so the textures — hundreds of megabytes,
    once fetched — do not fill the panel.
    """
    folder = os.path.dirname(path)
    stem = os.path.splitext(path)[0]
    wanted = [path, stem + ".xodr"]
    wanted += [
        os.path.join(os.path.dirname(folder), name)
        for name in sorted(os.listdir(os.path.dirname(folder) or "."))
        if name.endswith(".json")
    ]
    wanted += [os.path.join(folder, "Textures", "polyhaven.json")]
    return [_entry(one) for one in wanted if os.path.isfile(one)]


def _in_carla_package(path):
    """Whether an .xodr is the one beside a CARLA package's mesh.

    By looking, rather than by knowing where the exporter puts things: a CARLA map is
    an .fbx and an .xodr of the same name in one folder, and that is the whole test.
    """
    stem = os.path.splitext(path)[0]
    return os.path.exists(stem + ".fbx")


def _drawn(format_, path, render, files=None):
    try:
        return _view(format_, path, svg=render(path), files=files)
    except Exception as exception:
        return _view(format_, path, error=str(exception), files=files)


def _osm(path):
    """Both OSM exports are OSM XML, and the page hands both to Leaflet.

    Which one this is comes off the `generator` attribute, which is the writer saying
    so — a Lanelet2 map and a plain one differ in what the ways *mean*, not in
    anything a reader could infer from the geometry. The file itself is not read in
    here: the page reads it out of the working directory, as it does for the `source`
    and `save` buttons.
    """
    with open(path, encoding="utf-8") as file:
        head = file.read(512)
    lanelet2 = 'generator="lanelet2"' in head
    return _view(
        "Lanelet2" if lanelet2 else "OpenStreetMap",
        path,
        note=(
            "Drawn by Leaflet, over OpenStreetMap tiles: this export carries latitudes "
            "and longitudes, so it has a place on Earth rather than only a shape."
        ),
    )


def _view(format_, path, svg=None, note=None, error=None, files=None):
    """One panel's worth of answer.

    The picture is either an SVG the viewer drew, or nothing — in which case the page
    draws the file itself and `note` is what there is to say about it.

    `files` is what the panel lists under the switch. It defaults to the export — one
    file, or a directory's contents — and is given explicitly where an export is
    neither: a CARLA package is a mesh, a road network and a descriptor spread across
    two folders.
    """
    relative = os.path.relpath(path, WORK)
    if files is not None:
        contents = sorted(files, key=lambda entry: entry["path"])
    elif os.path.isdir(path):
        contents = sorted(
            (_entry(os.path.join(path, name)) for name in os.listdir(path)),
            key=lambda entry: entry["path"],
        )
    else:
        contents = [_entry(path)]

    view = {"format": format_, "title": relative, "files": contents}
    for key, value in (("svg", svg), ("note", note), ("error", error)):
        if value is not None:
            view[key] = value
    return view
