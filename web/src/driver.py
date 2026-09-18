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
ORDER = ["OpenDRIVE", "Lanelet2", "OpenStreetMap", "SUMO", "ClipGT", "GPUDrive"]


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
    found = []
    for root, _, names in os.walk(WORK):
        for name in sorted(names):
            path = os.path.join(root, name)
            found.append(
                {
                    "path": os.path.relpath(path, WORK),
                    "size": os.path.getsize(path),
                }
            )
    return sorted(found, key=lambda entry: entry["path"])


def _views():
    """One panel per thing that was written, whatever it turned out to be."""
    views = []
    for root, directories, names in os.walk(WORK):
        for directory in sorted(directories):
            views.extend(_directory(os.path.join(root, directory)))
        for name in sorted(names):
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
        return [_drawn("OpenDRIVE", path, roadgen.render_opendrive)]
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


def _drawn(format_, path, render):
    try:
        return _view(format_, path, svg=render(path))
    except Exception as exception:
        return _view(format_, path, error=str(exception))


def _osm(path):
    """Both OSM exports are OSM XML, and the page hands both to Leaflet.

    Which one this is comes off the `generator` attribute, which is the writer saying
    so — a Lanelet2 map and a plain one differ in what the ways *mean*, not in
    anything a reader could infer from the geometry.
    """
    with open(path, encoding="utf-8") as file:
        xml = file.read()
    lanelet2 = 'generator="lanelet2"' in xml[:512]
    return _view(
        "Lanelet2" if lanelet2 else "OpenStreetMap",
        path,
        xml=xml,
    )


def _view(format_, path, **rest):
    relative = os.path.relpath(path, WORK)
    contents = [relative]
    if os.path.isdir(path):
        contents = sorted(
            os.path.join(relative, name) for name in os.listdir(path)
        )
    view = {"format": format_, "title": relative, "files": contents}
    view.update(rest)
    return view
