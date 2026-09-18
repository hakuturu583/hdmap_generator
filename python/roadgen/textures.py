"""Fetching the textures a CARLA package asks for.

`Map.export_carla()` writes a package that *references* its textures and does not
hold them: the FBX points at `Textures/asphalt_02_diff_2k.jpg`, and beside it is
`Textures/polyhaven.json` saying exactly which Poly Haven asset that file is and at
what resolution. This module is the other half — it reads that manifest and downloads
what it lists.

The split is deliberate. An exporter that reached for the network could not run
offline, could not run in CI and could not run in the browser, and the demo page runs
roadgen compiled to WebAssembly. So the Rust side never opens a socket, and this does.

    import roadgen

    m.export_carla("Import/")
    roadgen.fetch_textures("Import/Town01")     # downloads what the manifest lists

Nothing here is required. A package with no textures in it is a complete, importable
CARLA package: every material carries a flat colour that stands in for its texture,
and CARLA replaces them all anyway when `use_carla_materials` is on.

Poly Haven publishes under CC0 — public domain, no attribution required — which is
why these are the textures a package that may be redistributed can use. The package
names its sources in `Textures/CREDITS.md` as a courtesy rather than a condition.

The slugs in a manifest are the ones roadgen was written against, and Poly Haven's
catalogue is theirs to change. A slug that has gone is reported by name rather than
guessed at, and the manifest is JSON in the package: swapping one is editing a file.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request

__all__ = ["fetch_textures", "texture_manifest", "TextureError"]

#: Where the manifest sits inside a map's folder.
MANIFEST = os.path.join("Textures", "polyhaven.json")

#: How long to wait on one request, seconds. A texture is a few megabytes and a
#: fetcher that hangs on a dead connection is worse than one that fails.
TIMEOUT = 60


class TextureError(RuntimeError):
    """A texture could not be fetched, with what was wrong and for which asset."""


def texture_manifest(package):
    """Reads the manifest out of a map's folder.

    `package` is the folder holding the `.fbx` — the one `export_carla` reports as
    the map's directory, not the folder above it holding the descriptor.
    """
    path = os.path.join(package, MANIFEST)
    try:
        with open(path, encoding="utf-8") as file:
            return json.load(file)
    except FileNotFoundError:
        raise TextureError(
            "%s is not there: it is written by export_carla, so either this is not a "
            "package folder or the map's own folder is one level down" % path
        ) from None


def fetch_textures(package, *, overwrite=False, resolution=None, on_progress=None):
    """Downloads the textures a package's manifest lists, into that package.

    Returns the list of paths written, relative to `package`. Files already there are
    left alone unless `overwrite` is set, so running this twice costs one API call per
    asset and no downloads.

    `resolution` overrides what the manifest asks for — `"1k"` for a package to be
    moved around, `"4k"` for one to be looked at closely. The file keeps the name the
    FBX references either way, because the FBX references a path and not a resolution.

    `on_progress` is called with `(path, index, total)` before each download, for a
    caller that wants to say something while several hundred megabytes arrive.

    Raises `TextureError` naming every asset that could not be fetched, rather than
    stopping at the first: a catalogue that has moved on has usually moved on for more
    than one of them, and one run should say so once.
    """
    manifest = texture_manifest(package)
    api = manifest.get("api", "https://api.polyhaven.com").rstrip("/")
    files = manifest.get("files", [])

    written = []
    problems = []
    # One API call per asset rather than per file: an asset's diffuse, normal and
    # roughness maps are three entries of one listing.
    listings = {}

    for index, entry in enumerate(files):
        path = entry["path"]
        target = os.path.join(package, *path.split("/"))
        if os.path.exists(target) and not overwrite:
            continue

        slug = entry["slug"]
        if slug not in listings:
            try:
                listings[slug] = _listing(api, slug)
            except TextureError as error:
                listings[slug] = error
        listing = listings[slug]
        if isinstance(listing, TextureError):
            problems.append(str(listing))
            continue

        try:
            url = _url(listing, entry, resolution)
        except TextureError as error:
            problems.append("%s: %s" % (path, error))
            continue

        if on_progress is not None:
            on_progress(path, index, len(files))
        try:
            _download(url, target)
        except (urllib.error.URLError, OSError) as error:
            problems.append("%s: %s" % (path, error))
            continue
        written.append(path)

    if problems:
        raise TextureError(
            "%d of %d textures could not be fetched:\n  %s\n"
            "The manifest at %s is editable: a slug Poly Haven no longer has can be "
            "replaced with one from https://polyhaven.com/textures."
            % (
                len(problems),
                len(files),
                "\n  ".join(problems),
                os.path.join(package, MANIFEST),
            )
        )
    return written


def _listing(api, slug):
    """What Poly Haven has for one asset: maps, resolutions and formats."""
    url = "%s/files/%s" % (api, slug)
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        if error.code == 404:
            raise TextureError(
                "%s: Poly Haven has no asset by that name (%s)" % (slug, url)
            ) from None
        raise TextureError("%s: %s returned %s" % (slug, url, error.code)) from None
    except urllib.error.URLError as error:
        raise TextureError("%s: %s could not be reached (%s)" % (slug, url, error.reason)) from None
    except ValueError:
        raise TextureError("%s: %s did not answer with JSON" % (slug, url)) from None


def _url(listing, entry, resolution):
    """The download URL for one map of one asset.

    Poly Haven nests its listing map, then resolution, then format. Each level is
    looked up rather than assumed, and a level that is not there says what *is*, so a
    resolution a texture was never published at is a message rather than a KeyError.
    """
    maps = listing.get(entry["map"])
    if not maps:
        raise TextureError(
            "%s has no `%s` map (it has %s)"
            % (entry["slug"], entry["map"], ", ".join(sorted(listing)) or "none")
        )
    wanted = resolution or entry["resolution"]
    sizes = maps.get(wanted)
    if not sizes:
        raise TextureError(
            "%s has no `%s` at %s (it has %s)"
            % (entry["slug"], entry["map"], wanted, ", ".join(sorted(maps)) or "none")
        )
    file = sizes.get(entry["format"])
    if not file or "url" not in file:
        raise TextureError(
            "%s has no `%s` at %s as %s (it has %s)"
            % (
                entry["slug"],
                entry["map"],
                wanted,
                entry["format"],
                ", ".join(sorted(sizes)) or "none",
            )
        )
    return file["url"]


def _download(url, target):
    """Fetches one file, through a temporary name.

    Through a temporary name so that an interrupted download does not leave a
    half-written JPEG that the next run takes for a file it already has.
    """
    os.makedirs(os.path.dirname(target) or ".", exist_ok=True)
    partial = target + ".part"
    with urllib.request.urlopen(url, timeout=TIMEOUT) as response:
        with open(partial, "wb") as file:
            while True:
                chunk = response.read(1 << 16)
                if not chunk:
                    break
                file.write(chunk)
    os.replace(partial, target)
