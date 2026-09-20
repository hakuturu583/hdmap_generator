"""`python -m roadgen <command> …` — the package's commands.

    python -m roadgen textures Import/Town01                    # fetch a package's textures
    python -m roadgen sky /opt/carla Town01Package Town01       # give an imported map a sky
    python -m roadgen furniture /opt/carla Town01Package Town01 # stand its lights and signs

All three are what the script `export_carla()` writes beside a package runs. Commands
live here rather than in the modules that do the work, so that those stay modules
the package can import eagerly.
"""

import sys


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    commands = {"furniture": _furniture, "sky": _sky, "textures": _textures}
    if not argv or argv[0] not in commands:
        print("usage: python -m roadgen {%s} …" % "|".join(sorted(commands)), file=sys.stderr)
        return 2
    return commands[argv[0]](argv[1:])


def _sky(argv):
    from .sky import main

    return main(argv)


def _furniture(argv):
    from .furniture import main

    return main(argv)


def _textures(argv):
    import argparse

    from .textures import TextureError, fetch_textures

    parser = argparse.ArgumentParser(
        prog="python -m roadgen textures",
        description="Download the Poly Haven textures a written CARLA package asks for.",
    )
    parser.add_argument("package", help="the map's folder inside the package, the one with the .fbx")
    parser.add_argument("--resolution", help="1k, 2k or 4k, overriding the manifest")
    parser.add_argument("--overwrite", action="store_true", help="fetch files that are already there")
    args = parser.parse_args(argv)
    try:
        written = fetch_textures(
            args.package,
            overwrite=args.overwrite,
            resolution=args.resolution,
            on_progress=lambda path, index, total: print("%d/%d %s" % (index + 1, total, path)),
        )
    except TextureError as error:
        print(error, file=sys.stderr)
        return 1
    print("%d textures fetched into %s" % (len(written), args.package))
    return 0


if __name__ == "__main__":
    sys.exit(main())
