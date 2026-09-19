"""`python -m roadgen <command> …` — the package's commands.

    python -m roadgen sky /opt/carla Town01Package Town01

There is one so far. Commands live here rather than in the modules that do the
work, so that `roadgen.sky` stays a module the package can import eagerly.
"""

import sys


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    commands = {"sky": _sky}
    if not argv or argv[0] not in commands:
        print("usage: python -m roadgen {%s} …" % "|".join(sorted(commands)), file=sys.stderr)
        return 2
    return commands[argv[0]](argv[1:])


def _sky(argv):
    from .sky import main

    return main(argv)


if __name__ == "__main__":
    sys.exit(main())
