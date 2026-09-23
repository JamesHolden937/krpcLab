"""Where things are in the tree, so no script has to count directory levels.

Tools live one or two directories below the root (``tools/``,
``spaceplane/tools/``); each one puts the root on ``sys.path`` in its first
lines and from then on asks this module for every other path.

``use_venv()`` re-executes a tool under the project's ``.venv`` when the
interpreter running it has no ``krpc``.  The tools carry a
``#!/usr/bin/env python3`` shebang and system Python has no kRPC client, so
``./spaceplane/tools/quickglide.py`` would otherwise fail on its first import.
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INSTANCES = os.path.join(ROOT, "testInstances")
LOGS = os.path.join(ROOT, "logs")
SAVES = os.path.join(ROOT, "saves")
VENV_PYTHON = os.path.join(ROOT, ".venv", "bin", "python")


def instance_dir(instance):
    """``testInstances/ksp<N>``, the game copy of farm instance ``N``."""
    return os.path.join(INSTANCES, "ksp" + str(instance))


def ports(instance):
    """(rpc, stream) ports of farm instance ``N``, or (None, None) for the
    live game on kRPC's defaults."""
    if instance is None:
        return None, None
    base = instance_dir(instance)
    with open(os.path.join(base, ".rpc_port")) as fh:
        rpc = int(fh.read())
    with open(os.path.join(base, ".stream_port")) as fh:
        stream = int(fh.read())
    return rpc, stream


def use_venv():
    """Re-exec under ``.venv/bin/python`` if ``krpc`` is not importable here."""
    try:
        import krpc  # noqa: F401
        return
    except ImportError:
        pass
    if (os.path.exists(VENV_PYTHON)
            and os.path.realpath(sys.prefix) != os.path.realpath(
                os.path.join(ROOT, ".venv"))):
        os.execv(VENV_PYTHON, [VENV_PYTHON] + sys.argv)
    raise SystemExit("no krpc: run ./run.sh once to create %s" % VENV_PYTHON)
