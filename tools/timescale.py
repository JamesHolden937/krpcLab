#!/usr/bin/env python3
"""Set and read back the in-game time scale of a measurement instance.

The command-line face of ``common/timescale.py``, which the harnesses import.

    ./tools/timescale.py 0 2.0        # instance ksp0 at 2x
    ./tools/timescale.py 0 max        # as fast as its main thread will go
    ./tools/timescale.py 0 off        # back to 1x
    ./tools/timescale.py 0            # what is it actually achieving?

The distinction that matters is **commanded** versus **achieved**.  KSP's
physics is single-threaded, so asking for 4x on a machine that can sustain 1.6x
gets you 1.6x -- and from inside the game the two are indistinguishable, which
is why the plugin measures game seconds against real seconds and writes the
answer back out.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common import timescale  # noqa: E402

if __name__ == "__main__":
    sys.exit(timescale.main(description=__doc__))
