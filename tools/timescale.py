#!/usr/bin/env python3
"""Set and read back the in-game time scale of a measurement instance.

The command-line face of ``common/timescale.py``, which the harnesses import:

    ./tools/timescale.py 5 max
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common import timescale  # noqa: E402

if __name__ == "__main__":
    sys.exit(timescale.main())
