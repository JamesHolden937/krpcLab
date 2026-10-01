#!/bin/sh
# pypysetup.sh -- the PyPy environment the simulator's fast path flies in.
#
#   .pypy/        a portable PyPy (download pypy3.11 for linux64 and unpack it
#                 here, or point PYPY at any pypy3)
#   .venv-pypy/   krpc 0.6.0 under it, plus kspsim_fastclient.pth: krpc's
#                 client patched at startup to encode and decode without
#                 protobuf's pure-Python runtime (kspSim/fastclient.py; the
#                 same bytes on the wire).  KSPSIM_FASTCLIENT=0 turns it off.
#
# Both are git-ignored.  Re-run after moving the repository: the .pth holds
# its absolute path.
set -eu
cd "$(dirname "$0")/../.."
PYPY=${PYPY:-$(ls -d .pypy/pypy3*/bin/pypy3 2>/dev/null | head -1)}
[ -n "$PYPY" ] || { echo "no PyPy: unpack one under .pypy/ or set PYPY" >&2; exit 1; }
[ -x .venv-pypy/bin/python ] || "$PYPY" -m venv .venv-pypy
.venv-pypy/bin/python -c "import krpc" 2>/dev/null || .venv-pypy/bin/pip install -q krpc==0.6.0
SITE=$(.venv-pypy/bin/python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
cat > "$SITE/kspsim_fastclient.pth" <<PTH
$(pwd)
import kspSim.fastclient as _kf; _kf.install()
PTH
echo "wrote $SITE/kspsim_fastclient.pth"
.venv-pypy/bin/python -c "import krpc.client as c; print('fast client:', c.Client._invoke.__module__)"
