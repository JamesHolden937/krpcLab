#!/bin/sh
# Restart simulated instance N (default 0): kill by PID, start detached.
N=${1:-0}
cd "$(dirname "$0")/.."
for p in $(ps -eo pid,args | grep "[k]spSim.run --instance $N\b" | awk '{print $1}'); do kill $p; done
sleep 0.5
# PyPy runs the physics several times faster; CPython if it is not set up
# (see kspSim/CLAUDE.md, "Speed").  KSPSIM_PYTHON overrides.
PY=${KSPSIM_PYTHON:-.venv/bin/python}
[ -z "$KSPSIM_PYTHON" ] && [ -x .venv-pypy/bin/python ] && PY=.venv-pypy/bin/python
setsid $PY -m kspSim.run --instance $N > /dev/null 2>&1 < /dev/null &
sleep 1
