#!/usr/bin/env bash
#
# run-glide.sh -- set up the venv if needed and fly the spaceplane.
#
# The sibling of run.sh, and the same rules: never source the venv (run
# .venv/bin/python by path, so there is nothing to deactivate), forward SIGINT
# to the script so Ctrl-C shuts the autopilot down cleanly, funnel stdout and
# stderr to logs/stderr.log because nothing is supposed to print there, and
# print the log the run produced.
#
#   ./run-glide.sh
#   ./run-glide.sh --set GATE_ALT_M=1600 --set DEORBIT_LONG_BIAS_M=30000
#   ./run-glide.sh --address 192.168.1.5
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
mkdir -p "$HERE/logs"

if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install --quiet --upgrade pip
  "$VENV/bin/pip" install --quiet -r "$HERE/requirements.txt"
fi

before="$(ls "$HERE/logs" 2>/dev/null | grep -c '^LOG' || true)"
"$VENV/bin/python" -m spaceplane.autopilot "$@" \
    >> "$HERE/logs/stderr.log" 2>&1 &
pid=$!
trap 'kill -INT "$pid" 2>/dev/null' INT TERM
wait "$pid"
status=$?
trap - INT TERM

newest="$(ls -t "$HERE/logs" | grep '^LOG' | head -1)"
echo "log: $HERE/logs/$newest"
exit "$status"
