#!/usr/bin/env bash
# Set up the venv, run the autoland script, and tear both down cleanly on
# Ctrl-C or when the in-game TERMINATE button ends the script.
#
# Usage:  ./run.sh [--address HOST] [--set FIELD=VALUE ...]
set -uo pipefail
cd "$(dirname "$0")"

VENV=".venv"
PY="$VENV/bin/python"

if [ ! -x "$PY" ]; then
    echo "creating venv in $VENV"
    python3 -m venv "$VENV" || exit 1
fi
if ! "$PY" -c "import krpc" 2>/dev/null; then
    echo "installing requirements"
    "$VENV/bin/pip" install --quiet -r requirements.txt || exit 1
fi

mkdir -p logs

# The script itself never writes to stdout; this file only catches crashes
# that happen before the logbook is open (bad args, no kRPC server, ...).
"$PY" -m boosterland.autoland "$@" >>logs/stderr.log 2>&1 &
pid=$!

# Ctrl-C is forwarded as SIGINT so the script can null the throttle, drop the
# autopilot and remove its panel before exiting.
trap 'kill -INT "$pid" 2>/dev/null' INT TERM

echo "running (pid $pid) - use the in-game panel to start or terminate"
wait "$pid"
status=$?
trap - INT TERM

latest=$(ls -1v logs/LOG* 2>/dev/null | tail -n 1)
echo "exit $status; log: ${latest:-none}"
# Nothing to deactivate: the venv is used by path, never sourced.
exit "$status"
