#!/usr/bin/env bash
#
# run.sh -- the one way to fly an autopilot against a running game.
#
#   ./run.sh                                  # in-game launcher: a button per autopilot
#   ./run.sh --address 192.168.1.5            # ... against a game on another machine
#   ./run.sh --pilot spaceplane               # one autopilot; its panel waits for START
#   ./run.sh --pilot booster --autostart      # ... or starts at once
#   ./run.sh --pilot plane --set GATE_ALT_M=1600 --set LOG_INTERVAL_UT=1
#
# --pilot takes spaceplane (plane) or boosterland (booster); the list is
# PILOTS in common/launcher.py.  Every other flag goes to the autopilot (or the
# launcher) unchanged.
#
# Creates .venv on first use and runs .venv/bin/python by path -- never
# sourced, so there is nothing to deactivate.  Nothing prints: stdout and
# stderr go to logs/stderr.log, which only ever holds crashes from before a
# logbook opened.  The flight runs in the foreground, so Ctrl-C reaches it
# directly and it hands the vessel back before exiting.  (It used to run in
# the background with a trap forwarding SIGINT, which did nothing: bash starts
# background jobs with SIGINT ignored, and Python keeps an ignored SIGINT.)
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

pilot=""
args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --pilot)   pilot="${2:?--pilot needs a name}"; shift 2 ;;
        --pilot=*) pilot="${1#--pilot=}"; shift ;;
        -h|--help) sed -n '3,21p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         args+=("$1"); shift ;;
    esac
done

if [ -n "$pilot" ]; then
    module=$("$PY" -c "import sys; from common.launcher import module_for; \
print(module_for(sys.argv[1]) or '')" "$pilot")
    [ -n "$module" ] || { echo "unknown pilot: $pilot" >&2; exit 2; }
    what="$pilot"
else
    module="common.launcher"
    what="launcher"
fi

mkdir -p logs
before=$(ls -1 logs 2>/dev/null | grep -c '^LOG[0-9]*$')
echo "running $what - use the in-game panel; Ctrl-C to stop"
"$PY" -m "$module" "${args[@]}" >>logs/stderr.log 2>&1
status=$?

# Log numbers only grow, so the new ones sort last.
new=$(ls -1v logs 2>/dev/null | grep '^LOG[0-9]*$' | tail -n +$((before + 1)))
if [ -n "$new" ]; then
    echo "exit $status; logs: $(echo "$new" | sed 's#^#logs/#' | tr '\n' ' ')"
else
    echo "exit $status; no new log (see logs/stderr.log)"
fi
exit "$status"
