#!/usr/bin/env bash
#
# nosleep.sh start|stop|status -- hold a systemd sleep inhibitor for as long
# as flights are being flown.
#
# A suspended machine does not look like a suspended machine from inside this
# harness.  It looks like: every KSP process dying in the same second with an
# orderly Unity OnDestroy cascade in its log, and then, after resume, flights
# running about ten times slower than real time until they hit quickfly's
# wall-clock timeout with the booster still at 32 km.  Both were misread here
# once -- the first as a cgroup reaper killing the process group, the second
# as the machine being short of memory -- and a session was spent on each.
#
# The inhibitor lives in its own scope so it survives whatever started it.
# **Release it when the flying is done**; it stops the machine idling to
# sleep, which is the user's setting and not ours to keep.
set -uo pipefail
UNIT=boosterland-nosleep
case "${1:-start}" in
  start)
    systemctl --user is-active --quiet "$UNIT.scope" && { echo "already held"; exit 0; }
    systemd-run --user --scope --collect --unit="$UNIT" -- \
        systemd-inhibit --what=sleep:idle:handle-lid-switch \
                        --who=boosterland --why="KSP test flights in progress" \
                        --mode=block sleep infinity >/dev/null 2>&1 &
    disown
    sleep 1
    systemctl --user is-active --quiet "$UNIT.scope" \
        && echo "sleep inhibited (scope $UNIT.scope)" \
        || { echo "FAILED to take the inhibitor" >&2; exit 1; }
    ;;
  stop)
    systemctl --user stop "$UNIT.scope" 2>/dev/null
    echo "sleep inhibitor released"
    ;;
  status)
    systemctl --user is-active --quiet "$UNIT.scope" && echo held || echo "not held"
    systemd-inhibit --list 2>/dev/null | grep -i boosterland || true
    ;;
  *) echo "usage: nosleep.sh start|stop|status" >&2; exit 2;;
esac
