#!/usr/bin/env bash
# stop.sh [n]... -- stop the named instances, or all of them.
#
# Kills by pid rather than pkill pattern: a pkill -f pattern also matches the
# shell that is running it, which silently kills the caller.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Whether this is "stop the farm" or "stop these instances".  It decides
# whether the global sweep at the bottom is allowed to run: gamescope and
# wineserver are matched by name across the whole machine, so running it for
# a named instance takes down the *other* instances' wine sessions too.  The
# watchdog found that within a minute of existing -- it cleared one corpse
# with `stop.sh 2` and the sweep killed ksp3, which it then dutifully
# restarted, and so on.
ALL=0; [ $# -eq 0 ] && ALL=1
# A deliberate stop outranks the watchdog, which would otherwise put back
# everything this is here to take down.  Only when stopping the whole farm:
# watchdog.sh calls this per instance to clear a corpse before relaunching it,
# and that call must not kill the supervisor making it.
if [ $# -eq 0 ] && [ -x "$HERE/watchdog.sh" ]; then
  "$HERE/watchdog.sh" stop >/dev/null 2>&1
fi
if [ $# -eq 0 ]; then
  set -- $(ls -d "$HERE"/ksp[0-9]* 2>/dev/null | sed 's#.*/ksp##' | grep -E '^[0-9]+$')
fi
for N in "$@"; do
  hit=0
  for p in $(pgrep -f "ksp$N/KSP_x64.exe" 2>/dev/null); do
    grep -qa "bash" /proc/$p/comm 2>/dev/null && continue
    kill "$p" 2>/dev/null && hit=1
  done
  [ "$hit" = 1 ] && echo "stopped ksp$N" || echo "ksp$N not running"
done
sleep 3
# The nested compositor does *not* exit with the game, and leaving it behind
# is not harmless: it keeps holding the `kspN-wl` wayland socket, so the next
# start.sh brings up an instance that collides with the corpse of the last
# one.  Three of them accumulated over two restarts here and every instance
# launched afterwards died about 75 s after its kRPC server came up -- which
# reads exactly like the unexplained mass exits above and is not one.  Match
# on the instance's own .inner.sh path, which only its compositor carries.
for N in "$@"; do
  for p in $(pgrep -f "kwin_wayland --virtual.*ksp$N/.inner.sh" 2>/dev/null); do
    kill "$p" 2>/dev/null && echo "stopped ksp$N compositor"
  done
done
sleep 1
# gamescope and its wineserver exit on their own once the game is gone; give
# them a nudge if they linger.  Only when stopping everything -- see ALL.
if [ "$ALL" = 1 ]; then
  for p in $(pgrep -x gamescope 2>/dev/null) $(pgrep -x wineserver 2>/dev/null) \
           $(pgrep -x gamescopereaper 2>/dev/null); do
    kill "$p" 2>/dev/null
  done
  sleep 1
fi
echo "remaining: $(pgrep -x gamescope 2>/dev/null | wc -l) gamescope, $(pgrep -x wineserver 2>/dev/null | wc -l) wineserver, $(pgrep -fc 'kwin_wayland --virtual' 2>/dev/null) compositor"
