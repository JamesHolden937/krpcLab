#!/usr/bin/env bash
# stop.sh [n]... -- stop the named instances, or all of them, and prove it.
#
# Exit 0 only when nothing of any named instance is left alive; exit 1, with
# what survived on stderr, otherwise.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Whether this is "stop the farm" or "stop these instances".  It decides
# whether the global gamescope sweep at the bottom is allowed to run: it
# matches by name across the whole machine.  The watchdog calls this per
# instance to clear a corpse; a machine-wide sweep there once killed the
# other instances, which it then dutifully restarted, and so on.
ALL=0; [ $# -eq 0 ] && ALL=1
# A deliberate stop outranks the watchdog, which would otherwise put back
# everything this is here to take down.  Only when stopping the whole farm:
# the watchdog's own per-instance call must not kill the supervisor making it.
if [ $# -eq 0 ] && [ -x "$HERE/watchdog.sh" ]; then
  "$HERE/watchdog.sh" stop >/dev/null 2>&1
fi
# shellcheck source=procs.sh
. "$HERE/procs.sh"
[ $# -eq 0 ] && set -- $(farm_instances)
# **By scope and by /proc scan, then verified.**  Matching the exe's path
# with a forward slash never found the game (Proton names it
# `X:\...\kspN\KSP_x64.exe`), so this used to kill the umu-run wrapper,
# report "remaining: 0" and leave every game running (procs.sh).  Now:
# stop the instance's systemd scope (start.sh's launches: the whole cgroup
# goes), TERM whatever procs.sh still finds (the watchdog's setsid launches
# have no scope), KILL what survives 10 s of that, and fail if anything is
# left.  The nested compositor and its dbus-daemon go too: a compositor left
# holding `kspN-wl` makes the next instance on that socket die ~75 s after
# its kRPC port comes up.
for N in "$@"; do
  was=$(inst_pids "$N" | wc -l)
  systemctl --user stop "ksp$N-boosterland.scope" 2>/dev/null
  pids=$(inst_pids "$N"); [ -n "$pids" ] && kill $pids 2>/dev/null
  if [ "$was" -gt 0 ]; then echo "stopping ksp$N ($was processes)"; else echo "ksp$N not running"; fi
done
for _ in $(seq 20); do
  left=0; for N in "$@"; do left=$(( left + $(inst_pids "$N" | wc -l) )); done
  [ "$left" = 0 ] && break
  sleep 0.5
done
for N in "$@"; do
  pids=$(inst_pids "$N")
  [ -n "$pids" ] && { echo "ksp$N: SIGKILL to $(echo $pids | wc -w) survivors"; kill -9 $pids 2>/dev/null; }
done
sleep 1
# gamescope and its reaper (the old launch path) exit with the game; nudge
# them if they linger.  Only when stopping everything -- see ALL.
if [ "$ALL" = 1 ]; then
  for p in $(pgrep -x gamescope 2>/dev/null) $(pgrep -x gamescopereaper 2>/dev/null); do
    kill "$p" 2>/dev/null
  done
fi
bad=0; summary=()
for N in "$@"; do
  pids=$(inst_pids "$N")
  [ -n "$pids" ] || continue
  bad=1
  summary+=("ksp$N: $(for p in $pids; do cat /proc/$p/comm 2>/dev/null; done | sort | uniq -c | awk '{printf "%s%s ", $2, ($1>1 ? "x"$1 : "")}')")
done
if [ "$bad" = 1 ]; then
  printf 'STILL RUNNING: %s\n' "${summary[@]}" >&2
  exit 1
fi
echo "stopped: nothing of $(printf "ksp%s " "$@")left; KSP games machine-wide: $(game_count)"
