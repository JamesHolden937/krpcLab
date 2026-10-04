#!/usr/bin/env bash
#
# start.sh [n]... -- start the named instances, or 0-5, each in its own
# systemd user scope, and wait until every one has its kRPC port open,
# relaunching any that crash during boot (see below).
#
# The scope keeps the instances out of the calling session's process tree, so
# a shell or task that exits does not take the games with it.  --collect
# cleans the unit up when it exits, so a re-run does not trip over a stale
# one.
#
# It is *not* what stops all four dying at once.  That was the machine going
# to sleep, and it was misread here first as a process-group kill and then as
# a cgroup reaper before anyone looked at the wall clock.  Run `./nosleep.sh
# start` before a sweep and `./nosleep.sh stop` after it; see the comment in
# that script for the two signatures a suspend leaves in these logs.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ $# -eq 0 ] && set -- 0 1 2 3 4 5     # six: the standard since 2026-10-02
P="$HERE/prefix%s/drive_c/users/steamuser/AppData/LocalLow/Squad/Kerbal Space Program/Player.log"
launch() {
  local N="$1"
  systemctl --user stop "ksp$N-boosterland.scope" 2>/dev/null
  systemd-run --user --scope --collect --unit="ksp$N-boosterland" \
      -- "$HERE/kwinRun.sh" "$N" > "$HERE/ksp$N-boot.log" 2>&1 &
  disown
  started[$N]=$(date +%s)
}
port_up() { local p; p="$(cat "$HERE/ksp$1/.rpc_port" 2>/dev/null)"; [ -n "$p" ] && ss -ltn | grep -q ":$p "; }
declare -A started tries
want=()
for N in "$@"; do
  [ -d "$HERE/ksp$N" ] || { echo "no instance ksp$N -- ./mkclone.sh $N" >&2; continue; }
  launch "$N"; tries[$N]=0; want+=("$N")
  echo "started ksp$N (scope ksp$N-boosterland)"
  sleep 10        # stagger: four simultaneous loading screens thrash the disk
done
[ "${START_WAIT:-1}" = 0 ] && exit 0

# **Wait for every port, and relaunch any instance that crashes on the way.**
#
# About one boot in twenty-five dies: a native crash inside DXVK's d3d11.dll
# just after "Preloading Asset Bundle Definitions" in Player.log ("Crash!!!",
# then a d3d11 stack), which KSP.log shows as its last ReStock "Removing ..."
# line.  It is a race while several instances load at once -- not VRAM (six
# peak at 5.5 of 16 GB), not NTSync (5 deaths in 66 boots with it off,
# 2026-10-04) -- and a lone relaunch has always come up.  The game process
# exits; the compositor stays, so nothing else notices, and a batch chained
# on a port wait that timed out flies five instances instead of six.
#
# So this does not return until every named port is open.  A crash is a
# missing game process with no port, at least GRACE s after launch; that
# instance is cleared with stop.sh (which takes its compositor too) and
# launched again, up to RETRIES times.  START_WAIT=0 restores the old
# fire-and-forget behaviour.  Exit status 1 if any instance never came up.
GRACE=30; RETRIES="${START_RETRIES:-3}"; DEADLINE=$(( $(date +%s) + ${START_TIMEOUT:-900} ))
while :; do
  pending=()
  for N in "${want[@]}"; do
    port_up "$N" && continue
    pending+=("$N")
    pgrep -f "ksp$N/KSP_x64.exe" >/dev/null 2>&1 && continue
    [ $(( $(date +%s) - started[$N] )) -lt $GRACE ] && continue
    why="exited"; grep -q 'Crash!!!' "$(printf "$P" "$N")" 2>/dev/null && why="crashed"
    [ "${tries[$N]}" -gt "$RETRIES" ] && continue          # given up
    if [ "${tries[$N]}" -eq "$RETRIES" ]; then
      echo "ksp$N $why at boot $RETRIES times; giving up" >&2
      tries[$N]=$(( RETRIES + 1 )); continue
    fi
    tries[$N]=$(( tries[$N] + 1 ))
    echo "ksp$N $why at boot (KSP.log: $(tail -1 "$HERE/ksp$N/KSP.log" 2>/dev/null | cut -c1-70)); relaunch ${tries[$N]}/$RETRIES"
    "$HERE/stop.sh" "$N" >/dev/null 2>&1
    launch "$N"
  done
  [ ${#pending[@]} = 0 ] && { echo "all up: ${want[*]}"; exit 0; }
  live=0; for N in "${pending[@]}"; do [ "${tries[$N]}" -le "$RETRIES" ] && live=1; done
  if [ $live = 0 ] || [ $(date +%s) -gt $DEADLINE ]; then
    echo "timed out; not up: ${pending[*]}" >&2; exit 1
  fi
  sleep 5
done
