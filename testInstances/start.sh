#!/usr/bin/env bash
#
# start.sh [n]... -- start the named instances, or 0-3, each in its own
# systemd user scope.
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
for N in "$@"; do
  [ -d "$HERE/ksp$N" ] || { echo "no instance ksp$N -- ./mkclone.sh $N" >&2; continue; }
  systemctl --user stop "ksp$N-boosterland.scope" 2>/dev/null
  systemd-run --user --scope --collect --unit="ksp$N-boosterland" \
      -- "$HERE/kwinRun.sh" "$N" > "$HERE/ksp$N-boot.log" 2>&1 &
  disown
  echo "started ksp$N (scope ksp$N-boosterland)"
  sleep 10        # stagger: four simultaneous loading screens thrash the disk
done
cat <<'NOTE'

Each takes about three minutes to reach its kRPC port.  Wait for all of them
before starting a sweep -- a sweep compares columns, and a column flown with
three instances is not the same configuration as one flown with four:

  until [ "$(for n in 0 1 2 3 4 5; do p=$(cat ksp$n/.rpc_port);
             ss -ltn | grep -c ":$p "; done | grep -c '^1$')" = 6 ]
  do sleep 10; done
NOTE
