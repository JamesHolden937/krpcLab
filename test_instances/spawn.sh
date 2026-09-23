#!/usr/bin/env bash
#
# spawn.sh <n>... -- start the named instances and wait until each one's kRPC
# server is listening.  Each gamescope brings up its own Xwayland, so the
# instances do not contend for a display.
#
# Env: BACKEND (headless|sdl|none), FPS, RES -- passed through to run-ksp.sh.
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ $# -gt 0 ] || { echo "usage: spawn.sh <n>..." >&2; exit 1; }

for N in "$@"; do
  DIR="$HERE/ksp$N"
  [ -d "$DIR" ] || { echo "no instance ksp$N" >&2; exit 1; }
  RPC="$(cat "$DIR/.rpc_port")"
  if ss -ltn 2>/dev/null | grep -q ":$RPC"; then
    echo "ksp$N already up on $RPC"
    continue
  fi
  rm -f "$DIR/KSP.log"
  setsid nohup "$DIR/run-ksp.sh" > "$HERE/ksp$N-boot.log" 2>&1 < /dev/null &
  echo "ksp$N starting (rpc $RPC), log $HERE/ksp$N-boot.log"
done

echo "waiting for kRPC ..."
for N in "$@"; do
  RPC="$(cat "$HERE/ksp$N/.rpc_port")"
  for _ in $(seq 1 120); do
    ss -ltn 2>/dev/null | grep -q ":$RPC" && { echo "  ksp$N up on $RPC"; break; }
    sleep 5
  done
  ss -ltn 2>/dev/null | grep -q ":$RPC" || echo "  ksp$N TIMED OUT -- see ksp$N-boot.log and ksp$N/KSP.log"
done
