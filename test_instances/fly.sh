#!/usr/bin/env bash
#
# fly.sh <n> [quickfly args...] -- fly instance ksp<n> with the project's
# quickfly harness, on that instance's kRPC ports.
#
#   ./fly.sh 0 -n 3
#   ./fly.sh 1 -n 3 --set AIM_BIAS_EAST_M=0
#   ./fly.sh 2 --compare DRAG_WARMUP_SAMPLES=1,4 -n 2
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(dirname "$HERE")"
N="${1:?usage: fly.sh <n> [quickfly args...]}"; shift
DIR="$HERE/ksp$N"
[ -d "$DIR" ] || { echo "no instance ksp$N" >&2; exit 1; }
RPC="$(cat "$DIR/.rpc_port")"
STREAM="$(cat "$DIR/.stream_port")"

ss -ltn 2>/dev/null | grep -q ":$RPC" || {
  echo "ksp$N is not listening on $RPC -- start it with $DIR/run-ksp.sh" >&2
  exit 1
}

cd "$PROJ"
# Use the project venv's interpreter, the way run.sh does: krpc is installed
# there, not system-wide, and quickfly.py's shebang picks up system python.
PY_BIN="$PROJ/.venv/bin/python"
[ -x "$PY_BIN" ] || { echo "no venv at $PY_BIN -- run ./run.sh once to create it" >&2; exit 1; }
exec "$PY_BIN" ./quickfly.py --rpc-port "$RPC" --stream-port "$STREAM" "$@"
