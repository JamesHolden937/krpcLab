#!/usr/bin/env bash
# armfly.sh "<set>;<set>" ... -- one flight per instance, each with its own
# --set list, launched together.  TEMPORARY TEST HARNESS: ladder.py flies one
# configuration across the farm, this flies a different one on each, which is
# what a coarse bracket of a single knob wants.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
SAVE="${SAVE:-qs_plane}"
TS="${TS:-20}"
n=0
for spec in "$@"; do
  args=()
  IFS=';' read -ra parts <<< "$spec"
  for pp in "${parts[@]}"; do [ -n "$pp" ] && args+=(--set "$pp"); done
  echo "ksp$n: ${args[*]}"
  ( "$ROOT/.venv/bin/python" "$HERE/quickglide.py" $( [ "${PYPY:-1}" = 0 ] && echo --cpython ) --save "$SAVE" \
      --instance "$n" -n 1 --timescale "$TS" "${args[@]}" \
      | sed "s/^/ksp$n ${spec} /" ) &
  n=$((n+1)); sleep 8
done
wait
