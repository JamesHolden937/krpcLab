#!/usr/bin/env bash
# armfly.sh "<set>;<set>" ... -- one flight per instance, each with its own
# --set list, launched together.  TEMPORARY TEST HARNESS: ladder.py flies one
# configuration across the farm, this flies a different one on each, which is
# what a coarse bracket of a single knob wants.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAVE="${SAVE:-qs_plane}"
TS="${TS:-8.0}"
n=0
for spec in "$@"; do
  args=()
  IFS=';' read -ra parts <<< "$spec"
  for pp in "${parts[@]}"; do [ -n "$pp" ] && args+=(--set "$pp"); done
  echo "ksp$n: ${args[*]}"
  ( "$HERE/.venv/bin/python" "$HERE/quickglide.py" --save "$SAVE" \
      --instance "$n" -n 1 --timescale "$TS" "${args[@]}" \
      | sed "s/^/ksp$n ${spec} /" ) &
  n=$((n+1)); sleep 8
done
wait
