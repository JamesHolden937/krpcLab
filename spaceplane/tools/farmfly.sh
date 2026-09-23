#!/usr/bin/env bash
#
# farmfly.sh -- one configuration, N flights on each instance, tagged.
#
# TEMPORARY TEST HARNESS -- not part of the flight software.  `ladder.py`
# flies several *saves* across the farm and `armfly.sh` flies a different
# configuration on each instance; this flies **one** configuration many times,
# which is what CLAUDE.md's rule asks for -- the first batch of any comparison
# is n flights of one arm, not one flight of n arms.
#
# It records which instance flew which log, because that split is what
# separates "this control loop amplifies" from "these two games are not the
# same", and nothing else could answer it after the fact.
#
#   OUT=/tmp/base.txt N=8 ./spaceplane/tools/farmfly.sh
#   OUT=/tmp/arm.txt  N=8 SETS="CROSS_DEADBAND_PER_KM=4;CROSS_DEADBAND_MAX_M=6000" ./spaceplane/tools/farmfly.sh
#   OUT=/tmp/arm.txt  N=10 INSTANCES="0 2 3" TS=6.0 ./spaceplane/tools/farmfly.sh
#
# Then: ./spaceplane/tools/landsum.py logs/LOG16*   (and read the arrival and the landing bias
# as two separate numbers -- see docs/spaceplane/design.md).
#
# Before believing the result, three one-line farm checks that have each cost
# this project a day; see docs/testInstances.md:
#
#   pgrep -af spaceplane.autopilot                 # one per busy instance
#   md5sum testInstances/ksp*/saves/default/qs_plane.sfs | awk '{print $1}' | sort -u | wc -l
#   swapon --show                                  # after the batch, not just before
#
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"
SAVE="${SAVE:-qs_plane}"
N="${N:-8}"
TS="${TS:-6.0}"
INSTANCES="${INSTANCES:-0 1 2 3}"
SETS="${SETS:-}"
# GOVERN=--no-govern turns the time-scale governor off, so --timescale means
# a fixed multiplier for the whole flight -- the old meaning.  Only for
# measuring the governor against itself; see docs/spaceplane/design.md.
GOVERN="${GOVERN:-}"
OUT="${OUT:-$ROOT/logs/farmfly.txt}"

args=()
IFS=';' read -ra parts <<< "$SETS"
for pp in "${parts[@]}"; do [ -n "$pp" ] && args+=(--set "$pp"); done

# Matched on the *shape* of the process -- python running the module -- and
# not on the bare module name, which any shell that mentions it in its own
# command line also matches.
stale=$(ps -eo args | grep -c -- '-m spaceplane\.autopilot' || true)
if [ "${stale:-0}" -gt 0 ]; then
  echo "refusing to start: $stale autopilot(s) already flying." >&2
  echo "A killed batch leaves its flight running and the next batch reads" >&2
  echo "as a regression in whatever changed last -- spaceplane failure 40." >&2
  echo "  pkill -9 -f spaceplane.autopilot" >&2
  exit 1
fi

: > "$OUT"
echo "save=$SAVE n=$N timescale=$TS instances='$INSTANCES' sets='$SETS'" >> "$OUT"
for i in $INSTANCES; do
  ( "$ROOT/.venv/bin/python" "$HERE/quickglide.py" --save "$SAVE" \
      --instance "$i" -n "$N" --timescale "$TS" ${GOVERN:+$GOVERN} "${args[@]}" 2>&1 \
      | sed "s|^|ksp$i |" >> "$OUT" ) &
  sleep 6
done
wait
echo "BATCH DONE" >> "$OUT"
swapon --show --noheadings 2>/dev/null | awk '{print "swap after the batch: "$4}' >> "$OUT"
