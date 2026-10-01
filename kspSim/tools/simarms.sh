#!/bin/sh
# simarms.sh SAVE K "ARM0 sets" "ARM1 sets" ... -- screen arms on the sim.
#
# One kspSim instance per (arm, flight): instance i flies arm i % n_arms, so
# K flights of every arm run at once (n_arms * K instances).  Sets are
# space-separated --set arguments, e.g. "HAC_WEAVE_MAX_DEG=75 DRAIN_RESIDUAL=True".
# Output: one line per flight, prefixed "armN", as quickglide prints it.
# Screening only (kspSim/CLAUDE.md): confirm on the farm.
SAVE=$1; K=$2; shift 2
cd "$(dirname "$0")/../.."
NA=$#
OUT=$(mktemp -d)
i=0
for k in $(seq 1 "$K"); do
  a=0
  for sets in "$@"; do
    ./kspSim/restart.sh $i
    args=""
    for s in $sets; do args="$args --set $s"; done
    PYPY=""
    [ -x .venv-pypy/bin/python ] && PYPY="--pypy"
    ( ./spaceplane/tools/quickglide.py -n 1 --save "$SAVE" --instance sim$i \
        --timescale 8 --timeout 3000 $PYPY $args 2>&1 | sed "s|^|arm$a |" \
        > "$OUT/$i.txt" ) &
    i=$((i+1)); a=$((a+1))
  done
done
wait
cat "$OUT"/*.txt
rm -rf "$OUT"
