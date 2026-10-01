#!/bin/sh
# Fly one save on N simulated instances at once, K flights each.
#   ./kspSim/tools/simfly.sh qs_plane 6 1 [--set FIELD=VALUE ...]
#   PILOT=booster ./kspSim/tools/simfly.sh quicksave 4 3
# Each instance is a kspSim server on 50200+2i; results go to stdout, one
# line per flight, exactly as the harness (quickglide.py, or quickfly.py for
# PILOT=booster) prints them.
SAVE=$1; N=$2; K=$3; shift 3
cd "$(dirname "$0")/../.."
OUT=$(mktemp -d)
i=0
while [ $i -lt "$N" ]; do
  ./kspSim/restart.sh $i
  if [ "${PILOT:-spaceplane}" = booster ]; then
    ./boosterland/tools/quickfly.py -n "$K" --save "$SAVE" --instance sim$i \
        --rpc-port $((50200 + 2 * i)) --stream-port $((50201 + 2 * i)) \
        "$@" > "$OUT/$i.txt" 2>&1 &
  else
    PYPY=""
    [ -x .venv-pypy/bin/python ] && PYPY="--pypy"
    ./spaceplane/tools/quickglide.py -n "$K" --save "$SAVE" --instance sim$i \
        --timescale 8 --timeout 3000 $PYPY "$@" > "$OUT/$i.txt" 2>&1 &
  fi
  i=$((i+1))
done
wait
cat "$OUT"/*.txt
rm -rf "$OUT"
