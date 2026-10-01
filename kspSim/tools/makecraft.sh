#!/bin/bash
# makecraft.sh INSTANCE MAIN_SAVE AIR_SAVE [ORBIT_SAVE]
#
# Everything a new craft needs before the simulator can be trusted with it,
# in order, on one farm instance (see kspSim/CLAUDE.md, "A new craft"):
#
#   1. probe the model: MAIN_SAVE's state, tables and part data, with the
#      control, gear and airbrake tables taken on AIR_SAVE (the same craft,
#      in the air -- surfaces do not answer input in vacuum).  No air save?
#      kspSim/tools/airsave.py makes one by deorbiting an orbital save.
#   2. fly the flight-test battery in the game, every physics frame recorded:
#      on AIR_SAVE free/pitch/roll/yaw/mix/trim/gear/brakes (with the reported
#      torques), and on ORBIT_SAVE (if given) inertia/rcs/throttle -- the
#      vacuum tests measure inertia, RCS and gimbals exactly.
#   3. the fidelity summary of every recording against the model.
#
# Then read the summary: a torque or force bias in one test is a defect to
# find (fidelity.py --segments for the detail, oracle.py to tell a table
# from the physics).  Fly one autopilot flight with the passive recorder
# (flighttest.py --attach) and read regimes.py on it before trusting a
# closed-loop comparison.  calibrate.py is the last resort, not the first;
# thermal needs no fit -- record a game entry with observe.py --fluxes and
# score it with thermcheck.py.
set -u
N=$1; MAIN=$2; AIR=$3; ORBIT=${4:-}
cd "$(dirname "$0")/../.."
OUT=logs/kspsim/ft
mkdir -p $OUT
./kspSim/tools/probe.py --instance $N --save $MAIN --controls-save $AIR || exit 1
# Sideslip tables, then the levels near zero where trim lives (drag is
# quadratic in deflection; the base solo inputs stop at +-0.5).
./kspSim/tools/probe.py --instance $N --save $MAIN --controls-save $AIR --solo-beta --solo-levels || exit 1
if [ "$AIR" != "$MAIN" ]; then
  ./kspSim/tools/probe.py --instance $N --save $AIR --aero-from $MAIN --no-terrain || exit 1
fi
for s in free pitch roll yaw mix trim gear brakes; do
  ./kspSim/tools/flighttest.py --instance $N --save $AIR --script $s --torques \
      -o $OUT/g_${AIR}_$s.jsonl
done
if [ -n "$ORBIT" ]; then
  [ "$ORBIT" != "$MAIN" ] && ./kspSim/tools/probe.py --instance $N --save $ORBIT \
      --aero-from $MAIN --no-terrain
  for s in inertia rcs throttle; do
    ./kspSim/tools/flighttest.py --instance $N --save $ORBIT --script $s -o $OUT/g_${ORBIT}_$s.jsonl
  done
fi
./kspSim/tools/fidsum.py $OUT/g_${AIR}_*.jsonl ${ORBIT:+$OUT/g_${ORBIT}_*.jsonl}
