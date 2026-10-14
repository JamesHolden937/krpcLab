#!/bin/bash
# Fly simmission over every destination x thrust x start date, in parallel,
# and print one line per flight (outcome, total dv, wall seconds).
#
#   lowThrust/tools/sweep.sh OUTDIR "THRUSTS_N" "START_UTS" [DESTINATIONS...]
#   lowThrust/tools/sweep.sh /tmp/sw "100 200 500 2000" "0 3000000 6000000 9000000" moho duna
#
# From LKO 100 km, 5 t vessel.  JOBS (default 14) flights at once, TMO
# (default 900) seconds each, EXTRA extra simmission arguments, SIM the
# command (default lowThrust/tools/simmission).
cd "$(dirname "$0")/../.."
out=$1; th=$2; uts=$3; shift 3
dests=${@:-moho mun minmus duna eve ike dres jool eeloo}
sim=${SIM:-./lowThrust/tools/simmission}
mkdir -p "$out"
declare -A alt=([moho]=50 [mun]=50 [minmus]=30 [duna]=200 [eve]=150 [ike]=50
                [dres]=50 [jool]=500 [eeloo]=50 [gilly]=20 [pol]=20 [tylo]=50
                [laythe]=100 [vall]=50 [bop]=50)
for d in $dests; do for t in $th; do for u in $uts; do
  echo "$d $t $u ${alt[$d]}"
done; done; done | xargs -P "${JOBS:-14}" -L1 bash -c 'f='"$out"'/$0_$1_$2.txt; s=$SECONDS; timeout '"${TMO:-900}"' '"$sim"' --from "kerbin: ap=100 pe=100" --to "$0: ap=$3 pe=$3" --thrust $1 --ut $2 '"${EXTRA}"' > $f 2>&1; echo "wall $((SECONDS-s))" >> $f'
for f in "$out"/*.txt; do
  n=$(basename "$f" .txt)
  r=$(grep -oE '^(ARRIVED|FAILED): .*' "$f" | cut -c1-70)
  [ -z "$r" ] && r="CRASH/TIMEOUT $(grep -m1 -E 'Error|Exception' "$f")"
  printf "%-22s %-72s %s %s\n" "$n" "$r" "$(grep -o 'total [0-9.]* m/s' "$f")" "$(grep wall "$f")"
done
