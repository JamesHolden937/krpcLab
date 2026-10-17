#!/usr/bin/env bash
# boosterland/tools/brot.sh TAG "name:SET" ... -- every (arm, save) once per instance, rotated
TAG=$1; shift; ARMS=("$@"); P=$(cd "$(dirname "$0")/../.." && pwd); OUT=$P/logs/rot-$TAG.txt; : > "$OUT"
SAVES=(${SAVES_LIST:-qs_heavy quicksave}); NS=${#SAVES[@]}; NA=${#ARMS[@]}; NC=$((NA*NS))
LAPS=${LAPS:-1}
for i in 0 1 2 3 4 5; do
 (
  for k in $(seq 0 $((NC*LAPS-1))); do
    j=$(( (k + i) % NC )); arm=${ARMS[$((j/NS))]}; save=${SAVES[$((j%NS))]}
    name=${arm%%:*}; set=${arm#*:}
    args=(-n 1 --save "$save" --timescale 1 --set AIM_BIAS_NORTH_M=0 --set AIM_BIAS_EAST_M=0)
    [ -n "$set" ] && for s1 in ${set//,/ }; do args+=(--set "$s1"); done
    res=$("$P/testInstances/fly.sh" $i "${args[@]}" 2>&1 | grep -E "^\s+[0-9]+/[0-9]+ " | tail -1)
    echo "ksp$i $name $save |$res" >> "$OUT"
  done
  echo "ksp$i done" >> "$OUT"
 ) &
done
wait
echo "ROT DONE" >> "$OUT"
