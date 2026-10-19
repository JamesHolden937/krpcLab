#!/usr/bin/env bash
# ascent/tools/srot.sh TAG "name:SET" ... -- every (arm, save) once per instance, rotated.
# SAVES_LIST (default: the three stacks), LAPS (default 1).  Writes logs/rot-TAG.txt,
# ending "ROT DONE".
TAG=$1; shift; ARMS=("$@"); [ ${#ARMS[@]} -eq 0 ] && ARMS=("def:")
P=$(cd "$(dirname "$0")/../.." && pwd); OUT=$P/logs/rot-$TAG.txt; : > "$OUT"
SAVES=(${SAVES_LIST:-qs_stack qs_stack_small qs_stack_stretch}); NS=${#SAVES[@]}; NA=${#ARMS[@]}; NC=$((NA*NS))
LAPS=${LAPS:-1}
for i in 0 1 2 3 4 5; do
 (
  for k in $(seq 0 $((NC*LAPS-1))); do
    j=$(( (k + i) % NC )); arm=${ARMS[$((j/NS))]}; save=${SAVES[$((j%NS))]}
    name=${arm%%:*}; set=${arm#*:}
    args=(--instance $i -n 1 --save "$save" --timescale 1)
    [ -n "$set" ] && for s1 in ${set//,/ }; do args+=(--set "$s1"); done
    res=$(timeout 1800 "$P/ascent/tools/stackfly.py" "${args[@]}" 2>&1 | grep -E "^[0-9]+/[0-9]+ " | tail -1)
    echo "ksp$i $name $save |$res" >> "$OUT"
  done
  echo "ksp$i done" >> "$OUT"
 ) &
done
wait
echo "ROT DONE" >> "$OUT"
