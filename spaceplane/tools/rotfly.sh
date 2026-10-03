#!/usr/bin/env bash
# rotfly.sh "I0 I1 .." ROUNDS OUT TREE "save|sets" "save|sets" ...
# Each round instance idx i flies arm (round+i) mod n_arms; all run in parallel.
# The autopilot flies under PyPy (.venv-pypy) by default -- PYPY=0 for CPython: ~6x cheaper ticks, so the
# time-scale governor holds a higher multiplier.  TS (default 20) is only the
# ceiling: the governor holds each phase to what its control interval allows,
# and the `loop rate` line at the end of every log says whether it did.
set -u
busy=$(ps -eo args | grep -cE '^[^ ]*python[^ ]* (spaceplane/tools/quickglide|-m spaceplane\.autopilot)')
if [ "$busy" -gt 0 ]; then echo "refusing: $busy flight process(es) already running" >&2; exit 1; fi
INST=($1); R=$2; OUT=$3; TREE=$4; shift 4; ARMS=("$@"); NA=${#ARMS[@]}
cd "$TREE"
fp=$(.venv/bin/python -c 'from spaceplane.config import defaults_fingerprint; print(defaults_fingerprint())')
: > "$OUT"; echo "rot instances='${INST[*]}' defaults=$fp pypy=${PYPY:-1} ts=${TS:-20}" >> "$OUT"
for a in "${!ARMS[@]}"; do echo "arm$a='${ARMS[$a]}'" >> "$OUT"; done
for (( r=0; r<R; r++ )); do
  echo "-- round $r" >> "$OUT"
  for i in "${!INST[@]}"; do
    a=$(( (r + i) % NA )); spec="${ARMS[$a]}"; save="${spec%%|*}"; sets="${spec#*|}"
    args=(); IFS=';' read -ra parts <<< "$sets"
    for pp in "${parts[@]}"; do [ -n "$pp" ] && args+=(--set "$pp"); done
    ( .venv/bin/python spaceplane/tools/quickglide.py --save "$save" --instance "${INST[$i]}" -n 1 --timescale "${TS:-20}" $( [ "${PYPY:-1}" = 0 ] && echo --cpython ) "${args[@]}" 2>&1 | sed "s|^|arm$a ksp${INST[$i]} |" >> "$OUT" ) &
    sleep 8
  done
  wait
done
echo "ROT DONE" >> "$OUT"
