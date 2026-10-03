#!/usr/bin/env bash
# savefly.sh ROUNDS OUT "sets A" "sets B" ... : instance i flies ${SAVE:-qs_s2_hac}<i>;
# the arm alternates by (round + i) so every save sees every arm.
set -u
# Each save is pinned to one instance, so every arm sees every save -- rotfly.sh
# with more arms than instances flies only some of them (rot-flarep2-1003).
busy=$(ps -eo args | grep -cE '^[^ ]*python[^ ]* (spaceplane/tools/quickglide|-m spaceplane\.autopilot)')
if [ "$busy" -gt 0 ]; then echo "refusing: $busy flight process(es) already running" >&2; exit 1; fi
R=$1; OUT=$2; shift 2; ARMS=("$@"); NA=${#ARMS[@]}
cd "$(dirname "$0")/../.."
fp=$(.venv/bin/python -c 'from spaceplane.config import defaults_fingerprint; print(defaults_fingerprint())')
: > "$OUT"; echo "savefly defaults=$fp pypy=1 ts=20" >> "$OUT"
for a in "${!ARMS[@]}"; do echo "arm$a='${ARMS[$a]}'" >> "$OUT"; done
for (( r=0; r<R; r++ )); do
  echo "-- round $r" >> "$OUT"
  for i in 0 1 2 3 4 5; do
    a=$(( (r + i) % NA )); args=(); IFS=';' read -ra parts <<< "${ARMS[$a]}"
    for pp in "${parts[@]}"; do [ -n "$pp" ] && args+=(--set "$pp"); done
    ( .venv/bin/python spaceplane/tools/quickglide.py --save "${SAVE:-qs_s2_hac}$i" --instance $i -n 1 --timescale 20 "${args[@]}" 2>&1 | sed "s|^|arm$a ksp$i |" >> "$OUT" ) &
    sleep 8
  done
  wait
done
echo "ROT DONE" >> "$OUT"
