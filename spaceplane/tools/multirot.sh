#!/usr/bin/env bash
# spaceplane/tools/multirot.sh TAG CYCLES "save|SET=1;SET2=2" ...
# 2 rounds per farm start (the farm restarted between cycles); the arm list
# is rotated by 2 per cycle so the rotation continues across restarts.
set -u
TAG=$1; C=$2; shift 2
ARMS=("$@"); NA=${#ARMS[@]}
cd "$(dirname "$0")/../.."
for (( c=0; c<C; c++ )); do
  if [ $c -gt 0 ]; then (cd testInstances && ./stop.sh >/dev/null 2>&1; ./start.sh >/dev/null 2>&1); fi
  ROT=()
  for (( k=0; k<NA; k++ )); do ROT+=("${ARMS[$(( (k + 2*c) % NA ))]}"); done
  spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 2 logs/rot-$TAG-c$c.txt "$PWD" "${ROT[@]}"
  swapon --show | tail -1 >> logs/rot-$TAG-c$c.txt
done
echo "ALL ROT DONE $TAG"
