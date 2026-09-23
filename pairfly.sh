#!/usr/bin/env bash
#
# pairfly.sh "<sets A>" "<sets B>" -- two configurations, flown against each
# other with the instance and the clock controlled.
#
# TEMPORARY TEST HARNESS -- not part of the flight software.
#
# `farmfly.sh` flies one configuration n times and `armfly.sh` flies a
# different one on each instance.  Neither is what a two-arm comparison
# wants, and this project has been bitten by both of the confounds they
# leave:
#
#   * **the instance** -- three clones drifted to three different mod sets
#     and it cost a session (CLAUDE.md), so an arm that lives on ksp0 and a
#     control that lives on ksp2 differ by more than the arm; and
#   * **the clock** -- four instances left up for seven hours biased the
#     arrivals 27 km with the spread unchanged, so the batch flown second is
#     not the same experiment as the batch flown first.
#
# So each round splits the farm between the two arms and the *next* round
# swaps which half flies which.  Over an even number of rounds every arm has
# flown on every instance the same number of times, and the two halves are
# interleaved in time rather than run end to end.
#
#   ROUNDS=4 SAVE=qs_plane_inc ./pairfly.sh "" "BRAKE_FOR_DISTANCE=False"
#   ROUNDS=4 OUT=/tmp/pair.txt ./pairfly.sh "A=1" "A=2"
#
# The first argument is arm A and the second arm B; an empty string means the
# committed defaults.  Read the result with:
#
#   ./rollsum.py logs/LOG24*        # what the rollout was handed
#   ./landsum.py logs/LOG24*        # arrival, handover, wheels
#
# Every log says which arm flew it: the `config:` line carries the --set list
# and the defaults fingerprint, and SAVE_NAME carries the entry state.  Two
# logs with different fingerprints are not the same experiment however
# identical their difference lists look -- which is how three rounds of one
# batch came to be three experiments when config.py was edited underneath a
# running farm.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
# `${1?...}` and not `${1:?...}`: an *empty* arm is the committed defaults,
# which is the most common control there is, and the colon form rejects it.
A="${1?usage: pairfly.sh \"<sets A>\" \"<sets B>\"}"
B="${2?usage: pairfly.sh \"<sets A>\" \"<sets B>\"}"
# **Two saves, optionally.**  `ladder.py` rotates arms across instances only
# when the arm count is coprime to the instance count -- its own docstring
# says so -- and the commonest comparison there is, two arms on four
# instances, is exactly the case that fails: each arm is pinned to half the
# farm for the whole batch, and an instance effect cannot then be told from
# an arm effect.  Observed, as a 900 m split that was unreadable as a result.
# This script swaps halves every round by construction, so SAVE_A/SAVE_B
# compares two *saves* under one configuration with that confound removed.
SAVE="${SAVE:-qs_plane}"
SAVE_A="${SAVE_A:-$SAVE}"
SAVE_B="${SAVE_B:-$SAVE}"
ROUNDS="${ROUNDS:-4}"
TS="${TS:-6.0}"
INSTANCES="${INSTANCES:-0 1 2 3}"
OUT="${OUT:-$HERE/logs/pairfly.txt}"

# **Killing this script must kill the round.**  The flights run in
# background subshells and quickglide starts the autopilot in a session of
# its own, so `pkill -f pairfly.sh` used to leave four autopilots flying four
# vessels -- which is spaceplane failure 40, and the batch started afterwards
# reads as a regression in whatever changed last.  Observed twice in one
# session before this trap existed.
# Only on an interrupt, never on a normal exit: a normal exit has already
# waited for its flights, and pkill-ing autopilots there would reach into any
# other batch that happened to be flying.
cleanup() {
  trap - INT TERM
  echo "interrupted -- killing this batch's flights" >&2
  pkill -9 -P $$ 2>/dev/null
  pkill -9 -f 'quickglide\.py --save' 2>/dev/null
  pkill -9 -f '\-m spaceplane\.autopilot' 2>/dev/null
  wait 2>/dev/null
  exit 130
}
trap cleanup INT TERM

stale=$(ps -eo args | grep -c -- '-m spaceplane\.autopilot' || true)
if [ "${stale:-0}" -gt 1 ]; then
  echo "refusing to start: $stale autopilot(s) already flying." >&2
  echo "  pkill -9 -f spaceplane.autopilot" >&2
  exit 1
fi

# **The code must not change underneath the batch.**  Every flight re-imports
# config.py, so an edit between rounds silently splits one batch into two
# experiments -- observed, and only caught because the fingerprint is logged.
# Recorded here so the result file says which build it belongs to.
fingerprint=$(.venv/bin/python -c \
  'from spaceplane.config import defaults_fingerprint; print(defaults_fingerprint())')

: > "$OUT"
echo "saves=A:$SAVE_A B:$SAVE_B rounds=$ROUNDS timescale=$TS instances='$INSTANCES'" >> "$OUT"
echo "defaults=$fingerprint" >> "$OUT"
echo "A='${A:-(defaults)}'" >> "$OUT"
echo "B='${B:-(defaults)}'" >> "$OUT"

read -ra LIST <<< "$INSTANCES"
half=$(( ${#LIST[@]} / 2 ))

for (( round=0; round<ROUNDS; round++ )); do
  echo "-- round $round" >> "$OUT"
  idx=0
  for i in "${LIST[@]}"; do
    # Swap which half flies which arm on alternate rounds.
    if (( (idx < half) != (round % 2 == 1) )); then
      name=A; spec="$A"; save="$SAVE_A"
    else
      name=B; spec="$B"; save="$SAVE_B"
    fi
    args=()
    IFS=';' read -ra parts <<< "$spec"
    for pp in "${parts[@]}"; do [ -n "$pp" ] && args+=(--set "$pp"); done
    ( "$HERE/.venv/bin/python" "$HERE/quickglide.py" --save "$save" \
        --instance "$i" -n 1 --timescale "$TS" "${args[@]}" 2>&1 \
        | sed "s|^|$name ksp$i |" >> "$OUT" ) &
    idx=$((idx+1))
    sleep 6
  done
  wait
done
echo "PAIR DONE" >> "$OUT"
current=$(.venv/bin/python -c \
  'from spaceplane.config import defaults_fingerprint; print(defaults_fingerprint())')
if [ "$current" != "$fingerprint" ]; then
  echo "WARNING: defaults changed mid-batch ($fingerprint -> $current)." >> "$OUT"
  echo "These rounds are not one experiment.  See CLAUDE.md on the" >> "$OUT"
  echo "defaults fingerprint." >> "$OUT"
fi
swapon --show --noheadings 2>/dev/null | awk '{print "swap after the batch: "$4}' >> "$OUT"
