# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-06 night -> 10-07 (~2045-0440).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-06 night / 10-07".

Defaults fingerprint **`8f16d628`** (= `42d938f0` + new off flags and
instruments; **no default changed**).  Everything committed and pushed.
Farm **stopped**, sleep inhibitor released.

## Where it stands (rigoff, the bimodal save)

| | arrival at the cone | intact on runway | vehicle kept |
|---|---|---|---|
| defaults (3 batches) | +4.0..+5.4 km, sd 3.7-5.3 | 23/56 | 46/56 |
| `GLIDE_PITCH_OFFLOAD` 0.6, Mach >= 3 | **+0.8..+1.1 km, sd 0.5-1.0** | 22/56 | **55/56** |

On `_inc` (4 vs 3 of 12) and `_high` (8 vs 8) the offload is neutral.
The user: the target is this shuttle family (cargo/bigger-wing and
mass-distribution variants); other people's craft are secondary.

## What was found

1. **The glide's long mode is kRPC's attitude controller.**  Holding 36 deg
   at 2-4 kPa takes the shuttle's whole pitch input, mostly integral, and
   kRPC keeps its pitch/yaw integrators in a roll-invariant frame
   (`kspSim/attitude.py`).  At the ~38 km reversal the long flights drop to
   +0.4 of pitch with 8-12 deg of error and 8-10 deg of slip for 30 s.
   `GLIDE_PITCH_OFFLOAD` (a body-frame trim carrying the standing input)
   removes it: long arrivals 17/24 -> 1/24.  Its two laws, both paid for:
   never learn from a saturated read-back (at 1.0 it pinned the elevons and
   departed at Mach 3-4), and only above Mach 3 (held lower it costs ~2 km
   of energy).
2. **With the long mode gone, landings are set by energy, in two places.**
   (a) The glide aims a 12 km crossing (12.9 km of energy height); the cone
   needs ~18.5 at handover.  Defaults delivered ~24 only because their
   propagator is pessimistic (`ph=`: 100 km out it predicts 13-15, the
   vehicle hands over at 17-25).  (b) The cone itself loses flights "out of
   height" whatever they bring in: speed collapses to 68-92 m/s (rolled-out
   flights exit at 106-115) with the bank 30-40 deg off its command.

## Flags and instruments added (all off / log-only)

- `GLIDE_PITCH_OFFLOAD`, `_TAU_S` (0 = pitch time_to_peak), `_MAX` (0.8;
  flown at 0.6), `_MIN_MACH` (0; flown at 3).  rot-offload, rot-offload2,
  rot-offmach, rot-hmach, rot-offorb, rot-ph, rot-hacalt (-1007).
- `HOLDABLE_BY_MACH` (+ `HOLDABLE_MACH_EDGES`): Holdable per Mach regime.
  rot-hmach-1007: 3/16 vs 5/16, null.
- `GLIDE_ENERGY_AIM`: **refuted by construction in the sim, never flown**
  -- the predicted speed at 12 km is pinned (114-147 m/s) by the
  propagator's gate-speed alpha cap.
- Log columns: `yrin=` yaw/roll input read-back; `pv=`/`el=` predicted
  speed at the gate altitude; `ph=` the prediction's own handover
  (height/speed where the arc meets the cone's entry test).

## Next

1. **The cone's speed collapse** (rot-hacalt-1007 "out of height" cluster,
   LOG7930, 7917, 7922, 7934): find why speed falls from ~220 to ~100 and
   below at 10-8 km -- the bank tracking 30-40 deg off in the cone, the
   alpha trim at +4, the speed loop.  `conesum2.py`-style table: exit speed
   splits OOH 68-92 from ok 106-115 cleanly.  This is upstream of every
   landing that reaches the cone with energy.
2. **The glide's energy target**: derive the high-gate target from the
   cone's own need instead of `HAC_ALT_M` (rot-hacalt gain: delivered
   energy follows the aim, but so did the cone's need -- fix 1 first).
3. Then promote the offload: needs a landing gain, not only arrivals and
   survival; re-fly on all three orbits and `qs_plane` (regression).
4. Carried: `TOUCHDOWN_AIM_M` per vehicle; spaceplane/CLAUDE.md "Next".
   CG-shifted `qs_shuttle2` copies as a third test axis (the user plans
   mass-distribution variants; the offload is about the standing moment).

## Traps paid this session

- `pkill`/`pgrep -f` loops kill their own shell (exit 144): use
  `scratchpad/killsims.sh`-style script files; a sim left running beside
  the farm was found that way.
- `start.sh` once failed silently after a stop; check `pgrep -c -f
  KSP_x64[.]exe` (36 when six are up).
- A scripted `str.replace` whose old text is a substring of another line
  (12-space vs 16-space indent) matches twice: assert counts.
- landsum's `surplus` is at the cone **exit**, not the handover.
- Swap 20-22 GB after every ~1 h batch; restarted before every batch.
