# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-08, afternoon to midnight (~1530-0000).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-08 afternoon/evening" and
its later sub-sections (evening, night).

Defaults fingerprint **`efc472d5`** -- **no default changed today**; every new mechanism is a flag, off.  Committed and pushed.
Farm **stopped**, sleep inhibitor released.

## What today established (in order of confidence)

1. **Below Mach 3 the shuttle holds 0.57-0.75 of the ~40 deg the glide
   commands**, so the glide's predicted cone-entry energy reads 5-7 km low
   and entries above the cone's ~20-21 km ceiling land 4-6 km long.
   Mach-banded `HOLDABLE_PRIOR` (`holdprior.py --by-mach --mach 0.8`, needs
   `HOLDABLE_BY_MACH`) fixes the prediction (entry 18.9 -> 17.4 km); landings
   null (21/36 v 21/36).
2. **The aero table is re-probed at the surfaces' present deflection**, so
   at cone entry its subsonic rows say L/D 0.85-1.7 where the cone flies
   3.3-3.7 (`cone ladder` log line).  **The flown polar**
   (`tools/conepolar.py` -> untracked `logs/conepolar/`): lift peaks ~139
   m^2 near 12 deg (table ~300); the upper cone flies 15-17 deg, stalled,
   L/D ~1.2.  kspSim cannot screen any of this (its subsonic lift is 1.5x the
   table, the game's 0.6x; kspSim/CLAUDE.md gap 8).
3. **The split rudder is a real speedbrake** (`tools/splitprobe.py`; the
   user's idea): both Big-S fins at Deploy Angle 38 -> approach drag +101%
   (L/D 4.17 -> 1.89), lower cone +52%, yaw/roll 0.0.  Sideslip is held to
   ~3.5 deg by the rudder (weathercock ~28 m^3 q/deg): weak.  The old
   approach airbrake never deployed the fins.
4. **The approach dive** (~1 flight in 5 all day: flare at 95-133 m/s, 57-93
   sink) is the largest killer.  It starts from handovers 0.8-1.1 km over
   need: S-turns bank 40 at alpha ~1 while the speed law dives (up to 35
   deg) to make speed.

## Flags built today (all off)

| flag | result |
|---|---|
| `GLIDE_CONE_ENERGY` / `GLIDE_CONE_CEILING` | ceiling right (19-23 km) but engages too late without the prior |
| `HOLDABLE_PRIOR` by Mach | entry energy -1.5 km; landings null |
| `HAC_SPEED_EAS` | **refuted on the farm** 1/12 |
| `AERO_REFRESH_NEAR_MACH` | half-connected, 7/12 v 8/12 |
| `HAC_LD_FLOWN_POLAR` | connected; exposes the no-lap/lap gap, 3/12 |
| `HAC_PAST_KEEPS_LINEUP` | swamped by dives, 3/12 v 2/12 |
| **`HAC_SPLIT_BRAKE`** (+`_ON_BRANCH`) | **7/12 v 5/12, long 1 v 5, high 4/4** (rot-split-1008); ungated it opened while stalled and lost 2.  Gated version only flown inside stacks |
| `ROLLOUT_SPLIT_BRAKE` | flown only in stacks; the roll is already ~380 m median |
| `APPROACH_SINK_GUARD` | v2: 0 dives, 0 lost -- but stalls short / lands long; **worse** pooled (2/12, 2/12, 4/12 stack) |
| `APPROACH_SPLIT_ON_GUARD` | 6/12 v 5/12, then 2/12 v 7/12 (the guard's stalls) |

Best configuration is unchanged (offload + canard trim + measured cone
L/D); on rigoff/inc/high tonight it landed 9/12, 6/12, 5/12, 7/12.

## Next, in order

1. **Fly `HAC_SPLIT_BRAKE` + `HAC_SPLIT_ON_BRANCH` + `HAC_LD_FLOWN_POLAR`
   alone against the base, 24+ an arm** -- the one mechanism with a
   positive first batch (long 1 v 5), never flown gated and unstacked.
2. **The approach dive, at its source**: high handovers.  The split brake
   in the cone should hand over lower; if dives remain, act on the
   *energy* (brake in the approach with the speed law's target as its
   floor, wings level instead of the 40 deg S-turn) rather than on the sink.
   Do not cap sink by flooring alpha (tonight's guard: stalls).
3. The Shuttle's missing pieces (journal): choose overhead/straight-in by
   energy; a TAEM segment with S-turns before the cone; dynamic-pressure
   schedule instead of a fixed 112 m/s *true* cone target.
4. Carried: eccentric orbit ~17-21 km long; `TOUCHDOWN_AIM_M` per vehicle;
   regenerate `logs/holdprior` and `logs/conepolar` after any attitude or
   trim change (conepolar skips `spb>0` ticks).

## Traps paid today

- **A cap on a symptom moves the failure**: the sink guard turned dives into
  stalls.  Check the flare entry speed, not only the sink.
- **Label arms from each log's `config:`**: `HAC_LD_MEASURED` contains
  "EAS"; rotfly with more arms than instances flies unevenly -- rotate.
- **A background command ending in `&` returns at once**; wait on `ROT DONE`.
- **Read per-orbit rates**: "65%" was rigoff only; blended ~50%.
- A "reserve" on a spender is a handover higher: the 400 m cone-brake
  reserve fed the approach dives.
- Code committed mid-batch reaches later rounds (rotfly runs the tree): keep
  new code flag-gated.
- Swap reaches ~20 GB in 2 rounds; restart every 2 rounds.
