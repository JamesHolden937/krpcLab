# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-06 (day, ~0735-1600).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-06 morning" and
"... afternoon".

Defaults fingerprint **`63ef8321`** (= `6a226a64` + the off flags below;
`6a226a64` = the overnight `d91cd0ad` + `ROLLOUT_STEER_LEAD_S` 3).
Everything committed and pushed.  Farm **stopped**, sims not used, sleep
inhibitor released.

## What changed this session

1. **Promoted: `ROLLOUT_STEER_LEAD_S` = 3** (sav-lead2-1006).  The rollout
   steered on cross-track alone (no damping) and every shuttle rollout
   weaved +-25-80 m with a ~20 s period; 5 of 17 losses in rot-newdef-1006
   touched down on the centreline and stopped off the tarmac.  Lead 3:
   every rollout closes monotonically, stops within 7 m.  From orbit
   (rot-bank-1006, rot-lapt-1006) no shuttle loss has been a rollout since.
2. **Correction to the last handoff: the old craft lands SHORT, not long.**
   `ROLLOUT -> STOPPED stopped N m along` is unsigned; the signed stops
   (-1.4..-2.6 km from the midpoint) put `qs_plane` 1.2-1.5 km short of
   the threshold, intact, ~340 m of rollout.  Its approach falls ~3 km
   short of the 1800 aim -- the shuttle-fitted `APPROACH_BEST_LD` 4.2 is
   the likely term.  0/12 in rot-bank-1006.  Not chased (the user: the old
   craft is a brick).
3. Tools/data: `qs_s2_rwy0-5` exist only in the instances (copies of
   `saves/qs_s2_rigoff2.sfs`, which lands on the runway reliably) for
   landing-only A/B.  `logs/brakecache/` (AIRBRAKE_CACHE, below).

## Flags added (all off) and what they measured

- `HAC_EXIT_LAP_FRACTION` (1.0 = old): cone saves, only rigoff4 lapped and
  it ended -1.6 km (sav-spend-1006).
- `HAC_CHOOSE_BY_ENERGY`: choose end/hand by the planner's fit.  Offline,
  from a lined-up arrival the other end costs more than a lap; unflown.
- `AIRBRAKE_CACHE`: arm the measured spoiler from the craft's last vacuum
  probe when engaged in the air (air-start saves never had a brake, which
  made sav-spend-1006's brake arm disconnected).  Unflown; the cache is
  written by any flight on the new code that starts in vacuum.
- `HAC_EXIT_LAP_AT_TARGET`: the exit prices a lap as the plan does (target
  speed's radius) less 500.  rot-lapt-1006: null (rigoff 4/12 vs 4/12) --
  its threshold (~6.3 km) is above most high exits (2.5-6.2 km).
- Flown, existing flags, refuted: `HAC_SPEED_PATH` (inc 0/12, all 7.5-12
  km short, rot-lapt-1006); `GLIDE_ALPHA_MAX_DEG` 37 (7/24 vs 8/24) and 34
  (0/24, most arrive at 11.8 km short) (rot-galpha-1006);
  `HAC_FLAP_BRAKE_ON_SURPLUS + _IGNORES_ROLL` from orbit 9/24 vs 11/24 --
  the brake deploys only on the cone's last tick, because the plan absorbs
  the surplus into path it never flies (rot-fbrake-1006);
  `HAC_BANK_MAX_DEG` 60 (22/36 vs 22/36, rot-bank-1006);
  `HAC_WEAVE_MAX_DEG` 70 / `HAC_WEAVE_HELD` (cone saves, null/worse).

## The blocker: rigoff arrives at the cone 4-5 km high on half its flights

Shuttle from orbit on the current defaults, five batches: rigoff
5/12, 4/12, 4/12, 8/24, 11/24 (~38%); inc 9/12, 8/12; high 9/12, 7/12.
rigoff's cone entries are bimodal, 15-17 km or 19-21 km (rot-galpha-1006,
24 flights).  The high ones exit the cone 2.5-10 km above what the approach
needs and break at the flare (48-68 m/s, sink 38-46).

**Mechanism, found this session** (LOG7092/7101/7129/7161 vs
7096/7116/7119/7138): the split is the learned alpha ceiling.
`Holdable` only creates a bin when the vehicle saturates.  Flights that
saturate once early (q ~1600 Pa, achieving 36 against 43) plan the rest of
the glide with a ~37 deg ceiling and arrive on profile.  Flights that do
not keep commanding 37-44 deg, carry more energy (Mach 4.9 vs 4.7 at 36
km), and at q ~3500 Pa achieve only 23-28 deg with bank pinned at 64-70
(bank-coupled alpha; `holdmap.py` over 264 logs: held alpha 31-34 at q 1-4 kPa, 25-28 above 5); the ceiling collapses late, the prediction jumps
from +500 to +5-11 km at 33-27 km.  A fixed cap (`GLIDE_ALPHA_MAX_DEG`)
does not fix it -- achieved alpha depends on bank as well as q.

**The cone cannot absorb 2-6 km**: lined up, the path is the run to the
gate at any radius; the next answer is a lap (~6.5 km of height at R 2000).
Between them only the weave and drag, and neither is enough.

## Next

1. **Lead: the lap stack moves the cone's error from high to low.**
   rot-lapstack-1006 (rigoff, 24 an arm): `HAC_EXIT_LAP_AT_TARGET +
   HAC_BANK_MAX_DEG 60 + HAC_RADIUS_MIN_M 1200` took the handover from
   mean +2012 sd 2430 to **+66 sd 1249** -- no 6-10 km-high exits left
   (max +3.4 km) -- but 12 of 24 now leave "out of height" 0.3-1.4 km under
   need, and landings are unchanged (10/24 vs 11/24).  The laps are now
   affordable; what is missing is a lap that ends at the gate, not below
   it.  Read where the out-of-height ones were (gate distance, turn) and
   whether the planned lap ratio (1.86) is above what a 60-deg lap flies
   (`conesum.py` on the arm-1 logs) before changing anything.
2. The glide: make the propagator plan with a bank-aware holdable alpha
   from the start rather than waiting to saturate (a prior over (q, bank)
   from the logs' `alpha ceiling` events, recomputed by a tool; or seed
   the ceiling with one early probe).  Four refutations of this mechanism
   now (failures 79, 85 and this session's two caps): change what is
   measured, not the constant.
3. Old craft: derive the approach ratio per vehicle (spaceplane/CLAUDE.md
   Next 0 with the sign corrected).
4. Carried over: the rest of spaceplane/CLAUDE.md "Next".

## Traps paid this session

- **Unsigned distances again**: the old craft's "rolls off the end" was a
  short landing.  Read the signed along.
- **Air-start saves have no measured brake** -- any flag about the flap
  brake is disconnected when flown from a cone save (now: AIRBRAKE_CACHE).
- **Cone saves no longer represent orbit** after an aim change: the
  qs_s2_rigoff states arrive 1.6-5 km high under HAC_AIM_DERIVED.
- Swap reaches 17-20 GB after every ~90 min batch; restart the farm
  between batches (done every time today).
