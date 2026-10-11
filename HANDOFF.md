# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-11 ~0020.  History: `docs/spaceplane/journal.md`,
"Session, 2026-10-11 (orbit shapes)".

Defaults fingerprint **`f0b8f9b9`** (was `bc9ac544`).  Committed and
pushed.  Farm stopped, sim servers stopped, sleep inhibitor released.

## The headline

**LOG9599 (the user's live flight, 160 km near-polar) overflew KSC at
Mach 6.7.**  The pass was good -- it crossed the extended centreline ~2 km
off the line, 40 km west of the threshold, at 47 km -- but it arrived
nearly perpendicular to the runway, so the glide's runway-frame `long`
was ~-30..-40 km everywhere near KSC and only reached 0 about 650 km past
it (`long=+1411 cross=-650146` at glide entry).  The solve aimed there
and flew for range (30 deg bank, a climb 49.5 -> 50.9 km).  **Bug 2 below
is the cause; bug 1 would not have refused that pass**, but is the same
blindness for passes that miss KSC.  Both invisible on the farm's
near-equatorial saves:

1. The deorbit decision was along-track only (`forward_arc` projects the
   gate into the orbit plane), so it committed to a pass nowhere near KSC.
   Now `guidance.deorbit_reach` propagates a reversing and a one-sided
   entry from the post-burn state and a pass is taken only if the gate is
   within `DEORBIT_CROSS_SHARE` (0.5) of the sideways reach (~105-135 km
   on the shuttle).  The plan scan screens states with a vacuum estimate
   (`offtrack_estimate`) and looks 3 days ahead (`DEORBIT_PLAN_HORIZON_S`
   64800).
2. The glide nulled `long` resolved on the **runway heading**; an arrival
   crossing the runway at an angle put its overshoot into `cross`.
   `miss_components` now resolves along the arrival's ground track.

For orbits up to 500 km: the entry limits are measured from the top of
the air instead of the burn (`DEORBIT_ENTRY_ARC_MAX_M` 1400 km,
`DEORBIT_MAX_ENTRY_TIME_S` 1005 s, calibrated so the 87 km save commits
on the same window as before; `DEORBIT_RANGE_MAX_M` is now a screen,
3600 km); the drain reserve is at least the apoapsis burn to a surface
periapsis; the drain and the burn budget count **usable** propellant
(the scarcer of fuel/oxidizer over the engine's mixture -- a 500 km burn
ran dry otherwise).

**Measured in kspSim only** (matrix of 8, `qs_shuttle2@...` specs,
LOG9630-9637): every shape reaches the field -- 87 km, 160/30, 500/0,
500/45, 250/90, 100x500/20, 80x300/60, 120/150 retro; Mach-5 states
120-195 km out with long within 1.2 km.  Landings 2-22 km short, which is
the sim's known short approach (kspSim gap 7).  Before: 200-650 km off.

## Next, in order

1. **Fly the farm batch that was killed** (memory: 8 stray kspSim servers
   were left beside the farm -- kill them with `pkill -f "kspSim[.]run"`
   from a script file first):
   `spaceplane/tools/multirot.sh orbits-1011 4` with arms
   `qs_shuttle2_{rigoff,inc_rigoff,high_rigoff,ecc_rigoff,i30_rigoff,
   i60_rigoff,retro_rigoff,e500_rigoff,e500i45_rigoff}|`.  The first four
   are the regression check against taemfix-1010 (23/24); the last five
   are the new shapes (`saves/derived.txt`).  Read `deorbit reach:` and
   `deorbit plan:` lines (and `refused:` counts when no window is found).
2. If the farm agrees, the how-it-works PDF's DEORBIT section (plan,
   sideways reach, entry limits from the air, usable propellant) and the
   GLIDE's miss frame.
3. Last session's list: `ABORT_NORTH`, `GROUND_SPOILER_ON_NOSE` (built,
   off, unflown on the farm); the inc glide energy deficit; low-speed
   rollouts off the edge; `constantsAudit.md`; `AIRBRAKE_CACHE`.

## Open from this session

- The 100x500 km pass reached Mach 5 with 50 km of cross-track (landed
  anyway in the sim); the plan's 12-hour look-ahead missed an 80x300 km
  pass by ~80 km and replanned (long-horizon RK4 drift) -- harmless,
  costs a replan.
- A burn whose stop test tapers can creep to the 60 s guard (polar run,
  LOG9607); seen once since the propellant fix, never on the farm.

## Traps paid this session

- **Kill sims with `pkill -f "kspSim[.]run"`**, not `kspSim/run.py` (the
  process is `-m kspSim.run`); left running they cost ~0.4 GB each and the
  farm batch was killed for memory.
- The sim's Mach < 4.5 glide departs in attitude on non-aligned arrivals
  (alpha 40-70 deg commanded 19): sim gap, judge the terminal glide on the
  farm.
- `forward_arc`/`miss_components` on the runway frame agree with the
  track frame only when the track is along the runway -- every farm save
  until today.

## Traps from before (still true)

- kspSim lands short of the game with TAEM on; don't judge range there.
- kspSim needs `DEORBIT_SAS_ALIGN=False` (stub SAS).
- `pkill -f "<pattern>"` in the same command line kills the calling shell.
- multirot's arm numbers rotate per cycle: group by the log's config line.
- Swap ends a 4-cycle multirot at ~21-23 GB; restart before comparing.
- `warp_to` is blocking: the panel does not answer during it.
