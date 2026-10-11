# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-11 ~1130.  History: `docs/spaceplane/journal.md`,
"Session, 2026-10-11 late morning (the user's polar dive)".

Defaults fingerprint **`7904409b`** (was `f0b8f9b9`; the only change is
three new config fields, the flag below, off).  Committed and pushed.
Farm stopped, sims stopped, sleep inhibitor released.

**Standing (the user, 2026-10-11): never ask before starting the farm or
flying a batch** -- now in root `CLAUDE.md`.

## The headline

**The live 186 km polar (LOG9641) dived in the glide below Mach 3**, then
tumbled in the cone's first bank swing.  kRPC held +0.2 of pitch against a
16-22 deg nose-low error for 100 s once the offload bled off at Mach 3
(its gains are scaled to the 8 MN m the surfaces *could* give, so its
integral runs on minutes): alpha 5 where 20-28 was commanded, 50 kPa, cone
entered at the Mach 1.5 veto at 441 m/s / 9.7 km.  There, ±45 deg weave +
alpha ceiling released 8 -> 32 + pitch retuned to 3 s at 28 kPa -> alpha
41, bank +79/+123/+172, recovered at 5.5 km and 85 m/s.  No abort in the
log; it ends in the cone (broken pipe -- the game closed).

**Built, off: `GLIDE_PITCH_RESCUE_DEG`** (`_FULL_DEG` 10, `_HOLD_S` 8,
`Autopilot._glide_pitch_rescue`): below the offload's Mach, a nose-low
pitch error past the band for 8 s engages an integral on the manual trim,
held to the end of the glide.  **Sim** (LOG9642-9661, 10 orbit pairs):
reproduces the dive in 2/3 polar baselines; with it, >+10 deg error ticks
below Mach 3 58 -> 21 and no cone entry above 245 m/s.  Engaging on a
nose-*high* error did not close it (LOG9645, 9651), hence nose-low only.

**Farm batch `rescue-1011` (logs/rot-rescue-1011-c0..4.txt, LOG9662-9721)
is unreadable for the flag because the baseline is broken:** default
`qs_shuttle2_rigoff` 2/5 on the runway (taemfix-1010: 23/24), arrivals
-23..+14 km.  Per arm (RWY/total): i90 off 4/5, on 3/10; e500 off 5/10,
on 5/10; e500i45 off 4/10, on 4/10; rigoff off 2/5, on 0 flown (the 8-arm
rotation never reached arm 7).  Swap ended at 20.7 GB.

**The cause on rigoff is last session's deorbit planner** (sim-only, never
farm-flown before this): the same save planned dv 25.9 m/s 436 s ahead
(LOG9673, 9721: on the runway), 27.1 at 2595 s (LOG9685, -5 km), **246 m/s
68 s ahead** (LOG9697, broke up) and **179 m/s 22685 s ahead** (LOG9709,
+8 km water).  Same save, four different windows -- the plan is not
deterministic in the state it starts from, and it accepts burns 10x the
nominal.

## Next, in order

1. **Fix the deorbit plan's window choice** before any other farm work:
   why the same save gets 26 vs 246 m/s (look at what differs when the
   scan starts -- drain timing; ksp4 "could not pause after the load"
   in c0), and reject/penalise windows whose dv is far above the
   orbit's minimum.  Then re-fly rigoff/inc/high/ecc against taemfix-1010.
2. Re-fly `GLIDE_PITCH_RESCUE_DEG=8` vs off on a working baseline
   (i90, e500, e500i45, rigoff; use an even arm count per 6 instances so
   every arm is flown); promote or delete.
3. A guard on entering the cone fast: refuse or soften (weave, ceiling
   release, pitch retune) when entering well above the cone's planned
   speed -- LOG9641's tumble path is still open to any fast arrival.
4. **SAS on the node, not surface retrograde (the user asked).**  Today
   the search burns along *surface* retrograde (rotating-frame -v) while
   the node is placed as pure orbital retrograde (`add_node(prograde=-dv)`,
   `sas_engage`); on inclined orbits those differ by 4-5 deg, which is why
   Maneuver mode never committed on LOG9598 and `DEORBIT_SAS_SURFACE` went
   in.  Fix it at the source: build the node from the solved burn vector
   (decompose surface -v * dv into prograde/normal/radial at node time) and
   hold SAS on Maneuver.  One direction for search, node, map view and SAS;
   the hold is then inertial during the burn instead of following the
   turning surface velocity.  Check with `DEORBIT_SAS_ALIGN` on a polar
   sim save (node error < 0.5 deg at commit), then the farm; delete
   `DEORBIT_SAS_SURFACE` if it wins.
5. Then the deorbit/glide items from before: the how-it-works PDF's
   DEORBIT section, `ABORT_NORTH`, `GROUND_SPOILER_ON_NOSE`, the inc
   glide energy deficit, `constantsAudit.md`.

## Traps paid this session

- `multirot.sh` with 8 arms on 6 instances x 2 rounds never flew arm 7
  in 5 cycles; arm 0 and 6 got 5.  Pick arm counts that divide 12.
- The live craft (`shuttle`, 41.7 t) is the farm's craft with full tanks;
  after the drain both are 31.4 t.  Not a discrepancy.
- `pgrep -af KSP_x64` matches its own shell -- it does not mean a game
  is up.

## Traps from before (still true)

- Kill sims with `pkill -f "kspSim[.]run"`.
- kspSim lands short of the game (gap 7) and its sub-Mach-4.5 glide
  departs on non-aligned arrivals: don't judge range there.
- kspSim needs `DEORBIT_SAS_ALIGN=False`.
- multirot's arm numbers rotate per cycle: group by the log's config line.
- Swap ends a 5-cycle multirot at ~21 GB; restart before comparing.
- `warp_to` is blocking: the panel does not answer during it.
