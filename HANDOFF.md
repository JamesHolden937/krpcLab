# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-10 afternoon (~1100 -> 1600).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-10 day".

Defaults fingerprint **`66f98be6`** (was `ca5fef06`).  Committed and
pushed.  Farm stopped, sleep inhibitor released.  No worktrees open.
`howItWorks.pdf` updated (SAS line-up, azimuth floor).

## The headline

1. **The deorbit burn lines up on stock SAS** (`DEORBIT_SAS_ALIGN`,
   promoted; the user's idea after flying it live).  A maneuver node 2 s
   ahead, SAS in maneuver mode, commit at <= 3 deg on two ticks or 120 s.
   rot-smoke-sas-1010 (LOG9340-9345): settled 0.1-0.6 deg in 20-24 s,
   0.6-1.0 deg while burning (5.7 on the high orbit) v ~7 before; 6/6 on
   the runway.
2. **`AZIMUTH_FLOOR_FROM_TURN` promoted.**  rot-hedazf-1010 (16 an arm,
   three orbits): glide-end reversals 3.5 v 4.9 a minute, on the strip 12
   v 11, long 1 v 4.  The flown-bank overshoot past 90 deg is unchanged
   (the roll loop).
3. **The defaults are strong:** rot-bfl-1010 defaults arm (23 flights,
   four orbits, LOG9412-9459) **21/23 stopped on the runway, 23/23 kept
   18+ parts, 0 in the water**; the two off it were high-orbit flights
   +1.35 / +1.44 km, just past the end.
4. **Two cone experiments deleted** (below).  The lesson both taught:
   **the cone's surplus is height, and it has to be spent as path.**
   Anything that shortens the path (tighter circle) or spends speed
   instead (alpha past the lift peak as a brake) lands high arrivals long.

## Deleted this session (code back to 3ecfc1a + the AZF promotion)

| flag | what | why deleted |
|---|---|---|
| `HAC_ENTRY_DERIVED` | cone entry from a holdable circle and the gate | gate-height trigger: LOG9347 out of height, 9 km short; plan-affordable trigger: rot-smoke-hed2-1010 4/6; veto alone changed 1 flight of 16 (rot-hedazf-1010, LOG9403, still long) |
| `HAC_BANK_FROM_LIFT` | cone bank limit `acos(1/n)` from lift measured in flight (`FlownLift`) at the mass now, floor 45, cap 75; also the entry veto | rot-bfl-1010: **17/23 v 21/23** on the runway, 2 in the water, all high-orbit misses +1.2..+2.5 km.  It read 45 at **every** cone entry (no lift measured at entry speed), and opened to 60-75 lower down, which tightened the circle so the high arrivals (entry h ~17.6 km) spent less path |

`docs/spaceplane/constantsAudit.md` (new): ~30 hard-coded values sorted by
what they stand in for, with what should compute each.  The user wants
them removed ("they hide structural issues").

## Next, in order

1. **The cone's surplus-height problem** (old item 3, the lap gap).  laps
   = 0 in every flight; entries are 14-18 km against `HAC_ALT_M` 12 (the
   3 km `HAC_ENTRY_DIST_M` test fires while the glide dives ~37 deg).
   The cone then spends the surplus with alpha past its ~13 deg lift peak
   (`HAC_ALPHA_MAX_DEG` 22 is a drag brake in practice), which bleeds
   **speed**, not height: LOG9410 arrived lined up, never banked, came out
   slow (~91 m/s) and 2 km high, the split brake faded on the slow side,
   +4.5 km long.  Make the plan spend surplus as path: a wider circle up
   to `HAC_RADIUS_MAX_M`, a partial lap, an S-turn in the cone, and cap
   the cone's alpha at the flown lift peak.  Screen in kspSim (guidance),
   then the farm, 24 an arm, four orbits.  Steeper bank is fine **when it
   lengthens the path** (more turn), not when it shrinks the circle.
2. LOG9375 (rot-hedazf-1010): exited the cone needing 6.6 km more height,
   lost 27 km short.  Not looked at.
3. The constants audit, top-down, one flag each: in-flight polar with
   drag as well as lift; part properties from kRPC (canard range literal
   37.5, `SPLIT_FULL_DEG`, `RESOURCE_KG_PER_UNIT`); wing/tail-strike
   limits from geometry; then tier 2.  Parked-off flags `AIRBRAKE_CACHE`,
   `ALPHA_TRACKING_ON`: promote or delete.
4. If the glide's overshoot past 90 deg of bank matters: the roll loop
   (30 deg/s command slew against a ~3 s response), not the sign law.
5. `APPROACH_CAPTURE_TAU_SHARE=0.35` (off, unflown).
6. The deorbit's "model said" log figure is wrong on the first burn tick
   (unlimited thrust before the limit applies); log-only.

## Traps paid this session

- **Cone entry is a fragile balance.**  Both derived entry triggers ran
  out of height; the 3 km test works because it fires early.  Change the
  plan's use of the surplus, not when the cone starts.
- A smoke run of a bank limit read "flown lift peak none yet" on all six
  entries: the glide is below Mach 1.5 for seconds before the cone.
  Check that a measured quantity exists *where it is used* before flying.
- Waiters: `pgrep -f` self-matches, and `ps | grep` catches the launching
  shell; wait on the real PID (`pgrep -f "^bash spaceplane/tools/..."`).
- Swap reached 23 GB by the end of rot-bfl-1010 (multirot restarts per
  cycle, but the last cycle still ends heavy).  Restart before comparing.
- "could not pause after the load": 1 per arm this batch; excluded.
