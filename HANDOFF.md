# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-11 ~2100.  History: `docs/spaceplane/journal.md`,
"Session, 2026-10-11 evening (the cone holds dynamic pressure)".

Defaults fingerprint **`b5651d39`**.  Committed and pushed to
origin/main.  Farm stopped, sleep inhibitor released.  The how-it-works
PDF is current (deorbit plan and burn node, the cone's speed law).

A global SessionStart hook (`~/.claude/hooks/pushOnStart.sh`) now pushes
unpushed commits in any GitHub repo at session start; read its
`pushOnStart:` message and fix a failed push before anything else.

## What changed this session

- **`HAC_SPEED_Q` (default).**  The cone's speed target is the speed whose
  dynamic pressure equals the gate target's at the height flown (constant
  equivalent airspeed: ~112 m/s true at the gate, ~245 at 12 km).  The
  old constant true airspeed was below the stall above ~10 km, so the cone
  flew past its lift peak (flown lift peaks near 12 deg; it sat at 14-18,
  L/D 1.3), bled to 70-90 m/s and ran RCS for half the cone.  The glide's
  TAEM target (`guidance.taem_target`) is deliberately left on the old
  speed.
- **The cone's split-rudder fade is against the stall under `HAC_SPEED_Q`**
  (`Autopilot.hac_split_brake`: `APPROACH_SPLIT_MIN_SPEED_FACTOR` x stall
  at the turn's load, in equivalent airspeed), not against the cone target
  the cambered wing often cannot reach at alpha 0.
- Measured: speedq-1011 (LOG9921-9967, before the brake fix): Q i90 7/8 v
  6/8, e500i45 8/8 v 6/8, rigoff 3/8 v 8/8 (rolled out high, brake
  throttled).  **speedq2-1011** (LOG9968-10015, with it): **Q 19/24 v
  14/24**; RCS in the cone 15-40 s v 92-99; slowest 92-107 m/s v 76-86.

## Where each orbit stands (speedq2-1011, default)

i90 6/8, e500i45 7/8, rigoff 6/8 (19/24, ~79%; goal >= 95%, 0 lost).
The misses changed side: Q's are high -- two rigoff rolled out 4.2 km
over (overruns +4.0, +5.2 km), one i90 high (-2.9) -- plus one e500i45 out
of height (-2.3) and one breakup on the runway (LOG9970, 19 parts).  The
old default's were mostly out of height (water, wrecks).

## Next, in order

1. **The constant-q cone's surplus.**  It can no longer dump energy as
   stall drag; its spenders are the split rudder, radius (pinned at 16 km
   on the high flights), weave (pinned at 50) and laps.  Measure the
   brake by Mach (`splitprobe.py --glide --angles ...` at Mach 0.4-0.9):
   `HAC_SPLIT_FACTOR` is one point at the lower cone's state (Mach ~0.35,
   8 deg) and the upper cone now flies Mach 0.7-0.9 where the brake adds
   less (+32% v +52%), so the plan over-credits it.  Then consider the
   Shuttle's answer: the brake as the energy control against a
   height-v-range reference.  Read LOG9994, LOG10010, LOG9991.
2. **`HAC_SHORT_POLAR`** (off, unflown): its premise -- inclined cones
   short at alpha 8-9 -- came from the constant-TAS cone.  Re-fly it on
   the new default or delete it; don't leave it parked.
   (`logs/conepolar/Untitled_Space_Craft_31.json` is from the old cone;
   regenerate with `spaceplane/tools/conepolar.py` before using it.)
3. The live craft (`shuttle|31`) has no flown polar of its own.
4. Older: `ABORT_NORTH`, `GROUND_SPOILER_ON_NOSE` (both off, unflown on the
   farm), `docs/spaceplane/constantsAudit.md`.

## Traps paid this session

- **The lift table under-reads this wing at low alpha** (flown ClA ~65-110
  at 0-2 deg v the table's ~50), so a speed derived from the table at
  `ALPHA_MIN_DEG` is far above what the vehicle actually needs -- a cap
  built on it never binds.
- A brake fade keyed to a speed target fights any law that makes the
  target unreachable; key "slow" to the stall.
- The test of a new flag asserting its *off* state breaks on promotion:
  set the flag explicitly both ways in the test.
- Base arms scatter a lot between batches at n=8 (e500i45 6/8 then 2/8):
  compare arms within one interleaved batch only.

## Traps from before (still true)

- Read the save name off each log before comparing.
- A `&&` chain ending in `| tail` commits on a failing test -- check the
  suite's own exit status.
- landsum drops some logs; for outcome counts use the harness lines in
  `logs/rot-*.txt` joined to each log's `config:` line.
- kspSim has no `qs_shuttle2_rigoff` model: orbit specs go on
  `qs_shuttle2@pe=..`, and it needs `DEORBIT_SAS_ALIGN=False`.  Its
  subsonic lift is wrong (gap 8) -- don't screen cone energy there.
- Six fresh instances put 10-17 GB into zram at boot; 4 cycles reach ~22.
- Kill sims with `pkill -f "kspSim[.]run"`.
- multirot: arm counts must divide 12; arm numbers rotate per cycle.
- `warp_to` is blocking: the panel does not answer during it.
