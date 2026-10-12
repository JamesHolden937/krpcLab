# HANDOFF — read this first, rewrite it last

## 2026-10-11 late: smoothing defaults, flown only by the user (fingerprint `160acc93`)

The user's live flight LOG10017 touched down **230 m south of the
centreline** (on the grass, rolled back onto the runway; the DOWN line only
reads where it stopped).  Cause: the gear swing at 796 m made the roll
damper slow roll to 8.4 s; the approach's roll-back-out bank limit
(`rate x (t_door - roll ttp)`) hit **0 deg at 400 m**, so the capture was off
while 50 m off and drifting; the split rudder's stow at 374 m left +5 deg
of sideslip, the airframe rolled 7-13 deg uncommanded and the track turned
5-9 deg south.  The user asked for smoother flying and to test it
themselves ("fix, don't test"), so these went straight in as defaults,
**unflown on the farm**:

- `APPROACH_BANK_FLOOR_DEG` 6 -- the final's bank limit never drops to 0.
- `HAC_WEAVE_CLOCK_FROM_START` -- the weave's clock starts with the weave
  (LOG10017 flew a 2-s stub swing, +31 then -28).
- `HAC_WEAVE_PERIOD_S` 24 -> 48.  The bank is *not* capped (the user: fast
  roll is fine, the swing is the problem).
- `BANK_ACCEL_SHARE` 0.5 (`guidance.bank_accel`, `ease_bank`,
  `Autopilot.bank_accel`): the bank command speeds up and brakes at half
  the bank's angular acceleration from kRPC's `available_torque` /
  `moment_of_inertia` (read 1 Hz, smoothed), roll and yaw combined about
  the velocity vector at the flown alpha, braking where the remaining angle
  equals the stopping distance.  Glide alpha 44 no RCS: 5.6 deg/s^2, an
  80 deg reversal 8.0 s; with RCS 12.5; cone alpha 5: ~200 (effectively
  unbraked -- any swing left in the cone is kRPC's tune, not the torque).
  Glide, cone (no slew before) and approach.  LOG10017's glide overshot
  every lean by 15-20 deg.
- `HAC_SPEED_ERR_MAX_MS` 25 -- the cone's speed law acts on at most 25 m/s
  of error (LOG10017 pushed over from 159 to 231 m/s at 160 m/s of sink).
- `AZIMUTH_FLOOR_TURN_FACTOR` 2 -- the glide's reversal band is twice the
  heading swept in one roll (LOG10017: nine reversals in two minutes,
  Mach 3 -> 0.8, at +-40).
- `ABORT_NORTH` on (the user's standing wish; sim 8/8, farm unflown).

Not fixed: the split-rudder stow leaving +5 deg of sideslip on short final
(LOG10017 at 374 m); the roll damper reading the gear swing as an
oscillation.  Next: fly these on the farm against the previous defaults
(`b5651d39`) if the user's live flights disagree, then the HOWITWORKS pdf
(the weave, glide band, cone speed clamp, final bank floor changed).

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
