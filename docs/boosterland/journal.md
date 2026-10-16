# boosterland journal

## Session, 2026-10-16: the heavy booster

The user flew a second booster live -- the first stage of "shuttle first
complete test flight": 35 parts, 99.4 t at separation, five Vectors on an
engine plate, six airbrakes on the top tank, six legs, twelve Vernors as RCS.
It landed 830 m short and broke up (LOG10236).

**New save `qs_heavy`** (`boosterland/tools/splicebooster.py`): the booster
cut from the stack in the live game's FMRS `before_launch.sfs` (the subtree
under `Decoupler.3`, rooted at the top tank), put in place of `quicksave`'s
vessel with its separation state, propellant set to the logged 99.36 t. The
first farm flight reproduced the live failure (LOG10237).

### What the live flight died of

1. **The predictor credited lift the booster cannot make.** Commanded 8-10
   deg off retrograde, it held 2-3 (`hold=` is the real angle) with RCS
   open: six airbrakes at 90 deg on the trailing end make it a weathervane.
   Lift goes as the angle squared, so the solve's prediction carried ~10x
   the real lift and the miss walked 800 m short below Mach 0.9, unseen.
2. **A correction burned past its best**: 900 -> 314 -> 1068 m, 7 t of
   landing propellant.
3. **The landing burn**: the throttle trim gain (0.12 /(m/s)) is twice as
   hard on a 9 g booster as on the 3.7 g original; it bang-banged into a
   climb, and retrograde -- the attitude reference -- pointed down once it
   climbed. Ran dry on the farm; broke up live.

### Changes (all default, all committed)

| change | evidence |
|---|---|
| `lift_slope_at` scaled by the measured aoa tracking squared (`AERO_STEER_TRACK_*`) | reads 0.1-0.3 on both boosters where it samples cleanly; samples after a correction were polluted (46-88 deg "held") until the 10 s settle + over-hold rejection |
| correction aborts against its **best** miss (and a floor of best + `CORRECTION_ENTER_M`) | replay of LOG10237's 900 -> 314 -> 1068 |
| `LANDING_THROTTLE_GAIN` 6 m/s^2 per m/s, divided by max_accel (replaces `LANDING_THROTTLE_KP`) | equal to the old gain on the original booster |
| landing reference: retrograde held inside `LANDING_MAX_TILT_DEG` of vertical below `LANDING_NULL_ALT_M`, vertical when ascending | pure vertical was tried first and lost the free lateral damping at the throttle cap (qs LOG10246: hs 5 -> 14 m/s) |
| RCS: saturation rule (the shuttle's) in COAST only; error relay elsewhere | saturation everywhere left the valve shut while a landing tipped 41 deg (LOG10251) |
| airbrakes deployed at START but **not** made pitch/yaw surfaces | 1x quicksave: 93/141/262/300 m with pitch/yaw on (rt6-1016) v 16-51 m on 6 of 8 with the action-group deploy (ab7-1016, commit 2984d8d) |

### Refuted, deleted

- **Pumping propellant to the top tanks for the coast** (`COAST_PROPELLANT_UP`,
  the user's idea; branch `pump`, commit 5b57168, deleted). Moved 13.6 t,
  centre of mass +2.2 m toward the airbrakes; tracking still <= 0.29, misses
  679/860/958 m + one lost against ~600 without (ab7-1016).
- **`DRAG_PROBE_DESCENT` on the heavy booster** (pd10-1016): 3 of 4 lost,
  628-884 m. Stays off.

### Where it stands (1x, the only valid numbers)

- heavy: lands 2-9 m/s but ~600 m short (trk9-1016: 605/611/695 + one lost;
  rt6: 586/604/639 + one lost). The live code broke it up.
- original `quicksave`: 19/71/84/431 m with hard arrivals (trk9) -- no better
  than 2984d8d's 16-51 m on 6 of 8.

### The lead for next time

The heavy booster's corrections are **deterministic** across flights
(pd10-1016, four of four): burn 1 at ~31 km 855 -> 1190, burn 2
1240 -> 403, burn 3 420-600 -> 780-840. The ~820 m it carries into the
landing burn is made by them. And entering CORRECTION while COAST steers
jumps the predicted miss from ~0 to 300-1500 m at zero throttle: the steered
prediction still over-credits lift. Read a correction tick by tick with
`DIAG_STATE` (trk9-1016 logs have it: LOG10310-10315) before changing it.

### Traps paid for

- **`--timescale` persists on the instance.** The booster has no governor;
  every batch until ~1010 flew at ~4x (including ones with no `--timescale`
  argument, which inherit the last file) and is worthless. Pass
  `--timescale 1` explicitly.
- `sweep.py` runs one job per (save, config) on one instance, sequentially,
  and its LINE regex no longer matches quickfly's output ("FAILED rc=0").
- A worktree's `fly.sh` looks for `ksp<N>` beside itself: symlink them.
- Six instances reach ~20 GB of zram in a batch from a 10 GB start.
