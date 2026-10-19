# ascent

MechJeb flies the climb; this drops the booster at the last moment it can
still fly home, throttled so the upper stage pulls away, and hands it to
`boosterland`. The root `CLAUDE.md` applies in full.

```
STANDBY -> LAUNCH -> ASCENT -> THROTTLE_DOWN -> SEPARATE -> HANDOFF
```

- `autopilot.py` -- the phase machine, the only kRPC-aware code. `Stack`
  finds the booster from the staging (the parts the next decoupling drops
  that carry lit engines); nothing about the vehicle is configured.
- `budget.py` -- pure: the propellant the booster would land with if it
  separated now. Boostback integrated with the flip under power (nose turns
  from prograde over the top at `FLIP_RATE_DEG_S`), ballistic vacuum return;
  landing burn from terminal velocity on the widest tank's area,
  `LANDING_CD` 1.0. Matches LOG12758's measured boostback (1251 m/s) and
  landing (~330) -- `tests/testBudget.py`.
- MechJeb's attitude: PVG kicks the nose over from `PITCH_START_VELOCITY`
  50 m/s at `PITCH_RATE_DEG_S` 1.0 deg/s, then follows prograde (MechJeb's
  0.5 flew 55 deg at 10 km). Roll: `vertical_roll`/`turn_roll` are set to
  the pad roll read against a pitch-over towards `ROLL_REF_HEADING_DEG` 90
  (`attitude.pad_roll`), so an eastward launch never rolls; telemetry
  carries `pit= hdg= roll=` (kRPC's euler angles are ambiguous while
  vertical: heading 270 / roll -90 is heading 90 / roll +90).
- Separation: when that propellant, projected `SEP_LEAD_S` ahead, falls to
  `RESERVE_FRACTION` (0.15) of what the two burns need.
- Throttle-down: booster thrust limits (MechJeb keeps the main throttle) to
  `SEP_ACCEL_RATIO` 0.75 of the upper stage's acceleration, outer pairs off
  first (`boosterland.engines.allocate`); stages once the *delivered*
  thrust is under target on two ticks. The limits stay on after staging
  until boosterland's cluster owns them (released at staging they let the
  booster reach 2140 kN against a 1280 target, LOG12760).
- The throttle is MechJeb's: never write `control.throttle` on the ascending
  vessel (kRPC holds it over MechJeb) and never read it (it shows 0.00 under
  MechJeb) -- `main_throttle` reads the booster engines.
- With boosterland's `FLIP_THROTTLE_LAW` (its default) `budget.boostback`
  flies the same throttle law, the booster's authority summed over its parts.
- `HANDOFF=booster` (default): switch to the booster, run
  `boosterland.Autoland` in-process (its own LOG, named in this one). The
  upper stage is abandoned. `HANDOFF=upper`: MechJeb to orbit instead -- **broken**: PVG burns every
  upper stage to depletion (HANDOFF.md).

**MechJeb over kRPC** needs KRPC.MechJeb built for MechJeb 2.15: the
released 0.7.1 (Genhis) cannot find 2.15's ascent module. Installed is
darchambault's fork **v0.8.1** (`krpc-mechjeb-0.8.1.zip`), in
`GameData/kRPC/` of the live game and every farm instance; MechJeb2 2.15.3
is in the farm's `base/` (no longer stripped; per-craft MechJeb settings
deleted so every instance starts at MechJeb's defaults). The API is
flat: `ascent_autopilot.ascent_type` 0 classic / 1 PVG.

Saves: `qs_stack` (the shuttle stack, 366.8 t), `qs_stack_small` (one-Vector
booster under a pod, 73.9 t), `qs_stack_stretch` (the small stack with a
second X200-32, `tools/stretchcraft.py`, 91.9 t) -- all on the pad.

```bash
./run.sh --pilot ascent                        # live: panel waits for START
./ascent/tools/stackfly.py --instance 0 -n 1 --save qs_stack_small
./ascent/tools/srot.sh TAG "arm:SET=1,X=2" ... # every save on all six instances
.venv-pypy/bin/python -m unittest ascent.tests.testBudget
```

Where it stands, what is open: the root `HANDOFF.md`, section "ascent";
history in `docs/ascent/journal.md`.
