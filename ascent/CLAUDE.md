# ascent

MechJeb's classic ascent flies the climb on a turn shape solved from the
craft at liftoff; this drops the booster at the last moment it can still
fly home, throttled so the upper stage pulls away, and stays with the upper
stage to an 80 x 80 km orbit. The booster is flown afterwards from FMRS's
separation save (launcher "BOOSTBACK FROM FMRS SAVE", `boosterland/fmrs.py`). The root `CLAUDE.md` applies in full.

```
STANDBY -> LAUNCH -> ASCENT -> THROTTLE_DOWN -> SEPARATE -> HANDOFF
```

- `autopilot.py` -- the phase machine, the only kRPC-aware code. `Stack`
  finds the booster from the staging (the parts the next decoupling drops
  that carry lit engines); nothing about the vehicle is configured. At
  liftoff it logs the upper stage's control source and warns when there is
  none (an uncrewed pod controls nothing) or when upper-stage autostruts
  run to a booster part (Heaviest/Root -- torn at separation; kRPC cannot
  change the mode, the craft must).
- `budget.py` -- pure: the propellant the booster would land with if it
  separated now. Boostback integrated with the flip under power (nose turns
  from prograde over the top at `FLIP_RATE_DEG_S`), ballistic vacuum return;
  landing burn from terminal velocity on the widest tank's area,
  `LANDING_CD` 1.0. Matches LOG12758's measured boostback (1251 m/s) and
  landing (~330) -- `tests/testBudget.py`.
- `turnshape.py` -- pure: MechJeb classic's pitch-vs-altitude path
  (`90 - ((h-h0)/(end-h0))**shape * 90`; the vertical climb ends at
  MechJeb's start height or start speed, whichever first, and h0 is the
  height it ended at; all read from MechJeb's auto path) flown as a point mass over a rotating body, with the craft's
  thrust/Isp/masses, a Cd*A table sampled from the game at zero AoA
  (`simulate_aerodynamic_force_at`, `DRAG_HEIGHTS_M` x `DRAG_SPEEDS_MS`),
  the game's pressure/density, separation by `budget.recovery` (the same
  rule), the upper stage to apoapsis and the circularization. `solve`
  (coarse scan + golden section, ~0.5 s) picks the shape that leaves the
  most mass in orbit; `Ascent.solve_turn_shape` runs it at every liftoff
  and sets MechJeb's `turn_shape_exponent`. `TURN_SHAPE` 0.4 is only the
  no-orbit fallback. Low shape = flat early turn. Downrange is along the
  ground. Every log carries `turn shape track:` (predicted t:h/dr/v);
  `tools/shapetrack.py LOG...` sets it against the telemetry.
- `ASCENT_TYPE` 0 (classic). PVG (1) plans the stack and never re-plans
  after this script stages the booster: it burned every upper stage to
  depletion. The PVG pitch-kick fields (`PITCH_*`) do nothing under classic.
  Roll: `vertical_roll`/`turn_roll` are set to
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
- `HANDOFF=upper` (default): MechJeb classic to orbit, autostage on after
  the separation; ends on MechJeb finishing (`orbit:` event with the mass in
  orbit), on the upper stage being lost (mass 0 / landed / splashed), or
  `ORBIT_TIMEOUT_S`. The booster keeps the throttle-down thrust limits it
  was saved with; `boosterland/fmrs.py` clears them for a one-group booster.
  `HANDOFF=booster`: switch to the booster and run `boosterland.Autoland`
  in-process (its own LOG); the upper stage is abandoned. On the first
  HANDOFF tick MechJeb's ascent settings are re-read and any that differ
  are put back (`check_settings`).

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
.venv-pypy/bin/python -m unittest ascent.tests     # testBudget, testTurnShape
```

Where it stands, what is open: the root `HANDOFF.md`, section "ascent";
history in `docs/ascent/journal.md`.
