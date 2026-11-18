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
  landing burn after a fall through the body's own density profile with
  the stack's zero-alpha Cd*A table (taken at liftoff), lit where
  boosterland's law lights it (`LANDING_THROTTLE_CAP`, `SUICIDE_MARGIN`,
  `SUICIDE_REACTION_S`), dv = v + 2 g s H / v. Matches LOG12758's boostback
  (1251 m/s) and the Heavy Shuttle booster's fall and burn (LOG16184: lit
  at 680 m/s, ~850 m/s) -- `tests/testBudget.py`. On `qs_finbooster2`'s
  separation, fed the drag its fall flew, it gives the coast mass within
  0.9 t, the burn's light speed within ~12 m/s and the landing within ~1 t
  (fin2trv-1118, 24 flights; `testBudget` holds LOG17830). The fall's drag
  is `boosterland/dragcache`, recorded at the landing burn's ignition. Not
  modelled: propellant the coast spends on RCS (0.7-5.2 t on fin2, a
  coast-attitude limit cycle in boosterland, not a budget term).
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
- Separation lights the upper stage *before* the decoupler: when no upper
  engine is lit by the decoupler's stage and the next stage holds engines
  and decouples nothing (decided at launch, `Stack._light_plan`), those
  engines are activated directly, the decoupler fires on the next call, and
  then their stage is fired so MechJeb's autostage does not stop on it. No
  read sits between them and nothing waits on autostage.
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
the KRPC.MechJeb of kRPC main (0.8.0-dev, since 2026-11-14), in
`GameData/kRPC/` of the live game and every farm instance; MechJeb2 2.15.3
is in the farm's `base/` (no longer stripped; per-craft MechJeb settings
deleted so every instance starts at MechJeb's defaults). The API is
split: `ascent_autopilot` holds the common settings and
`ascent_path_index` (0 classic, 2 PVG; `ASCENT_TYPE` 0/1 maps to it),
`ascent_path_classic` the turn (`turn_shape_exponent`, auto path),
`ascent_path_pvg` `desired_apoapsis` and the pitch kick. `MJ_OWNER` in
`autopilot.py` says which holds what; `mj_set` refuses a name the object
lacks (a kRPC object otherwise keeps an unknown attribute in Python and the
game never sees it).

Saves: `qs_stack` (the shuttle stack, 366.8 t), `qs_stack_small` (one-Vector
booster under a pod, 73.9 t), `qs_stack_stretch` (the small stack with a
second X200-32, `tools/stretchcraft.py`, 91.9 t) -- all on the pad.

```bash
./run.sh --pilot ascent                        # live: panel waits for START
./ascent/tools/stackfly.py --instance 0 -n 1 --save qs_stack_small
./ascent/tools/srot.sh TAG "arm:SET=1,X=2" ... # every save on all six instances
.venv-pypy/bin/python -m unittest ascent.tests     # testBudget, testTurnShape
```

Where it stands, what is open: `docs/ascent/handoff.md` (index: the root `HANDOFF.md`);
history in `docs/ascent/journal.md`.
