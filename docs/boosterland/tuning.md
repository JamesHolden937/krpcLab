# Booster: measuring and tuning

How to get a trustworthy number out of a change: replaying a flight, aiming off
the pad, in-game against the sim, and what the accuracy history says.
Numbered failures cited here are in [docs/boosterland/failures.md](failures.md).

## Replaying a flight offline

`Config.DIAG_STATE` (default off) writes a second log line per telemetry tick:
the state vector, mass, the predicted touchdown point, and the *whole* Cd\*A
curve, plus the atmosphere tables and frame constants once at startup.
`replay.py` reads that back and re-propagates, so a real tick can be re-flown
with air it did not have:

```bash
./boosterland/tools/quickfly.py -n 1 --set DIAG_STATE=True
./boosterland/tools/replay.py logs/LOG53                    # the walk, as flown
./boosterland/tools/replay.py logs/LOG53 --curve-from -1    # every tick, with the final curve
./boosterland/tools/replay.py logs/LOG53 --omega zero       # ... or without the frame terms
```

It reproduces the in-flight predictions to tens of metres out of 112 km. It
exists because `cda=` in the compact line records only the Cd\*A at the Mach
the booster happened to be doing, and the propagator reads a whole curve —
which made it impossible to tell a curve being revised under the propagator
from a propagator that disagrees with the vehicle (failure 13). Under
`DIAG_STATE` the current bin is also probed at *both* attitudes and logged as
`probe=M<mach>,<real>,<turned>,aoa<deg>`.

## Aiming off the pad

`AIM_BIAS_NORTH_M`/`AIM_BIAS_EAST_M` move `Environment.target` off the pad.
**Both are 0, and the section is history**: the overshoot every value was
fitted to — 628, then 170, then 90 — was the propagator's integration bias, not
the vehicle (failure 16). With RK4 the east residual over ten flights averages
+0.4 m.

The *method* is still right for a new booster, and the history is a record of
what a calibration costs when it stands in for a bug (re-fitted three times,
never generalised between entry states, worth nothing once the cause was
found):

- Fly several entry states at bias 0 and read the **signed** E column.
- **Fit the slope from two biases; do not add one flight's residual.** The
  response is not 1:1, because moving the aim point changes the trajectory —
  measured at ~1.29 m of miss per metre of bias. The naive fix (130 + the
  641 m residual = 771) overshot to +185 m; the two-point fit at 628 landed
  +9 m.
- **`pad=` is measured from the real pad**, not from `env.target`, or a bias
  would flatter itself by exactly its own size. `miss=` stays guidance's error
  against the aim point.
- A bias sets the mean and cannot touch the spread. Sorting a group of flights
  by boostback exit time sorts it by landing point: 0.8 s of exit jitter was
  ~250 m of range, and the three flights whose exits fell inside 0.12 s landed
  inside 38 m.

## Measured in game, not in the sim

`quickfly.py` loads the quicksave, presses START itself, flies to touchdown and
reports distance and N/E offset. Use it. Five changes that improved the sim
sweep made the real flight worse, and the sim could not have caught any:

| change | sim | in game |
|---|---|---|
| axial `Cd*A` probe (`DRAG_PROBE_AXIAL`) | — | 3.3 km short into the sea, vs 137 m |
| ZEM/ZEV terminal guidance (`LANDING_TERMINAL_GUIDANCE`) | 51 -> 21 m median | 347/881/680 m, 129 s burn, 22 t -> 7.5 t |
| divert throttle floor (`LANDING_DIVERT_MIN_THROTTLE`) | median 6 m | hovers: `alt 383 -> 586` climbing, then falls |
| tighter `BOOSTBACK_TOLERANCE_M` | worse | worse (137 -> 193 -> 259) |
| the *fixed* ZEM/ZEV divert (`LANDING_DIVERT_ON_PREDICTION` + budget) | 96 -> 73 m mean | 896/300 vs 145/255, and it relights after touchdown |
| the signed boostback exit (`BOOSTBACK_UNDERSHOOT_M` = 0) | 4/75/19/5 m | 236-885 m on every save |

All are still in the tree behind flags, defaulted off. The last ZEM/ZEV row is
what "Aim, then stop" in [design.md](design.md) answers.

**It runs the other way too.** `boosterland/tests/fakeksp` integrates its own truth with
semi-implicit Euler at `SIM_DT` 0.2, so it rewards a predictor that shares that
bias: the RK4 propagator (failure 16) is worse in every sim cell and better in
every in-game cell but one. A sim that models the vehicle as a point mass is
also modelling *an integrator*.

The lesson is not that the sim is useless — it caught the throttle floor's
hover and the deep-throttle regression — but that it models a point mass with
instant-ish pointing and no aerodynamic torque, so it flatters anything that
steers and cannot see estimator noise at all. **Treat a sim improvement as a
hypothesis and confirm it in game.** And measure the *signed* offset before
theorising: a great-circle distance cannot tell an overshoot from an
undershoot, which is how a systematic 105 m westward bias got mistaken for
imprecision for most of a session.

## Measure over entry states, with the bias off

One quicksave is one separation state, and a number tuned against it proves
nothing about the next. `savegen.py` writes extra saves by applying a dv at the
separation state (`--prograde`, `--radial`, `--normal`), and `sweep.py` flies a
grid of (save, config) cells across the test instances and prints one table.
Five states at two flights a cell is twenty flights, about half an hour on four
instances.

- **Fly comparisons with `AIM_BIAS_*` at 0**, so the east column *is* the
  systematic overshoot and is comparable between configurations.
- **Read the signed miss, not the distance.** `quickfly.py` reports N/E offsets
  and every telemetry line carries `long=`/`cross=` — the predicted miss
  resolved along the approach, positive when the booster will fly past. That
  ambiguity has cost this project two sessions.

## Where the accuracy came from

Sim numbers, 24 atmospheric flights (four entry states at 45/30/20/15/12/8
°/s), metres from the pad:

| | median | mean | worst |
|---|---|---|---|
| closed-form landing burn, tolerance 500 | 78 | 135 | 505 |
| closed form, tolerance 150 | 103 | 125 | 374 |
| integrated burn, tolerance 500 | 81 | 82 | 193 |
| integrated burn, tolerance 150 | 65 | 67 | 155 |
| + `FINAL_APPROACH_RATE` 0.5 | 58 | 62 | 154 |
| one scalar `Cd*A` (later grid) | 74 | 100 | 313 |
| `Cd*A` interpolated against Mach | 52 | 50 | 124 |

Most of it is the propagator flying the landing burn rather than approximating
it. The tolerance only pays off *after* that: aiming three times more precisely
at a point hundreds of metres wrong is not worth much, which is why 150 m made
things worse on the old predictor and better on the new one. Two cautions: a
tighter boostback burns longer and ends lower, and the marginal-TWR vehicle
(1.37 at separation) goes 434 -> 1055 m between tolerance 350 and 250; and in
vacuum the tighter tolerance is worse (82 -> 284 m), because there is no drag
to bleed off what the coast opens.

`FINAL_APPROACH_RATE` 0.7 -> 0.5 flattens the last stretch: slightly more
propellant, legs arrive slower (worst arrival 4.12 -> 4.06 m/s, vacuum 5.21 ->
3.95). Raising `LANDING_THROTTLE_KP` instead is much worse — 0.2 puts the worst
arrival at 6.3 m/s and 0.3 at 8.6, the trim term overshooting in the last few
metres.

The in-game figure to compare a change against is in "Flying the coast as a
wing" ([design.md](design.md)): **median 4 m / mean 4 m / worst
7 m** over 15 flights at zeroed bias.

When changing guidance, sweep parameters through `boosterland.tests.testFlightSim.fly`
with a modified `Config` rather than guessing.

**Sweep in atmosphere with a slew rate set.** `fakeksp.Vessel(slew_deg_s=...)`
rate-limits rotation and thrusts along where the vehicle actually points.
Without it the fake obeys every steering command instantly, which is the
assumption that hid failure 6. ~30 °/s is what LOG7's booster managed.

A slow-turning vehicle is punished out of proportion to the turn, and not
because of the seconds the flip costs: at separation the booster is *receding
from the pad* at several hundred m/s, so every second pointed the wrong way is
downrange the burn must undo — for the 30 km/40 km state (900 m/s outbound) an
11 s flip buys ~10 km of extra work. That stretches the burn, which ends lower
and faster, which can leave no coast at all (COAST lasting a fraction of a
second with `corrections=0` is that failure). Flip/boostback times, 30 km/40
km: 5 s/88 s at 30 °/s, 11/104 at 15, 17/180 at 8, the last hitting
`BOOSTBACK_MAX_BURN_S`. Thrust misdirection is *not* the mechanism — mean
alignment error while thrusting is under 7° in every case.

`fakeksp.Vessel(gimbal_deg_s=...)` adds turn rate in proportion to throttle;
`min_throttle=` models an engine that will not run below a floor. Treat sim
distances as a lower bound even so: a rate-limited turn is the only attitude
difficulty the fake knows about, and a real booster also has to fight the
airstream to hold one.

## Flying a different booster, or a different body

Everything physical is measured, not assumed: `mu`, body radius, atmosphere
depth and the whole density profile, and the rotation vector come from the
body; mass, thrust, Isp and the COM-to-feet length from the vessel; `Cd*A` is
re-probed in flight. These are the numbers that are not:

- `PAD_LAT`/`PAD_LON` are the KSC pad, and `TERRAIN_MAX_M` (7000) is Kerbin's
  tallest ground — a hard floor under `Snapshot.landing_height`, so on a body
  with higher terrain it would clamp a *correct* height.
- `STARTUP_ACTION_GROUP` (2) assumes grid fins on AG2, `ENABLE_RCS` assumes
  there is RCS, `GEAR_DEPLOY_ALT_M` assumes legs worth deploying at 350 m, and
  `CORRECTION_MAX_BURNS` (3) is a guess at the engine's relights.
- `LEG_CLEARANCE_FALLBACK_M` (10) is your booster's half-length while
  `bounding_box` is unreadable. `CLEARANCE_RADIUS_FALLBACK_M` (15) is the same
  idea for the clearance scan, and it is a *radius* — it has to contain the
  vehicle through a 150° rotation.
- `SOUND_SPEED_FALLBACK_M_S` (340) is only reached if the body will not answer
  `pressure_at`; it sets the Mach axis the drag curve is binned on, so being
  wrong makes the bins the wrong width rather than the physics wrong.

Measured envelope, LOG7's entry state at 30 °/s:

| TWR (liftoff) | free throttle | 40% throttle floor |
|---|---|---|
| 0.89 | fails outright, 139 m/s | 24 km out |
| 1.37 | 713 m | 1550 m |
| 1.79 | 75 m | 24 m |
| 3.09 | 74 m | 8 m at 5.2 m/s |
| 6.19 | 7 m at 6.3 m/s | 103 m at 7.2 m/s |

TWR below ~1.4 at separation is not flyable and no tuning fixes it. An engine
that cannot throttle below ~40% lands fine at moderate TWR but arrives harder
as TWR climbs, because its *minimum* thrust then exceeds a hover —
`landing_throttle` has no model of a throttle floor, which is the first thing
to add for a vehicle that has one. Vehicle mass from 5 t to 200 t all complete
the sequence (311 / 75 / 191 / 1294 m); the 200 t figure is partly an artifact
of `fakeksp` giving every vessel the same `Cd*A`.
