# The spaceplane: design reference

How the spaceplane (`spaceplane/`) is built and why: the runway, the vehicle's
measured airframe, the architecture phase by phase, testing without KSP, and the
entry states.  It changes when the design changes.

- **Current state and what to do next:** [`spaceplane/CLAUDE.md`](../../spaceplane/CLAUDE.md).
- **What each session measured, in order:** [journal.md](journal.md).  Session
  write-ups ("Session handoff, ...", "Session, 2026-09-23" and the sections
  between them) live there, not here.
- **What broke and why each guard exists:** [failures.md](failures.md).
- **The farm:** [../testInstances.md](../testInstances.md).

## The runway

```
threshold 09 (west end)  lat -0.048600  lon -74.724466  heading 090
threshold 27 (east end)  lat -0.048600  lon -74.495283  heading 270
                         2.4 km recorded of a 2.5 km strip
```

Both ends are carried and `Runway.choose` picks between them, because a glider
with a 3.4 glide ratio arriving from the east cannot turn round to land
westward. It chooses on the **arrival bearing** — the chord from the vehicle to
the threshold with the threshold's own vertical removed — and not on the
current heading: comparing a velocity here with a runway direction a thousand
kilometres away as plain 3-vectors is meaningless on a sphere, and doing so
picked the wrong end for an entire offline entry. The aim point is
`TOUCHDOWN_AIM_M` past the threshold, because the flare floats.

## The vehicle, measured

`Untitled Space Craft`: a Mk1-3 pod with wings, 23 parts.

```
wet                 9.495 t    (LF 250.2 + Ox 305.8 = 2.780 t, mono 0.600 t)
after the drain     6.715 t    -- 29% of the vehicle, and of the wing loading
lift units          4.72       capsule body lift 1.4 (30%), wings 2.0, ctrl 1.32
LV-T91, 125 kN vac, Isp 355    1206 m/s available; a deorbit needs 40-120
```

`testInstances/planeprobe.py` measures the rest against the real craft with no
flight at all (`planeprobeGearup.txt`) — **and its subsonic half is wrong by
about 1.8x; see failure 13.** The table below is kept because its hypersonic
rows and all of its *ratios* are sound and the design rests on those; every
number that is a magnitude subsonic — the stall speed, `ClA max`, the glide
ratio — is superseded by the flown polar underneath it.

```
                 max L/D    at    ClA max   at      CdA 0deg  30deg  90deg
  55 km 2270     1.28      20deg   9.21    30deg      0.99    9.19   29.89
  35 km 2150     1.44      20deg   9.27    30deg      0.83    8.50   28.60
  25 km 1500     1.50      20deg   9.33    30deg      0.78    8.27   28.12

  subsonic, 100 m, 6.715 t
  stall        37.1 m/s at 30 deg     ClA max 85 m^2 -- nine times hypersonic
  best glide   64.2 m/s at 10 deg     L/D 3.39, sink 18.9 m/s
  min sink     48.0 m/s at 16 deg     L/D 2.86, sink 16.8 m/s
```

**`spaceplane/airframe.py` reads these off the table the flight sweeps for
itself**, and every flight logs what it found beside what is configured:

```
airframe, from its own swept table: stall 55.4 m/s (ClA max 50.2,
discounted 30%), best glide L/D 2.01 at 12 deg and 68.9 m/s
airframe DISAGREES: STALL_SPEED_M_S is 48.00 and the swept table says
55.43 (15% out) -- one of them is not this aircraft
```

It does not adopt them — a derived number is a hypothesis until the game
agrees — but it makes the disagreement impossible to miss, which is exactly
what was missing when four constants were transcribed out of a file that was
wrong by 1.8x (failure 20).

The polar the airframe actually flies, from `./spaceplane/tools/polar.py` — `act=ClA/CdA` off
every subsonic GLIDE and APPROACH line ever logged, binned on the angle of
attack *achieved*, and tightest over the flights flown with the attitude
controller tuned:

```
  aoa      n     L/D    ClA    1g speed        planeprobe said
   10    186    1.75   15.3     83.7 m/s       L/D 3.39, ClA 28.4
   12    519    2.10   22.9     68.4           L/D 3.26, ClA 36.1
   14   1013    1.52   24.6     66.0           L/D 3.07, ClA 43.6
   16   2882    1.46   26.8     63.2           L/D 2.86, ClA 50.7

  best glide   L/D 2.1 at 12 deg, ~68 m/s
  stall        ~48 m/s -- the most lift ever measured is ClA 45-50 at 22-26
```

so `STALL_SPEED_M_S` is 48.0, `APPROACH_BEST_LD` 2.10, the approach speed
`1.60 x 48 = 77 m/s`, and the gate is 2400 m up rather than 1400: at L/D 2.1
the old gate could not reach the runway at all.

Four things in that table run the whole design:

- **Maximum lift is at 30 degrees and maximum L/D at 20**, at every altitude.
  Not 40 (the shuttle) and not 90.
- **`Cl*A` at 90 degrees is exactly 0.00**, while `Cd*A` is 29-33 m^2 against
  0.8 at zero. Holding the vehicle broadside up high is *maximum brake and zero
  actuator* — and only holdable while the dynamic pressure is negligible, which
  is exactly when the drag it buys is negligible too.
- **`Cl*A` varies ninefold between subsonic and Mach 5** (85 m^2 against 9),
  which is why the aerodynamic table is two-dimensional where boosterland's is
  one.
- **The best glide ratio is 2.1 flown** (3.39 probed) — a steep, fast,
  shuttle-shaped approach with a long rollout, not a light-aircraft one, and
  steeper than the probe claimed: 26 degrees, not 16.

**The approach speed is 1.7-1.9 times the stall speed, and 1.3 is not
flyable.** The shape of this conclusion survives failure 13; the numbers in
the table do not, because they are multiples of a stall speed that was 37.1
and is really about 48. At the real stall the old `1.8 x 37.1 = 67 m/s`
approach was **1.4 Vs**, which is inside the stalling half of this very
table — which is why every flight touched down at 25 m/s of sink. Integrating the flare against the measured table says the textbook
1.3 Vs approach *stalls in the manoeuvre*: arresting 16 m/s of sink needs a
high angle of attack, which drags at 50 m^2, which eats the airspeed the lift
is made from.

| approach | x Vs | outcome | flare uses | touchdown |
|---|---|---|---|---|
| 48.2 | 1.30 | **stalled** | 22.7 m | 37.1 m/s |
| 55.6 | 1.50 | **stalled** | 16.9 m | 37.0 m/s |
| 63.1 | 1.70 | arrested | 12.1 m | 46.3 m/s |
| 70.5 | 1.90 | arrested | 11.1 m | 54.4 m/s |

**The rollout has never been measured and the approach speed is chosen
against it.** There are about 2.2 km of tarmac past the touchdown aim, so
stopping from 115 m/s needs 3.0 m/s^2 and from 90 m/s needs 1.8. The only
rollout this project has completed managed **1.04 m/s^2** while shedding
twenty of twenty-three parts, which is a measurement of a vehicle
disintegrating and not of its brakes. Until a clean landing produces the
number -- the STOPPED line now carries it -- the fast approach that fixed the
flare is an unpaid bill at the other end.

Gear down costs little: L/D 3.39 -> 3.07, min sink 16.8 -> 17.7 m/s, flare
12.1 -> 14.6 m. 2.4 km of runway is not the binding constraint — stopping from
46-54 m/s needs 0.44-0.61 m/s^2 and gear-down drag alone gives about 1.

**Gear down costs 3% of the glide ratio in flight, not the 19% the probe
implies.** Controlled for altitude and phase (the 900-1700 m band, APPROACH
only, a gear-down arm against a gear-up one), `CdA` rises **16%** and the
ground-over-height ratio moves only **1.79 -> 1.74**. The mechanism is the
guidance, not the aerodynamics: `APPROACH_SPEED_PATH` commands the descent
angle that holds the commanded speed, so extra drag is absorbed by
re-trimming alpha (9.5 -> 9.2 deg) rather than by steepening the path.
**A drag device is not a range lever unless the guidance commands it against
a surplus.** `GEAR_DRAG_FRACTION` was 0.19 on the strength of the probe and
is now 0.03.

### This vehicle has no aerodynamic control at all

> **Retracted, 2026-09-23 -- this section is wrong.** Every surface on this
> craft has all three axes enabled. The probe below read the part menu's
> `Pitch=False`, which is the *ignore* flag (False means active), and
> `available_torque` on the pad at q=0. Measured live on `qs_cone` at
> ~10 kPa: 95-276 kN m of `available_control_surface_torque`, and a pitch
> input of +1 against -1 differs by 0.14 rad/s after one second **with the
> wheels off**, against exactly 0 once the axes are disabled. See "Session,
> 2026-09-23". The paragraph is left as written so the citations to it
> still resolve.

Probed live on the craft (`testInstances/ctrlsrf.py`, `surfacespan.py`):
**all six control surfaces have Pitch, Yaw and Roll disabled** -- 0 of 6 have
any axis enabled -- and `available_torque` is (15000, 15000, 15000) N m,
which is the reaction wheels alone. The aircraft is flown entirely on
reaction wheels; the six surfaces are fixed lift and drag.

| surface | position (vessel frame, y fwd, z down) | span axis | what it is |
|---|---|---|---|
| `elevon2` x2 | x=+-1.67, y=-1.97 | (+-1,0,0) | horizontal, wing elevons |
| `smallCtrlSrf` x2 | x=+-2.50, y=-1.78 | (0,0,1) | **vertical** -- winglets |
| `smallCtrlSrf` x2 | x=+-1.00, y=+2.76 | (+-0.94,0.34,0) | horizontal, canards |

Each is 1.0 m^2 against a craft `CdA` of ~5.6 m^2, and every one exposes
`Deploy`, `Deploy Direction` and `Deploy Angle` (20) plus `Extend`/`Retract`
through the generic module interface. kRPC's typed `ControlSurface` wrapper
does find them, despite the four mods this install keeps that replace the
stock module.

**This retro-explains a great deal.** It is why kRPC reports no aerodynamic
pitching moment; why the alpha ceiling behaves as a dynamic-pressure-dependent
*torque* limit that collapses as q rises rather than as a stall; and why
`logs/LOG2756`'s edited craft could not hold 26 deg of commanded alpha on the
same 15 kN m -- not a wing problem, the wheels were fighting a larger
aerodynamic moment with identical torque. Any future change that assumes the
elevons trim the vehicle is assuming hardware this craft does not use.

## Architecture

```
spaceplane.autopilot   phase machine, the only kRPC-aware control code
  |-- environment.py   air, frame, runway, and the (alpha, Mach) table
  |-- trajectory.py    RK4 propagator; gravity, drag, lift, rotating terms
  |-- guidance.py      the glide solve, the approach, the flare, the deorbit
  |-- telemetry.py     per-tick values through streams
  `-- gui.py           in-game panel
```

```
STANDBY -> DEORBIT -> DRAIN -> COAST -> GLIDE -> APPROACH -> FLARE
                                                               |
                                                          ROLLOUT -> STOPPED
```

Phases hand over at **altitudes and geometry, never at a miss distance**.

### The airflow is aimed, not the vessel

`simulate_aerodynamic_force_at` takes the velocity to evaluate at, so a table
cell can be probed at exactly the angle of attack it tabulates by constructing
a velocity that makes that angle with the vehicle's own axes and leaving the
attitude alone. That removes at a stroke the failure that cost boosterland two
sessions (13 and 14): every bin there was written broadside and then used to
fly a nose-on descent. Here the probe is *always* in the configuration it is
tabulating, because the configuration is an input.

The vehicle's axes are *measured*, not assumed — kRPC documents its vessel
frame as x-right, y-forward, z-out-of-the-bottom, and `transform_direction` of
body +y is checked against `Vessel.direction` before anything is believed. Two
bugs came out of not doing that:

- `planeprobe.py` first aimed the nose at **retrograde**, copying the booster's
  convention, and measured this aircraft flying tail-first. The tell was a
  `skew` column sitting at 173 degrees.
- `Telemetry` first took `abs(min(lower))` of the bounding box for the wheel
  clearance, which is right for a booster standing on its tail along -y and
  returns *half the wingspan* for an aircraft. The wheels are at **max z**.

The whole 13-row table is swept in STANDBY, in vacuum, before the flight
starts: 182 probe calls, under a second, so the first prediction is made on
measured coefficients rather than on a warm-up.

### The airframe's ceiling is learned, not tabulated

In dense air this vehicle does not hold what it is told: asked for 20 degrees
at Mach 4 it delivers 15-18, and the deficit grows with dynamic pressure. The
plant is a **saturation**, `achievable = min(command, holdable(q))` -- not a
gain, and the difference decides everything. A proportional model is undone by
the solve: told 0.85 of its command will be flown it asks for more, the factor
cancels, and the prediction believes the inflated number. In game that took
the shortfall from 45-63 km to 99-118 km. A ceiling cannot be talked round.

`holdable` cannot be probed, because `simulate_aerodynamic_force_at` returns a
force and kRPC will not report a pitching moment at any price. So
`trajectory.Holdable` **watches instead**: every tick the loop already knows
what it asked, what the vehicle achieved, and the dynamic pressure it happened
at, and a tick where the vehicle falls short by more than a threshold is
evidence about the ceiling. It bins by `q`, keeps the highest angle seen while
saturated (one low sample is a bank reversal, not a limit), and raises a bin
when the vehicle tracks comfortably.

It could have been a table -- 663 logs of this craft give 11280 samples of a
clean saturation. **It is not one on purpose.** A fitted table flies the next
vehicle on this one's trim, which is the same objection that makes the aero
table swept in STANDBY rather than written down, the vessel's axes measured
rather than assumed, and the frame's handedness computed rather than reasoned
about. What is in `Config` is the estimator's shape -- bin width, what counts
as saturated, how many samples before a bin is believed -- and no angle.

Two properties it has to keep:

- **A bin with no evidence returns `None`**, never a large angle a `min` would
  accept. A missing answer must not look like a good one.
- **It extrapolates downward, never upward.** The propagator asks about
  dynamic pressures the vehicle has not reached -- that is what a prediction
  is -- and the ceiling falls as the air thickens, so a query past everything
  seen is answered with the lowest ceiling so far. Predicting less lift than
  the vehicle turns out to have makes the burn larger, and surplus is the
  recoverable side.

The log carries what each flight learned (`holdable alpha, learned: ...`), so
the measurement outlives the process.

**And the control loop's own ratchet has to be allowed to believe it.** Its
floor was `GLIDE_ALPHA_DEG` -- the angle the guidance wants for range -- so
the ceiling walked down to 20 and stopped while `Holdable` was reporting
17.8 and the vehicle was achieving 14. Everything below 26 km then flew at a
commanded 20 that the airframe could not hold, with the solve propagating
that 20. The floor is `ALPHA_CEILING_FLOOR_DEG` now, and it is a statement
about the plant rather than about what would be nice. Failure 26.

### Monopropellant is spent on turns, not on holds

`common/rcs.py` is shared by both craft. A phase says where RCS is
*permitted*; the valve says when it opens, from the angle between the
commanded nose and the real one -- on above `RCS_ERROR_ON_DEG`, off once
inside `RCS_ERROR_OFF_DEG` for `RCS_SETTLE_S`.

Permission alone is not enough, because the expensive part of a permitted
phase is not the turn. kRPC's autopilot hunts around whatever it is given, and
a thruster held open through the hunt pays for every oscillation: the
spaceplane holding prograde in orbit and doing nothing spent 55 of 150 units
in two minutes, and the coast measured 3 kg/s against a 600 kg tank.

Three properties, one constant each, and each is a way propellant was wasted:

- **A relay, not a threshold.** One angle compared every tick switches the
  thrusters at the frequency of the oscillation they are damping. The gap
  between ON and OFF is what makes that impossible -- the same shape as the
  glide's bank deadband.
- **It does not drop out mid-swing.** `SETTLE_S` asks the error to *stay*
  small; a swing through zero is not an arrival.
- **A missing answer is not a good one.** No command yet, or a nose that
  cannot be read, holds the valve rather than deciding it is settled.
  Reporting zero error there shuts the thrusters in the middle of a flip.

And the spaceplane has a fourth, `RCS_Q_MAX_PA`: four RCS blocks are a
rounding error against the aerodynamic moment, and the shortfall in held angle
of attack is a saturation (failure 23) that thrusters cannot talk round. The
booster sets it to 0 -- off -- because its descent really is flown on RCS,
wheels and the gimbal, which is what `CORRECTION_GIMBAL_THROTTLE` is for.

**A turn nothing is waiting on is free -- and the deorbit flip is not one.**
It was flown wheels-only for exactly one session on the argument that a vacuum
flip has minutes to happen in; it cost 50 km on every arrival, because the
alignment lives inside `fly_deorbit_burn` and that phase is only entered when
it is time to burn. The flip is always on the critical path. Failure 43. What
the valve still saves is the *hold* afterwards, which is where the propellant
actually went.

`boosterland/tests/fakeksp` cannot check any of this from a flight: its `control.rcs` is a
bare bool and the slew does not depend on it, so the closed-loop test only
shows that nothing else broke. In game, read the `rcs on` / `rcs off` events
against the flip, and the monopropellant left at the interface.

### Angle of attack brakes, bank steers

The division of labour is forced by the measurements:

- **Angle of attack** is the downrange and energy lever. Drag at 30 degrees is
  ten times drag at zero.
- **Bank magnitude** is the other one: rolling lift off the vertical lets the
  vehicle sink into thicker air, which is what actually decelerates it.
- **Bank sign** is the *only* cross-track authority anywhere in the flight.

**The solve is decoupled, not a 2x2**, and that is a correction. The first
version inverted a measured Jacobian for both controls at once, the way
`boosterland.guidance.solve_steer` does. It does not work on this plant:
`d(long)/d(alpha)` **changes sign during the entry** (from 80 km an extra
degree lengthens the flight; by 45 km it shortens it), and alpha spends most of
the entry against its stop, where the bank solved alongside it was no longer
the right partner and nothing re-solved it — offline that sat at 32 degrees and
-4 of bank for five hundred seconds with a 72 km miss in front of it. So alpha
takes the miss, bank magnitude takes the residual, bank sign takes cross-track.

**Both controls have a floor, and both floors are the same lesson.** Range is
not monotone in either at the shallow end:

| alpha (bank 0) | 5 | 10 | 20 | 25 | 30 | 32 |
|---|---|---|---|---|---|---|
| range (km) | 1610 | 1283 | **1265** | 1437 | 1730 | 1823 |

| bank (alpha 30) | 0 | 30 | 50 | 70 |
|---|---|---|---|---|
| range (km) | 1730 | **1872** | 1679 | 1432 |

Max L/D gives the *shortest* flight, because a high angle of attack makes
enough lift to hold the vehicle up where the air is thin and it glides on; and
a little bank *lengthens* the flight, because the lift it gives up vertically
is more than repaid by the drag it stops making. `SOLVE_ALPHA_MIN_DEG` (20) and
`SOLVE_BANK_MIN_DEG` (30) put the solve on the monotone branch of each, with
560 km of authority still between the corners. **But see failure 10** — the
alpha floor sits at the bottom of the bowl and `_solve_range` now brackets
instead.

### The propagator flies the mean of a reversing entry

`Steer.reversing` makes `acceleration` apply only the *vertical* component of
lift, reduced by `cos(bank)`, with the lateral part averaging to nothing. That
is what the vehicle does over minutes: it holds a bank magnitude and swaps the
sign whenever the azimuth to the runway leaves a deadband.

Propagating one constant lean instead predicts a vast curving arc — offline it
reported cross-track misses of 80-98 km that *alternated sign with every
guidance cycle*, and the bank command read `+18.8, -21.2, +18.8, -21.2` on
every tick: a vehicle rolling through 40 degrees twice a second, which no
airframe flies, and which left cross-track completely uncorrected while looking
— in a plot of bank *magnitude* — like it was working. The sign is chosen
instead by an azimuth deadband that narrows with range to go.

The *truth* model in `spaceplane/tests/glidesim.py` sets `reversing=False`, because the
vehicle at any instant has a real bank and really turns. Left at the default it
simulated a vehicle that could never change heading at all.

**And the angle of attack is held through a transit, which is right, but
"transit" has to be asked as a question and not guessed from the lean's
size.** The first version held it whenever the commanded bank was under 25
degrees, on the argument that `SOLVE_BANK_MIN_DEG` is 30 so anything smaller
must be a reversal in progress. Below about 27 km the cross-track band
narrows and the solve genuinely asks for a few degrees of lean, so the test
became permanently true and the angle of attack was frozen for 57-83% of
every entry's glide — the whole second half, with the solve's answer
computed and discarded twice a second. `guidance.bank_in_transit` asks
instead whether the rate-limited command has *arrived* at the lean it is
committed to. Failure 25.

**So `Prediction.cross` is a rate, whatever its units say.** With no lateral
lift in the model, the cross-track it reports at the gate is the vehicle's
present lateral velocity carried forward — and the reversal test used to
compare it against a fixed 500 m at every range. A relay on a rate, with a
roll that takes thirteen seconds to cross between the stops, is a bang-bang
loop with lag: measured in game it reversed on an 18-second period, overshot
the band sevenfold, and spent 60% of the level-off with the roll against
`BANK_RATE_DEG_S`. **Both** deadbands scale with range to run now, for the
reason the azimuth one always did. Failure 10e, which corrects 10d.

### The arrival-speed cap

Holding the entry angle of attack all the way down is right while the vehicle
is fast, but held to the end it decelerates to *that angle's* equilibrium glide
speed, which the table puts at 37 m/s. The first entry propagations arrived at
the gate doing 37-44 m/s against a 37.1 m/s stall and a 67 m/s approach speed:
they arrived unable to flare. So above `SPEED_HOLD_FACTOR` times the approach
speed the cap is wide open; below it the cap becomes a speed hold and angle of
attack is what holds it. With it, every deorbit burn from 30 to 300 m/s arrives
at the gate at **65 m/s**.

**A cap is only half of a hold, and the missing half is where the flight was
lost.** Above the threshold the cap returns `ALPHA_MAX_DEG` — no constraint
at all — which is exactly the state a diving vehicle is in, so the hold
disengages precisely when it is needed. Solved on the miss at the gate's
*altitude*, the cheapest way for the glide to null a surplus is then to
**lower** the angle of attack: less lift sinks the arc onto that altitude
sooner and the predicted miss converges nicely. Measured, it converged to
+1.9 km while the vehicle arrived over the gate at 1381 m doing 96.7 m/s with
95.7 of it straight down. Later, with the attitude controller tuned and the
polar honest, the same mechanism handed the approach a vehicle at **219 m/s
and 74 degrees off the runway heading** — position nulled, everything else
spent. The energy is never dissipated in either case; it is moved out of
height and into speed, and nothing after the gate can spend speed.

`SPEED_FLOOR_ON` makes the hold two-sided below `SPEED_FLOOR_MACH`: the same
law used as a floor as well as a cap, so diving is range authority the solve
no longer has and the ranging goes back to bank, which is the division of
labour the entry design already claims. It is floored on the subsonic solve
floor as well as on the 1 g trim, because trim *falls* with speed — a floor
built on trim alone is loosest exactly where the dive is.

**The cap is applied by the propagator *and* the control loop**, and it was not
at first — boosterland's rule about `landing_command` being shared: a
prediction of a law nobody flies is a prediction of a trajectory nobody flies.
APPROACH and FLARE are deliberately outside it, because the flare has to be
able to ask for maximum lift at a speed the cap would refuse; while it could
not, the offline landings arrived at 18 m/s of sink with the log showing a
perfectly sensible flare command.

### The scatter is made by the burn, not by the glide

Fifteen flights from three entry-interface quicksaves -- `entrysave.py` takes
one at 57.9 km on the way down, and `Autopilot.engage` enters GLIDE directly
when handed a vehicle below `ENTRY_INTERFACE_M` -- five repeats from each,
one unchanged configuration:

```
  save             n    mean      sd    range
  qs_entry @ksp0   5   -25.1    2.50    5.8 km
  qs_entry @ksp4   5   -23.8    2.99    7.5
  qs_entry @ksp5   5    -3.8    0.88    2.4

  pooled within-save sd   2.31 km   (15 flights)
  full-flight sd          14.8 km   (9 flights of one configuration)
```

The glide handed an identical state repeats to **2.3 km**. The whole flight
scatters at **14.8**. As variance that is 5.3 against 219: **the glide makes
2.4% of the scatter and everything before the interface makes 97.6%.**

Two things follow and both are large:

- **The +-10-15 km this project has chased since failure 10 is a deorbit
  problem.** Every margin, schedule and floor tried inside the glide has
  been a small effect underneath noise it did not generate, which is why
  nine-flight arms kept coming back "inside the scatter".
- **Glide questions are now 25x cheaper.** Five flights from a fixed save
  give a standard error of 1.0 km, so a 3 km effect is resolvable in twenty
  minutes; the same question from orbit needs about 25 flights an arm.
  Comparisons are run paired across several saves, so the entry state -- the
  dominant term -- cancels.

The three saves land -25.1, -23.8 and -3.8 km, so there is also a real 21 km
spread in *what the deorbit delivers*, sitting on top of a real 24 km
shortfall from two of the three states. Those are separable problems now and
they were not before.

### One metre per second at the interface is eleven kilometres on the ground

Over 79 full flights off one save, the specific mechanical energy the vehicle
carries across the entry interface correlates with the landing miss at
**0.88**:

```
  speed at the interface   2115.6 +- 1.0 m/s
  altitude                 57901  +- 56 m
  landing                    -9.0 +- 11.1 km
```

So the plant's sensitivity is about **11 km of along-track per m/s of entry
speed**, and the deorbit delivers +-1 m/s. To land inside a kilometre it
would have to deliver +-0.1. That single number explains why every glide-side
margin measured this session came back "inside the scatter": they are 1-2 km
knobs sitting underneath an 11 km input error.

**Where the metre per second comes from is still open, and four candidates
are already eliminated.** The spread is present at burn exit and does not
grow during the coast (specific energy sd 0.0034 MJ/kg at 76 km, 0.0023 at
56 km -- the atmosphere takes slightly more from the hotter arcs, so the
coast is mildly self-correcting). It is not:

- **the cutoff.** Every flight exits on `+0.00 m/s still owed`; the taper
  and `DEORBIT_MIN_THROTTLE` resolve the last of the burn to ~0.03 m/s.
- **the commit point.** Cutoff UT spreads 17.6 s and correlates -0.08 with
  interface energy.
- **the solved dv**, at least as logged. Within a single integer dv the
  interface speed still scatters +-0.95 m/s and the landing +-10.6 km.
- **thrust pointing.** 3.6 +- 1.6 degrees off retrograde is a 0.2% cosine
  loss, 0.07 m/s on a 35 m/s burn, and correlates -0.09.

What is left is the stop test itself: it ends the burn when
`deorbit_remaining` says the trajectory already does the job, every flight
reports `range error +0 m` when it does, **and the flights then land 11 km
apart.** The propagation says the same thing about states that behave
differently, which makes this a model question rather than a control one --
and the first place to look is that `Holdable` has no evidence at all during
DEORBIT, so the burn is solved against a glide with no alpha ceiling.

### And that eleven is not a sensitivity to energy: the plateau, measured

**`ladder.py`, 63 flights, two entry states.** `savegen.py` rewrites a save's
orbital elements in place, and it works on an *entry-interface* save as well
as an orbital one -- same position, same flight path angle, `--prograde N`
more speed. That makes the interface energy a dial with nothing else attached
to it, which is the one thing the 79-flight correlation above cannot offer:
there, interface speed co-varies with everything else a different burn
changes, most obviously where the vehicle crosses 58 km and therefore how far
it still has to go.

Turned, the dial says the plant is **not** 11 km per m/s.

```
  qs_entry @ksp5, 48 flights, 4 an arm        qs_entry @ksp0, 15 flights, 3 an arm
  dv      along     sd    se                  dv      along     sd    se
  -16   -37.7 km   1.6   0.8                    0   -41.4 km   1.8   1.0
   -8   -21.1      2.1   1.1                   16    -7.3      0.7   0.4
   -4   -11.8      0.4   0.2                   24    -2.9      0.2   0.1
   -2    -9.2      0.7   0.4                   32    -0.4      0.8   0.4
    0    -6.9      0.5   0.3                   40   +20.8      0.9   0.5
   +8    -5.4      3.8   1.9
  +16    -3.3      0.3   0.2
  +20    +1.4      0.8   0.4
  +24   +10.4      1.8   0.9
  +28   +20.7      1.3   0.6
  +32   +31.3      2.3   1.2
```

Both states have the same shape and it is a **plateau with two saturations**:

```
  arrival below about -8 km    1.9 - 2.3 km per m/s   cannot stretch
  arrival -8 km to the runway  0.2 - 0.6              absorbed
  arrival past the runway      2.5 - 2.7              cannot shorten
```

Three things follow.

- **The plateau is about 16 m/s wide and it sits a few kilometres short of
  the runway, not on it.** Inside it the glide nulls what it is handed; the
  converged arrival is -7 to -3 km rather than zero, which is a bias and not
  noise. Outside it, in *either* direction, the vehicle is saturated and the
  energy passes straight through.
- **The current aim delivers energy at the bottom edge of the plateau on one
  state (+19 m/s would centre it) and 30 m/s below it on the other (+32).**
  That is the shortfall, and it is a *placement* problem: there is nothing
  wrong with the glide's authority, it is being handed states outside it.
- **Eleven km per m/s is a confound.** At fixed geometry the sensitivity near
  the operating point is 1.2 km per m/s and inside the plateau 0.2. The
  ±0.1 m/s burn accuracy the 79-flight correlation seems to demand is
  therefore chasing the wrong quantity; what co-varies with interface speed
  across real deorbits -- range to go at the crossing is the obvious
  candidate -- is carrying most of the 11, and has not been separated yet.

**The mechanism is in the logs and it is the one the entry design claims.**
Bank magnitude by 5 km band, one flight from each of three rungs of the first
ladder:

```
              |bank|  20-25 km   15-20   10-15    5-10      lands
  dv   0        18.4       1.3     0.0     0.0     0.0      -6.9 km
  dv +16        46.5      38.4    24.6    33.4     8.2      -3.3
  dv +20        51.2      30.5    53.8    37.3     0.9      +1.4
```

The baseline flight has **no bank left below 20 km**: the range solve has
spent all of it and is asking for 19 degrees of angle of attack that the
airframe answers with 14 (failures 22, 23, 26). The surplus flights hold 25
to 54 degrees of bank right down to 10 km. The alpha ceiling is identical in
both -- it simply stops being load-bearing, because range is coming from
bank, which is monotone and abundant all the way down, instead of from alpha,
which saturates exactly where it is needed.

So *aim long and spend it* is right, it is already this design's stated
principle (`DEORBIT_WINDOW_BIAS`, `guidance.glide_reserve`), and the airframe
flies it. What has never been true is the amount: the surplus deliberately
carried is 8 km of reserve and a 0.25 window bias, against a plateau that
would take three to four times that.

**What this does not measure.** The dial is +N m/s at a *fixed* interface
state. A deorbit that under-burns by N m/s does not produce it -- it also
crosses 58 km somewhere else -- so the ladder sizes the glide's capacity and
not the deorbit's mapping onto it. Moving `DEORBIT_WINDOW_BIAS` and flying
full flights from orbit, n≈9 an arm, is the test that closes that gap and it
has not been run.

### Picking this up: the saves exist, the next test does not

The ladder saves are written into `ksp0/1/4/5/saves/default/` and survive a
reboot. Two families, each generated from an entry-interface quicksave with
`savegen.py --prograde N`:

```
  qs_l0   the ksp5 entry state, unmodified     lands -6.9 km
  qs_l1..4, qs_l8/16/20/24/28/32               +N m/s at the interface
  qs_m2/4/8/16                                 -N m/s
  qs_b0   the ksp0 entry state, unmodified     lands -41.4 km
  qs_b16/24/32/40                              +N m/s
```

Re-making any of them, or making the same dial for a third state:

```bash
./spaceplane/tools/entrysave.py 0                      # alongside a normal flight, saves qs_entry
cp testInstances/ksp0/saves/default/qs_entry.{sfs,loadmeta} /tmp/
./tools/savegen.py --save-dir /tmp --source qs_entry -o qs_hot --prograde 24
./spaceplane/tools/ladder.py --arms qs_b0,qs_b24,qs_b32 -n 4 --instances 0,1,4,5
```

**The open test is the deorbit's mapping onto the plateau, and it is the
expensive one.** The ladder sizes the glide's capacity from a fixed interface
state; a deorbit that under-burns by N m/s does not reproduce it, because it
also crosses 58 km somewhere else. So the question "what value of
`DEORBIT_WINDOW_BIAS` puts the delivered state in the middle of the plateau?"
needs full flights from orbit:

```bash
./spaceplane/tools/quickglide.py --save qs_plane --instance 0 --set DEORBIT_WINDOW_BIAS=0.75
```

Arms 0.25 (control), 0.50, 0.75, 1.00, **n≈9 each** -- the full-flight
scatter is sd 14.8 km and three-flight arms against it are spaceplane failure
27, made and retracted once already. Expect ~40 min an arm on four instances.

Two things to check on the way that this session noticed and did not chase:

- **`qs_entry @ksp0` lands -41.4 km now; `docs/spaceplane/design.md` records -25.1
  for it over five flights.** Same file, same instance. Something changed the
  glide between those two batches and `defaults_fingerprint` on the logs will
  say what. `logs/LOG1192`-`LOG1206` are the new ones.
- **Every flight in all 63 broke up or splashed.** The along-track is sound
  -- that is what these measure -- but no arm produced an intact vehicle, so
  the survival problem is untouched and is now the thing standing between a
  centred arrival and a landing.

### A metric that is cheap, and one that looked cheap and is not

Interface energy is measurable about ninety seconds into a warped flight and
needs no landing, which makes deorbit arms affordable. `deorbitprobe.py`
was written to read the **first few glide predictions** instead, on the
reasoning that they summarise what the burn delivered. They do not: three
entry saves whose first-solve medians agree to +-2.7 km land at -25.1, -23.8
and -3.8. Any deorbit conclusion drawn from that proxy would have been
noise. Prefer the energy.

### The entry is right down to 30 km, and the miss is made below it

Counted over sixteen flights of the default configuration off one save
(`logs/LOG1000`-`LOG1015`), by 5 km band, medians:

```
  alt band     n    aoa cmd   aoa achieved   deficit   |bank|   long
  55-60 km   249      32.0        32.7         -0.7      35.9      -7 m
  45-50      661      31.9        32.5         -0.7      32.8     -35
  35-40      326      31.1        30.6         -0.2      29.1     -62
  30-35      269      26.8        23.1        **3.4**    31.5     -11
  25-30      246      20.8        15.2        **5.2**    30.0     -62
  20-25      289      18.9        14.3          4.5      10.3   -1832
  15-20      391      18.9        14.4          4.4       3.6   -2027
  10-15      250      19.9        13.4          6.4       3.6   -1236
   5-10      461      19.9        15.1          4.9       2.9   -3541
```

Three things happen at once below 30 km and they are one thing. The vehicle
stops holding what it is told (5-6 degrees short of a command already at
`SOLVE_ALPHA_MIN_DEG`); the bank collapses to 3 degrees, because the range
solve has taken all of it; and the predicted miss goes from **tens of metres
to kilometres**. Above 35 km the guidance is not approximately right, it is
right to 60 m.

So the entry does not fail. What fails is the last 25 km, and it fails in the
one way this airframe cannot answer: it is **short**, and stretching costs an
angle of attack that `Holdable` has already learned it cannot hold
(`logs/LOG1015`: 17.9 deg at 6.8 kPa, 14.9 at 10 kPa, against a solve floored
at 20).

### The glide aims long and gives it back on a schedule

`guidance.glide_reserve` is the answer to that, and it is the one-sided
principle of `DEORBIT_WINDOW_BIAS` moved one phase later. The solve nulls
`long - reserve` rather than `long`: the reserve is full at
`GLIDE_RESERVE_FROM_ALT_M`, decays linearly to zero by
`GLIDE_RESERVE_TO_ALT_M`, and is zero from there to the gate.

The vehicle is therefore deliberately long through the altitudes where being
long is cheap to fix, and is aiming at the gate itself by the time it gets to
the altitudes where being short is fatal. Bleeding surplus energy is cheap and
monotone -- bank rolls lift off the vertical, drag goes as `rho v^2`, and both
get *stronger* all the way down. Manufacturing range is not: it costs the one
control the plant takes away exactly there.

The decay is load-bearing. A reserve held to the ground is a long landing,
which is no more recoverable than a short one.

`env.spending`, which gates the speed floor (`SPEED_FLOOR_WHEN_LONG`), is
measured against the same target -- gate plus reserve, not gate. Read against
the gate, a vehicle flown deliberately 8 km long while still short of its own
aim point would have the brake on, which is failure 22 in the reserve's frame.

**Flown, and adopted.** Three flights per arm, all three arms in the air at
once on matched instances, off the same save; along-track at the gate:

```
  qs_plane          f1        f2        f3      mean
  off           -20.1 km  -34.5 km  -12.1 km   -22.2
  8 km           -2.0      +0.2      -1.7       -1.2
  16 km         -17.7     -12.7      +0.7       -9.9

  qs_plane_inc
  off           -19.8     -19.5      -4.9      -14.7
  8 km           -2.8      -7.2      -1.0       -3.7
  12 km         -10.3      -2.7      -8.3       -7.1
```

Six flights against six: **-2.5 km mean against -18.4**, and on `qs_plane`
the spread went from 22 km to 2.2. `GLIDE_RESERVE_M` is a **peak and not a
gain** -- 12 and 16 are both worse than 8, the same shape
`DEORBIT_WINDOW_BIAS` has -- and the peak between 8 and 12 is unresolved.

**Then flown properly, and it does not survive.** Nine flights per arm,
arms rotated across three instances *and* across running order so neither
could stand in for the other:

```
  arm      n    mean     sd     se    vs control
  off      9   -17.7   14.8    4.9
  decay    9   -12.3   11.4    3.8    +5.4 km +-6.2   (0.9 sigma)
  flat     9   -10.7    9.8    3.3    +6.9 km +-5.9   (1.2 sigma)
```

Both shapes land 5-7 km better than the control and neither clears two
standard errors; over every flight since `LOG1000`, classified by what
actually flew, reserve-off (n=22) and reserve-8 km (n=23) have means of
-9.1 and -8.9 km. **Not adopted, and not refuted** -- resolving an effect
this size needs about 25 flights an arm. The twelve-flight table above is
kept because believing it is spaceplane failure 27.

What the batch *did* settle is the number every future comparison needs:
**the along-track scatter of one unchanged configuration is sd 10-15 km**.

### The heading alignment cone: stop asking the entry for an energy

**The change that moved the landing from tens of kilometres to hundreds of
metres, and it is a change to what the entry is asked for rather than to how
well it does it.**

Every straight-in arrival makes the same demand: be at one point, at one
altitude, with one energy, after fifteen hundred kilometres of unpowered
flight. Measured, this vehicle meets it with about 15 km of along-track
scatter, and 97.6% of that variance is made before the entry interface where
no amount of glide tuning can reach it. Reserves, schedules, gate placement
and a burn ten times more accurate have all come back inside that noise.

A shuttle does not make that demand. It aims *over* the field with height in
hand and spends the surplus turning: a circle tangent to the extended
centreline, flown round as far as the energy needs, rolling out on final when
the height that is left matches the distance that is left. Rolling out of the
circle **is** being lined up, so the alignment and the energy management are
the same manoeuvre.

```
                 .--- circle of radius R, centre abeam the gate ---.
                /                                                   \
   arrival --->|                                                     |
                \                                                   /
                 '--------------------> gate ---> threshold --------'
                      rollout: turn = 0, tangent to the centreline
```

What it buys is **tolerance, not accuracy**. The control variable is the
radius:

```
path still to fly   = run in to the tangent + R x (turn still to go)
path still affordable = (height - GATE_ALT_M) x HAC_LD
```

Set them equal and solve for `R`. An energy error becomes a radius, and a
radius is a thing the vehicle can simply fly. When even the widest circle is
short of path, the answer is another lap, which is tens of kilometres of it.

Four properties, each of which a flight has already paid for:

- **The centre moves with the radius.** A circle is a heading alignment cone
  only if it is tangent to the final approach course *at the gate*. Pin the
  centre 8 km abeam and vary the radius and every circle but one misses the
  gate sideways by `8 km - R`. Flown: three arrivals rolled out on 3.0, 4.6
  and 5.7 km circles about a centre placed for an 8 km one and landed 0.2,
  1.2 and 1.6 km off the centreline of a runway 70 m wide. With the centre
  moved, the next four landed **+134, +18, -197 and -32 m** across.
- **The tangent point is `acos(R/d)` round from the vehicle *with* the turn.**
  Both offsets are tangent points and only one is reached travelling the way
  the turn goes. With the wrong sign the arithmetic still produces a
  plausible number, which is why it survived: a vehicle lined up 30 km out on
  the extended centreline -- the best arrival there is -- was costed a **149
  degree turn** instead of none, so every flight read as short of height and
  abandoned the cone immediately.
- **The turn wraps at the rollout, and the wrap has to be in the right
  place.** A vehicle sitting exactly on the rollout reads either 0 or 2pi
  depending on the last bit of the arithmetic, and the difference is a whole
  lap.
- **Lined up is not ready.** Reaching the rollout with a lap still owed, or
  with more height than the approach's S-turn can spend, is not an exit: the
  vehicle flies past, the turn wraps, and the lap it asked for happens. Two
  flights of the first clean batch read `rolled out turn=1 h=12697 laps=1`,
  handed a four-kilometre approach law a vehicle twelve kilometres up, and
  floated ten kilometres past the field.

**Mach is a veto on entering it at all.** Turn radius is `v^2 / (g tan(bank))`
-- 9 km at Mach 0.9, 30 km at Mach 1.8, **250 km at Mach 4.7**. There is no
cone to fly at entry speed, and the first flight to reach the phase entered it
at Mach 4.7 because a distance test fired while the vehicle was still 27 km up
and hot. The glide keeps the vehicle until `HAC_ENTRY_MACH`.

**And the entry's target moves up with it.** `Runway.gate` returns the *high*
gate under `HAC_ON`: the same ground point, `HAC_ALT_M` up. Two reasons, and
the second is the one that was not obvious. This vehicle crosses the field at
19 km still doing Mach 2.9 and only falls subsonic around 12-13 km, so the
cone has to be delivered to where it can be flown. And raising the target
gives back about 19 km of required range -- 12 km of height at the turning
glide ratio -- which is most of the surplus the entry was arriving with, so
the same change that makes the cone enterable also unsaturates the glide solve
that feeds it.

**The deorbit bias is now the other half.** With the cone, a long arrival is
absorbed and a short one is not, so the aim wants to sit far enough long that
the short tail never reaches zero -- and `DEORBIT_LONG_BIAS_M` at its old
40 km sends the vehicle over the field 52 km long, too far out to turn back.
See the numbers in the batches below.

### The gate, not the threshold

The glide is solved to arrive at a point on the extended centreline
(`GATE_DIST_M` out, `GATE_ALT_M` up) and everything from there in is flown
geometrically. A solve that ran to the tarmac would be solving a manoeuvre it
has no model of — boosterland failure 5, one phase earlier.

The gate has to sit **above** the best glide from it to the touchdown aim,
not below: arriving with surplus height is recoverable by raising the nose or
S-turning, and arriving low is not recoverable by anything. That
one-sidedness runs through the design and is why the deorbit aims past the
gate rather than at it.

For a long time the gate was on the wrong side of that line and nobody could
see it, because the glide ratio it was placed against was `planeprobe`'s 3.39
and the airframe flies 2.1 (failure 13). At 4 km out and 1400 m up the gate
had 2.9 km of glide in hand for the 4.6 km to the aim — **an approach flown
perfectly from it lands 1.7 km short**. The margin also has to be taken
against the *gear-down* polar, which is 1.78 rather than 2.19, so where the
gear comes down is part of where the gate goes: at 2600 m over an 800 m gear
height the reach is 5.4 km against 4.6 needed. `spaceplane/tests/testSpaceplane.py`
now asserts that inequality against `APPROACH_BEST_LD` rather than against a
written-down angle, so re-measuring the polar cannot leave the gate behind
again.

**How far past used to be a fitted fraction of the arc, and now it is the
middle of what the glide can still reach.** `guidance.deorbit_window`
propagates the two corners of `solve_glide`'s own search box — the shortest
entry it may command and the longest — and the burn is chosen to put the gate
between them, in the middle. The corners are not a guess about the airframe:
they are exactly the span of commands the glide is allowed to issue, so the
window *is* its authority. That deletes the aim, its floor, its tolerance
band and the one-sided acceptance rule, and costs one extra propagation per
candidate (2.9 s against 1.9 for a whole search, in a phase with an orbit to
spare). The burn's stop test still needs an offset and still shares one with
the search — it is read off the solution taken, by `deorbit_chosen_aim`.

Why: the fraction was re-fitted **five times in one session**, every move
forced by a change somewhere else, because it stands in for the difference
between the entry the search propagates and the entry the glide flies. See
failure 19; the rule it earned is in CLAUDE.md.

The old path is still there under `DEORBIT_AUTHORITY_WINDOW = False`, and
this is what it did:

**A fraction of the arc still to fly**, not a distance, and that shape was
measured: what the aim compensates is the
propagator's over-prediction of the glide's range, which is a *relative*
error. A fixed 300 km aim puts three entry states within 4-20 km and sends
the elliptical one 57-60 km long into the sea; the same proportion asks that
shorter entry for less and brings all four home. `DEORBIT_LONG_BIAS_M` is a
floor under it. See failure 10a for why a five-point sweep of the old fixed
bias found nothing at all.

**That 50 km is a guess, and only its sign is earned.** Nothing after the burn
can add energy, so arriving long is recoverable at any price and short is not.
The magnitude has never been measured — that means comparing the predicted gate
arrival at the burn's exit against the actual one over several entry states,
which needs flights that reach the gate. All that is known is that it is well
inside the authority that has to absorb it (the range table spans 1265-1823 km
on alpha alone), so it is conservative rather than tuned.

**Stopping the propagation at the closest approach to the gate and solving on
the height left over there was tried.** The reasoning is sound — for an
unpowered vehicle that height *is* the energy error — and it converges
beautifully and lands the vehicle 20 km away, because nulling the energy says
nothing about *where* it is nulled.

### The landing chain, and the one number that sizes all of it

**Everything from the rollout to the wheels is sized by one ratio, and that
ratio has been measured over the wrong span three times.**

The cone hands over when the vehicle is near the rollout point *and* low
enough that the approach can reach the touchdown aim from there. "Low enough"
is `(gate_range + GATE_DIST_M + TOUCHDOWN_AIM_M) / APPROACH_BEST_LD`
(`guidance.hac`'s `approach_needed`), and the approach's own S-turn measures
its surplus against the same constant. So `APPROACH_BEST_LD` decides the
handover altitude, and the handover altitude decides where the wheels touch,
at about two metres of runway per metre of height.

What the constant has to describe is the **whole** span the aim sits at the
end of -- the approach *and the flare*, because the flare is flat and long
and the wheels are what is being aimed. `landsum.py` measures exactly that,
from the rollout to first wheel contact:

```
                                   value    what it is
planeprobe, transcribed             3.39    an aircraft that does not exist
at the flare entry, APPROACH_FACTOR 2.40    1.55   one regime, one instant
over the APPROACH phase, stopping at the flare   1.95
rollout to the wheels, 18 flights   2.55 +/- 0.35   the span the aim ends
```

At the configured 1.85 against a flown 2.55, the handover from a 1500 m
rollout is `5700/1.85 - 5700/2.55` = **846 m too high**, which is 2157 m of
ground at the ratio actually flown. Measured landing bias: **+1754 m, sd
803.** The arithmetic closes.

**But the two ends cannot be moved independently, and that is the trap.**
The cone descends on its own profile -- `GATE_ALT_M` plus the circling path
still to fly at `HAC_LD` -- and the exit test compares that profile against
`approach_needed`. Raise `APPROACH_BEST_LD` alone and the test stops being
satisfiable at the rollout: the vehicle stays in the cone, descends to the
`GATE_ALT_M` floor, and leaves "out of height" from wherever it happens to
be. Flown at 2.08, **eight of eight** exits were out of height, 2.0 to 3.2 km
below profile, one of them 2435 m from the gate at 2597 m -- a geometry no
glide ratio can fly -- and none of them landed.

So the next experiment is a *pair*, and it is specified:

```
APPROACH_BEST_LD   1.85 -> ~2.20   (the pessimistic end of 2.55 +/- 0.35;
                                    a margin constant wants that end)
GATE_ALT_M         2600 -> ~1950   (= 4200 / 2.20, what the approach needs
                                    at the gate itself)
```

Check in the log that exits still read `rolled out` and not `out of height`;
that is the failure mode this pair exists to avoid. Predicted: the handover
falls about 700 m, the wheels arrive about 1.8 km earlier, and the mean
`stopped along` moves from +3324 m to roughly +1500 -- on the tarmac, with
`HAC_EXIT_SURPLUS_M` (worth ~1275 m of ground at 500) the next thing to trim.

### The last four hundred metres

Four separate laws in this stretch were commanding steps, and every one of
them was destroying the vehicle on a runway it had already reached. They are
listed in failures 31, 34, 36, 37 and 39; what they have in common is worth
more than any of them:

- **The approach's S-turn set a bank magnitude and never a track.** The
  direction came from the centreline capture, which points at the
  centreline, so the vehicle banked to its limit, crossed, banked back, and
  flew a 30 m limit cycle worth 1.5% of extra path for a manoeuvre costed at
  41%. It commands a *track angle* now, `acos(straight path / path the
  height can pay for)` -- the cone's weave one phase later.
- **The capture asked for a closure the flare then had to inherit.**
  "Arrest it in the offset that is left" permits 9 m/s fourteen metres out;
  the flare flies wings level for eight seconds and turns that into 86 m of
  drift. It asks for the closure that puts the cross-track at zero *when the
  wheels arrive*, and the time it divides by is written against the flare's
  own trigger, because the flare spends its last two hundred metres
  arresting exactly the sink rate the estimate would otherwise use.
- **The flare's arrest demand divides by the height left**, so its last two
  ticks ask for maximum lift whatever the approach did -- nine landings
  touched down within a degree and a half of the same attitude and the first
  part lost was `Elevon 4` on every one. `FLARE_ARREST_FLOOR_M` stops the
  demand growing where it can no longer change the outcome, and
  `FLARE_TOUCHDOWN_ALPHA_DEG` says what attitude to arrive in -- yielding to
  a sink rate that is still there, because capping a vehicle twenty metres
  up and coming down at 30 m/s lands it at 21.
- **The rollout's derotation ramp never ran.** It was scheduled from 55 m/s
  when the vehicle touched down at 60-100; it now touches down at 41-51, so
  every landing commanded zero angle of attack on its first tick out of the
  flare's sixteen degrees. The schedule is a fraction of the stall speed now,
  which is what it actually depends on.

Result: cross-track at rest is **30 m on average**, 8 of the last 10
landings inside the 35 m strip, and the best of them kept 20 of 23 parts.
The lateral problem is closed; the longitudinal one above is not.

### The approach holds speed, not path

A glider cannot choose its speed and its flight path independently: pick an
angle of attack and the polar gives you both. The one that has to be chosen is
the **speed**, because arriving slow is unrecoverable.

The first version commanded angle of attack from the sink-rate error, which is
the law an aircraft with a throttle uses and is exactly backwards here:
arriving high it asked for *less* alpha, which unloads the wing, and the
vehicle traded its excess height for speed rather than for drag. Offline it
reached the threshold at **102 m/s with a 36 m/s sink rate**.

Height is the free variable and the surplus is spent with S-turns. This
airframe's window on final is narrow, so there is little steepening available
before the speed floor bites, and rolling the lift off the vertical is the
only other drag there is. There is no speedbrake.

**And the speed to choose is a fast one.** The instinct is to arrive slow, and
it is wrong here in a way that is worth stating as a rule: the load factor
available to arrest the sink goes as the **square** of the approach speed
(`n_max = (V/Vs)^2`) while the sink to be arrested goes only as `V`, so a
faster approach makes the flare strictly easier. What it costs is runway, and
runway is the one thing this landing has plenty of — stopping from 91 m/s in
the 1.8 km past the touchdown aim needs 2.3 m/s^2 against about 1 from
gear-down drag before the brakes are touched.

So **horizontal speed is the cheap axis and vertical speed is the one that
breaks the vehicle**, and the approach should give away the first to protect
the second rather than the reverse. At 1.60 x stall the flare had 2.56 g in
hand against 38 m/s of sink and stalled partway through it, touching down at
12.65 m/s on the tarmac 13 m off the centreline and breaking up
(`logs/LOG803`). At 1.90 there are 3.6 g. The brakes now come on at
touchdown rather than below 60 m/s, for the same reason: the rollout is where
the horizontal speed is *meant* to go.

### The drain is the largest aerodynamic change in the flight

2.780 t of 9.495, so 29% of the vehicle and 29% off the wing loading, which
moves the stall speed by 16% and every aerodynamic answer with it. It gets its
own phase, in vacuum, after the burn and before anything aerodynamic depends on
the answer: a prediction made *during* the drain is a prediction at a mass the
vehicle is not going to have.

kRPC reports a module's actions and fields by their **display** names, not the
names in the part config: this valve answers to `"Drain"`, not to
`StartResourceDrainAction`, and the field is `"Drain"` rather than
`isDraining`. The first flight tried the config names, logged `drain COULD NOT
START`, and flew the whole entry 2.8 t heavy. Measured: 9.495 t -> 6.715 t in
under 5 seconds.

### The session that put it on the runway (2026-09-18, later)

The session below got the vehicle down intact on the centreline and left the
along-track 330 m outside the window, with two open problems: the arrival
"cannot be aimed", and the configuration is eight fitted constants.  Both of
those descriptions turned out to be wrong in instructive ways.

**The landing miss was never a landing problem.**  Over the eighteen flights
below, the touchdown is `+118 m + 0.324 * arrival` with a residual of
**258 m**.  The miss is inherited from the arrival almost in full, so
centring the arrival lands the vehicle with nothing else changed -- and
nothing downstream of the entry needed to move.

**And the arrival bias is not made over the 2000 km entry.**  `long` holds
within tens of metres all the way down to 26 km and then loses 5.4 km below
it.  What loses it is a solve with no room: `alpha_ceiling` ratchets to about
18 degrees, `SOLVE_ALPHA_MIN_DEG` is 20, and `floor = min(floor, top)`
collapses the bracket to a **single point**, so the solve runs twice a second
through the whole decisive stretch and can return only the angle it was
handed -- thirty byte-identical ticks.  That is spaceplane failure 46, and it
is the *third* mechanism to freeze this command (25 and 26 were the others);
fixing 26 is what created it.

**Two principled fixes were flown and both are inert.**  Freeing the bracket
(`SOLVE_MAX_RANGE_ON`) measured 0.6 standard errors, because the range
optimum sits at the *top* of the bracket against the ceiling -- the vehicle
wants more angle of attack and the airframe will not hold it.  Making the
propagator honest about that ceiling (`HOLDABLE_EXTRAPOLATE`, written for
exactly this and never previously flown) measured 0.45 standard errors and
destroyed the vehicle five times in six.  **Both fail for the same reason:
they act below 26 km, and the deficit is committed above it.**

**What works is aiming long, early.**  `GLIDE_RESERVE_M` at 4000, held
constant to the gate rather than decayed, twelve flights rotated across three
instances:

| | along | sd | on the runway |
|---|---|---|---|
| off | -3.2 km | 1.3 | 0 of 6 |
| reserve 4 km | **-0.4 km** | **0.5** | 5 of 6 inside the window |

**5.2 standard errors**, replicated over twelve more flights at -0.2 and
-0.3 km.  The mechanism is visible in the cone's exit reason rather than only
in the average: every control flight leaves the cone *"out of height"* and
every reserve flight *"rolled out"*.  Starved, the cone falls through its
floor from wherever it happens to be; fed, it flies the rollout it was
designed around.  The reserve is open-loop and therefore early, which on this
problem beats being correct and late.

**The binding constraint is now cross-track, and it is an energy problem.**
Across seventeen reserve landings the lateral miss is **-23.8 m sd 14**
against a 35 m half-width, where the same configuration without the reserve
sits at +5.4 m.  It is not a lateral control failure:

```
corr(cross-track at flare entry, final cross-track)  -0.99
corr(flare entry speed, |final cross-track|)         +0.88
corr(arrival, flare entry speed)                     +0.77
```

The vehicle only arrives with the energy the rest of the chain expects when
it arrives about 4.5 km **short** -- because every constant downstream was
fitted while the entry was delivering that shortfall and was quietly
absorbing it.  That is failure 47, and it is CLAUDE.md's rule at the scale of
a whole phase chain: *a quantity measured from flight data describes the
vehicle as it was flown.*

**What is still open, and it is now specific.**  `GLIDE_RESERVE_M` is a
calibration standing in for a missing model, and the model is named: with no
evidence at a dynamic pressure, `Holdable.limit` returns `None` -- *no limit*
-- so the propagator assumes the airframe holds whatever it is commanded in
air it has never met.  The default is optimism and optimism is the
unrecoverable direction.  The fix is a **pessimistic prior that relaxes
toward measurement**, computable in STANDBY from things the vehicle already
takes: the swept table's normal force against angle and Mach, and the
reaction-wheel and RCS torque `dump_torque` logs.  The ceiling is where the
aerodynamic moment overcomes the torque; the one unknown is the effective
moment arm, measured once at about 0.55 m and boundable from the vessel's own
geometry.

**A harness fact that outranks all of the above.**  The control loop's rate
sets the entry scatter, and it moves with machine load: the same
configuration measured 2.13 game-seconds per tick (spread 0.06) in the
morning and 2.56 (spread 0.19) with four instances up later the same day, and
the along-track scatter went from sd 0.3 km to sd 2.7 km with survival from
16/18 to 2/5.  **Three instances at `--timescale 5` restores it.**  Compare
nothing across that boundary; `logsum`'s `dt` is the check.

### The session that made it land (2026-09-18)

A new airframe (wider gear track, mains aft, low wing, flatter ground
attitude) and four fixes turned a vehicle that never survived into one that
survives 8 flights in 9 and lands on the centreline every time.

| | stopped along | lateral | intact |
|---|---|---|---|
| before | -12706 m, sd 715 | km-scale | 0 of 9 |
| after | **-1528 m, sd 299** | **4 m mean, 9 of 9 on the strip** | **8 of 9** |

The runway accepts +/-1200 m along and +/-35 m across, so **lateral is
solved** and the along-track is about 330 m outside the window.

**The four fixes, in order of how much they were worth:**

1. The deorbit counted a model of its burn and shut down a third early
   (failure 44). Worth ~250 km.
2. The throttle floor over-spent the last tick (failure 45). Arrival scatter
   sd 1592 -> 565 m.
3. `ENTRY_ALPHA_DEG` 30 -> 22. The entry was braking at "maximum drag with
   lift" and could not reach the field; the range curve has an interior
   optimum on a **plateau at 20-22** (20 and 22 differ by 153 m against sd
   530, and 24 costs 1.6 km). Worth ~11 km.
4. `APPROACH_BEST_LD` 1.85 -> 4.2 with `GATE_ALT_M` 2600 -> 2000, moved
   together per failure 42. Worth 1.1 km of bias -- and, far more than that,
   **survival went 2 of 9 to 7 of 9**: an approach that believes it cannot
   glide holds height it does not need and arrives steep.

**Three suspects that were raised and cleared**, each by one batch, and worth
not re-raising: the aero table (`LiftTrim` measures 0.98-1.00 in every Mach
bin over thousands of samples -- `aeroaudit`'s lift column is a residual of
two cancelling 9 m/s^2 terms and cannot be read high in the entry); the
`Holdable` alpha ceiling (-16840 against -17013 with it off); and the bank
floor (`SOLVE_BANK_MIN_DEG` 10 is 1.8 km *worse* than 30, so the documented
non-monotone range-vs-bank curve still holds on this airframe).

**What is left, and it is not tuning.** The entry's arrival cannot be aimed:
`DEORBIT_LONG_BIAS_M` reaches only the burn's stop test (failure 33) and
`DEORBIT_CENTRE_BIAS_M` is inert at -2500 and at -17000. The arrival is
wherever the propagator's ~1% optimism over a 2000 km entry puts it, and the
last 330 m is currently bought by dragging `TOUCHDOWN_AIM_M` to the far
threshold -- a geometry anchor used as a bias, with a gain that has fallen
from 0.5 to 0.17, which is saturation, not a fix. `log_deorbit_window` now
prints the five numbers the search was offered, so one flight can say whether
those knobs reach the decision at all.

**And none of it transfers yet.** The configuration below is eight fitted
constants against one airframe and one save. `HAC_LD`, `APPROACH_BEST_LD` and
the entry angle are all quantities the vehicle measures on itself every
flight -- the flown cone ratio, the rollout-to-wheels ratio, the swept
table's range curve. Until they are computed rather than transcribed, this is
a demonstration and not a result.

```
CROSS_DEADBAND_PER_KM=20  CROSS_DEADBAND_MAX_M=40000  HAC_LD=1.86
ENTRY_ALPHA_DEG=22  APPROACH_BEST_LD=4.2  GATE_ALT_M=2000
TOUCHDOWN_AIM_M=2400
```

### High alpha: what the airframe gives and what it will hold (LOG1615)

One flight, `BROADSIDE_PROBE_DEG=90`: deorbit as normal, then command 90
degrees from COAST down and let `Holdable` watch what comes back. It landed
562 km short and lost the vehicle, which is what an instrument flight looks
like -- the deliverable is the curve, not the arrival.

**What broadside is worth.** From the swept table at entry Mach (4-8):

| alpha | Cd*A | Cl*A | L/D |
|---|---|---|---|
| 30 (what the entry flies) | 8.5 | 10.0 | 1.18 |
| 65 | 27.7 | 4.1 | 0.15 |
| 90 | 28.2 | **0.0** | 0 |

Broadside is **3.3x the drag** of the current entry attitude, so the "slow down
while still high" intuition is right and larger than it sounds. But `Cl*A` at
90 degrees measures **exactly zero at every Mach in the table**, so there is
nothing to roll: bank steering needs a force across the flow and at true
broadside there is none. 65 degrees keeps 98% of the drag and leaves L/D 0.15,
which is Apollo's regime and is where this idea actually lives.

**What it will hold**, commanded 90 the whole way down, best angle achieved per
250 Pa of dynamic pressure:

```
   0-500 Pa   90 deg      1500 Pa   44        4000 Pa   26       8000 Pa   19
     750 Pa   71          2000 Pa   36        5000 Pa   22      10000 Pa   14
    1000 Pa   48          3000 Pa   29        6500 Pa   20
```

The vehicle holds full broadside until about 500 Pa and gives it up fast after
that; by 4-5 kPa it is back at the angle the entry already flies. So high alpha
is available **above roughly 45 km** and nowhere below it.

**Why it lets go, in numbers.** kRPC reports no pitching moment, but it does
report torque, and `dump_torque` now logs it: reaction wheels **15 kN m**, and
RCS **37.5 kN m in pitch and yaw** -- 2.5x the wheels, which is the opposite of
what this project assumed when it decided four blocks could not matter. At the
1 kPa breakaway the resultant aerodynamic force at the angle being held is
about 26 kN, so the wheels are losing to an effective moment arm of roughly
**0.55 m**. The arm is tiny; the force is not, which is why a small
centre-of-pressure offset beats the wheels so early.

**The curve above is the wheels-only curve** -- that flight flew with RCS shut
through the entry. The prediction from the torque numbers was that thrusters
would carry the same angles to about 3.5 kPa, roughly 10 km lower, and the
propellant was the doubtful half.

**It was measured, and the prediction was wrong in both halves (LOG1616).**
Commanded 65 with `GLIDE_RCS`, `COAST_RCS` and `RCS_Q_MAX_PA=50000`:

| | 65 deg holds to | altitude | monopropellant |
|---|---|---|---|
| wheels only (LOG1615) | ~900 Pa | 45 km | untouched |
| wheels + RCS (LOG1616) | ~1400 Pa | 42 km | **the entire tank** |

The torque bought a factor of **1.5 in dynamic pressure, not 3.5**, and three
kilometres of altitude. `available_rcs_torque` is a peak figure with every
thruster pulling the right way; against a *steady* aerodynamic moment kRPC's
attitude controller does not deliver it, which is the same lesson as
`ALPHA_TRACKING` -- a quantity measured one way describes the vehicle flown
that way.

And the propellant went exactly as feared: 150 units at 50 km, 125 by 45 km,
107 when 65 degrees was lost at 40.5 km, 50 by 29 km, **empty at 25.2 km**.
The whole tank, for three kilometres.

There is a free consistency check in the tail of that flight: once the tank was
dry the held angle fell back onto LOG1615's wheels-only curve (22-25 degrees at
5-8 kPa, against 22.3 at 5 kPa measured the other flight). Two flights, two
configurations, one curve -- the ceiling is a property of the airframe and not
of how it was asked.

**So high alpha is closed as an entry mode.** Not because it makes no drag --
it makes 3.3x -- but because the airframe will not hold it anywhere the drag
is worth having, and the one actuator that could argue buys 3 km for its
entire propellant load. What remains open is the much smaller claim: a
terminal energy dump inside the cone, at the low dynamic pressures where the
angle is free.

### The swept table is written into the log

`AERO_DUMP` prints the whole `(alpha, Mach)` table once, in STANDBY: one line
per Mach row, `alpha:ClA/CdA`, with the altitude the row was probed at,
followed by a summary naming where the drag peaks and what 30, 65 and 90
degrees are each worth. Then `dump_torque` prints what the vehicle has to hold
an attitude *with* -- kRPC reports no aerodynamic pitching moment at any price,
but it does report available reaction wheel and RCS torque, and "four blocks
are not enough" is an assertion until that number is in the log beside the
dynamic pressure the vehicle fell short at.

The bins have always run out to 90 degrees. Every flight of this project has
measured the broadside cell and no log has ever shown it, so the high-alpha
entry question was unanswerable from data that already contained the answer --
failure 13's shape exactly. An unprobed cell prints `-`, never 0, because an
airframe that makes no drag at 90 degrees is a plausible-looking lie.

## The telemetry line carries the model's own report card

Beside the model's `cla=`/`cda=` at the commanded angle, every atmospheric
line carries what the vehicle actually did:

```
act=  9.1/ 9.2   Cl*A / Cd*A the vehicle achieved, from flight.aerodynamic_force
mdl=  9.3/ 9.2   the table, read at the angle the vehicle is ACHIEVING
ld= 1.01/ 0.98   model L/D and actual L/D, the number that sets the range
rho=1.000        the game's air density over the table's
```

Four columns that between them retire a whole class of argument: if `rho=`
is 1.000 the atmosphere model is not the problem, if `ld=` reads the same
twice the aero table is not the problem, and if `act=` and `mdl=` agree while
`aoa=` shows a five-degree gap then the problem is the *attitude* and nothing
aerodynamic. That is exactly how it came out — see failure 10b.

`aoa=` remains `commanded/achieved`, and `mdl=` is deliberately read at the
achieved angle: comparing a measured force against the table entry for an
attitude the vehicle is not in answers a different question, which is what
`cla=`/`cda=` had quietly been doing.

`aeroaudit.py` still works and is still useful on old logs, but it
reconstructs forces from finite differences and carries a Coriolis term worth
10-30% of the drag. Prefer these columns where a log has them.

`slip=` is the game's own sideslip angle, and it is separate from `aoa=` on
purpose. `aoa=` is nose-to-velocity, which is the *total* angle and hides the
slip inside itself -- 20 degrees of alpha with 10 of slip reads identically
to 22.4 with none, so for the whole project a vehicle flying 25 degrees
sideways would have logged as slightly nose-high. It is under a degree
through the hypersonic reversals and reaches 25 below 34 km. See failure 12.

`n=` is the parts still attached, and the shutdown line carries the sink rate
and speed of the arrival. Before those existed, "on the ground in FLARE"
covered both a 37 m/s touchdown with the vehicle intact and a 28 m/s-sink
impact into terrain.

## Phases end, even when the arrival does not

Three exits exist because a flight that goes wrong used to cost an hour of a
test instance rather than a minute:

- **`grounded_early`.** Only ROLLOUT and STOPPED expect to be on the ground,
  and only APPROACH and FLARE can reach them. An entry that falls short lands
  *during GLIDE*, which has no ground test at all because the arrival it
  steers at is hundreds of kilometres away — three flights in one batch sat
  at `h=-0.2` logging GLIDE lines at 63 m/s until the harness gave up.
- **A backstop for when the game will not say "landed".** A vehicle on a
  slope can skid with `situation` still reading *flying* and negative height
  under its wheels. Height below ground for `GROUNDED_STUCK_S` is the same
  fact arrived at independently — held for a while, not taken on one tick,
  because a terrain sensor reading below ground for an instant is a bounce
  and boosterland lost five flights to trusting that sensor.
- **`ROLLOUT_TIMEOUT_S`.** The stop test is a speed threshold, and a vehicle
  that arrived 50 km short is not on a runway: it is wedged in terrain, or
  destroyed with kRPC reporting 0.00 t and a frozen velocity forever.

None of these is cosmetic. The autopilot's contract is to hand back rather
than keep commanding a vehicle it cannot help, and in a project whose
bottleneck is in-game measurement, a phase that can wedge is a phase that
needs a clock.

## Testing without KSP

**`spaceplane/tests/fakeplane.py` replays `planeprobe`'s table, and that table is wrong
subsonically by about 1.8x** (failure 13). Everything below still holds — the
step convergence, the self-consistency, the sign checks — because those are
properties of the *scheme* and not of the coefficients. What the sim cannot
be used for is any statement about the airframe: best glide, stall speed,
where the gate goes. Those come from `./spaceplane/tools/polar.py`, which reads them back off
the flown logs.

`spaceplane/tests/fakeplane.py` is not a physics model of a spaceplane — it is a *replay
of what the game actually returned*, wrapped in the interface the propagator
and guidance expect, using the same `environment.Table` class the flight code
reads. Its density profile interpolates the densities the probe reported; an
exponential fit was tried first and is 27% low at 10 km and 80% high at 45 km,
which for a term going as `rho v^2` through the part of the entry that does all
the braking is not a detail.

**`spaceplane/airframe.py` is the runtime half of the same idea.** It reads
the stall speed and the best glide ratio off the table the flight sweeps for
itself, logs them beside the configured constants, and says so when they
disagree by more than `AIRFRAME_DISAGREE_FRACTION`. It does not adopt them —
a derived number is a hypothesis until the game agrees — and on the first
flight it ran it immediately earned its keep, reporting `APPROACH_BEST_LD is
2.05 and the swept table says 3.06 (49% out)`. Which of those is right is an
open question; that the question is now asked on every flight is the point.

`spaceplane/tests/glidesim.py` flies the real guidance against it end to end. What it can
show is that the guidance converges, that the signs are right, that the phases
hand over sensibly. What it **cannot** show is whether the vehicle lands — it
has no attitude dynamics, so nothing can fail to hold a commanded angle, and
the sibling project has a whole section on sim results that did not survive
contact with the game.

Two properties are asserted rather than any particular scheme:

- **The step has stopped mattering.** An offline entry propagated at 4.0, 2.0,
  0.5 and 0.25 seconds a step agrees on the range to within 100 m of 1703 km.
  boosterland failure 16 is what this test exists for.
- **The prediction is self-consistent along its own path.** Re-asked from a
  point on its own trajectory it must give the same answer; measured, it moves
  12 m over 1400 s of entry. A propagation that changes its mind when re-asked
  is a discretisation and nothing else can do that.

## Entry states

`savegen.py` works on this vessel unchanged — it edits only the `ORBIT` block,
applying a delta-v at the position the vessel already occupies, so `lat`,
`lon`, `alt` and `nrm` stay correct.

```bash
./tools/savegen.py --source qs_plane -o qs_plane_inc  --normal   100   # inclination
./tools/savegen.py --source qs_plane -o qs_plane_high --prograde  80   # elliptical
./tools/savegen.py --source qs_plane -o qs_plane_ecc  --radial    60   # steeper
./tools/savegen.py --source qs_plane -o qs_plane_low  --prograde -40   # lower
```

The stock save's orbit is essentially equatorial (`INC = 0.097`), so the
inclined save is the one that exercises cross-track authority and the
runway-end choice at all.

**Inclination is much less of a problem on Kerbin than it looks.** The planet
turns once every 6 h and an 80 km orbit takes 31 min, so every equator crossing
lands about 16 degrees (164 km) west of the last, and the crossing longitudes
sweep past KSC continuously. Against a crossrange authority worth tens of
degrees on a 3770 km circumference, essentially every pass is reachable. The
case that produces a long wait is a *resonance* between the orbital and
rotation periods, and a few m/s of phasing burn breaks it; a plane change at
LKO is ~400 m/s for 10 degrees and is the wrong tool.

### The valve fires after the loop stops looking

`cutoff drift` is one line per flight and it settled an open question that a
dozen mechanisms had failed to settle from the other end.  It records the
vehicle's specific energy at the `DEORBIT -> DRAIN` tick and again at the
first COAST tick -- two states the control loop already samples, no
propagation, nothing steering on it -- and reports the difference as an
equivalent dv.

In vacuum, with the engine cold, the vehicle loses **3.1-3.5 m/s on
`qs_plane` and 9.0-9.4 m/s on `qs_plane_inc`**, to a spread under 0.2 m/s
inside each state.  `DRAIN=False` takes both to 0.3.  It is the release
valve, and it fires *after* the burn's closed loop has stopped looking.

That matters out of all proportion to its size, and the reason is
architectural rather than aerodynamic.  The deorbit's stop test is the one
part of this program with no model in it at all: it propagates from where the
vehicle *is* and stops when that arc is right, so it is immune to every
modelling error and to exactly one thing -- an impulse delivered after its
last sample.  The valve is that impulse, its magnitude is set by the
propellant the vehicle happens to be carrying, and at the entry's ~12 km of
arrival per m/s the 6 m/s between these two vehicles is ~70 km on the ground.
Every "the propagator is optimistic" investigation in this file was looking
at the shadow of it.

`DRAIN_BEFORE_BURN` moves the valve upstream of the loop: it runs before the
deorbit, stops at `DRAIN_RESERVE_UNITS`, and the burn is then solved and
flown at the drained mass with nothing left to disturb it afterwards.  The
reserve stays aboard -- 40 units is 200 kg, 3% of a 6.7 t entry against the
29% the drain exists to shed.  Measured, the drift falls to **-0.45 and
-0.66 m/s**.

**The valve is faster than the control loop, and that is a general trap.**
It empties 556 units in 2.0 game-seconds; this loop cannot tick faster than
about two of those.  The first implementation opened the valve and came back
on its next tick to a dry tank, kept no reserve at all, and flew a deorbit
that died on `burn guard at 60 s` with no propellant (LOG2118, LOG2122).
kRPC *reads* are milliseconds, so the phase now holds the valve open and
watches the tank directly, closing it on the reserve rather than waiting for
its own next tick.  A control loop's tick rate is the resolution of
everything it closes a loop on.


### A tick in the log is not a tick of the loop

The telemetry lines of the FLARE sit **2.02 game-seconds apart**, which looks
exactly like a control loop too coarse to fly a six-second manoeuvre -- and
it is not one.  `Logbook.telemetry` gates on `LOG_INTERVAL_UT`, which is
**2.0**, so the spacing measured is the logging cadence and nothing else.
The same 2.02 appears in the HAC at `--timescale 1.5`, where the loop has
four times the wall clock per game-second; that is the tell.

Filed because the misreading was a whole hypothesis: "the flare is flown at
two-second ticks and the farm's timescale is what destroys the landings" is
a good theory, it fits the wrecks, and it was built on the log's own
cadence.  To measure the loop, set `LOG_INTERVAL_UT` below the tick you are
asking for, or count something the loop itself writes.

### Where it stands after the valve

Three entry states, one configuration, `--timescale 6`, the arrival being the
predicted along-track miss at `GLIDE -> HAC`:

| | before | after |
|---|---|---|
| `qs_plane` | -1.1 km sd 0.4 | **+0.2 sd 1.6** (n=6) |
| `qs_plane_high` | -17.3 sd 1.4, 0 of 6 ever landed | **+2.8 sd 1.7** (n=8) |
| `qs_plane_inc` | -56.0 sd 6.0, 8 of 8 destroyed short | **-9.8 sd 2.8** (n=10) |

Spread across the three: **74 km -> 12.6 km**, with no constant re-fitted --
the changes are `DRAIN_BEFORE_BURN` (failure 61) and sizing its reserve from
a dv budget (failure 64).  `qs_plane_high` had never once reached the field
and is now within three kilometres of the state everything was fitted to.

**The binding constraint has moved off the entry.**  Across those twenty-four
flights every one reached the field, five of the six that stopped were
inside the +/-1200 m along-track window and five of six inside the 35 m
strip -- and **three kept their parts**.  The wrecks are all
`destroyed in ROLLOUT`, at touchdown speeds of 35-48 m/s against a 48 m/s
stall, which is the flare arriving below the speed its capped angle of attack
can hold: at 15.9 degrees and 6.6 t the vehicle needs **50 m/s** to fly and
it touches down at 41-48.  That is one well-posed problem
(`FLARE_LEAD_S`/`APPROACH_SPEED_PROFILE` decide how long it floats) and it is
the next thing to work on, not the entry.

What is left on the entry is `qs_plane_inc`'s remaining 9-11 km.
`DEORBIT_CENTRE_BIAS_M` does reach the aim in game -- -8000 moved it +4.7 km,
a gain of about 0.6 -- but it moves every state together, and `qs_plane` and
`qs_plane_high` are already where they should be.

### The farm was measuring its own tick latency, in one phase

Two hundred flights of this project were flown at `--timescale 6`, and the
control loop cannot always serve that: a tick costs wall-clock time whatever
the game clock is doing, so at 6x a phase asking for a command every 0.1
game-second gets one every `6 x tick cost`.

The fix is a change of which quantity is held constant.  **The time scale is
the free variable and the control interval is the constraint.**  Ask for a
fixed interval in game seconds, measure what one tick costs in wall seconds,
and the fastest honest scale is the ratio.  That is
`common.pacing.ScaleGovernor`, pointed at the plugin's control file by
`Config.TIMESCALE_GOVERNOR`; `quickglide --timescale` becomes a ceiling
rather than a setting, and `--no-govern` restores the old meaning for
measuring against.  It costs nothing: 5.0-6.0x is still achieved.

**Measured against itself** -- one save, the committed defaults, the same
farm in the same state, the only difference being `--no-govern`:

| | ungoverned 6x (n=6) | governed (n=18) |
|---|---|---|
| `DEORBIT` tick | **0.30 game-s** | **0.10 game-s** |
| `HAC` / `APPROACH` / `FLARE` tick | 0.11 | 0.11 |
| arrival at `GLIDE -> HAC` | **-3226 m, sd 806** | **+294 sd 171, +409 sd 100** |

**Every phase but the burn was already being served**, and that is the useful
half of the result.  A tick on final costs 3-6 ms, so even at 6x the approach
gets the 0.1 game-second interval it asks for; the burn's tick costs 44-62 ms
because it propagates, and 6x turns that into 0.3 game-seconds of a throttle
taper it is trying to close a loop on.  Three and a half kilometres of
arrival bias and five to eight times the scatter were being made there, and
at 1x -- where anybody actually flies this -- neither was ever present.

The remaining +300 to +400 m is not an error: `GLIDE_RESERVE_M` is 500 and
the glide is hitting its commanded arrival.  (`DEORBIT_CENTRE_BIAS_M=6700`
moved the burn's aim 7.3 km and the arrival by 81 m, for the same reason: the
knob is connected -- the log prints the aim it changed -- but it is upstream
of a phase that absorbs it.)

**Every log now says which controller flew it.**  `LoopRate.report` writes one
line at shutdown:

```
loop rate, game-s per tick / wall-s of work: DEORBIT 0.10/0.062 n=97
  COAST 2.01/0.016 n=144  GLIDE 1.01/0.029 n=568  HAC 0.11/0.003 n=1508
  APPROACH 0.12/0.003 n=437  FLARE 0.11/0.002 n=99  ROLLOUT 0.12/0.002 n=28
```

Read it before comparing two logs, and read it *instead of* subtracting log
timestamps: the telemetry cadence is `LOG_INTERVAL_UT` and has never been
anything else.  Failure 63 recorded that trap and failure 65 fell into it
anyway.

### The configuration that lands

It is the committed default now, so there is no list of `--set` flags to copy
and nothing to get wrong:

```bash
.venv/bin/python -m spaceplane.autopilot          # waits for START
./spaceplane/tools/quickglide.py -n 3 --instance 0 --timescale 6   # or fly the quicksave
```

Nine flights of it on `qs_plane`, no overrides at all, and nine more of the
identical configuration passed as flags the batch before (`armN`):

| | armN (flags) | default (no flags) |
|---|---|---|
| on the runway (\|along\| <= 1200 m) | 9 of 9 | 5 of 6 |
| inside the strip (\|across\| <= 35 m) | 9 of 9 | 6 of 6 |
| kept 18+ parts | 8 of 9 | 6 of 9 |
| stopped along | +485 sd 444 | +897 sd 420 |
| across, \|mean\| / max | 10 / 20 | 11 / 20 |
| arrival at `GLIDE -> HAC` | +409 sd 100 | +294 sd 171 |

Pooled: **14 of 18 intact, 15 of 15 inside the strip, 14 of 15 on the
runway.**  The historical figure for this save is 12 of 24, and the
immediately preceding batch of the old configuration was 2 of 16.

**Where the four wrecks go.**  All four are `destroyed in ROLLOUT` and the
first parts lost name two different things: a `Structural Wing Type A` and an
elevon is the wingtip (failures 70 and 71, and it is cross-track at the
flare's door that predicts it), while an `LV-T91` and an elevon is the tail --
the flare ending nose-high at 47 m/s and touching the engine bell down first.
The second one is the open item.  `FLARE_TOUCHDOWN_ALPHA_DEG` and the tail
cap are where it lives.

**What changed to get here**, all of it laws rather than fitted constants
except the last two:

| | |
|---|---|
| `TIMESCALE_GOVERNOR` | the control interval is the constraint and the time scale is the free variable (failure 65) |
| `APPROACH_SPEED_PATH` | the speed loop is two-sided: command the descent that holds the speed, fly it with load, against the descent the vehicle has (66, 67) |
| `FLARE_SINK_TRACK` | the flare tracks the sink it can still arrest, instead of arresting all of it wherever it happens to succeed (66) |
| `AIM_RUNWAY_TRUE_ALPHA` | pitch to `alpha + descent`, because the guidance computes an angle of attack and this was flying it as an attitude (69) |
| `GLIDE_RESERVE_M` 4000 -> 500 | a margin is a fitted constant with a standard deviation baked into it, and the deviation moved (68) |
| `HAC_ROLLOUT_M` 1500 -> 900 | the cone's roll-out distance is what sets the cross-track the flare inherits (71) |

### And on the other two entry states, with the same defaults

Six flights each, no overrides, immediately after the batches above.

> **Superseded for `qs_plane_inc`, by 35 flights against these six.** What
> follows reads the arrival as solved and the touchdown as the open item.
> It is the other way round. The along-track miss at `GLIDE -> HAC` is
> **bimodal** -- `+86 .. +1005` against `-3664 .. -20562` -- and half its
> variance is the bank reversal count, at **+2137 m per reversal,
> r^2 = 0.49**, monotone from six reversals to eleven. The "6 of 6 reach the
> runway" below is six draws that happened to fall in the good mode; the
> short mode puts the vehicle in the sea six to twenty kilometres out.
>
> The touchdown problem is real and is still open -- a flight arriving +844
> was destroyed in ROLLOUT -- but it **cannot be measured from an orbital
> save**, because the arrival scatter swamps any landing change. Two batches
> were flown and stopped for exactly that. See failures 77-78, and use
> `entrysave.py` with `ENGAGE_INTO_LANDING` to fly the landing from a
> repeatable state.

**`qs_plane_inc` -- the state that had never once reached the field.**
Historically 0 of 15, every one destroyed 60-90 km short.  Now **6 of 6 reach
the runway**: along +255, +338, +938, +1329, +1600 and -888; across -26 to
+30, *every one inside the 35 m strip*; flare entries 89-101 m/s with 32-37
of sink, touchdowns 48-53 m/s.  The arrival is -402 m sd 2237 with nothing
tuned for this save.

And **1 of 6 keeps its parts.**  The approach and the flare are doing their
job on this state -- better than on `qs_plane`, if anything, since the
touchdown speeds are higher -- and the vehicle comes apart on contact,
shedding a mixture of elevons, a wing, a mid-body fuel tank and the engine.
That is a *touchdown* problem and not a guidance one, and it is the open
item: the same chain that survives 14 of 18 arrivals on one save survives 1
of 6 on another while flying them to the same place at the same speed.

**`qs_plane_high` -- still broken, and differently.**  The entry is long by
**+2460 and +6577 m** at `GLIDE -> HAC` where the reserve asks for +500, the
cone leaves *out of height* 4.5-6.1 km from the gate, and the approach has
nothing left to do: three flights stop 4.1 km long in the water and three
never reach the field at all, 3.5-4.3 km across.  The burn itself is clean
(`range error +0 m`, 120.8 of a solved 121 m/s delivered, cutoff drift
-0.00 to -0.10 m/s), so the miss is made between the burn and the interface
on this state -- which is exactly the open question failures 57-60 left, now
with the loop's tick latency subtracted from it and a 100 m-scale arrival on
the other two saves to measure against.

So: **the bar is most spaceplanes, not this one**, and this is one of three
saves landing well, one landing on the runway and breaking, and one not
arriving.  A craft file of its own is the test after that.
