# The spaceplane (`spaceplane/`)

The second autopilot: deorbit, entry, and a runway landing. Its failure modes
are in [spaceplane-failures.md](spaceplane-failures.md).

A second autopilot in the same tree: deorbit from orbit, fly an atmospheric
entry, and land a winged vehicle on the KSC runway. `boosterland/` is untouched
by it and its 152 tests still pass; what the two share is `boosterland.vec` and
`boosterland.logbook`, the `test_instances/` farm, and every convention in
`CLAUDE.md`.

```bash
.venv/bin/python -m spaceplane.autopilot            # fly it, waiting for START
./quickglide.py -n 3 --instance 0                   # fly the quicksave, in game
./quickglide.py -n 3 --instance 0 --timescale 4.0   # ... about 3.5x faster
./quickglide.py -n 1 --save qs_plane_inc --set GATE_ALT_M=1600
./glidesum.py --band 33000 26000 logs/LOG7*         # reversals, per flight
OUT=/tmp/base.txt N=8 ./farmfly.sh                   # one config, n per instance
./landsum.py --arrivals-within 5000 logs/LOG16*     # arrival -> handover -> wheels
./conesum.py --settled logs/LOG16*                  # what HAC_LD should be
./armsum.py --by GLIDE_RESERVE_ON --against '(default)' logs/LOG10*
python3 -m unittest tests.test_spaceplane           # offline, no KSP
python3 -m tests.glidesim --dv 60                   # one entry, traced
./test_instances/planeprobe.py 0 --mass 6.715       # measure the airframe
```

An entry is ten minutes of game time and the orbital wait is more, so
`--timescale` is most of what makes a repeatability question affordable here:
it paces the control loop on `ut` (`LOOP_PACING_GAME_TIME`) and prints what
the instance *achieved* rather than what it was asked for. Read
`docs/test-instances.md` before trusting a multiplier.

**Where it is** (2026-09-17, from `qs_plane` on a freshly restarted farm).
Read this with `landsum.py`, which prints all of it. **The two rows are two
different configurations and the difference between them is not committed:**

```
                              arrival mean   sd      inside 5 km
committed defaults   n= 7      -1.5 km     17.0 km     1 of 7
+ CROSS_DEADBAND arm n=30      +2.8 km     12.8 km    17 of 30   (--set, not
                                                                  committed)

both configurations, of the flights that then landed:
landing bias (stopped-arrival) mean +1.75 km  sd 0.80 km
cross-track at rest            |mean|  30 m   8 of 10 inside the 35 m strip
best landing                   20 of 23 parts, stopped +917 m along, -40 across
```

The baseline row is n=7 and the arm is one arm; neither is a measurement by
this project's own standard (failure 27). What the arm is good for is saying
where to point the next proper batch.

**The lateral problem is closed and the longitudinal one is not.** That is
the reverse of what this file said for several sessions, and the reversal is
the cone's doing: it picks which way to turn from where the vehicle actually
is, so it absorbs kilometres of cross-track and delivers tens of metres.

**Judge the entry and the landing separately.** They fail independently and
each hides the other:

- the **entry** on the committed defaults delivers an arrival inside 5 km
  about **one flight in seven**, and nothing downstream can rescue the rest;
  the uncommitted deadband arm made that roughly one in two;
- the **landing chain** adds **+1.75 km** of its own on top of whatever the
  entry did, against a runway that accepts +/-1200 m -- so even a perfect
  arrival runs off the far end today.

Both have a specified next step and neither needs new invention.

**What is left, in the order the evidence points:**

- **The landing bias is one constant measured over the wrong span**, and the
  fix is a *pair* of numbers that have to move together: `APPROACH_BEST_LD`
  1.85 -> ~2.20 and `GATE_ALT_M` 2600 -> ~1950. See "The landing chain, and
  the one number that sizes all of it" below for the arithmetic, the
  predicted result, and the failure mode to watch for (`out of height`
  exits). This is the cheapest remaining kilometre and a half.
- **The entry's scatter is made by a relay.** The glide spends **87% of its
  time with the bank off its stops** -- 24 reversals at 17.5 s per
  stop-to-stop slew -- and entry range is non-monotone in bank, so every
  reversal crosses the branch `SOLVE_BANK_MIN_DEG` exists to avoid. Raising
  `BANK_RATE_DEG_S` is not the lever (the relay just cycles faster: 33
  reversals, no change in arrival). Widening the cross-track deadband is:
  flown at `CROSS_DEADBAND_PER_KM=4, CROSS_DEADBAND_MAX_M=6000`, arrivals
  inside 5 km went **1 of 7 -> 17 of 30**, with the arrival cross-track
  *unharmed* (-138, -35, +16 m on the first three). The knob's original
  rejection was paid for entirely in cross-track, which the cone has since
  made free. **It is not committed**: the 30-flight arm is one arm, the
  baseline it beats is n=7, and the cap wants a sweep (3 km moved the
  reversal count only 24 -> 21; 6 km moved it to 17).
- **The deorbit still sometimes takes a different opportunity**, 2133-2186 km
  to run instead of 2244-2254, and those flights arrive **+36 km** on
  average against +5.3 for the rest -- 13 of 41 flights. A hard floor
  (`DEORBIT_COMMIT_ARC_MIN_M`) is implemented and left at 0: as a veto it
  makes the phase *wait*, and what it waits for may be badly out of plane.
  It wants to be a preference among the opportunities available at one
  moment, not a veto that spends them.

**Before trusting any of the numbers above, read
[test-instances.md](test-instances.md) on the farm.** Three separate faults
were silently corrupting measurements here until 2026-09-17 -- orphaned
autopilots surviving a killed batch, per-instance quicksaves drifting apart,
and a farm left up long enough to swap -- and the last of those biased
arrivals **27 km** with the spread unchanged, which reads exactly like a
guidance regression. All three are fixed or checkable in one line; the
checks are worth running before a batch and after it.

## Start here

If you are picking this up cold, in this order:

1. **Check the farm before anything else** -- three one-line checks in
   [test-instances.md](test-instances.md). Until 2026-09-17 all three were
   silently wrong at various points and a whole session's comparisons went
   with them. `farmfly.sh` refuses to start if the first one fails.
2. **Run `./landsum.py logs/LOG16*`** on the newest batch. It prints the two
   numbers that matter -- the *arrival* (the entry's doing) and the *landing
   bias* (everything after it) -- and they have to be read separately.
3. **Take the landing bias first.** It is the cheaper of the two, it is
   fully diagnosed, and the fix is two constants moved together:
   `APPROACH_BEST_LD` 1.85 -> ~2.20 with `GATE_ALT_M` 2600 -> ~1950. The
   arithmetic, the predicted result and the failure mode to watch for are
   under "The landing chain, and the one number that sizes all of it".
4. **Then the entry.** `CROSS_DEADBAND_PER_KM` looks like the lever and is
   not committed; it needs a cap sweep against a proper baseline, not
   another single arm. See "What is left" above and failure 41.

What *not* to spend time on, because it has been measured and written down:
cross-track anywhere (solved by the cone), `BANK_RATE_DEG_S` (connected, not
the lever), a minimum-width requirement on the deorbit window (backwards --
the wider windows are the worse passes), and `DEORBIT_COMMIT_ARC_MIN_M` as a
veto (it makes the phase wait for something worse).

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

`test_instances/planeprobe.py` measures the rest against the real craft with no
flight at all (`planeprobe-gearup.txt`) — **and its subsonic half is wrong by
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

The polar the airframe actually flies, from `./polar.py` — `act=ClA/CdA` off
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

Probed live on the craft (`test_instances/ctrlsrf.py`, `surfacespan.py`):
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

`boosterland/rcs.py` is shared by both craft. A phase says where RCS is
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

`tests/fakeksp` cannot check any of this from a flight: its `control.rcs` is a
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

The *truth* model in `tests/glidesim.py` sets `reversing=False`, because the
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
./entrysave.py 0                      # alongside a normal flight, saves qs_entry
cp test_instances/ksp0/saves/default/qs_entry.{sfs,loadmeta} /tmp/
./savegen.py --save-dir /tmp --source qs_entry -o qs_hot --prograde 24
./ladder.py --arms qs_b0,qs_b24,qs_b32 -n 4 --instances 0,1,4,5
```

**The open test is the deorbit's mapping onto the plateau, and it is the
expensive one.** The ladder sizes the glide's capacity from a fixed interface
state; a deorbit that under-burns by N m/s does not reproduce it, because it
also crosses 58 km somewhere else. So the question "what value of
`DEORBIT_WINDOW_BIAS` puts the delivered state in the middle of the plateau?"
needs full flights from orbit:

```bash
./quickglide.py --save qs_plane --instance 0 --set DEORBIT_WINDOW_BIAS=0.75
```

Arms 0.25 (control), 0.50, 0.75, 1.00, **n≈9 each** -- the full-flight
scatter is sd 14.8 km and three-flight arms against it are spaceplane failure
27, made and retracted once already. Expect ~40 min an arm on four instances.

Two things to check on the way that this session noticed and did not chase:

- **`qs_entry @ksp0` lands -41.4 km now; `docs/spaceplane.md` records -25.1
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
height the reach is 5.4 km against 4.6 needed. `tests/test_spaceplane.py`
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

**`tests/fakeplane.py` replays `planeprobe`'s table, and that table is wrong
subsonically by about 1.8x** (failure 13). Everything below still holds — the
step convergence, the self-consistency, the sign checks — because those are
properties of the *scheme* and not of the coefficients. What the sim cannot
be used for is any statement about the airframe: best glide, stall speed,
where the gate goes. Those come from `./polar.py`, which reads them back off
the flown logs.

`tests/fakeplane.py` is not a physics model of a spaceplane — it is a *replay
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

`tests/glidesim.py` flies the real guidance against it end to end. What it can
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
./savegen.py --source qs_plane -o qs_plane_inc  --normal   100   # inclination
./savegen.py --source qs_plane -o qs_plane_high --prograde  80   # elliptical
./savegen.py --source qs_plane -o qs_plane_ecc  --radial    60   # steeper
./savegen.py --source qs_plane -o qs_plane_low  --prograde -40   # lower
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
`boosterland.pacing.ScaleGovernor`, pointed at the plugin's control file by
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
./quickglide.py -n 3 --instance 0 --timescale 6   # or fly the quicksave
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

## Session handoff, 2026-09-20: what is measured, what is refuted, what is open

Read this before touching the spaceplane. It is the state a whole session of
in-game measurement left, and most of it is *negative* results -- which is the
expensive kind to re-derive.

### The committed configuration, and what it actually does

Fingerprint `6debf0c5`. On `qs_plane`, two **independent** eight-flight
batches of this exact configuration, which agreed at 7 of 8 each:

    n=16   intact 14 (88%)   on the runway 14 (88%)   on the strip 12 (75%)
    stopped along  +939 m  sd 455        (the runway is +-1200 m)

Two changes were added this session and both are **flown, not argued**
(`pairfly.sh`, 8 an arm, `qs_plane`): intact 4/8 against 2/8, on the strip
6/8 against 3/8, cross-track at rest **27 m against 66**.

- `BRAKE_FOR_DISTANCE` -- the rollout braked flat out from the first tick
  (`brakes = speed < 200 m/s`, i.e. always), measured at **12.3 m/s^2**, which
  is 83 kN on 6.9 t -- more than the vehicle weighs -- for a rollout whose own
  docstring computes a requirement of 0.5 and notes drag alone gives 1.0.
  Now `needed = v^2 / (2 * remaining)` less the drag already present.
- `APPROACH_BANK_COMPENSATION` -- `alpha_for_speed` solves for a load in the
  *vertical plane* and nothing divided by `cos(bank)`, so at the S-turn's 40
  degree limit the wing carried 77% of the load the loop computed, in exactly
  the moments the approach had decided it was high.

### Refuted this session -- do not re-try without reading the failure

| what | result | where |
|---|---|---|
| `TOUCHDOWN_AIM_M` 2400 -> 1200 | lands 245 m earlier, **halves survival** (3/8 vs 7/8) | 84 |
| S-turn stop as a time, not a distance to the aim | wrecks at cross-track +125..+145 m | 84 |
| `CROSS_DEADBAND_MAX_M` 40 km -> 10 km | +3.8 km, t=1.80 at n=8; **looked decisive at 4v4 and dissolved** | 78 |
| `ALPHA_TRACKING_ON` | table is *correct* (re-measured, 25 007 ticks) and enabling it is **worse** | 77 |
| `GLIDE_BANK_DUTY_ON` (propagate the reversal duty cycle) | -1350 m controlling for reversals | 79 |
| landing gear spring/damper, **both directions** | stiffer 1/8, softer 4/8, **stock 7/8** | 83 |

### Open, in the order I would take them

1. **Inclined orbits.** The arrival at `GLIDE -> HAC` is bimodal, `+86..+1005`
   against `-3664..-20562`, and **half its variance is the bank reversal
   count: +2137 m per reversal, r^2 = 0.49 over 35 flights**, monotone from
   six reversals to eleven. `Steer` holds one constant lean while the vehicle
   reverses 6-12 times, spending 88-138 s of a 700 s glide near wings level
   where it sinks less and flies further -- a range term the propagator has no
   representation of (CLAUDE.md's "the propagator must fly the law the vehicle
   flies"). Failure 79 explains why modelling it *alone* makes things worse:
   it cancels another error nobody has found, and **the pair has to be fixed
   together or not at all**.

   The lead I was on when the session ended: the dwell that limits the
   reversal rate (`BANK_REVERSAL_DWELL_SLEWS`) is switched off below
   `SPEED_FLOOR_MACH`, and that constant's own comment says the subsonic
   phase "still turns over every 12-18 s against a 17.5 s slew -- so it is
   permanently in transit, and that is where the flights that still diverge
   lose it." Splitting the dwell's Mach gate out from `SPEED_FLOOR_MACH`
   makes that a one-flag experiment. **Not run.** Note the caveat that sent
   me back to the drawing board: the divergence I measured on `qs_plane_inc`
   and `qs_e80` is at 24-19 km and Mach 3.7-2.5, which is *above* the gate,
   so the dwell was already active there -- the documented subsonic churn is
   a different window from the one the arrivals diverge in.

2. **Higher-energy orbits.** A five-arm ladder (`savegen.py --prograde`
   0/20/40/60/80 m/s, four flights each, every arm on every instance) puts the
   cliff between a **104 km and a 130 km apoapsis**: 3 of 4 on the runway
   becomes 0 of 4 and never recovers. **The failure is dispersion, not bias**
   -- arrival sd 157 m at circular against **5836 m** at 130 km. Peak q does
   not explain it (the worst arm has the *lowest* q), nor does the alpha
   ceiling crossing `SOLVE_ALPHA_MIN_DEG`, nor Kerbin's rotation (checked:
   burn-to-arrival is 1095-1114 s across all five arms, so the surface turns
   the same ~193 km under every one). What survives: the glide shortens
   598 -> 489 s while burn-to-arrival stays fixed, and the deorbit range comes
   out at 1945-1950 km on every arm against a `DEORBIT_RANGE_MAX_M` of
   2300 km it never uses. Failure 80.

3. **It lands late.** +939 of a +-1200 m runway is the last quarter of the
   tarmac; a flight that stops at +1737 has rolled off the end, which is the
   complaint this session opened with. Four levers have been measured against
   it and rejected (the table above). Nothing cheap is left.

### Two method errors this session, both now rules

- **"Off until flown" is not the conservative choice.** Two changes were
  committed as defaults, then turned *off* as unvalidated, then measured and
  found to be worth about half the landing performance. Disabling an unflown
  change swaps in the opposite hypothesis and calls it caution. Failure 82.
- **An eight-flight batch resolves a factor of two and nothing finer.** The
  same configuration gave 4 of 8 and 7 of 8 in consecutive batches; a
  four-flight arm looked "decisive at 4.4 sigma" twice and dissolved both
  times. Quote a rate with its n, and prefer `pairfly.sh`, whose
  round-by-round interleaving protects a *comparison* even when the absolute
  level drifts. Failures 78 and 83.

### Tooling added or repaired

`rollsum.py` (what the rollout was handed vs whether it kept the vehicle),
`appsum.py` (what the approach flew vs the descent it asked for),
`pairfly.sh` (two arms **or** two saves, halves swapped every round, so the
instance and the clock are both controlled; it also records the defaults
fingerprint at launch and re-checks it at the end). `armsum.py` was **silently
dead** since the heading-alignment cone was added -- it greps for
`GLIDE -> APPROACH`, a transition that no longer exists -- and grouped by the
per-instance governor path, making twelve flights into twelve arms of n=1.
Both fixed. Every log now records `SAVE_NAME`, without which a campaign across
three entry states misfiles itself.

## Session handoff, 2026-09-21: the aim's gain, and what it fixes

Read this with the 2026-09-20 handoff above, which it corrects in two places.

### The one number this session found

**One metre of arrival costs 6.4 m of `DEORBIT_CENTRE_BIAS_M`**, because two
gains multiply and neither had been measured:

    the window moves  -19.7 km per m/s of dv   (offline, every energy in the ladder)
    the arrival moves  -3.06 km per m/s        (in game, qs_e60, 16 flights)

Every test this project has run on the deorbit aim used 2.5-17 km of bias,
which buys 0.4-2.6 km of arrival -- at or under the noise of an eight-flight
batch, every time.  So "the aim is not a lever" (CLAUDE.md, failures 33, 68,
84, 86) is what a *correct* knob looks like when it is read at a seventh of
its scale.  Flown at the right scale on `qs_e60` the aim moves the arrival
3-4 km per 20 km of bias, with the delivered dv moving exactly as predicted
beforehand (+1.2 m/s at 28000 against a prediction of +1.1; +0.65 m/s from
28000 to 44000 against 0.66).  Failure 88.

**Check the gain before believing a null on any aim knob.**  The gain is
computable offline in one command (`deorbit_solution` at two biases), and
that check is cheaper than the batch it saves.

### What that buys on the high-energy save

`qs_e60` (a 157 km apoapsis, failure 80's arm that went 0 of 4 onto the
runway) is now a bracketed one-parameter problem:

| `DEORBIT_CENTRE_BIAS_M` | delivered dv | arrival | where it stops |
|---|---|---|---|
| 0 (committed) | 96.2 | +3939 sd 3100 | cone out of height, mostly lost |
| 28000 | 97.4 | -48 sd 3040 | mixed; the two runway landings of that batch |
| 44000 | 97.6 | -1354 | ~+3000, rolled off the end |
| 67000 | 98.6 | ~-7000 | **intact 3 of 6**, 7 km short |

**and then, on a farm restarted for it, 51000 lands it** (8 an arm,
`pairfly.sh`, swap 4.0 GB):

| | stopped along | on the runway | intact | arrival | cone surplus |
|---|---|---|---|---|---|
| `DEORBIT_CENTRE_BIAS_M=51000` | **+793 sd 123** | **8 of 8** | **6 of 8** | +131..+531 | +496..+499 |
| committed defaults | +152 sd 3350 | 1 of 8 | 0 of 8 | +4000..+5064 | -465..-751 |

Re-flown after `GOVERN_ON_PEAK` became the default (below), the same arm is
**8 of 8 on the runway and 8 of 8 intact** against 0 of 8 -- sixteen flights
of the configuration across two batches, sixteen on the runway, fourteen
intact.  Failure 93.

`sd 123 m` is tighter than `qs_plane`'s own committed configuration
(+939 sd 455).  **The three rows above it locate the optimum in the wrong
place** -- they were flown with 10.6 GB in zram after two and a half hours
of farming, which is the drift CLAUDE.md's farm rule is about: `pairfly.sh`
protects the comparison and nothing protects the level.  Failure 89.

**The target is the reserve, and that is not a fitted number.**  The winning
arm arrives at +131..+531, which is `GLIDE_RESERVE_M`, and earns a cone
surplus saturated at +499.  Only the *bias that achieves it* is per-state,
and 51000 must never be committed as a default -- on `qs_plane`'s 32 m/s
burn it is 2.2 m/s and 6-7 km short.

**`conesum.py`'s surplus is the outcome variable on this save**: positive
and the cone rolls out and the vehicle lands, negative and it exits *out of
height* 3-6 km from the gate with the approach having no vote.

### Two corrections to the 2026-09-20 handoff

- **The bank reversal count is a symptom of saturation, not a range term.**
  Its sign is +2137 m per reversal on `qs_plane_inc` and *negative* on
  `qs_e60` (8-9 reversals give +2689..+5003, 13 give -238).  A range term
  cannot change sign between two entry states of one vehicle.  What is the
  same on both is that a saturated glide stops reversing -- pinned at bank
  0.9 when short, at bank 70 when long -- and misses in the direction it is
  pinned.  Failure 87.  This retires the modelling programme of failures 78,
  79 and 85.
- **Failure 79's pre-registered "both together" arm is refuted.**
  `GLIDE_BANK_DUTY_ON` + `ALPHA_TRACKING_ON` on `qs_plane_inc`, 8 an arm:
  arrival -11664 sd 10700 against -4348 sd 4400, and the arm moved the
  reversal count *down* (9.0 to 7.4), which is the mechanism of its own
  failure.  Failure 85.

### The governor was reading a mean where it needed a tail

`ScaleGovernor` keeps a decaying *maximum* of the tick cost -- its own
comment says "Not the mean: what binds is the tail" -- and it was being fed
`LoopRate.busy`, an exponential average.  Two filters in series, and the
second never sees what the first removed.  `DEORBIT`'s search ticks cost
50-80 ms and its waiting ticks 3-12, so while the phase waits the estimate
falls to the cheap ones, the scale ramps to the 6x ceiling, and the burn
ignites there -- 0.3 game-seconds between commands at 17.8 m/s^2, which is
5.3 m/s of dv.

Sorted by achieved `DEORBIT` interval, every `qs_plane_inc` flight on disk
splits at 0.22 game-s/tick **with nothing in between**: 0.10-0.21 arrives
within 2.6 km, 0.24-0.32 arrives 6.5-25 km short.  `GOVERN_ON_PEAK`
(committed) governs on `LoopRate.peak`, the phase's worst tick, undecayed:

| | `DEORBIT` interval | arrival | on the runway |
|---|---|---|---|
| `qs_plane_inc`, peak | **0.10-0.14 every flight** | **+199 sd 420** | 4 of 6 |
| `qs_plane_inc`, mean | 0.10-0.31, bimodal | -4378 sd 5369 | 4 of 8 |
| `qs_plane`, peak | -- | -- | 8 of 8, +838..+971 |
| `qs_plane`, mean | -- | -- | 7 of 8, +554..+1258 |

4-5% of throughput for a thirteenfold reduction in arrival scatter and no
regression anywhere.  Failures 91-92.  Asking for the interval earlier does
*not* fix it (`DEORBIT_PACE_WHOLE_PHASE`, flown, null): the interval was
never the problem, the cost estimate was.

**The rule: a governor must be driven by the worst tick of the phase it
governs, and any smoothing upstream of it is the bug.**

### Open, in the order I would take them

1. **`qs_plane_inc`'s touchdown.**  The arrival is fixed; the vehicle still
   comes apart on contact on most flights, from flare states
   indistinguishable from `qs_plane`'s (h 123-160, v 90-99, sink 31-45,
   touchdown 45-62 m/s), `Elevon 4` first.  Failure 81's reading stands and
   it is a craft change.
2. **The deorbit bias is a per-state calibration and must not become a
   default.**  51000 works on `qs_e60`; on `qs_plane`'s 32 m/s burn it is
   2.2 m/s and 6-7 km short.  A proportional-to-dv law fails on inspection:
   `qs_plane_inc` already arrives at +480 with no bias at all, so 3% of its
   43 m/s burn would push it 3.5 km short.  What the aim needs is a
   *measured* correction, and the one early measurement nobody takes is in
   the coast -- the propagator predicts the interface crossing minutes
   before the glide begins and nothing compares it with what arrives.
   `DIAG_INTERFACE` exists for exactly this.
3. **A superseded lead, recorded so it is not re-taken.**  Failure 90 read
   `qs_plane_inc`'s bimodality as made *during the coast*, because the first
   GLIDE tick's `long=` predicts the arrival 600 s early.  That observation
   is correct and the inference was wrong: what the first tick was reporting
   is the state a burn flown at 0.3 game-seconds per command leaves behind
   (failure 91).  The coast is fine.
4. **`GLIDE_RESERVE_M` on inclined orbits.**  500 -> 4000 measured +5.0 km
   of arrival on `qs_plane_inc`, 8 an arm, and it is *not* committed: on
   `qs_plane` it would land later still, and that save's complaint is
   already that it lands late.  Worth re-measuring now that the arrival
   scatter there is 420 m rather than 5.4 km -- the whole reason the
   reserve was cut to 500 was a save whose entry had stopped scattering.

## Session handoff, 2026-09-21 (second): uncommanded authority, and four refuted arms

Baseline reproduced CLAUDE.md exactly -- **+900 to +1081 m, sd 184-210, 5 of 5
on the runway**, committed configuration, four in-game batches of
`pairfly.sh` at 8 an arm.

| arm | result | verdict |
|---|---|---|
| `AIRFRAME_DERIVED` (batch 1) | +1405 sd 364, 1 of 4 on the runway | worse |
| `HAC_LD` 1.49, the *achieved* ratio (batch 2) | 3.9 km from the gate, stopped 5.2 km | far worse, killed at n=2 |
| `AIRFRAME_DERIVED`, corrected (batch 3) | +2751 sd 639, **0 of 7** on the runway | worse, twice measured |
| `GEAR_FOR_ENERGY` (batch 4) | +800 vs +947, ratio 4.12 -> **3.76**, 6 of 8 vs 4 of 8 intact | mechanism confirmed, size below noise |

`AIRFRAME_DERIVED` is refuted on this craft and stays off. `GEAR_FOR_ENERGY`
is the one arm whose *mechanism* was confirmed -- the glide ratio moved in the
commanded direction -- and its size is under an eight-flight batch, which is
consistent with the 3% gear cost above rather than the 19% it was sized on.

### Corrections to things the project believed

- **`APPROACH_BEST_LD` = 4.2 is right; its comment is stale.** The comment
  says "2.08 sd 0.27 over 41 flights"; `landsum.py` measures the
  rollout-to-wheels ratio at **3.96 sd 0.41** today. Do not "fix" the
  constant. `APPROACH_LD_DERIVED` is documented do-not-use.
- **`HAC_LD`'s "tracking 1.07-1.11" reads 1.27-1.39 today** (`conesum.py`, 58
  flights, the 30 that roll out). The constant 1.86 is what the cone must
  *plan* with and is not the achieved ratio -- flying the achieved ratio
  stopped 5.2 km from the runway, which `HAC_LD`'s own comment had already
  recorded once (LOG1315, "1.35 was 25% low", overflew by 3.5 km).
- **Do not raise `APPROACH_SCURVE_MAX_DEG`.** The S-turn is saturated and it
  is the tempting knob, but its cost is cross-track at the flare door, which
  is already at the wreck threshold: mean **73 m** against a measured
  survivor/wreck split at **70 m**. It trades a late landing for a destroyed
  vehicle.

### Refuted mechanisms, each with its account in the code

Do not re-try these without reading the docstrings, which record why:
`airframe.lift_discount` (`LiftTrim` for `MARGIN` -- different quantities, and
the bin reads 0.74/0.49/2.54 across three flights); `airframe.approach_ld`
(omits the flare's flat 400-500 m); `airframe.cone_ld` at the achieved ratio
(`PLANNING_BIAS` -- the cone must plan *above* what it achieves).

### The next change, now built and unflown: a split-rudder airbrake

The vertical `smallCtrlSrf` pair deployed in *opposing* directions is a
speedbrake: side forces at equal and opposite moment arms, so yaw and roll
both cancel and what is left is drag. **No control authority is lost, because
these surfaces have none** (see "This vehicle has no aerodynamic control at
all" above).

**The identification rule must stay geometric and must refuse rather than
guess.** `test_instances/surfacespan.py` implements it: span axis from the
part rotation quaternion; vertical when `|span.z| > |span.x|` in the vessel
frame; require a pair mirrored about the centreline; **no pair -> no
airbrake**, and the vehicle flies as it does now. Horizontal surfaces are
never candidates -- cancelling a canard against an elevon needs a balance of
areas and moment arms that differs on every aircraft, which fails the
generality bar.

**Trigger it on the S-turn saturating, not on an altitude.** Measured over
batches 3-4: the approach's weave command `sc=` sits at its 45 deg cap for
**52% of approach ticks**, with **+639 m of surplus still unspent**. That is
the guidance's own signal that its only range control has run out, so it is a
principled trigger rather than a fitted constant.

**Do not simply bolt drag on.** The gear result above is the warning: the
speed loop re-trims around added drag and absorbs most of it. The brake has to
be commanded against surplus, the way `GEAR_FOR_ENERGY` is.

**Built as `spaceplane/airbrake.py`, `AIRBRAKE_SPLIT_RUDDER`, default off and
never flown.** `find_split_rudder` is the geometric rule above and returns a
reason on every path, armed or refused, because a brake that silently did not
arm reads in the log exactly like one that armed and did nothing.
`Brake.update` is the policy, and **every threshold in it is derived from a
constant some other phase already owns** -- a brake with three fitted numbers
of its own is a brake that must be re-fitted on the next aircraft:

| what | derived from | not |
|---|---|---|
| the surplus that arms it | `APPROACH_SCURVE_M`, the surplus that started the weave | a second constant for the same quantity |
| the window | `AIRBRAKE_TAU_CYCLES` x 2 x `APPROACH_SCURVE_PERIOD_S` | a fitted number of seconds |
| where it stows | the flare door this tick computes (`FLARE_ALT_M + FLARE_LEAD_S * sink`) plus `AIRBRAKE_STOW_LEAD_S` **seconds of sink** | a height fitted to this craft's ~470 m door |

There is an unconditional stow on entering FLARE as a backstop.

#### Flown once, 8 flights, and stopped early: two defects, both in the
#### measuring rather than the mechanism

The identification worked on the live craft at the first attempt --
`airbrake armed: x=-2.50/+2.50, 1.0 m^2 each`, `deploy angle 20 deg set on 2
of 2` -- and the policy fired as specified: `airbrake out at 702 m: S-turn
saturated 50% with +226 m left`, then `in at 430 m: surplus spent (+98 m)`.
The batch was killed at round 2 of 4 because it could not have answered
anything, and both reasons are worth more than the batch would have been.

**1. The saturation measure was an EMA, and an EMA starts cold.** The window
spanned two weave cycles -- 40 s of a ~75 s approach -- so the average was
still charging when the vehicle reached the stow height. Of four `B` flights,
**two deployed for about seven seconds** and **two peaked at 0.44-0.48 and
never armed at all** (LOG2815-2818). Half the treatment arm was therefore
identical to the control, and the other half carried seven seconds of a 1 m^2
pair. *Fixed:* the share is now **capped seconds over elapsed seconds**,
believed once `AIRBRAKE_MIN_CYCLES` of weave has been sampled -- which is
the quantity the mechanism was built from in the first place ("the cap for
52% of approach ticks"), unbiased from the first tick. Nudging
`AIRBRAKE_SATURATED_FRAC` down to 0.4 would have "fixed" the same symptom by
fitting a constant to this craft.

**2. The check that was supposed to be able to contradict the mechanism could
not.** It logged each half's `available_torque` and their sum, expecting near
zero -- but `available_torque` is the torque a surface *could* produce, given
as positive and negative magnitudes, so two halves can never cancel in it
however perfectly they oppose. `deflection` read `+0.00` through a deployment
that demonstrably happened, because with every axis disabled there is no
control-input deflection to read. Both agreed with themselves and neither
could ever have disagreed: failure 13's shape, rebuilt by accident.
*Fixed:* the log now records the **sideslip and body rate** when the brake
comes out, and the settling comparison is `slip=` with `ab=out` against
`ab=in` over a batch -- a clean split leaves them indistinguishable.

**What the batch did establish**, incidentally: both arms wrecked at a
similar rate (4 of 8 kept 18+ parts), and every wreck entered the flare
outside the 70 m cross-track split (-103, +240) while the survivors were
inside it. That is failure 34's known mechanism and it indicts neither arm.
 The approach line now
carries `ab=` and `sat=`, and `sat=` is logged even when the brake is not
armed -- it is the measurement the whole mechanism rests on and it costs
nothing to keep taking.

**What to fly:** one `pairfly.sh` arm of 8 against the committed default, on
`qs_plane`, reading `stopped along` first and the intact count second. The
prediction is a *smaller* overshoot with `ab=out` on the ticks where `sc=`
was capped; the failure modes to watch for are a yaw the identification was
supposed to make impossible (`xt=` growing while `ab=out`) and the gear
result repeating -- the speed loop absorbing the drag, which would show as
`ab=out` with `exc=` unmoved. 1 m^2 of extra `CdA` against a craft total of
~5.6 is the size of the effect available, so it is not a subtle-enough
question to need more than one batch.

**Two generality questions it does not yet answer**, both worth taking before
any constant in it is touched:

- *Other craft.* The identification refuses on anything but a clean mirrored
  vertical pair, so it is safe by construction -- but "safe" and "works" are
  different claims and only `qs_plane`'s geometry has been run through it,
  offline. `test_instances/mkheavy.py` and a craft file with a centreline fin
  or with two fin pairs are the cheap tests, and the second of those needs no
  game at all.
- *Other situations.* The trigger is the APPROACH's weave saturating, because
  that is where the surplus was measured. The same saturation signal exists
  one phase up -- the cone's `wv=` against `HAC_BANK_MAX_DEG`, and the
  glide's bank pinned at its stops, which is what failure 87 reads the
  reversal count as. A vehicle that arrives with the surplus the *entry*
  could not spend reaches the approach with more than the approach can spend,
  and the brake would be a phase too late. Do not extend it there by
  analogy: each phase's stow constraint is different, and the flare's is the
  one that destroys vehicles.

### Also unflown and still worth a batch

- **`HAC_ENTRY_AFFORDABLE`** -- enter the cone when it can pay for itself.
  Separates 21/3 from 9/24 over 57 flights offline, but note it asks
  `needed`, computed with `HAC_LD`, so it may be inert until that is honest.
- **`test_instances/mkheavy.py`**, which builds `qs_plane_heavy` (+20%
  landing mass via MonoPropellant, which `DRAIN` does not dump) -- the only
  generality test available, since every save in the farm holds the same
  airframe. It writes binary to preserve CRLF; reading a save in text mode
  silently strips 14 kB of line endings.

### State of the tree

**The defaults fingerprint is now `61adf259`, was `6debf0c5`.** Nothing that
flies changed -- `GEAR_DRAG_FRACTION` 0.19 -> 0.03 only bites with
`GEAR_FOR_ENERGY` on, and the airbrake fields are new and off -- but a batch
flown before this session carries the old hash, and `armsum.py` will file it
as a different baseline. That is the mechanism working (failure 28), not a
fault: the eighteen-flight and sixteen-flight results quoted above remain the
reference for this configuration's *flight* behaviour.

New this session, **all default off**: `AIRFRAME_DERIVED`,
`APPROACH_LD_DERIVED`, `HAC_ENTRY_DERIVED`, `HAC_ENTRY_AFFORDABLE`,
`LIFT_TRIM_DISCOUNT`, `GEAR_FOR_ENERGY`; plus `STALL_CALIBRATION_M_S` 55.6 and
`airframe.PLANNING_BIAS` 1.09, which exist so a derived quantity is expressed
in the units its consumers were fitted in and is therefore inert on the
reference craft. New modules: `spaceplane/airframe.py` additions,
`tests/flownpolar.py` (a real swept polar from `logs/LOG2747`),
`test_instances/{ctrlsrf,surfacespan,mkheavy}.py`, and now
`spaceplane/airbrake.py` with `AIRBRAKE_*` in `config.py`. The last four
batches are `logs/LOG2757-2812`. 530 offline tests pass.

### How the session went, which is itself information

Three changes were refuted in flight and one analytic claim was retracted
after being built on truncated shell output. The pattern is now written as
rules in CLAUDE.md: a measurement that shares a constant's name is usually a
different quantity; re-run the tool instead of reading the prose; and ask what
authority the vehicle has that the autopilot never commands. **The user
supplied the two most productive ideas of the session** -- pitch-down and bank
as uncommanded controls, and the split rudder -- and both were *missing
mechanisms* rather than mis-set constants.


## The airbrake flew, worked, and destroyed two vehicles doing it

The corrected estimator did what it was meant to -- `airbrake out at 1077 m:
S-turn saturated 100% with +459 m left`, against 702 m and a seven-second
deployment before -- and the drag is real and large:

```
alt=1238  v=106.3  ab=in     sc=45.0
alt=1023  v=105.6  ab=out    <- brake out
alt= 869  v= 93.4  ab=out
alt= 747  v= 84.1  ab=out    <- 21 m/s of speed gone in ~11 s
alt= 594  v= 85.3  ab=out
alt= 307  v= 99.5  ab=in     <- stowed; the speed loop dives to recover
```

Both braked flights then entered the flare at **h≈250 m with 79-82 m/s of
sink** (LOG2824, LOG2825) against **h 119-139, sink 29-36** for the unbraked
controls of the same round -- and both came apart on contact, while the two
controls landed intact at +900 and +953. The arrival did move the right way
(+186 against +900), which is the mechanism working; the vehicle just could
not fly the state it was handed.

**The cause is not the drag, it is which currency the drag was spent in.** A
glider makes speed only by trading height for it. The brake took the speed,
the speed loop bought it back with a dive, and the dive arrived at the flare
door. The surplus the brake exists to spend is *height*; the speed is the
approach's margin and was never the brake's to take.

*Fixed:* `AIRBRAKE_SPEED_GUARD` -- the brake retracts the moment the airspeed
falls below `guidance.approach`'s own target, which the command now
publishes (`command.target_speed`) rather than a second constant that could
drift from it. Worked through LOG2825's own speed profile the guard fires at
**747 m**, 250 m and eight seconds earlier than the flare-door stow did, with
the speed deficit at stow ~0 instead of 21 m/s. Unflown.

**And this is the same lesson the landing gear taught, from the other side.**
Gear-down drag was absorbed because `APPROACH_SPEED_PATH` re-trims alpha to
hold the commanded speed (3% of glide ratio, not 19%). The airbrake's drag
was large enough to break that loop instead of being absorbed by it -- so the
loop bought the speed back the only way it can. **On a speed-holding path
law, a drag device converts into flight-path angle or into a dive, and into
range only if the guidance spends the surplus deliberately.**

## The cone's dissipation is pinned at both of its caps

Found by asking the airbrake's question one phase earlier -- *what is
saturated, and what authority does the vehicle have that nothing commands?*
-- and measured over 722 circling HAC ticks from 126 logs (`logs/LOG27*`,
`logs/LOG28*`; the script is `coneradius.py` in the session scratchpad, kept
below as arithmetic anyone can redo).

| | ticks | bank flown, median | at `HAC_BANK_MAX_DEG` (45) |
|---|---|---|---|
| weave commanded | 345 | **45.0** | **57%** |
| no weave | 377 | 25.0 | 15% |

and the weave command itself, whenever it is on, reads **median 50.0, p90
50.0, max 50.0** -- that is `HAC_WEAVE_MAX_DEG` exactly, on every tick it
fires. So the cone asks for all the weave it is allowed and all the bank it
is allowed, and still hands a surplus down.

**But only one of those two caps is real**, which the next section works out:
the bank clamp sits downstream of the weave, so the 50 degrees the weave
commands is never flown -- about 32 is. `wv=50.0` in the log is a request,
not a measurement of the path.

That matters because the phase below does the same thing: the approach's
S-turn sits at its own cap for 52% of ticks with +639 m unspent (the
measurement that motivated the airbrake). So the chain is one story, not two
-- **the entry hands over a surplus, the cone cannot dump it, the approach
cannot dump the remainder, and the vehicle lands +939 m down a +-1200 m
runway.** Every lever tried against that overshoot so far has been in the
last phase or on the ground: the aim (68, 84), the S-turn stop (84), the
brake law (82), the gear's suspension (83), the gear's deployment
(`GEAR_FOR_ENERGY`, mechanism confirmed, ~150 m), the split rudder (1 m^2 of
`CdA`). The cone's bank redirects the whole lift vector, which is a
different order of authority.

**Why 45 is a policy and not a limit**: the same airframe flies
`BANK_MAX_DEG` 70 in the glide, one phase earlier, and pins itself there.
The cone is rationed to 45 with nothing in the file saying why.

**What the wing can pay for, measured** (`q * ClA / mg` over 634 HAC ticks):
median **1.06 g**, p90 1.36, max 1.91. Holding *altitude* in a turn needs
`1/cos(bank)` -- 1.41 g at 45 degrees, 2.00 at 60 -- so the vehicle cannot
hold a level 45-degree turn now and does not try to: the cone is a
*descending* spiral and more bank means more sink, which is the point when
the surplus is what you are trying to spend. The load ceiling still bounds
how tight the circle can be, and that bound is what `hold_radius` gets wrong
(below).

### `HAC_WEAVE_MAX_DEG` is decorative above about 32 degrees

Worth doing the arithmetic before touching either cap, because the two are in
series and only one of them binds. The cone's bank is
`clamp(forward + HAC_HEADING_KP * error, +-HAC_BANK_MAX_DEG)` with
`HAC_HEADING_KP` 1.2 and the feed-forward measured at a median 6.9 degrees,
so the heading error that saturates the bank is `(cap - 6.9) / 1.2`:

| `HAC_BANK_MAX_DEG` | saturates at | weave actually flyable | path stretch |
|---|---|---|---|
| **45 (committed)** | 31.8 deg of error | **31.8 deg** | 1.18x |
| 55 | 40.1 | 40.1 | 1.31x |
| 60 | 44.2 | 44.2 | 1.40x |
| 70 | 52.6 | **50.0** (the weave's own cap) | 1.56x |

So the configured `HAC_WEAVE_MAX_DEG` of 50 **cannot be flown at all** at the
committed bank cap -- the weave asks for it, the bank clamp refuses, and the
log dutifully prints `wv=50.0` while the vehicle flies about 32. That is
failure 33's clamp shape for the third time in this project, and it means
raising the *weave* cap alone would change nothing, while raising the *bank*
cap alone unlocks weave the configuration already asks for.

The dissipation this buys is the path stretch column: **1.18x today against
1.56x at bank 70**, on the phase whose surplus the approach then inherits.

### The arm this suggests, and what would refute it

`HAC_BANK_MAX_DEG` 45 -> 60, as a ladder if the farm allows (45/55/65),
read on **two** numbers: the surplus handed to the approach (`exc=` at the
first APPROACH tick) and the cross-track at the flare door. The second is
the failure mode: the cone's roll-out distance is what sets the cross-track
the flare inherits (`HAC_ROLLOUT_M` 1500 -> 900 moved `gate=1324 -> 778` and
`cross=+206 -> -37`), a tighter circle changes that geometry, and the flare
door's 70 m split is where this vehicle is destroyed. A batch that lands
earlier *and* wider has not won anything.

### Two diagnostics that are missing, and one model that is wrong

- **Nothing logs the flown turn rate.** `turn=` is re-solved every tick, so
  `d(turn)/dt` mixes flying with re-planning and the flown radius cannot be
  recovered from the logs: the estimate comes out median 7042 m against a
  planned 10800 with a p10-p90 of 0.5-13x, which is not a measurement. One
  `omega=` column would settle which radius law the vehicle obeys.
- **Nothing logs the cone's heading error**, so the split between the
  feed-forward bank and the proportional correction has to be inferred. It
  is worth knowing: the feed-forward `atan(v^2/gR)` is median **6.9 deg**
  while the bank flown is median 33.8, so **the geometry asks for a seventh
  of the bank the vehicle is using** -- the rest is weave and track
  correction.
- **`hold_radius` uses `tan(bank)`, which assumes the wing pulls
  `1/cos(bank)` g.** It pulls about 1.06. The lateral acceleration actually
  available is `n g sin(bank)`, so the honest radius is
  `v^2 / (n g sin(bank))` with `n` measured -- and the `tan` form is
  optimistic at every bank angle, increasingly so as bank rises. This is the
  same shape as `HAC_LD` having to be planned 30% above what the cone
  achieves: a plan that assumes authority the vehicle does not have, with a
  fitted constant absorbing the difference. **Fix the model before raising
  the cap much past 60**, or the plan gets tighter exactly where it is least
  affordable.

### And why 90 degrees is not the limiting case of this

At 90 degrees the lift vector is horizontal: there is no vertical component
at all, the vehicle stops gliding and falls, and it arrives *low and fast*
rather than high. The turn does not tighten the way the formula suggests
either -- lateral acceleration goes as `n g sin(bank)`, so 45 -> 90 buys a
factor of 1.41 in turn rate (a real radius of 9006 -> 6371 m at 1 g and 250
m/s) while the *planned* radius goes 8282 -> 0 and is caught by
`HAC_RADIUS_MIN_M` = 2000. A 2000 m circle at 250 m/s needs 3.2 g laterally
against the 1.91 this wing has ever delivered, and the log would faithfully
print `R=2000` while the vehicle flew something else -- failure 33's clamp,
again.


## Flying at a sideslip: the prediction, written before the data

A forward slip is how a glider with no spoilers dumps energy, and **nothing
in either autopilot has ever commanded one** -- measured over every log on
disk, the vehicle sits at |slip| median 0.8 deg in GLIDE and 2.1 in APPROACH.
The user proposed it; this section is the prediction, recorded before the
probe flights land, so the measurement can contradict it.

`CdA(b) = CdA(0) + (broadside - CdA(0)) sin^2 b`, with `CdA(0)` 6.1 flown
subsonic at aoa 8 and broadside 29 from the swept table:

| held slip | predicted `CdA` | change |
|---|---|---|
| 5 deg | 6.27 | +3% |
| 10 deg | 6.79 | +11% |
| 20 deg | 8.78 | **+44%, +2.7 m^2** |
| 30 deg | 11.82 | **+94%, +5.7 m^2** |

For scale, the split rudder measured about **+2 m^2 (+36%)** and took 21 m/s
out of the approach in eleven seconds. So 20 degrees of slip is worth more
than the airbrake and 30 degrees is worth three of them -- *if the wheels can
hold it*, which is the only thing `SLIP_PROBE_DEG` measures.

**Two ways this prediction can be wrong, and both matter.** The broadside
figure comes from the same probe table that was wrong by 1.8x subsonically
(failure 13), so the *magnitude* is suspect even though the shape is not.
And `sin^2` is the flat-plate form: a fuselage with a wing on it will not
follow it exactly, particularly where the wing starts shadowing the fin.
Which is why the number that decides is `act=` -- the achieved aerodynamic
force, which needs no model and no density assumption.

**Early result, from the coast (q < 500 Pa):** every commanded angle holds
at 96-98% of command, 30 degrees included. That is the regime where the
wheels have no weathercock moment to fight, so it proves the yaw authority
exists and proves nothing about the approach. The ceiling against `q` is
what the probe flights are for, and `slipsum.py` reads it back.

### First correction, from the glide: hypersonic slip buys nothing

Measured on the probe flights themselves, all four at a commanded alpha of
32 degrees and M > 6.5, so the only difference between them is the slip:

| commanded slip | `ClA` | `CdA` | L/D |
|---|---|---|---|
| 5 deg | 9.30 | 11.50 | 0.81 |
| 10 deg | 9.20 | 11.70 | 0.79 |
| 20 deg | 8.90 | 11.60 | 0.77 |
| 30 deg | 8.50 | 11.20 | 0.76 |

**No drag** (+0/+2/+1/-3%, i.e. nothing), and lift down 9% at 30 degrees, so
L/D falls 0.81 -> 0.76. The prediction above is wrong *here* and the reason
is plain once seen: `sin^2 b` assumes a streamwise baseline, and the
hypersonic glide already flies at **32 degrees of alpha** -- the body is
broadside to the flow before any yaw is added, so yawing it adds almost no
projected area. The prediction's regime is the subsonic one, where alpha is
8-12 and `CdA` about 6, and that is what the rest of the probe flight
measures.

Worth keeping either way: **slip is a cheap lift-spoiler at hypersonic
conditions** (-9% `ClA` for -3% `CdA`), which is a different tool from a
brake and might belong to the entry's range control rather than the
approach's.

**Sign convention, measured:** a commanded `slipc=+30.0` reads back as
`slip=-30.3` -- kRPC's `sideslip_angle` runs opposite to a rotation about the
lift axis. The magnitude tracks to a third of a degree; only the sign is
flipped, and any control law that acts on the sign has to know it.

**And the obvious second use of a lift-spoiler is dead on this save.** If
slip spoils lift without adding drag, the place to spend it is a glide that
is long and already out of bank -- failure 87's "pinned at bank 70 when
long". Measured over 23083 GLIDE ticks: **59 of them (0.3%) are at the
70-degree stop.** The glide is not bank-saturated on `qs_plane`, so there is
no opportunity there, and the saturation this project keeps rediscovering is
specifically the *cone's*. One query, one line, one dead end closed.

### The ceiling, measured: it is a saturation in `q`, like the alpha one

`slipsum.py` over the four probe flights (5/10/20/30 degrees commanded, one
per instance, each sweeping `q` on the way down):

| commanded | holds through | partial | lost |
|---|---|---|---|
| 20 deg | **5000 Pa** (101% of command) | 5-7k (74%) | 7k+ (43%) |
| 30 deg | **4000 Pa** (90%) | -- | 4-5k (23%), above (2-8%) |

So the authority is real and bounded, and the bound is exactly the shape
`Holdable` already learns for alpha: `achieved = min(command, holdable(q))`.
The approach's median `q` is about 5300 Pa, which puts **20 degrees at the
edge of what final can hold and 30 degrees comfortably inside the cone's**
(median `q` 4293). Anything built on this has to learn the ceiling in flight
rather than carry a table -- the same argument as everywhere else in this
file, and now with a measured curve behind it.

**If it holds, it changes the order of the whole session's work.** Slip is
*continuously variable*, where the split rudder is a fixed 20-degree deploy,
so it can be commanded in proportion to the surplus and backed off as the
speed margin closes -- which is exactly the failure that destroyed two
vehicles with the airbrake. And it can be spent in the cone, where the
saturation actually is, with 10 km of path and no flare underneath it.


## Sideslip, measured end to end, and the law it earned

The user proposed flying at a sideslip; four probe flights (5/10/20/30 deg
commanded, one per instance, `SLIP_PROBE_DEG`) settled every part of it in
one round. **The idea works, but not for the reason it was proposed, and not
in the regime it was proposed for.**

**1. The authority is there and nothing has ever used it.** The vehicle holds
what it is asked for -- 98-102% of command -- and the bound is a saturation
in dynamic pressure, the same shape `Holdable` already learns for alpha:

| commanded | holds through | partial | lost |
|---|---|---|---|
| 20 deg | 5000 Pa (101%) | 5-7k (74%) | 7k+ (43%) |
| 30 deg | 4000 Pa (90%) | -- | 4-5k (23%), 5k+ (2-8%) |

**2. It is not a brake, at any speed.** Hypersonic (M>6.5, alpha 32): `CdA`
+0/+2/+1/-3% across 5/10/20/30 degrees -- nothing, because the glide already
flies broadside. Subsonic (HAC and APPROACH): `CdA` flat again.

**3. It is a glide-ratio spoiler, and that is better.** Subsonic, at ~15 deg
of held slip, `ClA` falls **15.7 -> 10.9** with `CdA` flat, so

    L/D  1.64 -> 1.23     a quarter of the glide ratio, airspeed untouched

`tan(gamma) = D/L`: less lift at the same drag is a **steeper path at the
same speed**. Compare the split rudder, which spent *speed* (21 m/s in
eleven seconds) and forced the speed loop to dive into the flare door. The
approach's problem is a surplus of height with too much glide ratio to spend
it; this is a direct control on the glide ratio and the speed loop has
nothing to fight.

**4. And it must be straight before the wheels.** The 10, 20 and 30 degree
probes were all destroyed in ROLLOUT; the 5 degree one landed normally.
Failure 34 again -- a crabbed touchdown tears the gear off.

### `APPROACH_SLIP_FOR_ENERGY`, built, tested offline, unflown

Proportional to `guidance.approach`'s own `excess` above a deadband, so it is
inert on profile; ramped at `SLIP_RATE_DEG_S` in both directions; capped by
`SLIP_MAX_DEG` *and* by the ceiling `Holdable` learns live off the yaw axis;
zero at the flare door plus its lead.

**Its ramp test paid for itself before the arm flew.** The first version fell
back to an unbounded step whenever `dt` was zero -- which is the first tick of
every APPROACH -- so the law opened with twenty degrees of yaw in one command,
which is exactly what the ramp exists to prevent.

### Flown once, stopped at one flight, and the flight found a sign bug

`LOG2836`, the first B flight: **stopped +865 intact** against controls at
+900 and +953, and -- the thing that mattered -- **the flare door was
normal**: `h=137.9 v=93.8 sink=35.9 cross=+65` where the unbraked controls
arrive at h 119-139 and sink 29-36. Speed held 96-105 m/s through the whole
slip with no dive. So the currency is right: this spends height without
touching the speed the flare needs, which is exactly what the split rudder
could not do.

**But the law was running at about half strength, by construction.** The
sign was taken from the commanded bank -- and the bank alternates with the
S-turn weave, so the command reversed twice a cycle and spent the approach
ramping through zero:

```
alt=1925  exc=+857  slipc= -8.3
alt=1835  exc=+813  slipc=-12.3
alt=1741  exc=+762  slipc= -6.7     <- reversing
alt=1649  exc=+715  slipc= +3.5
alt=1569  exc=+679  slipc= +9.6
alt=1065  exc=+423  slipc= -4.5     <- reversing again
```

A law asking for 8-12 degrees delivered a mean of **5.9**. That is
structural, not statistical -- it follows from the code -- so the batch was
stopped at one flight rather than spending eighteen on a half-strength
configuration. *Fixed:* the side is chosen once, away from the centreline
the vehicle is already off, and held for the phase. What spoils lift is the
magnitude; every reversal is authority spent going through nothing.

**What would refute it:** the arrival moving earlier but the *flare door*
state degrading -- cross-track outside 70 m, or sink above the 29-36 m/s the
unbraked controls arrive with. Read `landsum.py` for the stop and the
`APPROACH -> FLARE` line for the door, in that order, and remember that this
save's controls land at +900 to +953 intact.


## The slip has two authorities, and the second one was found by wrecking a vehicle

Flown as a pure lift-spoiler with the sign chosen once per approach
(`logs/LOG2838-2846`, four an arm before the batch was stopped):

| | along, median | \|across\|, median | intact |
|---|---|---|---|
| A defaults | **+1122** | 13 m | 2/4 |
| B slip | **+762** | 59 m | 3/4 |

**The along-track mechanism is real and it is not an artefact of the arms
starting differently.** The surplus each flight was handed at its first
APPROACH tick is matched -- A +919/+864/+875, B +802/+911/+927, means 886
against 880 -- so the ~350 m separation is made in the phase where the slip
acts. Speed was held throughout (96-107 m/s, no dive) and the flare doors
were ordinary, which is the whole difference from the split rudder.

**The second authority is much larger than expected.** Over the four slipping
flights the cross-track moves **in the slip's own sign at about 50 m per
degree**:

    mean slip +6.8 -> cross +372 m      mean slip -7.1 -> cross -477 m
    mean slip +8.1 -> cross +367 m      mean slip +9.1 -> cross +411 m

So a *held* sign is a 400 m lateral kick, and `logs/LOG2846` is what that
costs: it began at cross -259, chose a positive slip -- the correct
direction -- and rode it through zero to **+301 at the flare door**,
destroyed 227 m off the centreline. The direction was right; the absence of
feedback was fatal.

*Fixed:* the sign is now feedback on the current cross-track with a
`SLIP_CROSS_DEADBAND_M` band so it does not chatter at the centreline, and
the existing ramp bounds how fast it reverses.

**And this reframes what the control is.** The approach has exactly two
lateral authorities today -- the capture's bank, capped at 40 degrees, and
the S-turn, which is pinned at its own cap for 52% of ticks. Slip adds a
third worth ~50 m per degree that is *paid for in height rather than in
path*, which is the one currency this phase has a surplus of. A mechanism
built to spend energy turns out to be a lateral control as well, and the
phase is short of both.

**What to watch when it next flies:** the cross-track should now converge
rather than diverge -- the failing signature is the monotone
`+80 +102 +152 +215 +268 +297 +302` of LOG2846 -- and the along-track gain
should survive, because the magnitude law is unchanged and only its sign
moved.

## The slip, flown nine an arm: the result, and the lateral bill it comes with

`APPROACH_SLIP_FOR_ENERGY` against committed defaults, `pairfly.sh`, nine an
arm, `qs_plane`, one fresh farm, fingerprint `86c9c646`:

| | mean along | sd | median | on the runway | intact |
|---|---|---|---|---|---|
| defaults | +1263 | 456 | +1129 | 6 of 9 | 4 of 9 |
| `APPROACH_SLIP_FOR_ENERGY=True` | **+790** | **215** | +860 | **9 of 9** | 7 of 9 |

**-473 m of overshoot and less than half the scatter.** The defaults ran off
the end twice (+2055, +2108, both `splashed`); the slip arm's longest was
+1009. The arms are matched on what they were handed -- surplus at the first
APPROACH tick +903 against +838 -- so the separation is made in the phase
where the slip acts.

**The bill is lateral.** Door cross-track median **26 m against 14**, and the
arm's two wrecks were its two highest-slip flights, arriving at `cross=+151`
and `+176`. At ~50 m per degree a sign taken from the offset *now* is still
leaning when the offset reaches zero, so the control **arrives** at the
centreline instead of arresting at it -- `logs/LOG2846`'s failure one order
smaller. *Fixed, unflown:* `SLIP_CROSS_PREDICT` takes the side from the
cross-track predicted at the flare door (`cross + rate * time`), the way
`APPROACH_LATERAL_CAPTURE` already reasons; the capture now publishes
`cross_rate` and `cross_time` so the two lateral authorities solve the same
problem rather than two different ones.

**Do not answer the lateral cost by shrinking the magnitude.** The magnitude
is the energy control and it delivered a mean of 6-8 degrees where 15 was
measured to be worth a quarter of the glide ratio.

Still default off, and **every slip flight so far is `qs_plane`**. Generality
is unmeasured; the batch to run first is `qs_plane_inc` (whose touchdown is
separately broken -- failure 81 -- so read its *arrival* and door state, not
its wreck count), then `qs_e60` with `--set DEORBIT_CENTRE_BIAS_M=51000`,
then a different airframe.

## Flying the entry for drag, and buying the burn with it

Two changes that are one design, both default off, prompted by the question
*what authority does the vehicle have that nothing commands?* asked of the
entry rather than the approach.

### `ENTRY_MAX_DRAG`: the entry's alpha is bounded by a lift argument

`ENTRY_ALPHA_DEG` is 22 because a *range* sweep plateaus at 20-22;
`ALPHA_MAX_DEG` is 32 because "past 30 the lift curve turns over". Both are
lift arguments bounding a phase whose job is to destroy energy. Drag does not
turn over. This airframe's own swept table at entry Mach:

| alpha | ClA | CdA | L/D |
|---|---|---|---|
| 22 (flown) | 6.9 | 5.7 | 1.21 |
| 35 | 9.3 | 14.4 | 0.65 |
| 65 | 4.1 | 27.7 | 0.15 |
| 90 | **0.0** | 28.2 | 0 |

Five times the drag, never asked for. And at 57.9 km the log reads
`aoa=22.0/22.7` at `q=128 Pa` -- the command is tracked exactly, so up there
the authority is free.

**The guard is the plant, not a constant.** The command is `Holdable`'s
ceiling -- live evidence first, `HOLDABLE_PROBE` (LOG1615's wheels-only
curve) as the prior -- so the vehicle is never asked for an angle it has been
seen to refuse. No standing shortfall means no ratchet fighting it and no
monopropellant spent arguing, which is how the whole tank once went for three
kilometres (LOG1616).

**This looks refuted by "High alpha: what the airframe gives and what it will
hold" above, and is not.** That section closed high alpha *on a committed
entry*, which crosses the thin-air band in seconds. A shallow entry is
defined by living in it. Same airframe, same curve, different trajectory.

### `DEORBIT_SHALLOWEST`: the smallest burn that still captures

Every other rule in this file picks the burn by *range*, which is not
monotone in dv -- hence a grid, a window and a tolerance. "Does this burn
commit?" **is** monotone, so it is a bisection: twelve propagations, no
fitted constant, and the answer is a property of the vehicle and the air.

It buys propellant, which this craft does not need and the next one will.
Measured offline (fakeplane, so a hypothesis until the game agrees), one
state, same arrival: **dv 43.23 -> 37.04 m/s, 14% of the burn.**

And the two halves need each other: broadside has `ClA` *exactly zero at
every Mach in the table*, so a broadside entry has nothing to skip on, and it
is the lift that makes a shallow entry bounce. The smallest committing burn
moves **16 -> 12 m/s** when the entry is flown for drag.

**What actually binds the shallowness today is not the skip.** It is
`DEORBIT_MAX_TIME_TO_GO_S` = 1500, the fitted stretch bound this file already
admits is standing in for "do not commit to an entry so shallow the
propagator cannot be trusted". Raising it drops the threshold to ~12 m/s.
Removing it once cost three flights and 95-132 km; the no-lift argument
undercuts the *reason* for it, which makes raising it the next arm and not a
silent edit.

### Where the drag stops and the lift starts: solved, not configured

The first version switched at a fixed Mach 3. That is the shape this project
has a rule against -- and it is the worst possible constant to fix, because
it sets the entry's length and the entry's length sets **where the burn
goes**. So it is solved at the burn, against the propagator, and carried on
the `Steer` as `drag_until` (`guidance.drag_switch_for`). Monotone: switch
early and the entry is long, late and it is short. Bounded below by the
arrival speed the landing chain is sized on, or the solve walks it to
"broadside all the way to the gate", which is the shortest arc and not a
landing.

A closed form will not do this. The equilibrium-glide range equation reads
**647 km where this vehicle flies 2317**: the arc is flown near circular
speed with centrifugal lift carrying it, nowhere near equilibrium.

Solving it turns the ignition point from an instant into a **window** -- ~4
degrees of orbit in the offline sweep -- because the switch trims inside it.

**And on this airframe the switch is nearly inert, which is the finding.**
Shallowest burn, sweeping the switch speed:

```
until=   0 m/s   arc = 2618 km      until=1500   arc = 2618 km
until= 600       arc = 2618 km      until=2000   arc = 2618 km
until=4500       arc = 2842 km      (never broadside)
```

Everything from 0 to 2000 m/s is the same trajectory. The airframe takes the
decision away: the holdable ceiling collapses from 90 to ~25 degrees between
500 Pa and 4 kPa -- about 45 km -- and the vehicle is still doing 2100-2400
m/s when it gets there. **The handover is a plant limit, not a policy**, and
the plant makes it at 45 km. Total authority in the parameter: 224 km of a
2618 km arc.

That is a statement about *this* vehicle, which has **no aerodynamic control
at all** (0 of 6 surfaces with an axis enabled, 15 kN m of reaction wheel
doing everything). On an airframe with working elevons the holdable band
reaches far deeper and the switch becomes a real control. The code is right
and the test vehicle is the degenerate case -- which is an argument for
`qs_plane_heavy` and for a craft file of one's own.

### What the drag is worth, and how it gears

| burn | arc at alpha 22 | arc at max drag | removed |
|---|---|---|---|
| 30 m/s | 2318 km | 1943 km | **375 km** |
| 60 | 1605 | 1529 | 76 |
| 90 | 1325 | 1290 | 35 |

It gears hard with shallowness, because shallow is what keeps the vehicle in
the band where the angle is free. Which is the same reason the two flags
belong to one design.

### What is general here and what is not

| piece | general? |
|---|---|
| bisection on "does this burn commit?" | yes -- monotone predicate, no constant |
| alpha bounded by `Holdable` | yes -- learned from what the airframe gives back |
| `ENTRY_ALPHA_CEILING_DEG` = 90 | yes -- broadside is broadside |
| the switch solved against the propagator | yes -- and it is why the burn and the entry agree |
| `HOLDABLE_PROBE` | prior only; live evidence overrides it |
| `DEORBIT_MAX_TIME_TO_GO_S` = 1500 | **no**, and it is what currently binds |
| `DEORBIT_SKIP_MARGIN_MS` = 2.0 | **no** -- a placeholder that names its replacement (the log's `cutoff drift`) |

## A second airframe, and the five things it found in an afternoon

The user built a Mk3 shuttle -- 30 parts, 37.5 t against the old craft's
6.7-14.5, seven Big-S control surfaces with **every axis enabled** against 0
of 6, two drain valves, best glide **L/D 5.91** against 3.06 -- and the
quicksave was copied into the farm as `qs_shuttle`. It took four flights to
get an entry out of it, and each failure was a quantity that had been right
*by inheritance* on one vehicle.

**None of these were findable on the old craft**, and none of them were
visible to 566 offline tests or to ~2800 flights. This section is the
argument for a second craft, made in numbers.

### 1. `Drain Mode` was never commanded

`ModuleResourceDrain` chooses between draining the part it is bolted to and
draining the whole vessel. The old craft was *saved* with it True, so the
autopilot never set it. The shuttle's valves came in False: both opened, both
drained their own empty parts, the tank never moved, and DRAIN re-entered
forever -- `drain watched down to 1750.0 units`, four times, unchanged.
Probed directly, toggling the mode empties the same tank in **two seconds**.

Fixed: `_drain_whole_vessel` reads the mode and sets it, toggling only as a
last resort -- a toggle applied to an unknown state is how you turn it off.

### 2. `Module.set_field_value` does not exist in this kRPC

It was the documented fallback in *both* drain paths, under the action names,
wrapped in `except Exception: continue`. It has therefore failed silently for
the life of the project, and nothing noticed because the actions always
worked on the one craft being flown. **A fallback that is never reached is a
fallback nobody has tested.** The setters this kRPC has are
`set_field_bool`, `set_field_int`, `set_field_float`, `set_field_string`.

### 3. Isp read from an unlit vessel is zero, and a floor is not a budget

`vessel.vacuum_specific_impulse` aggregates over *active* engines, so with
the throttle shut it reads 0 -- and the drain runs in vacuum before the burn,
which is exactly when it is 0. The old craft happened to report 355 there, so
`drain_reserve_units` ran its rocket equation and kept 96 units ("117 m/s of
burn"). The shuttle read 0, the budget collapsed to the 40-unit floor, the
drain dumped what the burn needed, and the deorbit **ran dry 9 m/s short of
its own solution** -- solved 26.2, delivered 17.4, speed pinned at 2050.1
with the engine commanded on (`logs/LOG2871`).

The log had been saying `at Isp 0` the whole time. Fixed:
`vehicle_vacuum_isp` asks the *engines* (Isp is a property of the engine, not
of whether it is lit), and a fallback to the floor now says so in capitals.
With the fix the drain keeps **430 units** instead of 30 and the burn
delivers 25.7 of 26.6 m/s.

### 4. `ATTITUDE_TIME_TO_PEAK_S` is worth 198 km on this airframe

The committed 3.0 s was fitted to the old craft, and its own comment says
what for: *"the controller is acting through a 2.4 s lateral mode"*. This
craft has no such mode. Flown as a bracket, one flight per instance,
`qs_shuttle`:

| `ATTITUDE_TIME_TO_PEAK_S` | aoa error | aoa sd | slip p-p | along |
|---|---|---|---|---|
| 1.0 | +34.2 | 14.1 | 68.2 | **-262 km** |
| 3.0 (committed) | +15.9 | 15.6 | 95.1 | -180 km |
| 6.0 | **-3.0** | **4.9** | 48.4 | **+17 km** |

**The mechanism is in the same logs as the outcome**, which is what makes
this more than the ordered dose-response failure 27 warns about: at 3.0 s the
vehicle flies 16-34 degrees of alpha *more than commanded*, with sideslip
swinging 95 degrees peak to peak at Mach 7, and `CdA` at 63-72 where the plan
assumed 45. The range error was never a guidance error. **The airframe was
not being pointed.**

Still n=1 an arm. It does not go near a default until it is replicated.

### 5. "RCS cannot help" was measured on a vehicle with no usable RCS

`GLIDE_RCS` is off because LOG1616 measured the whole monopropellant tank
buying 3 km of altitude. Read the torque line of that craft:

    old craft:  reaction wheel 15 kN m    RCS (0, 0, 0)
    shuttle:    reaction wheel 15 kN m    RCS (290112, 42625, 290134) N m

The shuttle flies the hypersonic glide on **the same 15 kN m of reaction
wheel as a craft a third its mass**, several times the inertia, at dynamic
pressures too low for the surfaces to bite -- with 290 kN m of RCS and 429
units of monopropellant switched off. `mono=419.9` does not move for the
whole glide. And the axis that wallows is **yaw**, which is exactly the weak
RCS axis (42.6 against 290).

Unflown. The general form is not a flag but a decision: compare the torque
available against what the airframe is failing to deliver, which the vehicle
already measures every tick.

### The tool this needed, which did not exist

`oscsum.py` -- alpha error, its spread, and peak-to-peak sideslip, per phase.
Nothing in the tree reported whether the vehicle was *pointed*; `landsum.py`
and `glidesum.py` would both have reported only the 200 km. Measured on the
two craft under the committed configuration:

| | aoa err | aoa sd | slip p-p |
|---|---|---|---|
| old craft, GLIDE | -1.5 | 2.8 | 9.2 |
| shuttle, GLIDE | +20.8 | 16.5 | **96.2** |

### And the fifth one had a general form, which is the point

The first reaction to `ATTITUDE_TIME_TO_PEAK_S` being worth 198 km was to
fly a batch at 9.0 s. That is not a fix -- it is a second airframe's trim,
and the third craft would need a third number. The user said so, and was
right.

**A constant in seconds cannot be general; the ratio to the vehicle's own
slew time can.** Angular acceleration is `tau / I`, so the time to swing a
commanded angle goes as `sqrt(I / tau)` -- and kRPC reports both terms
(`moment_of_inertia`, `available_torque`), one of which this autopilot has
been printing in STANDBY all along without anyone joining them up:

| | mass | I (pitch, roll, yaw) | slew, wheels | slew, +RCS |
|---|---|---|---|---|
| old craft | 14.5 t | (34931, 13452, 37195) | **1.57 s** | 1.57 s |
| shuttle | 37.5 t | (2068357, 94184, 2106086) | **11.85 s** | **2.63 s** |

**59x the pitch inertia for 2.6x the mass, on the identical 15 kN m of
reaction wheel.** So 3.0 s on the shuttle is a fifth of its own time scale,
which is the +16 degrees of alpha error and the 95 degree sideslip swing.

`ATTITUDE_TIME_TO_PEAK_DERIVED` computes `ATTITUDE_SLEW_FACTOR * sqrt(I/tau)`
**per axis** (first written on the slowest axis only; that over-slowed roll,
the bank control, and cost 37 km in `logs/LOG2879` -- see the 2026-09-22
evening handoff below; `moment_of_inertia` is `(pitch, roll, yaw)` and
`time_to_peak` wants `(pitch, yaw, roll)`). The
factor is 1.91, **read off the craft the constant was fitted to** (3.0 /
1.57), so the law reproduces committed behaviour on that airframe by
construction. That agreement is the reason to believe it and equally the
warning: a derivation that reproduces a fit says the fit was right for *that*
aircraft, and nothing yet about the next.

**What it predicts is testable and untested**: 22.6 s for the shuttle, above
the whole flown bracket (3.0 could not point the vehicle; 6, 9 and 12 all
could). The batch that tests the *law* is derived-against-committed on
**both** craft. The batch that merely fits it is 9.0 against 3.0 on one, and
that is the batch not to run.

**And the same two numbers say the constant was never the real fault.** With
RCS the shuttle's slew time is 2.63 s, near the old craft's 1.57 -- so 3.0 is
about right for this airframe *if it is allowed the actuators it carries*.
`GLIDE_RCS` is off because of a measurement on a craft whose RCS torque reads
`(0, 0, 0)`. Derive the tune from the actuators in use, or use the actuators
the vehicle has; it is one cause seen from two ends.

## Where the mechanisms have come from, counted

Worth recording because the pattern is now four for four and it should change
how a session is spent.

| mechanism | whose | outcome |
|---|---|---|
| raise the cone's bank | user | found a *double* saturation and the third clamp bug in this project |
| fly the approach at a sideslip | user | **-473 m and half the scatter**; an entire unused control axis |
| canards opposed to elevons | user | built; the split rudder's mechanism with the sideslip's currency |
| the split rudder | mine | weakest of the three brakes, and the only one that destroyed vehicles |
| `ENTRY_MAX_DRAG` / `DEORBIT_SHALLOWEST` | mine | correct, and inert on the airframe they were built against |

The two of mine are not *wrong* -- the drag work is right and found its home
the moment a second airframe existed -- but both were chosen from inside the
code, by asking which constant to vary. The user's four were chosen by asking
what the *vehicle* can do that nothing commands, and every one of them opened
an authority rather than tuning one.

**So when the user proposes a mechanism, build it before the thing already on
the list.** The hit rate is the evidence, and the reason is structural: a
search over `config.py` cannot find an authority that was never wired, and
that is exactly the class of thing this autopilot keeps turning out to be
short of.

## Session handoff, 2026-09-22 (evening): a second airframe, the derived attitude tune, and the slip not generalising

Moved here from CLAUDE.md verbatim; fingerprint at the time `b31b6887`.


### The result: there is a second airframe now, and it found five bugs in an afternoon

The user built a Mk3 shuttle and it is in the farm as **`qs_shuttle`**
(copied from `~/Kerbal Space Program/saves/default/quicksave.sfs`; all parts
stock, all present in `base/`). 30 parts, 37.5 t, **seven control surfaces
with every axis enabled** against the old craft's 0 of 6, best glide L/D
**5.91** against 3.06, two drain valves.

It found five things that were one airframe's trim, none of them visible to
566 offline tests or ~2800 flights. Four are fixed, one is a law:

| | what | state |
|---|---|---|
| 1 | `Drain Mode` never commanded -- right by inheritance on one craft | fixed |
| 2 | `Module.set_field_value` does not exist in this kRPC, and was the documented fallback in both drain paths | fixed |
| 3 | `vacuum_specific_impulse` is 0 on an unlit vessel, so the drain reserve took a floor and the burn **ran dry 9 m/s short of its own solution** | fixed |
| 4 | `ATTITUDE_TIME_TO_PEAK_S` is a constant in seconds and cannot be general | replaced by a law, below |
| 5 | `GLIDE_RCS` is off on the strength of a measurement taken on a craft whose RCS torque is `(0,0,0)` | open, unflown |

Full account: docs/spaceplane.md, "A second airframe, and the five things it
found in an afternoon".

### The attitude tune is now derived, and it beat every value I fitted by hand

`ATTITUDE_TIME_TO_PEAK_DERIVED` (default off) computes
`ATTITUDE_SLEW_FACTOR * sqrt(I / tau)` **per axis** from kRPC's
`moment_of_inertia` and `available_torque`.

    old craft  14.5 t  I=(34931, 13452, 37195)      -> (3.0, 1.8, 3.0) s
    shuttle    37.5 t  I=(2068357, 94184, 2106086)  -> (22.4, 4.8, 22.6) s

**59x the pitch inertia for 2.6x the mass, on the identical 15 kN m of
reaction wheel.** The factor 1.91 is read off the craft the constant was
fitted to (3.0 / 1.57), so the law reproduces committed behaviour there *by
construction* -- and in flight the old craft's derived value logged exactly
3.0 and landed +1151 m, an ordinary flight for it.

On the shuttle, attitude quality against the hand-flown bracket:

| `TIME_TO_PEAK` | aoa err | aoa sd | slip p-p |
|---|---|---|---|
| 3.0 (committed) | +15.9 | 15.6 | 95.1 |
| 6 / 9 / 12 | +4.0 / -1.5 / -0.6 | 10.6 / 5.4 / 5.0 | 71 / 54 / 53 |
| **22.6 (derived)** | **-0.6** | **1.4** | **24.8** |

The derived value is better than every number in the bracket and would have
been reached on the first flight instead of the eighth.

**Two cautions.** `available_torque` never reports aerodynamic surfaces, so
the derived slew time describes a wheels-only vehicle -- which matters the
moment `ENABLE_CONTROL_SURFACES` (below) is used. And getting the axis order
wrong costs 37 km: `moment_of_inertia` is `(pitch, roll, yaw)` and
`time_to_peak` wants `(pitch, yaw, roll)`; a single figure taken from the
slowest axis over-slows **roll**, which is the bank control and the entry's
only cross-track authority (`logs/LOG2879`, 37 km off the centreline with the
alpha tracked to 1.4 degrees of spread).

### High alpha: the question is answered, and the answer is "it depends on the craft"

`BROADSIDE_PROBE_DEG=90` on the shuttle (`logs/LOG2881`), read with the new
`./alphaceiling.py`, against the old craft's LOG1615 curve:

| q (Pa) | old craft | shuttle |
|---|---|---|
| 250-500 | 89.6 | 80.4 |
| 750-1000 | 71.1 | 45.3 |
| 1500-2000 | 43.9 | 38.5 |
| 2000-3000 | 36.3 | **50.3** |
| 4000-5000 | 25.6 | **60.4** |
| 8000-10000 | 19.4 | **47.4** |

Below ~2 kPa the shuttle holds *less* (59x the inertia, same wheel, air too
thin for surfaces). Above 2 kPa it holds *far more*, and that is the band
where `q * CdA * v` lives. **`ENTRY_MAX_DRAG` was never wrong, it was
homeless** -- the "switch is inert" finding is a property of the old craft.
One flight, small bins; the level is soft, the trend across six bins is not.

### Built and unflown, in the order I would fly them

1. **The combined brake** (`AIRBRAKE_OPPOSED_FLAPS`, `FLAP_BRAKE_PROBE_DEG`)
   -- **the user's mechanism, and fly it on the OLD craft first.** Canards
   deflected against elevons: the pitching moments cancel at the ratio of
   their area times arm, while both groups spoil lift *and* make drag. That
   is the currency the split rudder got wrong (drag alone spends speed, a
   glider dives to get it back, two vehicles died doing it) and sideslip got
   right (spoils lift at constant drag, spends height).
   `airbrake.find_all_brakes` arms the vertical pair **and** both horizontal
   groups -- they cancel on different axes so they compose -- and on
   `qs_plane`'s measured geometry that is **all six surfaces, 6.0 m^2,
   against a whole-craft `CdA` of 5.5**, with every control axis disabled so
   it costs nothing.
   `FLAP_BRAKE_PROBE_DEG` is the instrument and flies first, the way
   `SLIP_PROBE_DEG` did. It has to answer three things geometry cannot:
   does the moment actually cancel (`aoa=cmd/actual` across the deployment --
   the *behavioural* check the split rudder never had, because its
   verification summed `available_torque` and could not fail), what does it
   cost in `ClA` and give in `CdA` (`act=`), and is it therefore a lift
   spoiler or merely a brake.
   **Why the old craft and not the shuttle**: the shuttle's baseline `CdA`
   is 37.3 against the old craft's 5.5 at M 0.3 / alpha 8, so the same
   surfaces are worth ~6x less proportionally there -- and the old craft is
   the one with the unsolved +939 m overshoot.

2. **`ENABLE_CONTROL_SURFACES`** -- the old craft's six surfaces have pitch,
   yaw *and* roll disabled and kRPC exposes all three as writable. The
   autopilot already configures the vehicle it is handed (it locks gimbals,
   disables the nose brake, sets deploy angles, drives the drain valves);
   this is the same act. **It changes the plant and with it every constant
   measured on the old one** -- the stall, `ALPHA_TRACKING`,
   `HOLDABLE_PROBE`, `MARGIN`, the attitude tune (failure 23). Read it with
   `oscsum.py` and `alphaceiling.py`, not the arrival. Prediction: the alpha
   ceiling stops collapsing at 900 Pa and holds deeper, the way the
   shuttle's does above 2 kPa.
   **Note it interacts with (1)**: once the axes are on, the brake is no
   longer free -- it borrows from surfaces doing real work, as on the
   shuttle.

3. `SLIP_CROSS_PREDICT` (on by default inside the still-off
   `APPROACH_SLIP_FOR_ENERGY`) -- the slip's side is now taken from the
   cross-track predicted at the flare door, `cross + rate * time`, with
   `ApproachCommand` publishing `cross_rate` and `cross_time` so the two
   lateral authorities solve the same problem.

4. `HAC_LOAD_MODEL` -- the cone's radius is `v^2 / (n g sin(bank))` with `n`
   the load the wing can pay for (`airframe.turn_load`), not
   `v^2 / (g tan(bank))` which assumes a level turn the cone never flies.
   **This has to land before `HAC_BANK_MAX_DEG` is raised**, or the plan
   gets optimistic exactly where it is least affordable (1.33x too tight at
   45 degrees, 1.89x at 60, 2.8x at 70). The user has explicitly opened up
   raising that cap.

5. `ATTITUDE_TIME_TO_PEAK_DERIVED` on both craft, as a *law* test: the old
   craft is a null by construction and **that null is the test**.

### Refuted, or parked, and do not spend flights defending them

- **`DEORBIT_SHALLOWEST`** -- the bisection on "does this burn commit?" is
  sound (monotone predicate, no fitted constant) but on `qs_plane` it picks
  a *larger* burn than the committed rule, because its clock test is
  stricter than the one it replaces, and its first flight committed to a
  burn outside its own authority window and landed 484 km short. The window
  check is now in. Still default off and it has no proven value; leave it
  idle rather than feed it flights.
- **`ENTRY_MAX_DRAG`'s drag/lift switch is inert on the old craft** -- 0,
  600, 1500 and 2000 m/s all give the identical 2618 km arc, because the
  holdable ceiling collapses at ~45 km while the vehicle is still at
  2100-2400 m/s. That is a plant limit, not a policy. It should come alive
  on the shuttle; untested.
- The split rudder alone: superseded by (1).

### Method notes, paid for this session

- **I quoted failure 27's warning and then made the mistake.** A 1/3/6 s
  attitude bracket at n=1 an arm looked like a 198 km monotone gradient; the
  6.0 s arm then repeated at -59 km having given +17 km. **76 km of scatter
  on one configuration.** What survived was the *mechanism* -- attitude
  error measured within each flight over 100+ ticks -- not the ordered
  arrivals. Prefer a within-flight measurement to an outcome ordering.
- **I edited `config.py` while a batch was flying** and split it into two
  fingerprints (`f1916f8e` -> `7dafb344`). The first ten flights were
  discarded. The fingerprint caught it, which is the mechanism working; the
  fix is to develop in a copy of the tree (`cp -r spaceplane tests
  boosterland` somewhere, plus `test_instances/planeprobe-gearup.txt` which
  `fakeplane` reads) and apply the patch when the farm is free.
- **A batch died to memory pressure at 5.7 GB of zram** and I only read the
  number in the post-mortem. Watch it every round; it dropped to 712 MB the
  instant the farm stopped, so it is the farm accumulating and a restart
  fixes it.
- **The mechanisms that come from the user are four for four**, and the ones
  I chose from inside `config.py` are the weak ones. See docs/spaceplane.md,
  "Where the mechanisms have come from, counted". When the user proposes a
  mechanism, build it before whatever is already on the list.

**595 offline tests pass** (`python3 -m unittest discover -s tests`).

### New tools

- **`oscsum.py`** -- was the vehicle *pointed*? alpha error, its spread and
  peak-to-peak sideslip, per phase. Nothing else reported it: the shuttle
  wallowed **96 degrees** of sideslip at Mach 7 and `landsum.py` said only
  that it landed 200 km short.
- **`alphaceiling.py`** -- what angle of attack the airframe actually held,
  binned against `q`. Reproduces LOG1615's published curve exactly from its
  own log, so it is a like-for-like reader for any `BROADSIDE_PROBE_DEG`
  flight.

### The slip does not generalise, and that is this session's clearest result

`APPROACH_SLIP_FOR_ENERGY` was the tree's one measured win -- **-473 m and
half the scatter over 9 an arm on `qs_plane`** -- and default off only
because it had never been flown on a second entry state. It has now been.
`pairfly.sh`, `qs_plane_inc`, 4 rounds, 6 an arm, fresh farm, fingerprint
`7dafb344`:

| | n | mean along | sd | median | \|across\| med | on the runway |
|---|---|---|---|---|---|---|
| defaults | 6 | +497 | 596 | +354 | 3 m | 5 of 6 |
| `APPROACH_SLIP_FOR_ENERGY=True` | 6 | **+913** | **330** | +918 | 12 m | 3 of 6 |

**It lands later here, not earlier -- the opposite sign.** The difference is
+416 +- 278 (t ~ 1.5), so it is not significant either way; what matters is
that the -473 m does not reproduce. Two things *do* carry over: the scatter
halves (330 against 596, matching 215 against 456) and the lateral cost
appears (12 m against 3, matching 26 against 14).

So the mechanism is real -- sideslip spoils lift at constant drag, measured
end to end -- but **the benefit was a property of one entry state's energy
profile.** A lever worth 473 m on one save and nothing on another is a fitted
lever. It stays off. This is what a second entry state is for, and it is the
same shape as the five airframe constants above: something measured
convincingly on one configuration turning out to describe that configuration.

Read the touchdowns on that save as noise: 11 of 12 flights came apart on
contact, which is failure 81 and a craft problem, not guidance.

### Logs of that session

This session's logs are `logs/LOG2868-2903`; the batch files are
`logs/pairfly-slip-inc.txt` (discarded -- config edited mid-batch) and
`logs/pairfly-slip-inc2.txt` (the result above).

## Session, 2026-09-23: the surfaces were never off, and the audit that found it

### The old craft's control surfaces are live, and always were

The 2026-09-21 handoff's headline -- "this craft has no aerodynamic control
at all, 0 of 6 surfaces with any axis enabled" -- is **false**, and a lot of
reasoning since was built on it. What is actually true, measured this
session:

- The save has `ignorePitch = False`, `ignoreYaw = False`,
  `ignoreRoll = False` and `authorityLimiter = 150` on **all six** surfaces
  of `qs_plane` and all seven of `qs_shuttle`. kRPC's typed
  `ControlSurface.pitch_enabled` reads True on every one.
- In flight (`qs_cone`, ~10-15 kPa) `available_control_surface_torque` is
  **95-276 kN m** in pitch -- 6-18x the 15 kN m of wheels.
- The behavioural check, with the reaction wheels **off**: a pitch input of
  +1 against -1 differs by **0.14 rad/s after one second**; with the axes
  explicitly disabled the two inputs give **identical** rates (-0.1005 both).
  The surfaces have authority, and switching them off removes it.

How the error happened, since it is a trap for the next probe:

1. KSP's part menu shows the *ignore* flags under the names "Pitch", "Yaw",
   "Roll", so `Pitch=False` in `Module.fields` means **active**. Both craft
   read identically there -- the "0 of 6 against 7 of 7" contrast was never
   in the data.
2. `available_torque` was read on the pad or in vacuum, where q=0 and every
   surface reports zero.
3. kRPC's `ControlSurface.deflection` reads **0.00 at all times** under this
   install's `SyncModuleControlSurface`, even at 15 kPa with full input -- so
   the one per-surface number that could have disagreed could not.
4. The "Authority Limiter=30" (old) / "37.5" (shuttle) in the module fields
   is the deflection in *degrees* (150% x 20 deg, 150% x 25 deg); the limiter
   is already at its maximum.

**What it changes.**

- `ENABLE_CONTROL_SURFACES` is a no-op on both craft. Drop it from the queue.
- The alpha ceiling falling with q (LOG1615) is not "a constant wheel losing
  to a moment that grows with q". Surfaces and airframe moments both scale
  with q; the wheels do not. So at low q the wheels dominate and the ceiling
  is high, and at high q it asymptotes to the surfaces' **trim limit** --
  about 20 deg on the old craft, 50-60 deg on the shuttle, which is exactly
  the shape `alphaceiling.py` shows on both. It is a property of the surface
  geometry and deflection range, which is why it is craft-specific.
- **The opposed-flap brake is not free.** A deployed surface's deploy angle
  and its control deflection share one travel; deploying all six borrows
  from the only aerodynamic control the vehicle has. The `FLAP_BRAKE_PROBE_DEG`
  flight still answers the question, but `aoa=cmd/actual` degrading under
  deployment is now the *expected* failure, not a surprise.
- The shuttle's and the old craft's attitude behaviour differ for inertia and
  surface-geometry reasons, not because one of them has surfaces.

### The rollout's "+939 m overshoot" is mostly the brake law's own aim

`guidance.brake_fraction` brakes to stop `ROLLOUT_STOP_RESERVE_M` (300 m)
short of the far end, and the runway is +-1200 m from the midpoint, so its
target stop is **+900 m**. The intact stops of `logs/pairfly-slip3.txt`
cluster at +811..+973. The number that describes where the *approach* puts
the vehicle is the **touchdown**, and it is late: 1558-2274 m down a 2400 m
runway on that batch (`rwy=` on the first ROLLOUT line). Read the touchdown,
not the stop, when judging the approach and flare; read the stop only for the
brake law.

A smaller point on the same law: `apply_brakes` writes
`wheel.brakes = 100 * fraction`, but kRPC's `Wheel.brakes` runs 0-200 and the
old craft ships at 200 (the shuttle at 50). Measured across ~210 rollout
intervals, the wheels give about **7-8 m/s^2 per unit fraction** at partial
braking, against the 11 the law assumes (`BRAKE_DECEL_FULL_M_S2`, taken
flat-out at the craft's 200%). The law is closed-loop on distance, so this
mostly delays the braking rather than moving the stop. Not changed.

### Missing-controls audit, as a tool and as a log line

`test_instances/actuators.py <instance> [save]` lists every part module's
fields, events and actions, the torque by source, and the kRPC auto-pilot's
tuning. Diffing the two craft's inventories is how the above was found.
Other differences it shows, none of them commanded by the autopilot:

| setting | old craft | shuttle | commanded? |
|---|---|---|---|
| main-gear `brakes` % | 200 | 50 | yes, overwritten by the brake law (0-100) |
| RCS master in the save | off | **on** | yes, by the RCS valve |
| nose-wheel steering | on | on | yes |
| main-gear steering | off | (no module) | no; mains do not steer on either |
| engine gimbal | free | free | locked at STANDBY |

The flight log now carries two new lines (from the patch applied after the
2026-09-23 shuttle batch): `actuators:` at STANDBY -- surfaces with axes on
and limiter, wheel brake % and steering, reaction wheels, RCS blocks and
master, engine thrust limits -- and `authority kN m ... at q=` on **every
phase change**, torque by source (wheel, RCS, surfaces, engine) beside the
dynamic pressure. The second is the line that would have prevented the
error above: the surfaces' authority is now in every log at real q.

`slew_time_scale` now reads `available_reaction_wheel_torque` by name. It
used `available_torque`, which includes RCS whenever RCS happens to be on at
the moment of the call; the shuttle's save has RCS on, and the derived tune
would have come out 4.5x quicker than the wheels it flies on if the
initialisation order ever changed.

### The shuttle's 100 km overshoot is its deorbit burn, and the missing control is the thrust limiter

First arrival flights of the per-axis derived attitude tune on `qs_shuttle`
(`logs/farmfly-shuttle-derived.txt`, LOG2906-2911). The pointing is now
good -- alpha error -1..+2 deg, sd 4.5-9 in GLIDE, against +15.9/15.6 under
the committed 3.0 s -- and **every flight arrived at the cone 100-125 km long
and 33-43 km to the right.** A consistent bias, not scatter.

The glide is not at fault: `act=` and `mdl=` agree tick by tick, the solve
predicts +86..+118 km from its first tick, and it flies the whole entry
pinned at alpha 32 (the cap) and bank 70 (the cap). The error is made before
the interface. The exit line says where:

    DEORBIT -> COAST range error +0 m, +1.87 m/s still owed, solved dv 27 m/s, delivered 24.8 m/s

**Six of six burns quit 1.9-2.1 m/s short.** The mechanism:

1. The Rhino gives **65 m/s^2** on 30.6 t (the old craft's Cheetah: 8.6). The
   27 m/s burn is four ticks, and the unbalanced thrust on a locked gimbal
   swings the nose 10-15 deg off retrograde (`aoa=180/165`).
2. While it realigns, throttle is held at zero.
3. The floor rule prices "one more floored tick" as `floor x accel x
   horizon`, and `horizon` is the time since the engine last *burned* --
   which now includes ten seconds of realignment. One floored tick "costs"
   13 m/s, more than twice what is owed, so the rule stops the burn.

On this craft 2 m/s is worth ~50 km of arrival (the window moved from
2148-2429 km at the solve to 2227-2554 at cutoff), against ~3 km per m/s on
the old craft -- which is why the same rule was harmless there.

**Two fixes, both default off until flown** (fingerprint `aa66a7c8`):

- `DEORBIT_MIN_BURN_S` -- the engines' **thrust limiter**, which nothing in
  the autopilot ever set, is set at commit so the burn is never shorter than
  this at full throttle: `limit = dv / (MIN_BURN_S x accel)`. At 3 s it is
  ~0.14 on the shuttle and **0.60 on the old craft** -- whose engine reads
  17.8 m/s^2 at the burn, not the 8.6 an older comment claims, so it is
  *not* inert there (I wrote "inert by construction" from the comment before
  a flight corrected it). Restored at cutoff.
- `DEORBIT_FLOOR_ON_TICK` -- the floor rule charges the length of one real
  tick, not the dead time since the engine last burned.

**Flown** (`logs/farmfly-shuttle-burnfix.txt`, LOG2912-2917,
`ATTITUDE_TIME_TO_PEAK_DERIVED` + both flags): every burn delivered its
solution to **0.00-0.04 m/s** (against 1.9-2.1 short), the gate sat inside the
glide's reach at cutoff, and the first round arrived at the cone **+23, +35,
+48 km** against +100..+125. **But the thrust limiter never fired in this
batch** -- at commit the engine is not yet lit, `max_accel` read 0 and the
first version returned without a word. So this batch measures
`DEORBIT_FLOOR_ON_TICK` alone. Fixed afterwards (applied on the first tick
the engine answers, and a failure to set it is logged); the limiter itself
is still unflown.

### A bank reversal taught the learner a false alpha ceiling

With the burn fixed, the shuttle's glide starts on target -- predicted
+374..+526 m for three minutes -- and then walks out to +62 km between 40 and
28 km altitude. The commanded alpha drops from 28 to 16-18 deg there, and the
event log says why:

    aoa=25.5/14.2 bank=+14.0 q=6609
    alpha ceiling -> 24.0 (commanded 24.5, achieving 8.2, q=7095 Pa, learned 24.0)
    alpha ceiling -> 22.0 (commanded 24.0, achieving 9.6, q=7478 Pa, learned 10.6)
    aoa=20.0/22.6 bank=-58.0 q=8085

A reversal from +70 through 0 to -58 deg of bank dips the achieved alpha from
31 to 8 while the vehicle rolls; three seconds later it is back at 22.6. But q
climbs fast through 7 kPa, so the reversal fills a whole `Holdable` bin by
itself, the bin's "highest saturated sample" is a transient, and denser air
"carries the lowest seen" -- the predictor believes 10.6 deg for the rest of
the entry and the guidance flies less drag than it could.

`HOLDABLE_SKIP_REVERSAL` (default off): don't feed the learner while
`guidance.bank_in_transit` says the lean is between the stops, nor for one
pitch `time_to_peak` after -- the controller's own settle time, so 3 s on the
old craft and 22 s on the shuttle with the derived tune, not a fitted number.
The old craft reverses often too, so this is **not** null there by
construction and needs its own pair.

### The shuttle's arrival, three arms of six

`qs_shuttle`, `ATTITUDE_TIME_TO_PEAK_DERIVED=True` in all three, arrival at
`GLIDE -> HAC` (`long=`):

| arm | n | arrival, km | mean | sd | batch file |
|---|---|---|---|---|---|
| derived tune alone | 6 | +100 +104 +108 +108 +118 +125 | **+111** | 9 | `farmfly-shuttle-derived.txt` |
| + `DEORBIT_FLOOR_ON_TICK` (limiter silently inert) | 6 | +15 +23 +34 +35 +40 +48 | **+33** | 12 | `farmfly-shuttle-burnfix.txt` |
| + thrust limiter working + `HOLDABLE_SKIP_REVERSAL` | 6 | +30 +31 +34 +35 +36 +39 | **+34** | **3.3** | `farmfly-shuttle-all.txt` |

The floor fix is the 78 km. The limiter and the reversal gate together
leave the mean where it was and cut the scatter by nearly four. The
reversal gate works as designed -- every flight of the third arm logs
`learned -` through the whole entry -- so the remaining bias is not the
learner.

**What the remaining +34 km is.** The glide holds its prediction at
+0.4..+0.5 km for the first 20 km of descent, then **one bank reversal
between 34 and 26 km (q 3-8 kPa) moves it to +20 km.** Mid-roll the achieved
alpha falls from 30 to 9.6 deg and the `ratchet_alpha` ceiling backs off from
30 to 20 in five seconds (that ratchet is separate from `Holdable`, and not
gated). At that q the surfaces give 2928 kN m of pitch against 15 of wheel,
and the attitude controller is tuned to the wheels' 22 s: a roll about the
velocity vector at 30 deg alpha swings the nose in pitch and yaw, on axes
tuned to be slow. `ATTITUDE_TIME_TO_PEAK_LIVE` (built, unflown) re-derives
the tune from the live torque once a game-second.

Also seen and not chased yet: in the cone the shuttle wallows +-10-13 deg of
sideslip on a ~4 s cycle with alpha achieved 6-29 against 1-9 commanded,
and runs out of height (`needed 3587, had 1997`). Same cause is the first
suspect. And the landing chain still sizes itself on the old craft's
`STALL_SPEED_M_S`, `HAC_LD` and `APPROACH_BEST_LD` (`AIRFRAME_DERIVED` is
off); the log says so on every flight (`airframe DISAGREES ... 41% out`).

### Why the old craft is coming apart on the runway again

Intact rate on `qs_plane` has slid from 14/16 (fingerprint `6debf0c5`) to
4/9 on last session's defaults arm and 0/4 on today's. Tabulated from the
first ROLLOUT line of 22 flights (LOG2847-2864, LOG2924-2932): **every
flight whose rollout opened at `brk=1.00` at ~47 m/s was destroyed or badly
damaged; every one that opened at `brk=0.00-0.09` stopped intact.** Full
brakes at that speed means `guidance.brake_fraction` saw little runway left,
i.e. a late touchdown -- and flat-out braking at that speed is the 83 kN
through two small gear legs that the brake law's own docstring says tears
the aircraft apart. So the late touchdown is now the *cause of loss*, not
just of a late stop, and the thing to measure on the landing is the
touchdown point (`rwy=` on the first ROLLOUT line). Not a result of this
session's flags: the defaults arm is the worse one.

### The three flags on the old craft: harmless

`pairfly.sh`, `qs_plane`, 4 rounds on 3 instances, defaults against
`DEORBIT_MIN_BURN_S=3.0;DEORBIT_FLOOR_ON_TICK=True;HOLDABLE_SKIP_REVERSAL=True`
(`logs/pairfly-oldcraft-fixes.txt`, fingerprint `d7db6186`). Arrival at
`GLIDE -> HAC`:

| arm | n | arrivals, m | mean |
|---|---|---|---|
| defaults | 6 | +213 +442 +447 +495 +497 +766 | +477 |
| three flags | 6 | +92 +479 +480 +491 +503 +510 | +426 |

Indistinguishable -- on `qs_plane` the arrival is pinned at the cone's
`GLIDE_RESERVE_M` saturation, so this pair can only show harm and shows
none. The landing outcomes of this batch are read in the section above
(late touchdown -> full brakes -> breakup), in both arms alike.

**Every orbit on disk, for the floor bug.** Scanning the exit line of every
log since LOG2000 without `DEORBIT_FLOOR_ON_TICK`: ~870 old-craft burns across
`qs_plane`, `qs_plane_inc`, `qs_e20..e80`, `qs_gear`, `qs_soft` -- **none**
quit more than 0.5 m/s short (mean owed 0.03-0.25 m/s, ~1 km of arrival on
that craft, which the cone absorbs). `qs_shuttle`: **15 of 15**, mean 1.52.
It is a thrust-to-weight bug and no entry state of the old craft could have
shown it.

### `ATTITUDE_TIME_TO_PEAK_LIVE`: flown, refuted as built

`qs_shuttle`, with the three fixes above (LOG2936-2941,
`logs/farmfly-shuttle-live.txt`): arrival **-26, -31, -43, -49, -50, -51 km (n=6, mean -42)** against +34
without it; GLIDE sideslip peak-to-peak **63-69 deg** against 23-27. The
`attitude retune` lines show why. The live total includes engine and RCS
torque (time_to_peak read **0.0** during the burn), and kRPC's
control-surface torque depends on the current deflection, so the yaw figure
**chatters 2.3 <-> 10 s** from one second to the next while roll sits at
0.3-0.5 s. A controller that quick in thick air drives the lateral mode the
committed 3.0 s was chosen to damp. The idea -- the controller's time scale
should follow the authority the air provides -- survives; this
implementation does not. A version worth flying would use wheels plus a
*smoothed* surface torque, exclude engine and RCS, and floor the result at
the lateral mode's own period.

`RATCHET_SKIP_REVERSAL` (built, flying as of LOG2942): the other half of the reversal
problem. `ratchet_alpha` backs its ceiling off 2 deg/s during the roll (30 ->
20 in five seconds at 3.4 kPa, LOG2918) and recovers at 0.5 deg/s, and the
solve, pinned at bank 70, could not win back the +22 km. It skips the
back-off inside the same window `HOLDABLE_SKIP_REVERSAL` marks.

### `RATCHET_SKIP_REVERSAL`: the shuttle reaches the cone

`qs_shuttle`, derived tune + `DEORBIT_MIN_BURN_S=3` + `DEORBIT_FLOOR_ON_TICK`
+ `HOLDABLE_SKIP_REVERSAL` + `RATCHET_SKIP_REVERSAL`
(`logs/farmfly-shuttle-ratchet.txt`, fingerprint `0894b2b0`): arrival at the
cone **+1.9, +3.5, +3.2 km** (round one) against +34 without the ratchet gate
and +111 at the start of the session. The glide now holds its +500 m target
*through* the reversals, at ~47 deg of bank -- with authority in hand rather
than pinned at 70 -- and drifts to +1.8 km only in the last 20 km of altitude.

**Which moves the shuttle's frontier to the landing chain**, flown for the
first time from a reachable arrival (LOG2942-2944):

- the cone runs **out of height**: `needed 2724-3514, h=1998`;
- the flare opens at **28-53 m/s of sink** (the old craft: ~5);
- `landsum.py`'s rollout-to-wheels ratio reads **9.2**, where
  `APPROACH_BEST_LD` is the old craft's 4.2 (the log's `airframe DISAGREES`
  line says 5.91 from the swept table).

Every one of those is sized on the old airframe (`HAC_LD`, `STALL_SPEED_M_S`,
`APPROACH_BEST_LD`, with `AIRFRAME_DERIVED` off), plus the cone's sideslip
wallow noted above.

### Across orbits: two new shuttle entry states

`savegen.py --source qs_shuttle -o qs_shuttle_inc --normal 100` and
`-o qs_shuttle_high --prograde 80`, the same recipe as the old craft's
`qs_plane_inc` / `qs_plane_high`; md5 identical on ksp0-2. Best configuration
(derived tune + the four flags), arrival at the cone
(`logs/ladder-shuttle-breadth.txt`, `farmfly-shuttle-inc.txt`):

| save | n | arrivals, km | mean |
|---|---|---|---|
| `qs_shuttle` | 8 | +1.6 +1.9 +3.2 +3.5 +3.5 +3.6 +3.8 +4.5 | **+3.2** |
| `qs_shuttle_inc` | 4 | +2.4 +3.2 +3.6 +4.2 | **+3.4** |
| `qs_shuttle_high` | 2 | +6.1 +11.6 | **+8.9** |

Every burn closed to 0.00-0.01 m/s, including the high save's 66 m/s. The
inclined orbit is indistinguishable from the base one. The high orbit is
the one still out: its glide holds +0.5 km through the reversals and then
drifts to +8..+11 km below ~26 km altitude with alpha at 32 and bank at 70
-- **both caps** -- reversing on the cross-track every ~15 s. (I first read
it as "only 42-50 deg of bank"; those were mid-reversal samples.) So the
high-energy entry is beyond the glide's authority, the same shape as the
old craft's `qs_e60`, which needed `DEORBIT_CENTRE_BIAS_M` to move the aim
(failures 88-89). Compute that knob's gain on this craft before trying it.
**But read this first:** at the solve the window said the gate *was*
reachable -- `the glide can reach 2217 to 2303 km (86 km wide); the gate is
at 2267, +16% off centre` -- and both flights arrived 6-11 km long anyway,
and the post-cutoff window line returned `no answer`. So on this state the
reach model is optimistic by at least the arrival, or the reversals spend
the margin it counted on. n=2; a lead, not a finding.

`ladder.py` note: if any arm carries settings, separate arms with `;` --
with commas, `af:save=qs_shuttle,AIRFRAME_DERIVED=True` became two arms, the
second a save that does not exist, so the `AIRFRAME_DERIVED` probe on the
shuttle is **still unflown**. The help text now says so. And `farmfly.sh`
refuses to start while any autopilot is flying, so three single-instance
farmflys cannot share the farm: use `ladder.py` or `pairfly.sh`.

### `RATCHET_SKIP_REVERSAL` on the old craft: harmless

`pairfly.sh`, `qs_plane_inc`, 4 rounds, the three cleared flags in both arms,
`RATCHET_SKIP_REVERSAL` in B (`logs/pairfly-oldcraft-ratchet.txt`,
fingerprint `0894b2b0`). Arrival at the cone:

| arm | n | arrivals, m | mean | sd |
|---|---|---|---|---|
| three flags | 6 | -2153 -542 -534 -184 -22 +86 | -558 | 751 |
| + ratchet gate | 6 | -1783 -532 -266 +50 +152 +303 | -346 | 699 |

A difference of +212 against a standard error of ~420: nothing, which is
what an old craft whose reversals do not dip its alpha should show. The
landings are failure 81 unchanged -- 11 of 12 destroyed on contact, one
with 3 parts left, both arms, the same as last session's inc batch.
(I first tabulated these as 12/12 intact by reading the wrong column; the
per-flight lines say `0 parts`.)

**So all four shuttle fixes are now cleared on the old craft** -- the three
on `qs_plane` (arrival +477 vs +426) and the ratchet gate on `qs_plane_inc`
-- and on the shuttle together they take the arrival from +111 km to +3.2
(n=8), +3.4 on a second orbit (n=4). They are the candidates to become
defaults. `ATTITUDE_TIME_TO_PEAK_DERIVED` is still off by default too; on
the old craft it reproduces 3.0 s on pitch and yaw by construction, and it
is part of every shuttle arm above.

### Promoted to defaults, 2026-09-23

After each was flown and cleared on both craft, five flags are now the
committed configuration (fingerprint `83c8c65f`; `7521c020` had the first
four):

| flag | shuttle | old craft |
|---|---|---|
| `ATTITUDE_TIME_TO_PEAK_DERIVED` | precondition: at 3.0 s it cannot be pointed | null by construction, flown: -697 vs -686 (qs_plane_inc, n=6) |
| `DEORBIT_FLOOR_ON_TICK` | +111 -> +33 km | null (qs_plane, n=6) |
| `DEORBIT_MIN_BURN_S = 3` | limiter 0.14; sd 12 -> 3.3 km with the next | limiter 0.60; null |
| `HOLDABLE_SKIP_REVERSAL` | (with the above) | null |
| `RATCHET_SKIP_REVERSAL` | +34 -> +3.4 km | null (qs_plane_inc, n=6) |

The derived tune is per axis, so on the old craft roll is 1.8 s where the
constant gave 3.0; the pair above is what says that is harmless.

### The opposed-flap brake (the user's mechanism), probed

`FLAP_BRAKE_PROBE_DEG` 0 / 8 / 15 on `qs_plane`, two flights each
(`logs/ladder-flapprobe.txt`, LOG2980-2985, fingerprint `83c8c65f`). All six
surfaces arm (the vertical pair plus canards at x0.47 against the elevons,
6.0 m^2). Read within flight, by Mach band and *achieved* alpha, medians of
`act=`:

| Mach | alpha | control ClA / CdA / L/D | 8 deg | 15 deg |
|---|---|---|---|---|
| < 0.6 | 12 | 22.4 / 9.3 / **2.41** | -- | 18.6 / 8.9 / **2.11** |
| 4-8 | 20 | 5.2 / 6.2 / 0.84 | 6.3 / 4.5 / 1.42 | 5.9 / 4.4 / 1.34 |
| 4-8 | 24 | 6.3 / 7.2 / 0.88 | 7.4 / 5.7 / 1.30 | 7.0 / 5.5 / 1.27 |
| 4-8 | 32 | 9.0 / 10.9 / 0.83 | 9.3 / 10.0 / 0.93 | 9.0 / 10.3 / 0.87 |

1. **The moment cancels.** GLIDE alpha error -2.0 vs -1.4, spread 3.0 vs 2.8,
   sideslip p-p unchanged: canards against elevons at the computed ratio do
   not upset the vehicle -- the behavioural check the split rudder never had.
2. **Subsonically it is a lift spoiler**, exactly the currency the user
   argued for: at alpha 12 it costs 17% of ClA and 4% of CdA, L/D 2.41 ->
   2.11. The same shape as sideslip (1.64 -> 1.23 at 15 deg), with no
   lateral bill.
3. **Hypersonically it is the opposite** -- L/D *rises* 0.88 -> 1.27 at alpha
   24 -- so it must never be out during the entry. Holding it from COAST is
   why all four probe flights arrived 2-6 km short or worse and broke up; that
   is the instrument, not the brake.

So the brake belongs on the approach only, which is what
`AIRBRAKE_OPPOSED_FLAPS` does (deployed when the approach's S-turn authority
saturates). That is the arm to fly, against defaults on `qs_plane`, read on
the **touchdown point** -- see "Why the old craft is coming apart".

### The opposed flaps as a law: the first thing to move the touchdown

`AIRBRAKE_OPPOSED_FLAPS` was **disconnected**: it armed the flap brake and
`command_airbrake` returned on "no split-rudder pair", so nothing but the
probe ever deployed it (and the flare's stow backstop only stowed the pair).
Wired, tested, then flown -- `pairfly.sh`, `qs_plane`, 4 rounds, defaults
against `AIRBRAKE_OPPOSED_FLAPS=True` (`logs/pairfly-flapbrake.txt`,
fingerprint `83c8c65f`):

| arm | touchdown, m down the 2400 m runway | flare-door sink, m/s | intact |
|---|---|---|---|
| defaults | 1635 1650 1692 1706 1969 1978 (mean ~1770) | 29-36 | **6 of 6** |
| flap brake | 320 695 975 1478 2485 (one lost before contact) (mean ~1190) | **38-58** | **3 of 6** |

It deploys as designed -- `airbrake out at 1222 m: S-turn saturated 75%
with +286 m left`, back in when the speed guard fires -- and it moves the
touchdown **~600 m earlier**, where the aim, the S-turn stop, the brake law
and the gear spring (failures 68, 82-84) never moved it at all. The user's
mechanism is right. But the steeper path reaches the flare door with more
sink than the flare can arrest, and half the vehicles broke up.

The flare's own schedule says how much it can take: `sqrt(td^2 + 2 (L-1) g
h)` at its door `h = FLARE_ALT_M + FLARE_LEAD_S x sink`, whose fixed point on
the committed constants is **~39 m/s** -- the unbraked 29-36 is inside it,
the braked 38-58 is not. `AIRBRAKE_SINK_GUARD` (default off) stows the
brake whenever the sink exceeds that, every number the flare's.

**`AIRBRAKE_SINK_GUARD` as built is a no-op, and the reason reframes the
problem.** Two rounds (`logs/pairfly-flapguard.txt`, stopped between rounds
3 and 4 with nothing in flight): the guarded arm never deployed. The
*unbraked* approach already sinks 46-49 m/s at 1-2 km and only shallows to
~30 at the door, so a guard on the current sink is always tripped. And the
unguarded arm's worst flights did not need a long deployment to go wrong:
LOG2987 had the brake out for **three seconds** (1222 -> 1101 m, retracted by
the speed guard) and still reached the door at 42.7 m/s. That is not the
brake steepening the path; it is `APPROACH_SPEED_PATH` **diving to recover
the speed the brake took** -- the split rudder's failure (LOG2825), smaller.
So the next design is about how the brake and the speed loop share the
surplus -- e.g. deploy only while speed is *above* target by a margin the
brake can spend, or feed the brake's drag into the speed law's
`dv/dt = g sin(theta) - D/m` so the loop does not read it as a deficit --
not a sink threshold. Left off; the guard stays in the tree, off, with this
account beside it.

### `AIRFRAME_DERIVED` on the shuttle: refuted

`pairfly.sh`, `qs_shuttle`, 2 rounds, committed defaults against
`AIRFRAME_DERIVED=True` (`logs/pairfly-shuttle-airframe.txt`, LOG3007-3012):

| arm | cone arrival, km | flare entry, speed / sink | parts left |
|---|---|---|---|
| defaults | +3.9 +5.7 +6.2 | 50-126 / 20-68 m/s | 18, 11, 0 |
| `AIRFRAME_DERIVED` | +8.4 +10.1 +10.6 | **132-135 / 77-94** | 0, 1, 0 |

Sizing the cone and the stall off the shuttle's own table makes both the
arrival and the approach worse. The shuttle's landing chain -- out of height
in the cone on every flight, reaching the flare at 20-94 m/s of sink -- is
the frontier on that airframe and wants a design session flown from a
repeatable final-approach save (`entrysave.py --alt 3000`), not a constant
swap.

### Housekeeping from this session

- `STATE_FROZEN_S` (default 20): LOG3009 broke up in the flare, kept one
  fragment, and kRPC returned the same position at 124.9 m/s for 2800
  game-seconds -- 50 minutes of an instance in a phase with no clock.
  `frozen_early` ends such a flight. Final fingerprint **`64268a58`**.
- `farmfly.sh` refuses to start beside any flying autopilot (by design), and
  it writes `BATCH DONE` to its `OUT` file, not stdout -- a queue trigger
  that watches stdout waits forever. Use `ladder.py` (arms separated by `;`
  when they carry settings) or `pairfly.sh` to share the farm.
- `pgrep -f <pattern>` inside a wait loop matches the loop's own command
  line; test with `ps -eo args | grep "^python3 ./ladder.py"` or a file.

Logs of the session: LOG2906-3012. Batch files: `farmfly-shuttle-*.txt`,
`pairfly-oldcraft-{fixes,ratchet,derived}.txt`, `ladder-shuttle-breadth.txt`,
`ladder-flapprobe.txt`, `pairfly-flapbrake.txt`, `pairfly-flapguard.txt`,
`pairfly-shuttle-airframe.txt`.
