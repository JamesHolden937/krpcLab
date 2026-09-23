# Booster: control design

How the boosterland autopilot is put together and why each piece is shaped the
way it is. Read this before changing guidance, the propagator, or a phase.
Numbered failures cited here are in [booster-failures.md](booster-failures.md).

## Architecture

```
run.sh -> boosterland.autoland (phase machine, the only kRPC-aware control code)
            |-- environment.py  cached world data (density/sound tables,
            |                     the Cd*A-against-Mach curve, the lift
            |                     coefficient, omega)
            |-- proximity.py    nearby craft; when it is safe to flip
            |-- telemetry.py    every per-tick value, via kRPC streams
            |-- trajectory.py   local propagator + landing prediction
            |-- guidance.py     steering/throttle laws (pure functions)
            |-- gui.py          in-game panel, START / TERMINATE buttons
            `-- logbook.py      numbered log files, ut-gated
```

**Guidance acts on the miss in two places.** Boostback removes the bulk of it;
CORRECTION trims what the coast re-opens. The landing burn used to divert the
remainder and no longer does — that landed the booster sideways (failure 5).
Whatever miss survives CORRECTION is the miss you get.

**One prediction drives everything.** Each tick propagates the current state to
touchdown and every phase decision — burn direction, throttle, when to stop
boostback, when to light the landing burn, which way to divert — comes from
that fresh prediction. There is no stored plan that can go stale.

**The prediction is integrated with RK4, and the step is load-bearing.**
`predict_landing` steps with `trajectory._rk4` (`PREDICT_RK4`). Drag goes as
v^2 through a density that changes by a factor of e every 5 km, so a
first-order step over a hundred-second descent *biases* the answer rather than
adding noise — and boostback stops when the biased answer says it is on the
pad. That bias was 60-100 m of understated overshoot, which is what every
`AIM_BIAS_EAST_M` this project ever fitted was cancelling. See failure 16, and
do not trade the step back for CPU: 2.4 ms a propagation on a loop that sleeps
50 ms.

**The predictor flies the landing burn.** `predict_landing` propagates
ballistically to where the suicide burn has to start, then keeps integrating
with the engines on — `_fly_landing_burn` thrusts anti-velocity on the same
profile the booster tracks, under the same gravity and drag, down to the
ground. Braking sheds horizontal speed, so the answer lands *short* of the
ballistic impact point, by kilometres for a fast booster; boostback aiming at
a ballistic impact point would systematically overshoot.

Two things keep the prediction and the vehicle from disagreeing, both
load-bearing. `landing_burn_state` supplies the propagator's hand-over point
*and* the suicide-burn trigger, so the burn is predicted to start where it
really will. `trajectory.landing_command` is the throttle law itself, called by
the propagator and wrapped by `guidance.landing_throttle` for the control loop,
so the burn is predicted to be flown the way it will be. The closed form it
replaced — horizontal speed times half of `speed / net` — assumed a
full-throttle stop with no drag and no gravity turn and under-stated the creep
by hundreds of metres: across the sweep grid, 121 m mean / 459 m worst against
84/152 for the integrated burn. The burn costs a couple of hundred integration
steps and no kRPC calls.

**Remote calls are kept out of the integration loop.** A propagation is
hundreds of steps. `Environment` samples the density profile once at startup
into an interpolated table, samples the local speed of sound alongside it
(`c = sqrt(1.4 P / rho)`, from `pressure_at`/`density_at`, so no gas constant
and no temperature model), and keeps `Cd*A` as a local table probed with
`simulate_aerodynamic_force_at`. Integration itself is pure local Python.

**Cd\*A is a curve against Mach, not a number.** Cd has a transonic hump and
one propagation runs the booster from Mach 3 to Mach 0 through all of it:
LOG15 measured 15 m^2 subsonic, 50 m^2 near Mach 1.2 and 20 m^2 at Mach 2.5 on
the same vehicle in the same flight. Held at whatever the current Mach says,
the propagation is wrong in whichever direction the vehicle is about to move
(failure 8). `Environment` bins Mach 0 to `DRAG_MACH_MAX` into
`DRAG_CURVE_BINS` and `drag_area_at(speed, altitude)` interpolates between
filled bins, holding the end values rather than extrapolating.

The curve is **re-swept once the vehicle is flying the descent**, and
repeatedly after that: every bin probed during boostback was probed broadside,
and a side-on area used to fly a nose-on descent is what failure 13 cost. See
`DRAG_RESWEEP_AOA_DEG` / `DRAG_RESWEEP_INTERVAL_S` and failure 14.

Probing stays cheap. `simulate_aerodynamic_force_at` takes the velocity to
evaluate at, so the *whole curve* can be probed from one position and attitude
— dividing the force by `0.5 rho v^2` removes both. The first refresh that
finds usable air sweeps every bin; after that each refresh probes two bins (the
current one and one round-robin), two kRPC calls a second, so the curve tracks
altitude and attitude. Each bin smooths independently (`DRAG_SMOOTHING`,
`DRAG_OUTLIER_RATIO`), so a real change with Mach is no longer mistaken for the
noise one scalar had to absorb.

`trajectory.drag_lookup` threads it through the propagator: it turns either an
explicit scalar or `env.drag_area_at` into a `(speed, altitude) -> Cd*A`
function, and `predict_landing(..., drag_area=X)` still pins a constant.

**Rotating-frame terms are measured, not assumed.** kRPC reference frames are
left-handed, so the sign of a cross product is not obvious.
`Environment._measure_omega` compares the vessel's velocity in the rotating and
non-rotating frames (`w x r`) and picks the rotation vector that matches. All
Coriolis/centrifugal terms use it, so the handedness convention cancels. Do not
replace it with a hard-coded `(0, rotational_speed, 0)`.

**Heights are terrain-relative at the legs.** `Snapshot.landing_height` is
`surface_altitude` minus the distance from the centre of mass to the bottom of
the bounding box. The landing-burn trigger, the throttle profile and touchdown
detection all use this height; the propagator still uses the pad radius,
because that is what it is aiming at.

A sensor can be poisoned, and one has ended four flights, so the result is
**clamped on both sides** against the pad radius — the one number no sensor can
break. Terrain can rise `TERRAIN_MAX_M` above the pad and fall no further below
it than the pad's own height over the datum (`target_radius -
equatorial_radius`, measured) plus `TERRAIN_BELOW_SEA_M`. The floor was paid
for by LOG4/5/6, the ceiling by LOG2/LOG3. **Do not go back to
`min(height_above_pad, legs_altitude)`** — that is only conservative when the
ground is *higher* than the pad, and the booster is over lower ground on every
flight that misses. The COM-to-feet offset is re-measured every tick until
`Vessel.bounding_box` returns a believable length (failure 4).

### Burning for the gimbal

CORRECTION holds full thrust until it is within `CORRECTION_ALIGN_DEG` of the
aim, which is right when the vehicle is mid-turn and wrong when it cannot get
there at all: LOG7 spent 36 s commanding an attitude 45-72° away, timed the
burn out having done nothing, and let the miss grow 341 -> 966 m. Grid fins and
RCS are not much authority against an airstream; the gimbal is, and it only
exists while something is coming out of the engines.

So when the error is large *and not closing* — `Autoland.alignment_rate` under
`CORRECTION_STUCK_RATE_DEG_S` (3 °/s) — the throttle goes to
`CORRECTION_GIMBAL_THROTTLE` (0.25) rather than zero, and the vehicle turns
under power. The thrust is inefficient by construction (`cos(align)` useful at
best); that is the trade being made deliberately.

**The rate test is the whole design.** Burning whenever the vehicle is merely
off-aim was measured and is worse everywhere — it spends propellant on a turn
RCS was already making. With the gate it fires only when genuinely stuck: in
the mid-coast-kick regression on a 5 °/s vehicle it fires for a single tick and
takes the landing from 55 m to 14 m, and on vehicles that turn well it never
fires at all. This is the one change here whose sign depends on the vehicle:

| vehicle | gimbal burn off | on |
|---|---|---|
| `gimbal_deg_s=25`, 45-8 °/s | 100 mean / 98 median | 99 / 85 |
| `gimbal_deg_s=25`, 10-5 °/s (sluggish) | 84 / 85 | 84 / 83 |
| no gimbal modelled | 62 / 58 | 78 / 77 |

The last row is a vehicle where thrust buys no authority at all, so the burn is
pure cost — that is what a false premise looks like, not an argument against
it. `fakeksp.Vessel(gimbal_deg_s=...)` adds turn rate in proportion to
throttle; the default of 0 keeps every other test measuring the cost only. The
fake has no aerodynamic torque, so its RCS is never overwhelmed — the failure
this exists for is one the sim cannot reproduce. Set
`CORRECTION_GIMBAL_THROTTLE` to 0 for the old hold-thrust behaviour.

### The RCS valve is a relay, not a phase property

RCS used to be switched on at START and left on for the whole flight. Every
phase does need thrusters at some point -- the flip has no gimbal, and a
booster falling through thick air has little authority on anything else -- but
needing them *in the phase* is not needing them *in the tick*, and the ticks
where the vehicle is already pointed where it was told outnumber the ones
where it is turning. `update_rcs` runs after the phase handler each tick (so
the error is measured against the attitude this tick commanded) and hands the
angle to the shared `boosterland.rcs.Valve`; `docs/spaceplane.md` under
"Monopropellant is spent on turns, not on holds" has the design and the
numbers behind the three constants.

Two booster-specific points. The error is taken against the *routed*
direction `aim` commanded, not the final aim -- with `FLIP_VIA_VERTICAL` a
turn going exactly to plan would otherwise read as a hundred degrees of
failure. And `RCS_Q_MAX_PA` is 0: unlike the spaceplane, this vehicle has no
control surfaces that take over in thick air.

### Big turns go over the top

kRPC's autopilot rotates along the shortest arc. For the 150°-plus turns in
this profile (the boostback flip, and boostback into COAST or CORRECTION) the
two vectors are near enough to opposite that the arc is a coin toss, and it can
swing the nose *down* through the airstream — engines into the flow, and the
half the vehicle has least authority to recover from.

`guidance.flip_waypoint` checks the arc for turns beyond `FLIP_VIA_VERTICAL_DEG`
(90°), and when the shortest one dives it commands the up-side point of the
*same* great circle first. The nose then passes as near the vertical as the two
vectors allow, and the routing releases itself once the rest of the turn is
unambiguous, because it re-decides from the vehicle's real attitude every tick
rather than holding state. It costs about 20° of extra travel on a 170° flip
and never leaves the plane of the two vectors.

In `fakeksp` this is a no-op on all four entry states (identical distances bar
two runs differing by 2 m): with the current phase attitudes the direct arc
already climbs. That is the point — it is a guard on the ambiguous case, and
the fake has no aerodynamic torque with which to punish the dive it prevents.
`FLIP_VIA_VERTICAL=False` turns it off.

### Roll is commanded, not left free

`Autoland.aim` sends every attitude command through `set_autopilot_attitude`,
which uses kRPC's `set_direction_and_up` — a nose direction *and* the vector to
roll the roof toward. With no roll command kRPC's autopilot only drives the
roll *rate* to zero: the booster keeps whatever roll the flip left it in and
drifts from there.

`guidance.roll_reference` picks that up vector. Every steering command here
lives in the plane of the trajectory — boostback aim, coast bias, correction
tilt — so the roof is rolled into that plane, putting the pitch axis where the
steering is: grid fins working in the plane they steer in, and legs and fins in
the same place relative to the airstream every flight. `ROLL_OFFSET_DEG` clocks
the roof off that plane if the fins are not on the roof axis; `ROLL_ALIGN=False`
goes back to nose-only steering.

The reference is `n x nose` (`n` = the trajectory-plane normal), **not** the
autopilot's default zenith reference, which is ill-defined exactly where the
landing burn puts the nose. Two sign traps, both paid for in the tests:

- kRPC frames are left-handed, so the cross product's sign is not obvious. It
  is *measured* against the zenith, as `Environment._measure_omega` does for
  the rotation vector. Do not assume a handedness.
- `n x nose` reverses when the nose does, and the nose sweeps through the
  zenith on final and through 180° during the flip. Re-deciding the side from
  the zenith every tick snaps the reference through 180° mid-flight and
  commands a barrel roll. The zenith only *seeds* the choice; after that the
  side is carried forward (`Autoland.roll_up`).
  `test_the_roll_is_commanded_and_held_all_flight` asserts the reference never
  turns further in a tick than the nose itself did.

`set_direction_and_up` is only on current kRPC servers;
`set_autopilot_attitude` falls back to `up_reference`/`target_roll` and then to
nose-only, because a booster that drifts in roll still lands.

### When it is safe to flip

SEPARATION used to be a stopwatch: drift `SEPARATION_COAST_S` (3 s), then turn.
Three seconds is the wrong guess in both directions, and being too long is not
abstract — at separation the booster is *receding from the pad* at several
hundred m/s, so every second held is downrange the boostback burn must undo.

`proximity.ProximityScan` asks the question directly. Every craft within
`CLEARANCE_RANGE_M` is reduced to a sphere about its own bounding box (a sphere
because the booster is about to rotate through 150 degrees, and it is the only
shape still correct once it has), and the booster's own motion is marched
forward against each. Gravity is common to both and drops out; what is left is
the relative velocity and the booster's thrust. The phase ends the first tick
nothing is in the way.

**The keep-out is capped at the gap the two craft are already sitting at**, and
this is the whole model (`keep_out_distance`). A second before separation the
booster and its upper stage were *one vehicle*, so their centres are always
closer than the sum of their half-lengths: the sphere-sum is violated at t=0 by
construction and stays violated however fast they separate. Trusting it means
never flipping, which is what LOG16 cost (failure 9). Since the two craft are
intact and not touching, that distance is demonstrably survivable; what is left
to avoid is getting *closer* than it. A stage closing on the booster, or a burn
aimed back at one, both still read as conflicts.

**Two futures are checked and the worse one decides**: coasting, and
accelerating at full throttle along the boostback aim. The flip is not
instantaneous, so for the first seconds the booster is still drifting on
separation velocity with its thrust pointed between the old attitude and the
new one — the coasting case covers that. The thrusting case catches the burn
driving the booster *into* what it just dropped.

They get **different horizons**, because of attitude. The drifting case assumes
nothing about where the booster points, so `CLEARANCE_HORIZON_S` can be ten
seconds. The thrusting case assumes the vehicle is *already* on the boostback
aim, which it is not, so `CLEARANCE_BURN_HORIZON_S` is 2 s. Extrapolating that
thrust further is expensive fantasy: at 17 m/s^2 a ten-second powered lookahead
condemns anything within 800 m behind the booster, and the phase can then only
time out.

**While it is holding, it turns.** With nothing in the way the phase ends on
its first tick and BOOSTBACK flies the whole flip under power, which is the
better order (see "Light, then flip"). But once the scan *is* holding, the
choice is an unpowered flip against no flip, and no flip is how LOG16 lost an
airbrake. So `run_separation` commands the boostback aim whenever `blocker is
not None`, `SEPARATION_PRETURN` or no.

`SEPARATION_MAX_COAST_S` (5 s) goes anyway. A stage that matches the booster's
velocity and sits where the burn is going never changes, and a booster that
never flips is lost for certain, where one that flips next to its upper stage
only probably is. The log says which happened.
`SEPARATION_CLEARANCE_SCAN=False` returns to the old timer. Vessel enumeration
is a remote call per vessel so the *candidate list* is re-read only every
`CLEARANCE_RESCAN_UT`; their positions are read every tick and must be, because
at better than a kilometre a second a two-second-old position is two kilometres
of clearance the booster does not have.

### Phases (`autoland.py`)

```
STANDBY -> SEPARATION -> BOOSTBACK -> COAST -> LANDING_BURN -> TOUCHDOWN
                                       ^  |
                                       +--+ CORRECTION
```

- **STANDBY** — idle until the in-game START button; then RCS on, action group
  `STARTUP_ACTION_GROUP` (2) fired, autopilot engaged holding current attitude
  and roll.
- **SEPARATION** — coast until the upper stage is out of the way, holding the
  separation attitude. The engines are off here whatever else happens, so this
  is the one part of the flip that could only be flown on RCS — hence
  `SEPARATION_PRETURN` (default off); see "Light, then flip".
- **BOOSTBACK** — aim opposite the horizontal miss vector; full throttle from
  the first tick, tapering under `BOOSTBACK_TAPER_DV`, so the gimbal flies the
  flip. `FLIP_UNDER_POWER` (default on) disables the old `FLIP_ALIGN_DEG` gate
  that held the throttle shut until within 25° of the aim. Ends when the
  predicted touchdown is inside `BOOSTBACK_TOLERANCE_M` (150 m), or on the
  `BOOSTBACK_MAX_BURN_S` guard.
- **COAST** — engines off, retrograde with a light bias toward the pad; watches
  `landing_burn_state` for the trigger. With `AERO_STEER` it instead flies a
  *solved* angle of attack and steers on the air, the only two-way actuator in
  the flight — see "Flying the coast as a wing".
- **CORRECTION** — a throttled-down boostback (boostback's dv/throttle law,
  capped at `CORRECTION_MAX_THROTTLE`) entered from COAST when the predicted
  miss exceeds `CORRECTION_ENTER_M` (60 m; `CORRECTION_EXIT_M` must stay well
  under it — the gate was 1200 while the prediction still walked through the
  coast (failure 14), then 300, then 60 once the burns stopped being quantised
  (failure 15)) while there is still altitude and flight time to use it. It
  steers with `guidance.correction_attitude`, **not** boostback's aim: a tilt
  off retrograde bounded by `CORRECTION_MAX_TILT_DEG`, because a falling
  booster cannot hold boostback's horizontal attitude in thick air (failure 6).
  When it is off the aim *and not getting there* it lights the engines at
  `CORRECTION_GIMBAL_THROTTLE` instead of waiting; see "Burning for the
  gimbal". Both entry and every tick are gated on
  `Autoland.correction_gradient` — the metres of miss a m/s of burn actually
  closes, measured by propagating twice — so a burn that would make things
  worse never happens. Returns to COAST under `CORRECTION_EXIT_M`, then holds
  off for `CORRECTION_COOLDOWN_S` so the phase cannot flap on prediction noise;
  `CORRECTION_MAX_BURNS` bounds the relights. Set it to 0 to disable.
- **LANDING_BURN** — drops the gear on the first tick (`GEAR_ON_LANDING_BURN`;
  `GEAR_DEPLOY_ALT_M` is only a backstop, and `GEAR_ACTION_GROUP` fires an
  extra group for legs not on the stock Gear group). Throttle is a
  constant-deceleration profile (feedforward + `LANDING_THROTTLE_KP` trim),
  tapering into a constant-rate final approach (`FINAL_APPROACH_RATE`) so the
  legs arrive at `TOUCHDOWN_SPEED`. It also *steers*
  (`LANDING_TERMINAL_GUIDANCE`, on): ZEM/ZEV on the propagator's predicted
  touchdown down to `LANDING_NULL_ALT_M`, then lateral-velocity nulling only —
  see "Aim, then stop".
- **TOUCHDOWN** — throttle zero, log the distance from the pad, exit.

Shutdown (GUI TERMINATE, Ctrl-C, or a crash) always runs `Autoland.shutdown`:
throttle to zero, autopilot off, SAS on, streams and panel removed, connection
closed. It is deliberately a hand-back-to-player state, not a save attempt.

## Aim, then stop

The landing burn steers, and what makes that safe is that it stops steering *at
a point on the trajectory rather than at a miss*.

`guidance.terminal_command` is ZEM/ZEV on the horizontal double integrator,
gains 6 and 4 out of the energy solution rather than tuned, closed on the
propagator's predicted touchdown (`LANDING_DIVERT_ON_PREDICTION`) and capped by
the throttle budget (`tilt_budget`, so a tilt never comes out of the braking).
All of that existed and was defaulted off, because when flown it **toppled the
booster** — and neither the budget nor the gains were the problem.
`LANDING_DIVERT_MIN_ALT_M` (15 m) simply *stopped* the steering, and whatever
lateral speed the aiming had built was still on the vehicle when the legs
arrived: 3.4 m/s at 16 m up, against 0.1 for a pure-retrograde burn.

So below `LANDING_NULL_ALT_M` (250 m) the position term is dropped and the same
law runs on its velocity term alone down to a 4 m floor. Aiming and stopping
are different jobs and the last stretch belongs to the second. That is failure
5's lesson one layer down, and it needed re-learning because a law which steers
to *both* zeros still leaves a cliff at the altitude where you switch it off.
The mechanism in a sim log's `hs=` column: 3.5, 5.0, **9.6** (aiming, building
lateral speed), 5.8, 0.7, 0.1, 0.0.

**The sim cannot judge the guard and says to remove it.** Four entry states in
atmosphere, distance from the pad:

| | 45k/25k | log7 | 30k/40k | 60k/10k | mean |
|---|---|---|---|---|---|
| no steering (old default) | 125 | 144 | 176 | 49 | 123 |
| steering, nulling below 250 m | 118 | 113 | 166 | 17 | **103** |
| steering, nulled only below 15 m | 103 | 83 | 152 | 8 | 86 |

All twelve arrive at ~4.0 m/s with lateral speed under a millimetre a second,
*including* the bottom row — `fakeksp` points instantly and has no aerodynamic
torque, so it cannot topple and therefore cannot charge for arriving tilted.
The bottom row is the configuration that was flown in game and broke the
booster. Take the 17 m of sim accuracy the guard costs as the price of a
failure the sim is structurally unable to show.

## Flying the coast as a wing

`AERO_STEER` steers the descent with an angle of attack instead of holding
retrograde. It is **on by default** and is the largest single win in the file.

The case for it is the shape of what failure 16 left behind. Every other
actuator is the engine: CORRECTION spends propellant and a relight, steers
within `CORRECTION_MAX_TILT_DEG` of retrograde and is gated against lengthening
the flight, and the landing burn thrusts straight anti-velocity. So the
guidance could only land the booster *shorter*, and an undershoot was a miss
nothing could touch — which is what `qs_steep` kept doing, 130-160 m short,
with the prediction saying so from 34 km with fuel aboard. Meanwhile the
booster spends a minute falling through air thick enough to decelerate it at
more than a g. Held a few degrees off retrograde that air is an actuator: it
works in **both** directions, costs no propellant and no relight, and acts for
the whole time the miss is already known.

**Nothing here assumes the vehicle has lift.** `Environment.probe_lift_slope`
measures it with the same `simulate_aerodynamic_force_at` call that measures
Cd*A — the perpendicular component this project had been discarding — and
`guidance.solve_steer` *solves* for the angle against the propagator rather
than applying a fitted gain. With no sideforce the slope stays empty, the
solver's conditioning test trips, and the vehicle holds retrograde
(`test_a_booster_with_no_wing_is_not_steered`).

**What the booster actually has**, from `testInstances/liftprobe.py` against
the real craft with no flight (the probe takes an attitude, so it can be asked
about angles the vehicle is not holding):

| altitude | speed | Cd*A | lateral accel at 5 deg |
|---|---|---|---|
| 3 km | 300 m/s | 16.9 | 1.20 m/s^2 |
| 3 km | 700 m/s | 16.3 | 4.67 m/s^2 |
| 8 km | 450 m/s | ~12 | 1.12 m/s^2 |
| 15 km | 700 m/s | 15.2 | 0.60 m/s^2 |

- **It is body lift, not the grid fins.** Deployed and stowed give identical
  sideforce to the last decimal; what the fins change is *drag*, 2.75 -> 16.9
  m^2. So probe the deployed configuration for Cd*A — the fins and airbrakes
  are animated, and probing the instant the action group fires measures a
  vehicle mid-deployment reading 1-3 m^2 against the 12-50 seen in flight.
  That is what `--settle` is for.
- **L/D is only ~0.04**, not the 0.3 the mid-deployment probe implied. Still
  ample: 0.4 m/s^2 through the last 40 s is ~300 m against a ~130 m error, and
  the effect is quadratic.
- **The force goes as `alpha * |alpha|`.** Per radian the sideforce varies
  eightfold between 2 and 20 deg; per radian *squared* it holds at -149 to
  -115. A cylinder in crossflow is not a wing with a linear lift curve, and a
  two-point linear solve under-commands small angles and overshoots large ones.
  Both the propagator's term and the solver's inversion are quadratic.

**The wing steers in two axes.** `solve_steer` originally solved only the
*downrange* miss, with the lift plane fixed to `cross(r, v)`. Read the `cross=`
column of any flight and that is half the error going unopposed — 15-28 m
across 20 flights, and *not moving* from 28 km to the ground. Nothing else can
touch it: CORRECTION's tilt is a downrange lever and the landing burn has ~10 m
of authority. On a 20 m pad that term alone is the whole budget.
`AERO_STEER_CROSS` probes in-plane and out-of-plane, builds the 2x2 sensitivity
in the quadratic variable the plant is linear in, and inverts it to null `long`
and `cross` together. The roll is stored as a **rotated normal** rather than a
lift direction, because `Steer` takes `cross(normal, v)` every step — a stored
direction would stop being perpendicular as the velocity turns. A badly
conditioned 2x2 falls back to the downrange axis alone.

**The solve checks its own answer, and that is load-bearing.** Near the ground
the sensitivity collapses, so an *already closed* miss divided by nearly
nothing comes back as the maximum angle: LOG399/LOG403 held 4.1-4.7 deg flat
from 32 km with the miss pinned at 0-5 m, then saturated over the last three
ticks and took the miss 0 -> -19 m in one step; LOG402 went -62 -> -91. That is
the solver manufacturing the error it can no longer fix. `guidance.verified`
propagates the commanded angle, compares against the do-nothing propagation
taken first, and keeps the command only if it lands *nearer* — halving it once
before giving up. It is free (that prediction is computed anyway) and is the
same shape as `correction_gradient` refusing a backfiring burn.

**The obvious guard is the wrong one and it was measured.** An altitude floor
below the knee in the `aoa=` column is exactly the cliff `LANDING_NULL_ALT_M`
exists to avoid one phase later, and it cost `quicksave` 13 m (5/5 -> 18/19),
because the last two kilometres do real work when they are not saturating.
`AERO_STEER_MIN_ALT_M` is left in at 0.

**Where the guidance stands.** 15 flights, five entry states, committed
defaults, `AIM_BIAS_*` at 0 so this is guidance error and not calibration:
`qs_cold` 3/2/2, `qs_hot` 2/3/4, `quicksave` 5/7/2, `qs_north` 6/6/4,
`qs_steep` 2/6/6 — **median 4 m, mean 4 m, worst 7 m**, every flight intact,
final `hs=` 1.7 m/s or less. For scale the configuration this replaces measured
median 26 / mean 57 / worst 162 at the same zeroed bias (failure 16), and
30/43/137 at its own fitted bias (failure 15). **The KSC pad is about 20 m
across, so this is the first configuration that lands on it rather than near
it**, and with no calibration constant — east residuals are +2 to +6 m across
all five states.

What got there, in order of what each was worth: the wing waiting for the
prediction to settle, the throttle-saturation family of failure 17 (the whole
of the tail — 1106 m, 1697 m, and a 77 m tip-over that emptied the tanks), the
two-axis wing (cross-track averaged 14 m with no actuator at all), and the
landing burn's velocity nulling (without which four boosters in seven arrived
on their sides).

The default-on measurement, five entry states, two flights a cell, bias 0:

| | median | mean | worst |
|---|---|---|---|
| `AERO_STEER=False` | 20 | 51 | 151 |
| wing below 25 km | **3** | **3** | **5** |
| wing below 15 km | **3** | **3** | **5** |

Per save, no wing -> wing: `quicksave` 25 -> 3, `qs_hot` 68 -> 3, `qs_cold`
14 -> 2, `qs_north` 15 -> 3, `qs_steep` **151 -> 4**. The two thresholds tying
exactly says the win is *not acting on the boostback-exit prediction* rather
than any particular altitude; 25 km is committed for having more flight left to
act with, which should matter on a booster with less lift.

**Waiting is the whole result, and steering immediately is worth -100 m.** The
first two-axis grid acted from the boostback exit and came out bimodal:
`qs_steep` 102 -> 3 and `quicksave` 71 -> 6, but `qs_north` 8 -> 107 and
`qs_cold` 8 -> 36, damage always to the *short* side. On `qs_north` the exit
predicts `long=-181`, the *unsteered* coast walks it to `-5`, and the booster
lands 13 m out — that -181 is not a miss, it is what is left of failure 13's
walk, which the re-sweep clears a few tens of seconds later. A wing that acts
on it faithfully steers out an error that was going to correct itself.
`AERO_STEER_MAX_ALT_M` is the fix and it is one comparison.

**CORRECTION had to learn the same lesson, and the symmetry is wrong.** Ten
flights at the defaults came in at 2-6 m and one at 112, whose burn ran at
25-27 km — above where the wing starts — on exactly the walk the wing declines
to chase (`miss=238` -> `795` in three seconds, cut by
`CORRECTION_ABORT_RATIO` as designed, but 795 m is more than the wing's ~300 m
of authority can undo). `CORRECTION_MAX_ALT_M` closes that window and is a
clear regression:

| save | ceiling off | ceiling at 25 km |
|---|---|---|
| `qs_north` | 4, 5 | **118, 141, 107** |
| `qs_cold` | 3, 3 | **42, 58, 20** |
| `quicksave` | 2, 6 | 5, 3, 13 |

The reason is authority: regressed flights go `coast entry -182 -> burn entry
-120`, good ones `-174 -> -16`. CORRECTION's early burns take the first bite
out of the ~180 m boostback lead — acting on an unsettled prediction and still
usefully, because the bulk of that lead is real — and the wing cannot close
180 m alone. One backfire in ten flights is cheaper than removing the phase, so
the knob is left in at 1e9 (off). **The wing waits because it is a trim and the
walk is the same size as its authority; CORRECTION does not, because the miss
it removes is ten times larger than the walk on top of it.**

Two things still worth watching in a log, from when this was n=1: whether the
vehicle holds the commanded angle in thick air (`aoa=` against what the
autopilot achieves), and whether a steered descent hands the landing burn any
lateral speed (failure 5 is what that column is for). One known limit of the
measurement: `simulate_aerodynamic_force_at` may not model control-surface
deflection, in which case the probe is a *lower* bound on the real authority
and the guidance is simply conservative — the loop re-solves every tick.

## Light, then flip

`logs/LOG9` flipped a 30 t booster at 19 km and 584 m/s with the engines shut,
because SEPARATION pre-turned and BOOSTBACK then held the throttle at zero
until `align < FLIP_ALIGN_DEG`. It wallowed through ~150° on reaction wheels
and RCS with no gimbal and aero torque fighting it. `plan.txt` asked for "full
throttle and quickly flip", which is also what Superheavy does, and for the
same reason — thrust is what makes the turn controllable.

So `FLIP_UNDER_POWER` (default on) lights the engines on the first BOOSTBACK
tick however far off the aim, and `SEPARATION_PRETURN` (default off) keeps the
whole turn inside the powered phase. Four entry states in atmosphere, distance
from pad (all TOUCHDOWN at ~4 m/s):

| slew | state | flip then light (old) | light then flip (default) | both |
|---|---|---|---|---|
| 30 °/s | LOG7 / 45k / 30k-40k / 60k-10k | 75 / 78 / 400 / 14 m | 78 / 6 / 138 / 39 m | 54 / 6 / 333 / 43 m |
| 15 °/s | " | 265 / 286 / 345 / 89 m | 62 / 13 / 116 / 445 m | 62 / 51 / 88 / 252 m |
| 8 °/s | " | 342 / 496 / 748 / 86 m | 115 / 24 / 18 / 381 m | 103 / 36 / 1 / 417 m |

The 60 km/10 km state prefers the old order at slow slew — it is nearly
straight up with little downrange to undo, so the pre-turn's saved seconds beat
the gimbal. `SEPARATION_PRETURN=True` gets the overlap back on top of the
powered flip (the "both" column); note that reintroduces an unpowered turn the
fake cannot punish, since it models no aerodynamic torque.
