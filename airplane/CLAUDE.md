# airplane

An attitude autopilot for winged craft that the player flies through: it
holds the attitude and the player's stick goes on top. The root `CLAUDE.md`
applies in full.

```
STANDBY -> FLY -> DONE        (./run.sh --pilot airplane; launcher "AIRPLANE: CRUISE / HOLD")
```

`autopilot.py` is the phase machine and the only kRPC-aware code;
`control.py` holds the laws as pure functions (`tests/testControl.py`).

**Panel** (the window mod): a CRUISE / HOLD choice, **prop plane**,
**constant rates**, and the pitch / roll / yaw rate limits in rev/min. All
of them act in flight; a mode change re-takes the attitude.

**CRUISE**: the rudder nulls sideslip (PI on slip into the yaw-rate
command), bank is held (never past `BANK_LIMIT_DEG` 89), pitch *attitude*
is held, and the turn is coordinated: the bank asks for the heading rate
`g tan(phi) / V` (bank's tangent held to `TURN_MAX_G` of load), which the
Euler kinematics turn into a pull and a yaw -- banked 45 at 30 deg up, it
pulls until 30 is held. Heading is free.

**HOLD**: pitch, heading and bank held where taken. Past `VERTICAL_DEG`
80 of pitch heading is undefined: it is re-taken every tick.

**How**: attitude error / (`ATTITUDE_TAU_FACTOR` x tau) = Euler rates,
into body rates (`control.euler_to_body`), capped at `MAX_*_RATE_DEG_S`;
each body rate flown by a PI loop on its stick axis (`control.RateLoop`).
The integrals are the axes' trims. Attitude and rates come from the
vessel's axes and angular velocity in its surface frame, read off as the
axes' own motion (`d(axis)/dt = w x axis`, the convention measured for the
spaceplane). Senses: pitch +1 nose up, roll +1 right, yaw +1 nose right.
SAS is turned off while flying and put back.

**No gain is fitted to a craft by hand; each axis is measured in flight.**
kRPC's `available_torque` / `moment_of_inertia` (wheels, RCS, gimbals and
the surfaces *at the present q and Mach*, re-read every
`AUTHORITY_REFRESH_S`, `control.authority`) is only the starting point:
it is a model, and on the Electric Warbird it reads roll 3.7x low and
pitch 4.4x high. `control.Plant` fits each axis's `w' = k accel u - d w +
c` (effect relative to kRPC's figure, the airframe's own rate damping, the
out-of-trim acceleration) by recursive least squares, forgetting over
`PLANT_FORGET_S` 10 s, on the input actually flown (ours + the player's
stick + the trim) against the rate's change over the next tick (the tick's
mean rate as regressor). Only the `PLANT_WINDOW_S` 1 s after an input step
of `PLANT_EXCITE` 0.01 is fitted -- in a steady hold the input moves only
as feedback and the fit would slide; the covariance is capped at
`PLANT_P_MAX` 100. What is used is trusted as far as the fit explains the
rate (`quality`, 0..1): `k^quality` and `quality x d`, so an axis the
per-axis model cannot explain (HOLD's yaw, cross-controlled against a
held bank) stays near kRPC's figure. `control.pi_gains` puts both poles of
the rate loop at `-1/tau` on the measured plant: `kp = (2/tau - d) /
effect` (>= 0), `ki = 1/(tau^2 effect)`. tau is a number of the loop's
own measured interval (>= 0.15 s) set by the phase margin the loop keeps
through its delay (`control.tau_ticks`: `RATE_PHASE_MARGIN_DEG` through
`LOOP_DELAY_TICKS` 1.5 -- read a tick before acting, the command held a
tick; this PI crosses over at omega tau = 2.06 with 76.3 deg before any
delay), and by the surfaces' travel (below). **The default margin, 32.12
deg, is the old 4 ticks**: 47 deg (6.0 ticks) is unmeasured on the farm
(see Where it stands). A tick longer than the mean closes
that tick's loop on the same number of it (`control.tick_tau`): gains
sized on the mean tick overshoot on a long one and the rate flips sign tick
to tick. No axis asks for more than full input for an error of its own
rate cap (`kp <= 1 / MAX_*_RATE`; held there, `ki` puts both poles at
`-effect kp / 2`): with almost no authority (the runway at walking pace)
the poles would ask for 2/(tau effect), and the ailerons and rudder
slammed side to side. Where the loop still rings,
`control.Ringing` halves that axis's gain (4 reversals of 0.15 in 2 s;
floor 1/8) and lets it recover at 0.05/s. This is not INDI (the
spaceplane's `SURFACE_OWN`, too noisy at kRPC's tick): the fit averages
hundreds of ticks and sets only the gains, never the command. Log `kp=`,
`acc=` (kRPC, deg/s^2 per unit input), `k=`, `d=`, `fitq=`, `ring=`,
`tau=`; events `authority ...` at START, `<axis> ringing: gain xN`.

**The surfaces take time to move** (the user, 2026-11-05). KSP slews a
surface's input and angle at `actuatorSpeed` deg/s over `ctrlSurfaceRange`
(`Mathf.MoveTowards` in `ModuleControlSurface`; a `Lerp` lag when
`useExponentialSpeed`): stock 25-40 deg/s over 10-60 deg, 0.3-2.4 s end to
end. kRPC reports neither, so the window mod's `SurfaceActuators` reads
them, with each surface's and the vessel's potential torque, every
`AUTHORITY_REFRESH_S`. Per axis (`control.actuators`): slew = full travel
per second, torque-weighted harmonically, wheels/RCS/gimbals instant; lag
of exponential surfaces. Then (`control.closing_time`) each axis's tau is
at least `sqrt(SLEW_FACTOR x cap / (accel x slew))` -- on **kRPC's** figure,
not the measured effect: with the measured effect roll's tau fell 0.37 ->
0.26 s and it rang (journal 2026-11-05 evening); this floor holds roll at
a closing time the per-axis model does not explain -- the surfaces must
reach the input a step of the axis's rate cap needs within half the
closing time -- and `LAG_FACTOR` x the lag; and `RateLoop` never moves the
command faster than the slew (its integral does not wind against it).
Offline (`TestSlowSurfaces`, a slewed plant with a tick of latency): sized
on the tick alone a 0.2 rad/s step overshot by up to 0.76 rad/s; with both,
at most 0.058 over accel 1-20 rad/s^2, slew 0.4-3 travel/s. Engine gimbal
response is not modelled (counted instant). Without the window mod (kspSim)
every axis is taken as instant. Log `tau=` per axis; event `surface
actuators` at START.

**On the ground** (kRPC's situation landed, splashed or pre-launch;
`control.on_ground`; events `lift-off` / `touchdown`, `gnd=` on the FLY
line): the wings are held level; the pitch is held where it sits with the
proportional term only (no trim: against a tail wheel it would wind
nose-up all down the runway and pitch the plane up at lift-off); the
heading is held in both modes by the rudder and the steering (CRUISE's
slip and turn are off); and only the yaw plant is fitted (the steering is
part of the yaw input's effect; the gear holds pitch and bank, which would
fit as axes that barely answer). Off the ground the pitch and heading
targets are re-taken where the take-off left them.

**Pitch stiffness** (`control.Stiffness`, always fitted; used only with
`ATTITUDE_TRIM`, **off**): the input holding an attitude costs per radian
of angle of attack (`g` = K/E, the airframe's static stability over its
control power; both scale with q), fitted on the ticks with the pitch
rate under `STIFF_STILL_DEG_S`, trusted once alpha has spread
`STIFF_SPREAD_DEG`. With the flag, the pitch trim also integrates the
attitude error at `g / (ATTITUDE_TAU_FACTOR tau)`: on a stiff airframe the
rate PI (sized on E alone) winds its trim in at ~1/(tau^3 E) and the
attitude sags (the KL Jet Trainer: nose 8 -> -4 deg after its rotation,
0.05 of up input against the 0.5 it needed, LOG15521). Offline
(`TestStiffness`): K/E 3, an 8 deg hold's error after 6 s 6.6 -> 0.1 deg,
no overshoot. Log `stiff=` (-1 until trusted).

**The player's stick**: kRPC adds a client's input to the player's
(`PilotAddon.OnFlyByWire`: `ControlInputs.Add`) and KSP folds the trim into
the stick (`FlightInputHandler`), so the player's input is the read-back
less what we wrote less the trim (`control.stick`). Past `STICK_DEADBAND`
on an axis for `STICK_CONFIRM_S` 0.15 s (past `STICK_FIRM` 0.2 at once)
the hold lets that axis go (keeps its trim) and, once the stick has been
back inside the deadband as long, holds whatever attitude the stick left
it at (`control.StickGate`). The live read-back flickers past the
deadband a tick at a time with the stick untouched (LOG15411: 834 one-tick
crossings in 400 s, against 3-6 a minute on the farm), and every crossing
used to let the hold go and re-take its target wherever the plane had
drifted. **Constant rates**: the stick commands a
body rate instead, full deflection = the panel's limit (RPM x 2 pi / 60
rad/s), and the loop flies it (our input = loop output less the stick), so
a full stick at high speed turns no faster than the limit; the holds' own
rates are capped by the same limits.

**Prop plane**: runs `propPitch.PropPilot` in the same process, ticked with
the airplane (its own LOG, headless panel; its pitch/rpm/mode on the
airplane's panel). Unticking it hands the props back.

**Log** `FLY` line: `att=`/`tgt=` bank, pitch, heading (deg), `slip=`,
`rate=`/`want=` pitch, roll, yaw rates (deg/s), `out=` our inputs, `i=` the
integrals, `stick=` the player's input per axis, `kp=`/`acc=`/`k=`/`d=`/
`fitq=`/`ring=`/`tau=` the gains and where they came from.

## Where it stands (2026-11-08, fingerprint after `a8e0e27`)

**Airframes:** the Electric Warbird, the Seahawk (stock Serenity twin
prop) and three generated stock jets (`tools/craftgen.py`: KL Jet Trainer,
KL Jet Delta, KL Jet Canard -- Panther, Mk1 fuselage, wingConnector3 wings;
`--check` launches and photographs them). Suborbital hopper ground-loops
on its own (-130 deg/s yaw against full rudder from 3 m/s: its four rotors'
torque, unchecked); Arrow-E cannot launch on the farm (no control source).

**Measured (one flight per cell -- screening, not a result):** the
Warbird's roll after a stick release rings +/-130 deg/s at ~1.2 Hz until
`Ringing` halves the gain (LOG15483): the reason for the phase margin.
From the runway with `airfly --rotate 60`: trainer M32/M47 both bounce
back onto the runway after the rotation and sag (the stiffness problem
above); M47 crashed in CRUISE and HOLD (LOG15521, 15527), M32 survived
both (LOG15522, 15526) -- the sag, not the margin, is what kills it. The
delta never rotated at half stick (`ROTATE_STICK` now 1.0); the canard
rolls over on the runway at 55-170 m/s (craft or ground handling,
unread). Warbird/Seahawk fly either arm.

## Next

1. Fly the screening batch the memory reaper killed (scratchpad `ab3.sh`
   pattern; 6 instances put 21-28 GB in swap within two rounds -- fly 4):
   ATTITUDE_TRIM off/on x margin 32/47 on trainer, delta, Warbird,
   Seahawk; then 3+ flights a cell; promote or delete both flags.
2. Yaw has the same spring (sideslip); roll the dihedral coupling.
3. The canard's runway roll-over; the hopper's ground loop.
4. howItWorks.pdf (crash guard gone, margin, stiffness) and push.

## Commands

```bash
./airplane/tools/airstart.py --instance 0    # (re)make qs_warbird: flies it from the runway, saves
./airplane/tools/airfly.py --instance 0      # CRUISE from qs_warbird, stick pulses, one summary line
./airplane/tools/airfly.py --instance 1 --mode 1 --set RATE_PHASE_MARGIN_DEG=47   # HOLD, an override
./airplane/tools/airfly.py --instance 2 --runway   # from runway 09: full throttle, hands off; take-off summary
./airplane/tools/airfly.py --instance 2 --runway --rotate 60 --craft "KL Jet Trainer"   # a jet: stick back at 60 m/s
./airplane/tools/craftgen.py --check 5       # write the jets, launch and screenshot each on ksp5 (logs/shots/craftgen/)
./run.sh --pilot airplane                    # live: START on the panel
./run.sh --pilot airplane --set MODE=1       # start in HOLD
.venv-pypy/bin/python -m unittest airplane.tests
```
