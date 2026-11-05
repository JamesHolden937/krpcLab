# propPitch

Holds a propeller plane's collective blade pitch at whatever setting makes it
go fastest, continuously. The root `CLAUDE.md` applies in full.

```
STANDBY -> TRACK -> DONE        (./run.sh --pilot prop)
```

**It flies only the propellers.** The pilot (or the harness) flies the
plane. At START it finds every robotic rotor and the blades below it (any
part with a `Deploy Angle` field), disables the craft's KAL controllers
(they would overwrite the blades every frame), runs every rotor at
the throttle's RPM limit and `ROTOR_TORQUE_PCT`, **releases every rotor's brake**, and from
then on writes every blade's `Deploy Angle`. TERMINATE gives it all back: the KALs re-enabled, the
rotors' RPM and torque limits as they were.

**Law** (`bladeModel.py`): the best pitch is *computed*, not searched for.
KSP evaluates each blade as one lifting surface at one point
(`velocityOffset` along its span, ~6.0 m from the hub on the R-25), lift
along the blade's normal turned by the Deploy Angle, drag along the flow,
both from the part cfg's curves (read from Assembly-CSharp:
`ModuleControlSurface.FixedUpdate`, `ModuleLiftingSurface`). At START the
pilot reads every blade's position and axes once; every tick it reads the
airspeed vector in the vessel's axes, density and speed of sound, evaluates
that law for all blades at every angle (1 deg grid + golden section) and
commands the angle with the most thrust along the nose. No dither, no lag:
a change of conditions is answered on the next tick. Data for all nine stock
blades is in `bladeTable.py`, generated from the game's cfgs by
`tools/bladetable.py` (a test checks it against `testInstances/base`); a
blade not in it stops START with its name.

**Throttle = RPM limit** (`THROTTLE_RPM`, default on): every tick the
rotors' `target_rpm` is max(`THROTTLE_RPM_MIN`, throttle x `ROTOR_RPM`),
written when it moves by `RPM_WRITE_STEP`; the pitch is scored at that
rpm. Farm LOG15208/15206/15207: throttle 1/0.5/0 ran 460/230/0 rpm.

**Feather** (`THROTTLE_FEATHER`, 0.02): a throttle at or below it is a
cut -- 0 rpm (the stopped-rotor brake release is skipped) and the blades at
the most thrust at 0 rpm, i.e. the least drag, searched over the blades'
whole range: edge-on, Deploy 0 in axial flow. Model (R-25, 60 m/s): 0.16 kN
per rotor feathered against ~4 kN at 85 deg. Farm LOG15214: cut at 60 m/s,
0 rpm and 0.0 deg within ~1 s, coast J ~-2 m/s^2 (model's prop share 0.07);
no in-game baseline with the blades left at idle 90 yet.

**Prop brake** (`PROP_BRAKE`, default on): while `control.brakes` is on,
every rotor runs at `ROTOR_RPM`, the rotors' own brake latch (the craft's
Brakes group sets it) is released, and the blades go to the angle with the
most thrust *against* the motion: the air velocity's nose component
(vessel y) > 0 -> most reverse, < 0 -> most forward, searched over the
blades' whole Deploy Angle range, +/-1.5 x ctrlSurfaceRange (Assembly-CSharp
`ModuleControlSurface.OnStart`; `range` in `bladeTable.py`: 90 for
fans/props, 22.5 for heli blades). **Still** (|v| < `BRAKE_HOLD_SPEED`
0.5 m/s): every rotor at 0 rpm with its brake latch set, blades left where
they are, until |v| > `BRAKE_RESUME_SPEED` 2 m/s (`mode=brake-stop`).

**Tip limit** (`BRAKE_TIP_LIMIT`, on the wheels only; `tipping.py`): the
contacts are the bottoms of the bounding boxes (read at START) of every
wheel grounded since Brakes went on. About the foremost and the rearmost,
the moment of gravity, the measured aero force (its lift, and the
deceleration it alone gives, at the CoM) and the props' thrust at the hubs
is linear in the thrust, so the thrust that leaves `BRAKE_TIP_MARGIN` 0.5
of gravity's restoring moment is solved for; past the balance point the
bound is the balance. The props' share cancels out: only gravity, lift and
the thrust line's height over the CoM decide it. **The wheel brakes'
moment is not counted** (it opposes a tail-over with props above the CoM;
it would add to a nose-over with props below). Backstop, a hill climb: a
fraction of the bound halved when a contact wheel is off the ground 0.25 s
or the pitch moves > 10 deg/s, raised 0.25/s. Where no bound can be
computed (one wheel, a row across) it climbs from zero at 0.1 g/s. Log
`tip=` (the bound, kN, and the fraction); events `tip: ...`.
`qs_prop_noon` stands on its four wing gear with the CoM **0.15 m** ahead of
the rear pair: bound ~-17..-25 kN at low speed (the props make ~-570 kN),
~0 from 55 m/s (lift ~ weight). From 33-38 m/s it stops in 8-9 s with no
sideslip (LOG15245/15246; unlimited: ~1 s, and from 80 m/s it went over,
LOG15210). From 55-75 m/s it slides and hops whatever the props do: a
plain throttle cut there breaks up 4/6 (LOG15249-15254) -- the plane under
the harness, not the brake.

**Blade load and rpm**: each pitch is scored at the rpm its rotor would
settle at. KSP's rotor is a stiff velocity drive (damper 1000) capped at
`maxTorque` x torque limit (70 kN m, EM-32S); its EC draw is
0.06 x 70 x limit x rpm/460 / 0.75 = 5.6 EC/s at 100%, independent of
load, and with no EC it gives no torque. The motor's real torque is
*learned* from the measured rpm (`Rotor.observe`): below target and flat
for 0.6 s, the model's blade torque at that rpm is all the motor has; while
holding target it recovers toward the cap over ~5 s. A dead motor learns
~0 and the search then stops loading that rotor.

**Rotor brakes** (works mid-flight): a rotor's brake is a latched mode
(`ModuleRoboticServoRotor.brakingMode`) set by the Brakes action group *and*
by a power loss; a braked rotor holds 0 rpm whatever kRPC tells its motor,
and no kRPC property reads or clears it. The rotor's own `BrakeAction`
(kRPC `PartAction.activated = False`, i.e. Deactivate) releases it without
touching the wheel brakes. START releases all of them; each re-assert
releases a rotor below `BRAKE_RPM` while the Brakes group is off (a group
left on is respected after START). Before this, a START in flight with the
rotors braked flew 0 rpm while the model assumed 460 (LOG15188).

**Deflection sense** is not readable through kRPC (the CW/CCW variant sets
it), so `choose_signs` picks, per blade, the sense that pushes forward at
rest; the thrust only depends on that and the spin's product.

The log's `J=` is the measured forward aero force per unit mass (thrust less
airframe drag); `model=` the model's thrust per unit mass at the angle flown;
`motor=` the learned torque per rotor (kN m); `torque=` the worst rotor's
load over its cap; `thr=` the throttle; `mode=` thrust / feather /
brake-rev / brake-fwd / brake-hold.

## The test craft: `qs_prop`, `qs_prop_noon`

The user's live `prop.sfs` (2026-11-02): 108 parts, 12.8 t, four EM-32S
rotors (2 CW, 2 CCW) with four R-25 blades each, a KAL-1000 mapping throttle to
the blades' angle (90 at idle to 0 at full) and the rotors' RPM, 32 solar
panels, an RTG and **55 EC of storage**. `qs_prop` is at KSC sunrise, where
the panels give 1.5 EC/s and the rotors starve and stall. `qs_prop_noon`
is the same save warped +5100 s on the farm (53 EC/s): fly that one.
**Rotor brakes are on the Brakes action group**: brakes on means 0 rpm.

## Where it stands (2026-11-04, fingerprint `ca612601`)

2026-11-04: the tip limit and the stop when still (above; journal). The
reaction-torque counter is deleted (the user, 2026-11-04 late). propPitch
also runs inside the airplane pilot (`airplane/`, its "prop plane" box). Every brake from <= 40 m/s stops intact and still.

First live flights (LOG15184-15188, 2026-11-02 night): from the runway it
matches the farm (J tracks model within ~1 m/s^2 to 75 m/s; climbed to 57 km
on a zoom at 894 m/s before EC ran out). START in flight (LOG15188, 7.7 km)
failed: rotors braked, fixed 2026-11-03 (farm LOG15189, LOG15193: braked
rotors at 0 rpm spin to 460 in ~2 s after START, Brakes group on). The pilot
still dies with `ObjectDestroyedException` if a part it holds breaks off
(LOG15184, LOG15193).

Farm, earlier (`qs_prop_noon`), LOG15177-15183:

- The model's best angle: 85 at rest, 82 at 14 m/s, 75 at 52, 68 at 91,
  60 at 133 m/s; the old seeker's measured best at 90 m/s was 68.
- `bladeprobe` (LOG15182, angles 40-89 on the roll): J and model agree
  within ~1 m/s^2 at 40-80 deg at 0-40 m/s. With a starved motor (115 rpm,
  readout refreshed every 0.3 s) the model's peak was ~4 deg higher than
  measured -- not a clean test.
- At the optimum the plane accelerates ~3.5 g; J 29-34 against model 38-40
  at 70-110 m/s (the gap is airframe drag).
- **Every farm flight is destroyed by ~100-130 m/s**: sideslip of 25-38 m/s
  on the runway (or airborne under the harness's hold). The harness cannot
  fly this plane at this thrust; top speed is unmeasured.
- In the probe, EC fell 53 -> 2 while the plane tumbled to 200 m, and three
  motors disengaged; the learning took their torque to ~0. The draw law says
  4 rotors use 22 EC/s, so the drain was something else (panels out of the
  sun or lost).

## Next

1. Survive losing a blade/rotor (re-`find` on ObjectDestroyedException).
2. A harness that can fly it (rudder/heading on the runway at 3.5 g).
3. Feather's gain measured in game: a coast with the blades held at 90
   against feathered (needs an arm that holds an angle at a cut). Both farm
   coasts ground-looped ~5 s after the cut (harness steering, probably).
4. The prop brake flown live, and backwards (only forward was flown on the
   farm; the backward branch is unit-tested). The tip limit on a craft
   that sits on a wider wheelbase.
5. START costs ~2500 calls (~4 s): the blade field tables.

## Commands

```bash
./run.sh --pilot prop                                  # live: START on the panel
./propPitch/tools/propfly.py --instance 0              # farm: take off, hold 300 m, 300 game-s
./propPitch/tools/propfly.py --instance 0 --takeoff-speed 9999 --seconds 30 \
    --brake-at 40 [--throttle 0.5]                     # runway: prop brake past 40 m/s
    # ... --cut-at 60 instead: throttle cut (feather) past 60 m/s
    # ... --wheel-brakes 0: the craft's wheel brakes off (props only)
./propPitch/tools/brakesum.py logs/LOG<n> ...            # per brake/cut flight: stop time, slip, nose-up, bound
./propPitch/tools/propfly.py --instance 1 --probe --takeoff-speed 9999 \
    --seconds 40 --set LOG_INTERVAL_UT=0               # model vs game, angle sweep
./propPitch/tools/probesum.py logs/LOG<n>              # ... read it
./propPitch/tools/bladetable.py                        # regenerate bladeTable.py
.venv-pypy/bin/python -m unittest propPitch.tests.testBladeModel
```

`propfly.py` flies the plane from its own connection with kRPC's autopilot
(heading 90, climb at 5 deg past 40 m/s, then a 300 m height hold) and prints the
end speed, the best 10-s mean and the final pitch centre.
