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

**No gain is fitted to a craft** (the user, 2026-11-05: "shouldn't you use
something that adapts to the plane?"). Per axis, the authority is kRPC's
`available_torque` / `moment_of_inertia` -- wheels, RCS, gimbals and the
surfaces *at the present q and Mach* -- re-read every
`AUTHORITY_REFRESH_S` (`control.authority`); `kp = 1 / (accel x tau)`,
`ki = kp / (INTEGRAL_TAU_FACTOR x tau)` (`control.rate_gains`). tau is
`RATE_TAU_TICKS` (4) of the loop's own measured interval (>= 0.15 s): the
loop is as fast as it can serve, not as fast as a table says. What is left
is design, the same for every craft. kRPC *models* the surfaces; where its
figure is high the loop rings, and `control.Ringing` halves that axis's
gain (4 reversals of 0.15 in 2 s; floor 1/8) and lets it recover at
0.05/s. Offline, a 1-tick-latency plant whose real authority is 0.25x-8x
kRPC's settles on every step (backstop cutting to 0.33 at 8x). Not chosen:
identifying the effectiveness from differenced rates (INDI) --
`SURFACE_OWN` did that for the spaceplane on 2026-10-21 and kRPC's tick
made it too noisy to fly (0/12). Log `kp=`, `acc=` (deg/s^2 per unit
input), `ring=`, `tau=`; events `authority ...` at START, `<axis> ringing:
gain xN`.

**The player's stick**: kRPC adds a client's input to the player's
(`PilotAddon.OnFlyByWire`: `ControlInputs.Add`) and KSP folds the trim into
the stick (`FlightInputHandler`), so the player's input is the read-back
less what we wrote less the trim (`control.stick`). Past `STICK_DEADBAND`
on an axis the hold lets that axis go (keeps its trim) and holds whatever
attitude the stick leaves it at. **Constant rates**: the stick commands a
body rate instead, full deflection = the panel's limit (RPM x 2 pi / 60
rad/s), and the loop flies it (our input = loop output less the stick), so
a full stick at high speed turns no faster than the limit; the holds' own
rates are capped by the same limits.

**Prop plane**: runs `propPitch.PropPilot` in the same process, ticked with
the airplane (its own LOG, headless panel; its pitch/rpm/mode on the
airplane's panel). Unticking it hands the props back.

**Log** `FLY` line: `att=`/`tgt=` bank, pitch, heading (deg), `slip=`,
`rate=`/`want=` pitch, roll, yaw rates (deg/s), `out=` our inputs, `i=` the
integrals, `stick=` the player's input per axis, `kp=`/`acc=`/`ring=`/`tau=`
the gains and where they came from.

## Where it stands (2026-11-05, fingerprint `214ff2b8`)

Written 2026-11-04 on the user's spec; **never flown** (the user tests it
live). Its gains come from kRPC's authority and its own tick; offline they
hold over a 32x error in that authority. No farm harness yet.

## Next

1. The user's live flights: `acc=` against what the plane does (a
   `ringing` event means kRPC's authority is high on that axis; `out=`
   pinned at +/-1 with `rate=` short of `want=` means it is low).
2. If a sense is wrong (it rolls away instead of holding) it is one sign in
   `control.body_rates` or the input senses; `tests/testControl.py` pins
   the geometry, not the game's input senses.
3. A farm harness (`tools/`, from `propPitch/tools/propfly.py`) flying a
   step on each axis, for tuning.

## Commands

```bash
./run.sh --pilot airplane                    # live: START on the panel
./run.sh --pilot airplane --set MODE=1       # start in HOLD
.venv-pypy/bin/python -m unittest airplane.tests
```
