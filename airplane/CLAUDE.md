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

**How**: attitude error x gain = Euler rates (`*_GAIN`), into body rates
(`control.euler_to_body`), capped at `MAX_*_RATE_DEG_S`; each body rate
flown by a PI loop on its stick axis (`control.RateLoop`), gains x
`GAIN_Q_REF_PA / q` within [0.1, 5] (surface moment goes with dynamic
pressure). The integrals are the axes' trims. Attitude and rates come from
the vessel's axes and angular velocity in its surface frame, read off as
the axes' own motion (`d(axis)/dt = w x axis`, the convention measured for
the spaceplane). Senses: pitch +1 nose up, roll +1 right, yaw +1 nose
right. SAS is turned off while flying and put back.

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
integrals, `stick=` the player's input per axis, `scale=` the q gain scale.

## Where it stands (2026-11-04, fingerprint `a2ff2c90`)

Written 2026-11-04 on the user's spec; **never flown** (the user tests it
live). Gains are first guesses. No farm harness yet.

## Next

1. The user's live flights: read `out=`/`i=` against `rate=`/`want=` --
   a loop whose `out` sits at +/-1 or whose rate rings has its gain wrong
   (`*_KP`/`*_KI`, `GAIN_Q_REF_PA`).
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
