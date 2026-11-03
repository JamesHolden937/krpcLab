# propPitch

Holds a propeller plane's collective blade pitch at whatever setting makes it
go fastest, continuously. The root `CLAUDE.md` applies in full.

```
STANDBY -> SEEK -> DONE        (./run.sh --pilot prop)
```

**It flies only the propellers.** The pilot (or the harness) flies the
plane. At START it finds every robotic rotor and the blades below it (any
part with a `Deploy Angle` field), disables the craft's KAL controllers
(they would overwrite the blades every frame), runs every rotor at
`ROTOR_RPM`/`ROTOR_TORQUE_PCT`, and from then on writes every blade's
`Deploy Angle`. TERMINATE gives it all back: the KALs re-enabled, the
rotors' RPM and torque limits as they were.

**Objective**: `J` = the aerodynamic force along the vessel's nose divided by
its mass (`Flight.aerodynamic_force` in the vessel frame, which counts the
blades). That is propeller thrust minus airframe drag. At any airspeed the
pitch that maximises it accelerates fastest, so tracking it reaches the
highest speed. The log carries `acc=`, the measured specific force along
the nose (acceleration less gravity), as a check: the two agree to ~0.1-1
m/s^2 in flight (LOG15158).

**Law** (`seeker.py`): extremum seeking. The pitch is dithered by a sine;
J is fitted over the last periods as mean + trend + sin + cos; the in-phase
part over the amplitude is the slope dJ/dpitch; the centre climbs at
`RATE_MAX tanh(slope/SLOPE_SCALE)`. There is no model of the blade, rotor
or craft.

## The test craft: `qs_prop`, `qs_prop_noon`

The user's live `prop.sfs` (2026-11-02): 108 parts, 12.8 t, four EM-32S
rotors (2 CW, 2 CCW) with four R-25 blades each, a KAL-1000 mapping throttle to
the blades' angle (90 at idle to 0 at full) and the rotors' RPM, 32 solar
panels, an RTG and **55 EC of storage**. `qs_prop` is at KSC sunrise, where
the panels give 1.5 EC/s and the rotors starve and stall. `qs_prop_noon`
is the same save warped +5100 s on the farm (53 EC/s): fly that one.
**Rotor brakes are on the Brakes action group**: brakes on means 0 rpm.

## Measured (2026-11-02)

- At rest the best pitch is low (blades at 60 deg stall: EC drains and
  the rotor stutters, LOG15161). It rises with airspeed: ~28 deg at 5 m/s,
  ~65 deg at 80-120 m/s (LOG15158).
- The thrust is very large on the farm: 2.4 g at 80 m/s with pitch ~64.
  The harness's height hold could not keep up (LOG15158 pulled up to
  +46 m/s vertical speed at 123 m/s and hit the ground).
- The blade's "Angle of Attack" field reads 0 on the farm, so it can't
  be used as an incidence feedback.

## Commands

```bash
./run.sh --pilot prop                                  # live: START on the panel
./propPitch/tools/propfly.py --instance 0              # farm: take off, hold 300 m, 300 s
./propPitch/tools/propfly.py --instance 1 --set DITHER_AMP_DEG=0 \
    --set RATE_MAX_DEG_S=0 --set INITIAL_PITCH_DEG=45  # a fixed pitch, for reference
.venv-pypy/bin/python -m unittest propPitch.tests.testSeeker
```

`propfly.py` flies the plane from its own connection with kRPC's autopilot
(heading 90, climb at 5 deg past 40 m/s, then a height hold) and prints the
end speed, the best 10-s mean and the final pitch centre.
