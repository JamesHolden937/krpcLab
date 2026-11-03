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

**Law** (`seeker.py`): extremum seeking, no model of the blade, rotor or
craft. The pitch is dithered by a 1 s, +-2 deg sine. Every tick, J over the
last 2 s is fitted by least squares to mean + trend + trend^2 + sin + cos +
sin2 + cos2. The in-phase sine gives the slope J' and the cos2 term gives the
curvature J'' (as -J'' A^2/4). Well-measured curvatures (t < -3) feed a
smoothed J''. With one, the centre takes Newton steps toward -J'/J'' (tau
1 s, at most 5 deg/s). Before that it climbs at 5 tanh(t/3) deg/s on the
slope's t-statistic. Every second the KALs' Enabled flag and the rotors'
limits are read back and re-set if changed.

## The test craft: `qs_prop`, `qs_prop_noon`

The user's live `prop.sfs` (2026-11-02): 108 parts, 12.8 t, four EM-32S
rotors (2 CW, 2 CCW) with four R-25 blades each, a KAL-1000 mapping throttle to
the blades' angle (90 at idle to 0 at full) and the rotors' RPM, 32 solar
panels, an RTG and **55 EC of storage**. `qs_prop` is at KSC sunrise, where
the panels give 1.5 EC/s and the rotors starve and stall. `qs_prop_noon`
is the same save warped +5100 s on the farm (53 EC/s): fly that one.
**Rotor brakes are on the Brakes action group**: brakes on means 0 rpm.

## Where it stands (2026-11-02, fingerprint `9ce0c552`)

Unflown live. On the farm (`qs_prop_noon`):

- **The seeker works on the takeoff roll.** On 6/6 flights (LOG15168-15173)
  it went from 0 deg at rest to 60-70 deg by 90 m/s, at its 5 deg/s cap,
  while forward acceleration rose from 0 to 2-3 g. `J` agrees with the
  measured `acc=` to within ~5%.
- **Every flight ended destroyed once airborne.** At 2-3 g of thrust,
  `propfly.py`'s kRPC attitude hold loses the plane: vertical speed swings
  +25/-74 m/s and it hits the ground. The manoeuvring then dominates J,
  and the fit reads nonsense slopes (|t| ~ 100, lag ~180 deg).
  **Not yet known: is that the harness's hold or the airframe?** Fixed
  pitch 45 deg never leaves the ground (36 m/s top speed, LOG15160).
- The best pitch rises with airspeed: ~10-20 deg at rest, ~30 at 3 m/s,
  ~50 at 17 m/s, ~68 at 50-90 m/s. **Low pitch at speed is a brake**: the
  blades windmill. A false negative slope at 90 m/s once took the pitch
  68 -> 23 deg and decelerated the plane at 9 m/s^2 (LOG15167, the old
  6 s linear fit).
- The craft's KAL maps full throttle to 0 deg, which on the farm is no
  thrust at all. The live game may differ: ~50 extra mods.
- The blade's "Angle of Attack" / "Air Speed" fields read ~0 on the farm,
  so the blade gives no incidence feedback.

## Next

1. The user's live flight: does the plane fly at the pitch this picks?
   Read `slope/t/curv/rate` in the log. Do the Newton steps hold the
   centre still in steady flight?
2. A harness that can fly the plane (a gentler attitude law, or
   `RATE_MAX_DEG_S` limited on the roll), so top speed can be measured
   against a fixed-pitch sweep.
3. When the pilot manoeuvres, J changes for reasons that aren't the
   pitch. Consider gating the centre's motion on |vertical speed| or
   bank, or measuring J along the velocity with the lift removed.
4. START costs ~2500 calls (each blade's field table): read
   `field_list` by code name once instead.

## Commands

```bash
./run.sh --pilot prop                                  # live: START on the panel
./propPitch/tools/propfly.py --instance 0              # farm: take off, hold 300 m, 300 game-s
./propPitch/tools/propfly.py --instance 1 --set DITHER_AMP_DEG=0 \
    --set RATE_MAX_DEG_S=0 --set INITIAL_PITCH_DEG=45  # a fixed pitch, for reference
.venv-pypy/bin/python -m unittest propPitch.tests.testSeeker
```

`propfly.py` flies the plane from its own connection with kRPC's autopilot
(heading 90, climb at 5 deg past 40 m/s, then a 300 m height hold) and prints the
end speed, the best 10-s mean and the final pitch centre.
