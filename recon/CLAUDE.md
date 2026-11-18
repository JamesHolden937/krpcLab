# recon

The QUDRAC (the user's small RAPIER plane, `saves/craft/SPH/QUDRAC.craft`, the live craft with
its 400 LF tank filled: the live file has 0) flies from the runway over a waypoint the player
places on the map and back, as fast as heat, fuel and 15 g allow. **Fitted to this one craft on
purpose** (the user, 2026-11-17), but no number that does not make sense as a craft constant.
The root `CLAUDE.md` applies otherwise.

```
STANDBY -> TAKEOFF -> OUT -> HOME -> FINAL -> FLARE -> ROLLOUT -> DONE
./run.sh --pilot recon            launcher "QUDRAC: OVER THE WAYPOINT AND HOME"
./recon/tools/reconfly.py --instance 1 --lat 0.5 --lon -62 --timescale 3   (farm)
./recon/tools/probe.py --instance 0 --alt 20000    (kRPC-autopilot characterisation)
```

- **Target**: the newest custom waypoint on the body (stock map waypoint tool); `TARGET_LAT/LON`.
- **Attitude**: kRPC's autopilot. On the wheels pitch/heading; **in the air the path is flown**:
  `guidance.normal_accel` (path angle closed in `PATH_TAU_S` against gravity, turn to the
  course, held to the load budget, vertical first) -> lift along it, angle of attack off the
  swept lift curve at this q (`alpha_for`) plus a path integral -> `guidance.path_attitude`
  nose and roof -> `set_direction_and_up`.
- **Take-off** (`plan_takeoff`): lift curve from `simulate_aerodynamic_force_at` (0-30 deg),
  rotation angle = what the tail clearance measured at rest allows, lift-off speed for it,
  floored at `ROTATE_MIN_Q_PA` (the fin's dynamic pressure); pitch target ramped at 40 deg/s;
  a measured clearance guard stops the nose.
- **Speed**: throttle proportional on the hottest part's predicted fraction (`heat_throttle`,
  the backstop); `plan_speed` adds the fuel-limited cruise speed (drag from KSP's aerodynamics
  x measured LF per N s, level cruise only). The thermal prediction (`thermal.py`, KSP's
  heating formulas) is **logged only** (`THERMAL_FLY` off, `vpred=`): its constants come out
  ~1000x small (flux units unverified).
- **Altitude**: searched (`seek_altitude`): down when thrust-limited, up on spare thrust at
  `asin(a/g)`, up gently (`HEAT_SEEK_GAMMA_DEG`) when heat/fuel binds.
- **Route**: on round the planet on the same great circle unless even the most economical
  speed's fuel is short, then back the short way (always so far: ~620-780 LF needed, ~307 left).
- **Home**: coast speed planned in the present air's drag, descent on `DESCENT_SLOPE`; FINAL
  only at <= 1.5 x approach speed (circles the gate meanwhile); approach = max(1.3 Vs, fin
  speed); go-around past 40% of the runway; flare; rollout derotates 3 deg/s, brakes once the
  nose is down.

## Where it stands (2026-11-18 00:50)

Works on the farm: take-off, over the target (300-1550 m, ~130 s), home, centred approach,
touchdown (LOG17758: 80 m/s, sink 1.2, cross -3; LOG17761: 82 m/s, cross 0). **No intact
landing yet**: the nose slams down within 0.5 s of main-gear contact and the intake strikes.
Suspect: main wheels at friction 10 spinning up below the CoM (the mains sit 0.72 m behind
it). Waiting on the user (gear friction is their rule). Home leg slow (340-1325 s).
