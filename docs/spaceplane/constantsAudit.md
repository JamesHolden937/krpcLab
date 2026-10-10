# Hard-coded values in the spaceplane -- audit of 2026-10-10

`spaceplane/config.py` has 359 fields (312 float, 16 int, 28 bool, 3 str) and
the control code ~180 numeric literals of its own.  Most are legitimate
(where the runway is, how often to tick, numerical guards).  The ones that
matter are numbers standing in for **a property of the vehicle, the load
or the orbit** -- they are right for the shuttle they were fitted on and
silently wrong for the next one, and two found today were hiding structural
problems:

- `HAC_ENTRY_MACH` 1.5 stood in for a turn radius and admitted a 26 km
  circle against the cone's 16 km cap; three of the four entries it let
  through fast were that batch's misses.
- `HAC_ALPHA_MAX_DEG` 22 (initial commit, no reason recorded) sits past the
  cone's flown lift peak (~139 m^2 near 12 deg, falling to ~90-100 at
  15-25): above 13 deg the extra alpha is drag the speed law uses as a
  brake.  Read as a lift limit it overstates the wing about 2x.

Each item below says what the number stands in for and what should compute
it.  Work it top-down; one flag per replacement, measured, then promoted
or deleted.

## Tier 1 -- vehicle properties that change with load or craft

| constant | value | stands in for | replace with |
|---|---|---|---|
| `HAC_BANK_MAX_DEG` | 45 | the bank the wing can hold | `HAC_BANK_FROM_LIFT` (built: in-flight `FlownLift` peak at the mass now) |
| `HAC_ALPHA_MAX_DEG` | 22 | two things: the lift ceiling and the speed law's drag brake | lift: the flown lift peak (`FlownLift`); brake: a measured drag-vs-alpha choice, separately |
| `ALPHA_MAX_DEG`, `GLIDE_ALPHA_MAX_DEG`, `ENTRY_ALPHA_DEG`, `GLIDE_ALPHA_DEG`, `APPROACH_ALPHA_MAX_DEG`, `FLARE_ALPHA_DEG` | 32, 40, 22, 20, 28, 30 | the polar's shape per regime | the measured polar per Mach band (the glide already learns its ceiling, `ratchet_alpha`) |
| `FLARE_TOUCHDOWN_ALPHA_DEG` | 13 | tail-strike geometry | the measured tail-strike angle (logged every flight, 20.6 deg on the shuttle) x `TAIL_STRIKE_MARGIN` |
| `BANK_MAX_DEG`, `APPROACH_BANK_MAX_DEG`, `APPROACH_BANK_COMP_MAX_DEG`, `FLARE_BANK_MAX_DEG` | 70, 40, 60, 12 | the same lift limit as the cone's, per phase | `hac_bank_limit`'s law generalised |
| literal in `guidance.approach` | 10 deg below 150 m | wing-strike clearance | wingtip ground clearance from the part geometry and height (as the tail strike is) |
| `airframe.MARGIN` | 0.70 | the share of table lift the wing makes | `LiftTrim` measures it (reads 0.74) |
| `HAC_LD`, `HAC_GATE_LD`, `airframe.PLANNING_BIAS` | 1.86, 1.35, 1.09 | the cone's glide ratio (`cone_ld` returns the constant) | the flown polar's L/D at the cone's alpha and bank (`turning_ld`, fed by measured lift *and drag*) |
| `APPROACH_BEST_LD` | 4.2 | final's glide ratio | every log prints "airframe DISAGREES ... 4.20 v 2.95 (30% out)"; measure it (landsum computes it) |
| `HAC_HOLD_MARGIN`, `HAC_RADIUS_MIN_M`, `HAC_RADIUS_MAX_M`, `HAC_RADIUS_M` | 1.3, 2/16/8 km | circles the airframe can hold | follow from the bank limit; the max is the field geometry, the rest is derived |
| literal `37.5` (canard trim) | deg | the canard part's deflection range | read the control surface's range from the part |
| `CANARD_TRIM_MAX_ALPHA_DEG`, `CANARD_TRIM_LOCAL_MAX_DEG` | 15, 28 | where the canard stalls | the canard's local angle where its moment was measured to fall (tabprobe2) -- probe it at STANDBY |
| `SPLIT_FULL_DEG` | 38 | the split rudder's full deflection | read the part's range |
| `BRAKE_DECEL_FULL_M_S2` | 11 | wheel-brake deceleration at full brakes | mass- and friction-dependent; measure on the rollout |
| `RESOURCE_KG_PER_UNIT` | 5.0 | propellant density | kRPC gives each resource's density |
| `APPROACH_FACTOR`, `SPEED_HOLD_FACTOR`, `GLIDE_ARRIVAL_FACTOR`, `APPROACH_FLARE_FACTOR` ... | x stall | speeds | scale-free if the stall is right; the stall uses `MARGIN` (above) |
| `DEORBIT_RANGE_MIN_M`/`MAX_M` | 400-2300 km | this airframe's entry range ("590-1880 km" in its comment) | the deorbit search's own reach at the dv box edges |

## Tier 2 -- aim points and biases (structural problems wearing constants)

| constant | value | what it hides |
|---|---|---|
| `HAC_ALT_M`, `HAC_ENTRY_DIST_M` | 12 km, 3 km | the cone needs 2-5 km above `HAC_ALT_M`; the distance test supplies it by firing early. Both derived triggers lost (LOG9347; rot-smoke-hed2-1010). Real fix: aim the glide at the height the cone needs |
| `APPROACH_AIM_SHIFT_M` | 1000 | a bias on the approach aim -- a missing model of final's achieved L/D |
| `TOUCHDOWN_AIM_M`, `GATE_ALT_M`, `GATE_DIST_M` | 1800, 2000, 4000 | final's geometry; the gate distance is already partly derived (`gate_dist`) |
| `DEORBIT_WINDOW_BIAS`, `DEORBIT_LONG_BIAS_M` | 0.25, -3000 | residual along-track biases |
| `HAC_EXIT_SURPLUS_M`, `GLIDE_RESERVE_M` | 500, 500 | margins in metres for errors that are a fraction of the path |
| `ENTRY_INTERFACE_M`, `SKIP_ENTER_MARGIN_M` | 58 km, 5 km | Kerbin's atmosphere; kRPC gives the body's atmosphere depth |
| `DRAIN_RESERVE_DV_MS` | 200 | the burn's dv need, which the deorbit search knows |

## Tier 3 -- gains and time constants (~100)

`*_KP`, `*_TAU_S`, deadbands, rate limits.  Several are vehicle-dependent
(`HAC_HEADING_KP`, `APPROACH_SPEED_KP`, `BANK_RATE_*`) and would be better
scaled by the measured response the way the roll damper and the attitude
time-to-peak already are.  Lower priority: a wrong gain shows up as an
oscillation the logs measure; a wrong vehicle property shows up as a miss
nothing explains.

## Tier 4 -- legitimate

Runway coordinates and size, tick and prediction steps, logging and the
time-scale governor, warp, timeouts, numerical guards (`1e-6`, clamps on
`cos`), probe settings, `9.80665` (Isp's g0), `1.4` (air's gamma in
`environment`).

## Flags still parked off

`AIRBRAKE_CACHE`, `ALPHA_TRACKING_ON` -- promote or delete (root CLAUDE.md).
`LOOP_PACING_GAME_TIME` (the harness sets it) and `DIAG_INTERFACE`
(diagnostic) are infrastructure.
