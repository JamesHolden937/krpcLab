# boosterland: constants audit (2026-11-15)

Every number in `boosterland/config.py` and the flight code, sorted by what it is. The root rule: no
ship-fitted constant may shape a flight that is flown. **Fitted** rows must be replaced by a measured
or physical quantity before the next batch; **policy** rows are dimensionless choices allowed by the
rule (say so in the comment); **numerical / safety / game** rows are not about the craft.

## Fitted -- replace before flying (in flight order)

| constant | value | what it does | replacement |
|---|---|---|---|
| `SEPARATION_MAX_COAST_S` | 5 s | longest wait for clearance | safety cap; keep as a timeout only if the clearance scan cannot answer |
| `AIRBRAKE_DEPLOY_ANGLE_DEG` | 60 deg | brakes' angle from START to the deploy law (fitted: new1-1016) | the middle of the law's measured range, so it can move both ways |
| `DEPLOY_MIN_DEG` | 30 deg | floor of the deploy law (fitted on the heavy, dep1-1016) | the measured range's bottom (the part's own limit) |
| `DEPLOY_DEADBAND_M` | 30 m | no deploy step inside this miss | the prediction's measured noise |
| `DEPLOY_STEP_S` / `DEPLOY_SETTLE_S` | 10 / 2 s | one Newton step per; re-sweep after | the brakes' measured deploy time |
| `DEPLOY_NEWTON_FRACTION` | 0.7 | damping of the step | policy (dimensionless) or a full step once the step waits for the measured settle |
| `DEPLOY_PROBE_DEG` / `DEPLOY_LEARN_MIN_DEG` | 10 / 3 deg | first step; smallest step that teaches k | a share of the measured range |
| `DEPLOY_LATE_STEP_S` / `DEPLOY_LATE_TOL_M` | 5 s / 10 m | re-solve interval; switch tolerance | measured deploy time; prediction noise |
| `BOOSTBACK_TOLERANCE_M` | 190 m | boostback ends inside this miss | what the descent can still remove: the coast's measured aero reach + brakes |
| `BOOSTBACK_TAPER_DV` | 60 m/s | throttle down under this dv | the dv the throttle can resolve: thrust accel x (measured tick + spool) to the tolerance |
| `FLIP_ALIGN_DEG` | 25 deg | throttle held until aligned (FLIP_UNDER_POWER overrides) | check whether it still acts; delete if not |
| `FLIP_GIMBAL_SHARE` | 0.9 | gimbal torque credited in the flip law | the measured credited/real ratio (gimbalTorqueFix probe) |
| `ROLL_PLANE_MIN_HS_MPS` | 5 m/s | roof from the trajectory plane only above this | where the plane's direction is no longer measurable: its angle noise |
| `ROLL_BRAKE_DEG` | 5 deg | the roll-rate cap stops within this | the roll loop's measured lag x rate |
| `AERO_STEER_DEADBAND_M` | 2 m | no aero steering inside this miss | prediction noise |
| `AERO_STEER_MIN_AUTHORITY_M` | 1 m | least reach worth steering | prediction noise |
| `AERO_STEER_SOLVE_AOA_DEG` / `PROBE_AOA_DEG` | 4 / 8 deg | finite-difference angles | a share of the measured holdable aoa |
| `AERO_STEER_TRACK_*` | 10 s, 1 deg, 2 s / 1 deg | when a tracking sample is taken | the measured nose lag (settled after N lags) |
| `COAST_RCS_TRIM_START_M` / `DEADBAND_M` / `FULL_M` | 10 / 3 / 30 m | cross-track trim relay and gain | prediction noise; full input at the RCS's measured reach over the time left |
| `COAST_RCS_TRIM_CHECK_S` / `CHECK_M` | 4 s / 15 m | wrong-sign test | measured lag and noise |
| `COAST_RCS_TRIM_SETTLE_DEG` | 10 deg | start once the flip is this close | the pointing the trim needs: its own tilt |
| `LANDING_UPRIGHT_TILT_DEG` | 2 deg | lean allowed at contact | the legs' measured tip angle (atan(b/h)) x share; none known: 0 |
| `HOVER_CUT_ALT_M` / `TOUCHDOWN_HOVER_VS` | 3 m / -0.5 m/s | cut a hover near the ground | backstop: the leg clearance's measured error; review |
| `GEAR_LEAD_FACTOR` / `GEAR_LEAD_S` | 1.25 / 1 s | legs dropped this early | measured deploy time + one measured tick |
| `RCS_SAT_*`, `RCS_ERROR_ON/OFF_DEG`, `RCS_SETTLE_S` | 0.95, 0.5 s, 0.2, 0.6, 5/1.5 deg, 2 s | the RCS valve's relay | the nose's measured lag and pointing noise |

## Policy (dimensionless; allowed)

`LOOP_BANDWIDTH_SHARE` 0.5, `TOUCHDOWN_SPEED_SHARE` 0.25, `COAST_UNWIND_LAGS` 3, `LANDING_MAX_TILT_DEG` 15
(the throttle headroom planned for steering; an angle, but no craft dimension), `LANDING_ZEM_GAIN` 6
(optimal control), `AERO_STEER_HOLD_MARGIN` 0.7, `ROLL_SLOT_HYSTERESIS` 0.15, `BOOSTBACK_EXIT_TICKS` 4
(should be game seconds of lag, not ticks: review), `ENGINE_PAIR_TOL` 0.25.

## Estimators (statistical; review, lower priority)

`DRAG_SMOOTHING` 0.25, `DRAG_OUTLIER_RATIO` 2, `DRAG_WARMUP_SAMPLES` 1, `DRAG_PROBE_DESCENT_TILT_DEG` 16,
`DRAG_RESWEEP_AOA_DEG` 8, `DRAG_RESWEEP_INTERVAL_S` 10, `AERO_REFRESH_UT` 1, `AERO_STEER_HOLD_PROBES` 8,
`AERO_STEER_HOLD_PROBE_S` 2, `AERO_STEER_TRACK_SMOOTHING` 0.1, `AERO_STEER_TRACK_FLOOR` 0.05.

## Numerical, safety, game, site (not about the craft)

Pad coordinates and offset; `AIM_BIAS_*` (0); `PREDICT_*`, `DENSITY_SAMPLE_COUNT`, `DRAG_MACH_MAX`,
`DRAG_CURVE_BINS`, `LOOP_*`, `LOG_INTERVAL_UT`; clearance geometry (`CLEARANCE_*`); `BOOSTBACK_MAX_BURN_S`,
`GEAR_DEPLOY_ALT_M` and `GEAR_MEASURE_TIMEOUT_S`, `TOUCHDOWN_BACKSTOP_SPEED`/`WATCH_S`, `DEAD_STREAM_*`,
`THROTTLE_DIVERGENCE`, `MASS_LOSS_FRACTION` (guards and diagnostics); `TERRAIN_*`, `SOUND_SPEED_FALLBACK`,
`GAME_DEFAULT_CRASH_TOLERANCE` 9 (KSP's), `G0`; `LEG_CLEARANCE_*` (bounding-box padding/fallback);
`ENGINE_AXIS/RING_TOL_M` (grouping engines by geometry); `FLIP_VIA_VERTICAL_*`, `ROLL_THRESHOLD_DEG`,
`ROLL_OFFSET_DEG` (geometry/kRPC settings); `TOUCHDOWN_ALT_M` 0.2. Code literals: numerical tolerances,
the 20 m/s probe floor (environment.py), 1.4 (air's gamma).
