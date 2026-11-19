# boosterland: constants audit (2026-11-15)

Every number in `boosterland/config.py` sorted by what it is. The root rule: no ship-fitted constant may
shape a flight that is flown. The fitted ones found on 2026-11-15 were replaced the same night by
measured or physical quantities (commits 0da9516..HEAD, journal "2026-11-15 night"); this is what is
left and why it is allowed.

## Measured quantities the laws now run on

| quantity | where | replaces |
|---|---|---|
| nose lag behind its command (delay fit, powered ticks) | `common.attitudelag`, `Autoland.lag` | kRPC's time_to_peak, the slew constants |
| prediction's tick-to-tick scatter | `common.jitter`, `env.miss_band` | every miss deadband (deploy, late switch, aero steer, RCS trim, boostback exit) |
| pointing error's scatter | `ValveLimits`, `env.pointing_band` | RCS relay angles, tracking-sample steadiness, descent configuration, lift-probe angle |
| horizontal velocity's scatter | `Autoland.velocity_band` | ROLL_PLANE_MIN_HS_MPS |
| engine spool-up + control tick | `trajectory.reaction_time` | SUICIDE_REACTION_S, throttle gain, boostback taper, gear lead |
| RCS lateral acceleration | `Autoland.read_rcs_accel` | RCS trim gain |
| what the craft survives (parts, legs) | `env.touchdown_tolerance`, `lateral_tolerance`, `tip_angle_deg` | TOUCHDOWN_SPEED, upright tilt, the aim's brakeable speed |
| brakes settled (drag probe stops changing) | `Autoland.brakes_settled` | DEPLOY_SETTLE_S |
| the coast's commanded angle | `env.descent_aoa` | DRAG_PROBE_DESCENT_TILT_DEG |
| leg deploy time | gear timing | GEAR_LEAD_* |

## Policy (dimensionless; allowed by the rule)

`NOISE_SIGMAS` 2, `SETTLE_LAGS` 3, `LOOP_BANDWIDTH_SHARE` 0.5, `SURVIVAL_SHARE` 0.25, `FLIP_THRUST_LOSS` 0.1,
`LANDING_MAX_TILT_DEG` 15 (the throttle headroom planned for steering, as an angle), `LANDING_ZEM_GAIN` 6
(optimal control), `AERO_STEER_HOLD_MARGIN` 0.7, `ROLL_SLOT_HYSTERESIS` 0.15, `RCS_SAT_INPUT` 0.95 /
`RCS_SAT_RELEASE` 0.6 / `RCS_SAT_CLOSING_FRACTION` 0.2, `ENGINE_PAIR_TOL` 0.25, `FLIP_VIA_VERTICAL_DEG` 90 /
`FLIP_VERTICAL_PAST_DEG` 10 (the flip's route geometry).

## Estimators and cadences (statistics and cost; not about the craft)

`DRAG_SMOOTHING`, `DRAG_OUTLIER_RATIO`, `DRAG_WARMUP_SAMPLES`, `AERO_STEER_TRACK_SMOOTHING`,
`AERO_STEER_TRACK_FLOOR`, `AERO_STEER_HOLD_PROBES`; `AERO_REFRESH_UT`, `AERO_STEER_HOLD_PROBE_S`,
`DRAG_RESWEEP_INTERVAL_S`, `DEPLOY_LATE_STEP_S` (how often a measurement or a solve is re-taken),
`common.jitter.SAMPLES` 20, `common.attitudelag.WINDOW/MIN_TICKS/GAIN`.

## Guards, numerics, game and site

Pad coordinates; `AIM_BIAS_*` 0; `SEPARATION_COAST_S` (scan off only) and `SEPARATION_MAX_COAST_S`,
`BOOSTBACK_MAX_BURN_S`, `GEAR_DEPLOY_ALT_M`, `GEAR_MEASURE_TIMEOUT_S`, ``,
`TOUCHDOWN_WATCH_S`, `DEAD_STREAM_*`, `THROTTLE_DIVERGENCE`, `MASS_LOSS_FRACTION` (timeouts and
backstops); `CLEARANCE_*` (collision geometry); `PREDICT_*`, `DENSITY_SAMPLE_COUNT`, `DRAG_MACH_MAX`,
`DRAG_CURVE_BINS`, `LOOP_*`, `LOG_INTERVAL_UT`; `TERRAIN_*`, `SOUND_SPEED_FALLBACK_M_S`, KSP's default
crashTolerance 9; `DEPLOY_MAX_DEG` (the brake law's unit); `ENGINE_AXIS/RING_TOL_M`; `ROLL_THRESHOLD_DEG`,
`ROLL_OFFSET_DEG`; `TOUCHDOWN_ALT_M` 0.2.

## Still to look at

- (done 2026-11-18: `LEG_CLEARANCE_MARGIN_M` replaced by the legs' `Leg.contact_position`; `TOUCHDOWN_BACKSTOP_SPEED` by the weakest touching part's crash tolerance)
- The 20 m/s floor under every aerodynamic probe (`environment.py`): below it a probe is noise; tie it to
  the probe's own resolution.
