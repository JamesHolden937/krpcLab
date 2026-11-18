"""All tunables for the recon pilot (the QUDRAC only).

    ./run.sh --pilot recon --set HEAT_MAX=0.8

This pilot is fitted to one craft on purpose (the user, 2026-11-17: "this
autopilot is just for that plane, so you can ... tune it for that
specifically"); the craft is ``saves/craft/SPH/QUDRAC.craft``.  The numbers
quoted are from ``recon/tools/probe.py`` on the farm (logs/probe-*.txt).
"""

from dataclasses import dataclass

from common.config import apply_overrides, differences, fingerprint  # noqa: F401


@dataclass
class Config:
    # --- the target ----------------------------------------------------------
    # The last custom waypoint on the vessel's body (stock map view: "Place
    # waypoint" / the waypoint tool); with none, these (deg) if set.
    TARGET_LAT: float = float("nan")
    TARGET_LON: float = float("nan")
    # Overhead: the closest approach is logged; the target is passed once it
    # is behind (more than 90 deg off the nose) or within this distance.
    PASS_RADIUS_M: float = 500.0

    # --- limits ----------------------------------------------------------------
    # Total load on the pilot (thrust's push included), g.  The wings get
    # what is left after the axial load (guidance.normal_g_budget), less
    # G_MARGIN; measured g over G_LIMIT - G_MARGIN shuts the throttle and
    # rolls the pull back.
    G_LIMIT: float = 15.0
    G_MARGIN: float = 2.0
    # Heat: every part's internal and skin temperature against its own
    # limit, the hottest one.  The throttle is proportional on where it
    # will be HEAT_LOOKAHEAD_S from now: full HEAT_BAND under HEAT_MAX,
    # shut at it.  Probe 1: the cockpit skin (limit 2000 K) went 1000 ->
    # 2142 K in 2 s at Mach 4.8 at 5.5 km, and the craft burned.
    HEAT_MAX: float = 0.85
    HEAT_BAND: float = 0.08
    HEAT_LOOKAHEAD_S: float = 6.0
    # Heat-limited with the throttle shut, the descent stops (the flight
    # path is held at or above level) until the heat falls.
    HEAT_NO_DESCENT: float = 0.8
    # The fastest speed every part survives for the rest of the flight is
    # computed (recon/thermal.py: KSP's heating formulas, each part's
    # constants read off kRPC's flux numbers) every THERMAL_PLAN_S and
    # held on the throttle; the proportional governor above is the
    # backstop.
    THERMAL_PLAN_S: float = 2.0

    # --- the profile -----------------------------------------------------------
    # Take-off: full throttle; the nose comes up as early as it can, to the
    # best-lifting angle of attack short of a tail strike (both computed at
    # START: the lift curve from KSP's own aerodynamics,
    # ``simulate_aerodynamic_force_at``, swept 0..LIFT_SWEEP_MAX_DEG; the
    # tail-strike angle from the parts' bounding boxes about the main
    # wheels, less TAIL_CLEARANCE_DEG for the pitch loop's overshoot); the
    # rotation starts so the nose is up as the speed reaches the lift-off
    # speed that angle gives (thrust's lift counted).
    LIFT_SWEEP_MAX_DEG: float = 30.0
    TAIL_CLEARANCE_DEG: float = 1.0
    GEAR_UP_AGL_M: float = 50.0
    # The cruise altitude is searched for in flight, never set: the
    # altitude command climbs at ALT_SEEK_RATE_M_S while heat holds the
    # throttle back (thinner air: more speed for the same heat) and sinks
    # at it while the throttle is full and the speed no longer grows by
    # SEEK_ACCEL_M_S2 (thrust is the limit: thicker air gives the RAPIER
    # more).  It settles where the two limits meet: the fastest speed the
    # craft can hold without burning.  Probes (logs/probe1): heat-limited
    # at ~1370 m/s at 15 km, ~1475 at 20 km, ~1480 at 25 km.
    ALT_SEEK_RATE_M_S: float = 150.0
    SEEK_ACCEL_M_S2: float = 0.3
    # Never closer to the ground than this while seeking (m).
    ALT_FLOOR_AGL_M: float = 2000.0
    # Climbs and descents at most these flight-path angles.
    CLIMB_GAMMA_DEG: float = 30.0
    DESCENT_GAMMA_DEG: float = -15.0
    # Altitude error closed in ALT_TAU_S; flight path error into pitch
    # attitude at most PITCH_STEP_DEG off the present pitch.
    ALT_TAU_S: float = 15.0
    PITCH_STEP_DEG: float = 6.0
    # Heading error closed in HEADING_TAU_S (a coordinated turn's rate);
    # bank never past BANK_MAX_DEG.
    HEADING_TAU_S: float = 6.0
    BANK_MAX_DEG: float = 80.0

    # --- home ------------------------------------------------------------------
    # The runways (spaceplane's surveyed thresholds): 09 lands east from the
    # west end, 27 west from the east end; whichever the return comes in
    # nearer to.
    RUNWAY_09_LAT: float = -0.0485761
    RUNWAY_09_LON: float = -74.724466
    RUNWAY_27_LAT: float = -0.0501756
    RUNWAY_27_LON: float = -74.495283
    RUNWAY_LENGTH_M: float = 2400.0
    RUNWAY_ALT_M: float = 70.0
    # The final approach: from FINAL_M out on the centreline at
    # FINAL_SLOPE (tan of the glide slope), aimed AIM_M past the threshold,
    # at APPROACH_STALL_RATIO times the stall speed (the lift curve's peak
    # at the mass then; 1.3 is the airliner rule).
    FINAL_M: float = 12000.0
    FINAL_SLOPE: float = 0.07
    AIM_M: float = 300.0
    APPROACH_STALL_RATIO: float = 1.3
    # The descent home: altitude on DESCENT_SLOPE above the final's gate;
    # speed what a drag-only coast slows to APPROACH_SPEED at the gate
    # (guidance.coast_speed, on the drag measured in flight, times
    # COAST_DRAG_SHARE: drag falls as the air thins, and a coast that is
    # too fast cannot be slowed).
    DESCENT_SLOPE: float = 0.12
    COAST_DRAG_SHARE: float = 0.6
    # Localizer: cross-track closed in LOC_TAU_S, cut at most LOC_CUT_DEG.
    LOC_TAU_S: float = 12.0
    LOC_CUT_DEG: float = 30.0
    # Flare from FLARE_AGL_M: the sink rate ramps to TOUCHDOWN_SINK_M_S.
    FLARE_AGL_M: float = 25.0
    TOUCHDOWN_SINK_M_S: float = 1.5
    # Speed held by throttle: (want - speed) / SPEED_BAND_M_S.
    SPEED_BAND_M_S: float = 20.0

    # --- loop and log ------------------------------------------------------------
    LOOP_SLEEP_S: float = 0.05
    LOOP_PACING_GAME_TIME: bool = False
    LOG_INTERVAL_UT: float = 0.5
    LOG_DIR: str = "logs"
    # Stop after this many game seconds (0 = until stopped or landed).
    RUN_TIME_S: float = 0.0


def defaults_fingerprint():
    return fingerprint(Config)
