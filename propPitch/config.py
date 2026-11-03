"""All tunables for the prop-pitch optimizer.

    ./run.sh --pilot prop --set DITHER_AMP_DEG=2
"""

from dataclasses import dataclass

from common.config import apply_overrides, differences, fingerprint  # noqa: F401


@dataclass
class Config:
    # --- the seeker (seeker.py) ------------------------------------------
    # The collective pitch is dithered by +-DITHER_AMP_DEG with this period
    # (game seconds); the response is fitted over WINDOW_PERIODS periods.
    # Short: the takeoff roll doubles the speed in ~3 s (LOG15167), and the
    # blades' actuator (35 deg/s, largeFanBlade.cfg) and the rotor answer
    # within ~0.1 s (lag ~10-15 deg at a 3 s period, LOG15158).
    DITHER_AMP_DEG: float = 2.0
    DITHER_PERIOD_S: float = 1.0
    WINDOW_PERIODS: float = 2.0
    # The thrust's lag behind the pitch, in degrees of the dither: the fit
    # demodulates against sin(wt - lag).  Each log's telemetry prints the
    # measured lag (`lag=`); this should match it.
    PHASE_LAG_DEG: float = 0.0
    # The centre climbs at RATE_MAX tanh(t / SLOPE_T_SCALE) deg/s, t the
    # fitted slope over its standard error.
    RATE_MAX_DEG_S: float = 5.0
    SLOPE_T_SCALE: float = 3.0
    # Fits whose curvature (second harmonic) is negative by more than
    # CURVATURE_MIN_T standard errors feed a curvature smoothed over
    # CURVATURE_TAU_S; with one, the centre closes the Newton distance
    # -J'/J'' with time constant NEWTON_TAU_S instead (capped at RATE_MAX).
    CURVATURE_MIN_T: float = 3.0
    CURVATURE_TAU_S: float = 5.0
    NEWTON_TAU_S: float = 1.0
    # The blades' Deploy Angle range the seeker may use.  The craft's own
    # KAL-1000 sweeps 0..90 (qs_prop).
    PITCH_MIN_DEG: float = 0.0
    PITCH_MAX_DEG: float = 90.0
    # Where the centre starts; negative = the blades' Deploy Angle at START.
    INITIAL_PITCH_DEG: float = -1.0

    # --- the rotors -------------------------------------------------------
    # Every rotor is run at this RPM limit and torque limit (percent) while
    # the pilot flies: rotor_02s.cfg's traverseVelocityLimits top out at
    # 460, so that is the fastest the EM-32S will turn.
    ROTOR_RPM: float = 460.0
    ROTOR_TORQUE_PCT: float = 100.0
    # Disable every KAL robotic controller while flying (it would write the
    # blades' Deploy Angle back every frame); restored on shutdown.
    DISABLE_KAL: bool = True

    # --- loop and log -----------------------------------------------------
    LOOP_SLEEP_S: float = 0.05
    LOOP_PACING_GAME_TIME: bool = True
    LOG_INTERVAL_UT: float = 0.5
    # Every this many game seconds the KALs' Enabled flag and the rotors'
    # RPM/torque limits are read back and re-set if anything changed them
    # (two of five instances flew with the rotors at 0 and 229 rpm,
    # LOG15165-15166).
    REASSERT_S: float = 1.0
    LOG_DIR: str = "logs"
    # Stop after this many game seconds (0 = until TERMINATE).
    RUN_TIME_S: float = 0.0


def defaults_fingerprint():
    return fingerprint(Config)
