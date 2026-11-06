"""All tunables for the airplane pilot.

    ./run.sh --pilot airplane --set BANK_LIMIT_DEG=60
"""

from dataclasses import dataclass

from common.config import apply_overrides, differences, fingerprint  # noqa: F401


@dataclass
class Config:
    # --- modes ------------------------------------------------------------
    # 0 CRUISE (sideslip nulled, bank and pitch attitude held, turns
    # coordinated), 1 HOLD (pitch, heading and bank all held).  The panel's
    # choice overrides it in flight.
    MODE: int = 0
    # CRUISE never banks past this (deg).
    BANK_LIMIT_DEG: float = 89.0

    # --- gains: from the craft, not fitted to it (control.py) ------------
    # The rate loops close in RATE_TAU_TICKS of the loop's own measured
    # interval (never under RATE_TAU_MIN_S): a loop cannot be faster than
    # what it can serve.  The PI gains put both poles at -1/tau on the
    # axis's measured response (PLANT_* below), starting from kRPC's
    # available_torque / moment_of_inertia (the surfaces at the present q
    # and Mach), re-read every AUTHORITY_REFRESH_S of game time.  KP_MAX
    # bounds an axis with no authority.
    RATE_TAU_TICKS: float = 4.0
    RATE_TAU_MIN_S: float = 0.15
    # The control surfaces take time to move (KSP slews each by its
    # actuatorSpeed over its ctrlSurfaceRange, read from the game by the
    # window mod): an axis's loop is also no faster than its surfaces can
    # reach the input a rate step of the axis's cap needs in 1/SLEW_FACTOR
    # of the closing time, tau >= sqrt(SLEW_FACTOR x step / (accel x slew)),
    # nor faster than LAG_FACTOR x an exponential surface's lag; and the
    # command never moves faster than the surfaces can.
    SLEW_FACTOR: float = 2.0
    LAG_FACTOR: float = 3.0
    INTEGRAL_TAU_FACTOR: float = 4.0
    KP_MAX: float = 20.0
    AUTHORITY_REFRESH_S: float = 0.5
    # The attitude loops (and CRUISE's slip loop) close ATTITUDE_TAU_FACTOR
    # times slower than the rate loops: error / (factor x tau) is the rate
    # asked for; the slip loop's integral INTEGRAL_TAU_FACTOR slower again.
    # (RATE_TAU_TICKS 2.5 rang more in pitch and yaw and settled roll no
    # faster than 4: airfly batch 2026-11-05, LOG15339-15361.)
    ATTITUDE_TAU_FACTOR: float = 3.0
    # kRPC's authority is a model, wrong 4x either way on the Warbird
    # (control.Plant): each axis's response w' = k accel u - d w + c is
    # fitted in flight by least squares forgetting over PLANT_FORGET_S
    # (covariance capped at PLANT_P_MAX), k held in [PLANT_K_MIN,
    # PLANT_K_MAX], d in [0, PLANT_D_MAX] (1/s), both trusted as far as the
    # fit explains the rate; the rate loops' PI gains put both poles at
    # -1/tau on what it measures (control.pi_gains).
    PLANT_FORGET_S: float = 10.0
    PLANT_P_MAX: float = 100.0
    PLANT_K_MIN: float = 1.0 / 16.0
    PLANT_K_MAX: float = 16.0
    PLANT_D_MAX: float = 20.0
    # Only the PLANT_WINDOW_S after an input step of PLANT_EXCITE is fitted.
    PLANT_EXCITE: float = 0.01
    PLANT_WINDOW_S: float = 1.0
    # Where the loop still rings: an axis whose input
    # reverses RING_REVERSALS times by RING_SWING within RING_WINDOW_S has
    # its gain halved (to RING_FLOOR at least), raised back by
    # RING_RECOVER_PER_S while quiet.
    RING_REVERSALS: int = 4
    RING_SWING: float = 0.15
    RING_WINDOW_S: float = 2.0
    RING_RECOVER_PER_S: float = 0.05
    RING_FLOOR: float = 0.125
    # The rates the holds themselves may ask for (deg/s): how hard it may
    # manoeuvre on its own, a comfort limit, not the craft's.
    MAX_PITCH_RATE_DEG_S: float = 15.0
    MAX_ROLL_RATE_DEG_S: float = 60.0
    MAX_YAW_RATE_DEG_S: float = 10.0
    # Coordinated turn: psi' = g tan(phi) / V, the bank's share held to
    # TURN_MAX_G of load and V to at least TURN_MIN_SPEED_M_S.
    TURN_MAX_G: float = 4.0
    TURN_MIN_SPEED_M_S: float = 30.0
    # Past this pitch (deg) heading is undefined: HOLD stops holding it and
    # the turn's feed-forward is off.
    VERTICAL_DEG: float = 80.0
    # CRUISE's slip integral is held to this yaw rate.
    SLIP_I_MAX_DEG_S: float = 5.0
    INTEGRAL_MAX: float = 0.6

    # --- the player's stick -----------------------------------------------
    # A stick deflection beyond this is the player flying the axis: the
    # hold on that axis lets go (its trim stays) and re-takes the attitude
    # the stick leaves it at.  With constant rates on, the stick commands
    # a body rate instead: full deflection = the panel's rate limit.
    STICK_DEADBAND: float = 0.05
    # The panel's rate limits (rev/min) at start.
    PITCH_RATE_RPM: float = 3.0
    ROLL_RATE_RPM: float = 10.0
    YAW_RATE_RPM: float = 1.5
    CONSTANT_RATES: bool = False
    # Run propPitch beside it (the panel's "prop plane" box).
    PROP_PLANE: bool = False
    # SAS is turned off while flying (it would fight the holds) and put
    # back at the end.
    SAS_OFF: bool = True

    # --- loop and log -----------------------------------------------------
    LOOP_SLEEP_S: float = 0.05
    LOOP_PACING_GAME_TIME: bool = False
    LOG_INTERVAL_UT: float = 0.5
    LOG_DIR: str = "logs"
    # Stop after this many game seconds (0 = until TERMINATE).
    RUN_TIME_S: float = 0.0


def defaults_fingerprint():
    return fingerprint(Config)
