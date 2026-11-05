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

    # --- outer loops: attitude error -> Euler rate (1/s) ------------------
    PITCH_GAIN: float = 1.0
    BANK_GAIN: float = 1.5
    HEADING_GAIN: float = 0.6
    # The body rates the holds themselves may ask for (deg/s).
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
    # CRUISE's sideslip loop: yaw rate per radian of slip (1/s), and its
    # integral (1/s^2), held to SLIP_I_MAX_DEG_S.
    SLIP_GAIN: float = 1.5
    SLIP_KI: float = 0.5
    SLIP_I_MAX_DEG_S: float = 5.0

    # --- inner loops: body-rate error (rad/s) -> stick, PI ----------------
    # At dynamic pressure GAIN_Q_REF_PA; scaled by Q_REF / q (the surfaces'
    # moment goes with q) within [GAIN_SCALE_MIN, GAIN_SCALE_MAX].
    PITCH_KP: float = 1.5
    PITCH_KI: float = 1.5
    ROLL_KP: float = 0.6
    ROLL_KI: float = 0.4
    YAW_KP: float = 1.5
    YAW_KI: float = 1.0
    INTEGRAL_MAX: float = 0.6
    GAIN_Q_REF_PA: float = 5000.0
    GAIN_SCALE_MIN: float = 0.1
    GAIN_SCALE_MAX: float = 5.0

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
