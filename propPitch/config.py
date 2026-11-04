"""All tunables for the prop-pitch optimizer.

    ./run.sh --pilot prop --set ROTOR_RPM=400
"""

from dataclasses import dataclass

from common.config import apply_overrides, differences, fingerprint  # noqa: F401


@dataclass
class Config:
    # --- the optimum (bladeModel.py) ------------------------------------
    # The Deploy Angle range searched; the craft's own KAL-1000 sweeps 0..90
    # and KSP clamps a blade's deflection at 1.5 x ctrlSurfaceRange (90).
    PITCH_MIN_DEG: float = 0.0
    PITCH_MAX_DEG: float = 90.0
    # Grid step of the search (then golden section to SEARCH_TOL_DEG).
    SEARCH_STEP_DEG: float = 1.0
    SEARCH_TOL_DEG: float = 0.05

    # --- the rotors -------------------------------------------------------
    # Every rotor is run at this RPM limit and torque limit (percent) while
    # the pilot flies: rotor_02s.cfg's traverseVelocityLimits top out at
    # 460, so that is the fastest the EM-32S will turn.
    ROTOR_RPM: float = 460.0
    ROTOR_TORQUE_PCT: float = 100.0
    # The throttle sets the rotors' RPM limit: throttle x ROTOR_RPM, never
    # below THROTTLE_RPM_MIN (0 = throttle cut stops the props).  Off, every
    # rotor runs at ROTOR_RPM whatever the throttle.
    THROTTLE_RPM: bool = True
    THROTTLE_RPM_MIN: float = 0.0
    # The rotors' limit is re-written only when the throttle's RPM moves by
    # at least this much (rpm): one write per rotor per change.
    RPM_WRITE_STEP: float = 2.0
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
    # A rotor turning slower than this (rpm) at a reassert while the Brakes
    # group is off has its brake released (a power loss latches it).
    BRAKE_RPM: float = 1.0
    LOG_DIR: str = "logs"
    # Stop after this many game seconds (0 = until TERMINATE).
    RUN_TIME_S: float = 0.0


def defaults_fingerprint():
    return fingerprint(Config)
