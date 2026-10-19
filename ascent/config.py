"""All tunables for the ascent: MechJeb to orbit, the booster dropped when
it has just enough left to come home.

    ./run.sh --pilot ascent --set ORBIT_ALT_M=90000
"""

import hashlib
from dataclasses import dataclass, fields


@dataclass
class Config:
    # --- MechJeb ascent ----------------------------------------------------
    ORBIT_ALT_M: float = 80000.0        # MechJeb's target orbit
    INCLINATION_DEG: float = 0.0
    ASCENT_TYPE: int = 1                # MechJeb 2.15: 0 classic, 1 PVG
    # MechJeb's own autostaging stays off for the booster: this script
    # stages it.  After separation it is turned on again for the upper
    # stage's own staging (HANDOFF=upper only).
    MJ_AUTOSTAGE_AFTER_SEP: bool = True

    # --- Launch ----------------------------------------------------------
    # Lit but still on the pad this long after staging: release the next
    # stage (clamps) -- if it decouples nothing.  Give up after the timeout.
    LAUNCH_CLAMP_WAIT_S: float = 3.0
    LAUNCH_TIMEOUT_S: float = 20.0

    # --- When to separate --------------------------------------------------
    # Separate when the booster's propellant, after the boostback and the
    # landing it would fly from this state, is down to this reserve -- a
    # fraction of what those two burns need.  It covers what the model
    # leaves out: correction burns, the landing burn's throttling, RCS on
    # the Vernors, and the drag of the boostback.
    RESERVE_FRACTION: float = 0.15
    # Turn rate of the booster's flip under power.  The heavy booster swings
    # ~140 deg in the first 4-5 s of its boostback (LOG12758, hold=), and
    # 30 makes the model burn exactly the 1251 m/s it measured.  Read a
    # boosterland log's hold= column on a new booster; a slower flip wastes
    # more of the burn.
    FLIP_RATE_DEG_S: float = 30.0
    # With boosterland's FLIP_THROTTLE_LAW the flip is modelled from the
    # booster's own gimbal, RCS and inertia (summed over its parts), re-read
    # this often; FLIP_RATE_DEG_S is the full-throttle flip's model.
    AUTHORITY_REFRESH_S: float = 15.0
    # Drag coefficient on the booster's frontal area while it falls.  1.0 is
    # a bare cylinder; airbrakes only slow it more, so the landing estimate
    # errs towards keeping propellant.
    LANDING_CD: float = 1.0
    # Look-ahead: the reserve is projected this far forward at its present
    # rate of fall, so the throttle-down starts before the line, not on it.
    SEP_LEAD_S: float = 2.0
    # Don't judge the budget before the vehicle is clear of the pad.
    MIN_ALT_M: float = 1000.0

    # --- The throttle-down before separation -------------------------------
    # Once free, the booster's acceleration must be this fraction of the
    # upper stage's, so the upper stage pulls away and the two cannot meet.
    # The booster's engines go out centre-last (boosterland's EngineCluster
    # order, reversed), the way its landing burn throttles.
    SEP_ACCEL_RATIO: float = 0.75
    # Separate once the booster's delivered thrust is under the target by
    # this tolerance on SEP_CONFIRM_TICKS ticks in a row ...
    SEP_THRUST_TOL: float = 0.02
    SEP_CONFIRM_TICKS: int = 2
    # ... or give up waiting after this long and separate anyway.  An engine
    # with a slow spool-down is the only thing that should ever reach it.
    SEP_THROTTLE_TIMEOUT_S: float = 5.0

    # --- After separation --------------------------------------------------
    # "booster": switch to the booster and fly boosterland's autoland in this
    #   process (the upper stage is left to MechJeb, which stops flying it
    #   when it is no longer the active vessel).
    # "upper": stay with the upper stage and let MechJeb fly it to orbit.
    HANDOFF: str = "booster"
    # Seconds the switch to the booster may take.
    SWITCH_TIMEOUT_S: float = 10.0

    # --- Engine grouping (passed to boosterland.engines) --------------------
    ENGINE_AXIS_TOL_M: float = 0.3
    ENGINE_RING_TOL_M: float = 0.3
    ENGINE_PAIR_TOL: float = 0.25

    # --- Loop and logging ---------------------------------------------------
    LOOP_SLEEP_S: float = 0.1
    LOOP_PACING_GAME_TIME: bool = False
    LOOP_PACING_MAX_STRETCH: float = 5.0
    LOG_INTERVAL_UT: float = 2.0
    LOG_DIR: str = "logs"
    ORBIT_TIMEOUT_S: float = 900.0      # HANDOFF=upper: give up after this


def defaults_fingerprint():
    """A short hash of every default in ``Config`` (see boosterland's)."""
    base = Config()
    text = ";".join("%s=%r" % (f.name, getattr(base, f.name))
                    for f in sorted(fields(base), key=lambda f: f.name))
    return hashlib.sha1(text.encode()).hexdigest()[:8]


def apply_overrides(cfg, pairs):
    """Apply ``NAME=value`` strings to ``cfg`` in place, coercing types."""
    types = {f.name: f.type for f in fields(cfg)}
    for pair in pairs:
        name, _, raw = pair.partition("=")
        name = name.strip()
        if name not in types:
            raise KeyError("unknown config field: %s" % name)
        current = getattr(cfg, name)
        if isinstance(current, bool):
            value = raw.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(current, int):
            value = int(raw)
        elif isinstance(current, float):
            value = float(raw)
        else:
            value = raw
        setattr(cfg, name, value)
    return cfg
