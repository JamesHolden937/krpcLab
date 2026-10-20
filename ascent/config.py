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
    # MechJeb 2.15: 0 classic, 1 PVG.  PVG plans the stack it is engaged on
    # and never re-plans after this script drops the booster: every upper
    # stage burned to depletion (LOG12916-12922), re-engaging it after the
    # separation did not help (escape on 4/4, upper1-1019); classic from
    # the separation reached 80 x 80 km on 3/3 (LOG12942, 12949, 12952).
    ASCENT_TYPE: int = 0
    # Classic's "turn shape" (MechJeb's percentage / 100): low is a flat
    # early turn, high a steep climb that turns late.  It is solved at every
    # liftoff from the craft on the pad (turnshape.solve: the shape that
    # leaves the upper stage the most mass in orbit, the booster separating
    # by budget.recovery).  TURN_SHAPE is only the fallback when the solve
    # finds no orbit at all.
    TURN_SHAPE: float = 0.4
    # The drag table the solve flies, sampled from the game at zero angle
    # of attack (Flight.simulate_aerodynamic_force_at) on this grid.
    DRAG_HEIGHTS_M: str = "0,5000,10000,20000,30000,45000"
    DRAG_SPEEDS_MS: str = "100,200,300,350,400,500,700,1000,1500,2200"
    # Log the solved shape's predicted flight every this many seconds
    # ("turn shape track:"), for ascent/tools/shapetrack.py to read against
    # the telemetry: what would disagree with the model.
    SHAPE_TRACK_S: float = 10.0
    # PVG only (ASCENT_TYPE 1).  PVG's gravity turn: vertical to this surface speed, then the nose is
    # pitched over at this rate until it meets prograde, which it then
    # follows (AoA-limited) until PVG's optimizer takes over below its
    # dynamic-pressure trigger -- after the booster is dropped.  The kick
    # alone sets the turn's shape.  MechJeb's defaults (50, 0.5) flew the
    # stacks steep: 55 deg at 10 km, 37 at 30 km (LOG12916-12923, 12929).
    # 1.0 on the small stack: 70 at 2 km, 45 at 10, 36 at 20 (LOG12932);
    # 1.5 was 39 at 10 km and dropped the booster at 26 km (LOG12931).
    PITCH_START_VELOCITY: float = 50.0
    PITCH_RATE_DEG_S: float = 1.0
    # MechJeb rolls the vehicle to its roll setting; it is set to the roll
    # the vehicle sits at on the pad, read against a pitch-over towards
    # this heading, so a launch that way never rolls (attitude.pad_roll).
    ROLL_REF_HEADING_DEG: float = 90.0
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
    # The upper stage's engines are lit before the decoupler fires
    # (Ascent.upper_lit), and it fires once they deliver this fraction of
    # their thrust -- or after the timeout, for an engine that spools.
    SEP_UPPER_THRUST_FRACTION: float = 0.9
    SEP_IGNITE_TIMEOUT_S: float = 3.0

    # --- After separation --------------------------------------------------
    # "upper": stay with the upper stage and let MechJeb fly it to orbit;
    #   the booster is flown afterwards from FMRS's separation save
    #   (launcher "BOOSTBACK FROM FMRS SAVE", boosterland/fmrs.py).
    # "booster": switch to the booster and fly boosterland's autoland in this
    #   process (the upper stage is left to MechJeb, which stops flying it
    #   when it is no longer the active vessel).
    HANDOFF: str = "upper"
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
    # HANDOFF=upper: give up after this.  900 cut the shuttle stack off
    # before MechJeb finished (LOG12831); a coast to apoapsis alone can be
    # several minutes.
    ORBIT_TIMEOUT_S: float = 3600.0


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
