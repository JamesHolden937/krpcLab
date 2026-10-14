"""All tunables for the low-thrust autopilot and its stand-in simulator.

Control code reads ``cfg.X``; it never hard-codes a number.  Override from
the command line with ``--set FIELD=VALUE``.
"""

import hashlib
from dataclasses import dataclass, fields


@dataclass
class Config:
    # --- Arrival ----------------------------------------------------------
    # Done when every targeted apsis radius is within this.
    TOLERANCE_M: float = 100.0
    # ... and the orbit plane within this of the target plane.
    PLANE_TOLERANCE_DEG: float = 0.01
    # ... and a held periapsis direction within this (eccentric orbits).
    APSE_TOLERANCE_DEG: float = 0.5
    # The +-90 deg arcs of a held apse line wake once both apsis radii are
    # within this fraction of target (an end-game correction).
    APSE_ARC_GATE: float = 0.01

    # --- Guidance (Hohmann in small burns) -----------------------------------
    # The arcs round each apsis have half-width acos(eta): ETA_DEFAULT is
    # flown when there is no deadline (0.99 = +-8 deg: within ~2% of the
    # impulsive Hohmann on every transfer in tests/testGuidance), and the
    # planner searches ETA_MIN..ETA_MAX to meet one.  Never 1: the arcs
    # shrink to nothing.  Wide arcs are faster but each pass changes the
    # orbit more: eta 0.95 cost +20% on 2000x100 -> 2000 where 0.99 cost +2.
    ETA_DEFAULT: float = 0.99
    # A mission's orbit-shaping phases widen the arcs (down to this cutoff,
    # +-60 deg) until the apsis changes left fit this many passes.
    ETA_WIDE_MIN: float = 0.5
    ARC_PASSES_MAX: float = 30.0
    ETA_MAX: float = 0.999
    ETA_MIN: float = 0.2
    # The prograde step (m/s) the radius rates are differenced over.
    RATE_DV: float = 0.01

    # --- Missions (mission.py) ------------------------------------------------
    # Lowest periapsis above a body's atmosphere (or surface) any phase plans
    # (10 km let an approach error and a capture burn carry Duna's periapsis
    # into its atmosphere).
    SAFE_ALT_M: float = 30_000.0
    # Capture into an ellipse with apoapsis this fraction of the target's SOI
    # (or the final apoapsis, if higher); FINAL then lowers it -- after any
    # plane change, which is cheap up there.
    CAPTURE_AP_FRACTION: float = 0.3
    # ... or, when an engine is too weak to capture into that in one pass,
    # into one this loose (a moon has no moons to keep clear of).
    CAPTURE_LOOSE_FRACTION: float = 0.7
    # An engine too weak to capture inside the planet's SOI matches its
    # motion in the cruise (MATCH), arriving at MATCH_STANDOFF of the SOI
    # radius at this fraction of the fastest v_inf it can capture there (a
    # long capture starting within this fraction of the SOI, or one pass).  MATCH
    # starts once stopping there needs MATCH_SHARE of the engine and ends
    # MATCH_END_S before the aim point.
    ARRIVE_MARGIN: float = 0.8
    MATCH_STANDOFF: float = 1.05
    MATCH_SHARE: float = 0.8
    MATCH_END_S: float = 600.0
    # A one-pass capture must fit in this fraction of the time within
    # +-CAPTURE_ARC_DEG of periapsis; past that arc, still unbound, it failed.
    CAPTURE_ARC_DEG: float = 60.0
    CAPTURE_FILL: float = 0.8
    # A capture too long for that starts when it must, inside the SOI:
    # thrust along the velocity removes energy at the acceleration per metre
    # of path, so it burns once the energy to shed reaches this share of
    # what the path left to periapsis can remove.
    CAPTURE_PATH_SHARE: float = 0.8
    # The capture burn holds the periapsis between the safe floor and this
    # multiple of the aimed one.
    CAPTURE_RP_BAND: float = 1.5
    # Apoapses stay this many of a moon's SOI radii short of its orbit, and
    # a moon transfer's waiting orbit reaches only this fraction of the way.
    WAIT_SOI_MARGIN: float = 1.2
    WAIT_AP_FRACTION: float = 0.6
    # Departure parking ellipse apoapsis, as a fraction of the highest one
    # clear of the body's moons and well inside its SOI.
    PARK_AP_FRACTION: float = 0.9
    # The window is searched from the estimated end of the pump plus this.
    WINDOW_LEAD_S: float = 6 * 3600.0
    # A moon push is searched up to this multiple of the tangential push
    # that reaches the moon's orbit, this far ahead.
    PUSH_DV_FACTOR: float = 2.0
    # ... or, away from periapsis (every 30 deg), up to this multiple of
    # the periapsis Hohmann: a near-circular waiting orbit pushes from
    # wherever the phase is right, an eccentric one only at periapsis.
    PUSH_OFF_PERIAPSIS: float = 1.1
    PUSH_HORIZON_S: float = 30 * 21600.0
    # ... and only within this fraction of the post-push revolution.
    PUSH_REVOLUTION: float = 0.75
    # A departure push stops this far past periapsis (the rest is made up
    # in the cruise).
    DEPART_ARC_DEG: float = 60.0
    # A departure pass is rejected if its escape enters a moon's SOI within
    # this long.
    ESCAPE_CHECK_S: float = 3 * 21600.0
    # Mid-course corrections: made when the B-plane miss needs more than
    # CORRECT_DV, not within CORRECT_MIN_LEAD_S of the encounter, rechecked
    # after CORRECT_EVERY of the time remaining.
    CORRECT_DV: float = 0.5
    CORRECT_MIN_LEAD_S: float = 3600.0
    CORRECT_EVERY: float = 0.1
    # A moon correction bigger than this, or a closest approach beyond
    # MISS_REPLAN of the moon's SOI, means no encounter is ahead: back to
    # WAIT for a new push.
    CORRECT_MAX_DV: float = 150.0
    # A cruise miss beyond this many of the target's SOI radii, or this
    # fraction of its distance from the star (whichever is less), or a
    # linear correction above CORRECT_MAX_DV, is re-planned as a fresh
    # Lambert transfer, not corrected.
    RETARGET_MISS: float = 20.0
    RETARGET_FRACTION: float = 0.01
    MISS_REPLAN: float = 1.5
    # In the target's SOI the hyperbola's periapsis is nudged until within
    # this fraction of the capture altitude, no later than this before it.
    APPROACH_TOL: float = 0.02
    APPROACH_MIN_LEAD_S: float = 1800.0
    MISSION_MAX_DAYS: float = 4000.0
    # After a slingshot, brake until the apoapsis is inside this fraction of
    # the SOI.
    REBIND_SOI_FRACTION: float = 0.6

    # --- Tick ---------------------------------------------------------------
    # Guidance is re-decided once a tick and held (as the live pilot will
    # hold a command between ticks); a tick is this fraction of a period.
    TICKS_PER_ORBIT: int = 180
    # RK4 steps per thrusting tick (coasting ticks are propagated exactly).
    THRUST_SUBSTEPS: int = 4
    # A thrusting tick changes the velocity by at most this share of it
    # (only a strong engine is ever limited by it).
    THRUST_DV_SHARE: float = 0.02

    # --- The live pilot (pilot.py) ---------------------------------------------
    # Thrust only while the thrust axis is within this of the command (the
    # plugin turns the vessel at its wheels' rate; the planner assumes it
    # points at once).
    ALIGN_DEG: float = 2.0
    # The warp rate is at most this many times a tick's length in seconds:
    # rails time advances in frames of 0.02 x rate and the loop's latency is
    # a few hundredths of a second, so a tick then overruns by a few percent
    # at most.  (10 s ticks in low orbit: 50x.)
    WARP_PER_TICK: float = 5.0
    # Telemetry at most once per this many game seconds (events always).
    LOG_INTERVAL_UT: float = 600.0

    # --- Planner ---------------------------------------------------------------
    # Cutoffs flown between ETA_MIN and ETA_MAX to meet a deadline, evenly
    # spaced in arc half-width.
    PLAN_GRID: int = 13
    # A transfer still running after this many orbits is reported unfinished.
    MAX_ORBITS: int = 20000


def apply_overrides(cfg, pairs):
    """``FIELD=VALUE`` strings onto ``cfg``, coerced to the field's type."""
    types = {f.name: f.type for f in fields(cfg)}
    for pair in pairs or ():
        name, _, value = pair.partition("=")
        if name not in types:
            raise ValueError(f"unknown config field {name!r}")
        current = getattr(cfg, name)
        if isinstance(current, bool):
            value = value.lower() in ("1", "true", "yes", "on")
        else:
            value = type(current)(value)
        setattr(cfg, name, value)
    return cfg


def differences(cfg):
    """The fields of ``cfg`` that differ from the defaults, as ``k=v``."""
    base = Config()
    return [f"{f.name}={getattr(cfg, f.name)}" for f in fields(cfg)
            if getattr(cfg, f.name) != getattr(base, f.name)]


def defaults_fingerprint():
    """A hash of the whole default set: which baseline ``differences`` is from."""
    base = Config()
    text = ";".join(f"{f.name}={getattr(base, f.name)!r}" for f in fields(base))
    return hashlib.sha1(text.encode()).hexdigest()[:8]
