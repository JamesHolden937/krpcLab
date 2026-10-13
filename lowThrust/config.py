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

    # --- Guidance (Hohmann in small burns) -----------------------------------
    # The arcs round each apsis have half-width acos(eta): ETA_DEFAULT is
    # flown when there is no deadline (0.99 = +-8 deg: within ~2% of the
    # impulsive Hohmann on every transfer in tests/testGuidance), and the
    # planner searches ETA_MIN..ETA_MAX to meet one.  Never 1: the arcs
    # shrink to nothing.  Wide arcs are faster but each pass changes the
    # orbit more: eta 0.95 cost +20% on 2000x100 -> 2000 where 0.99 cost +2.
    ETA_DEFAULT: float = 0.99
    ETA_MAX: float = 0.999
    ETA_MIN: float = 0.2
    # The prograde step (m/s) the radius rates are differenced over.
    RATE_DV: float = 0.01

    # --- Tick ---------------------------------------------------------------
    # Guidance is re-decided once a tick and held (as the live pilot will
    # hold a command between ticks); a tick is this fraction of a period.
    TICKS_PER_ORBIT: int = 180
    # RK4 steps per thrusting tick (coasting ticks are propagated exactly).
    THRUST_SUBSTEPS: int = 4

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
