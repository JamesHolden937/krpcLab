"""What every autopilot's ``Config`` needs besides its fields: ``--set``
overrides and the fingerprint of its defaults.

Each pilot's ``config.py`` keeps its own dataclass and re-exports these, so
``from <pilot>.config import Config, apply_overrides, defaults_fingerprint``
reads the same everywhere.
"""

import hashlib
from dataclasses import fields


class UnknownField(KeyError):
    """``--set`` named a field the config does not have."""


def apply_overrides(cfg, pairs):
    """``--set FIELD=VALUE`` strings applied to ``cfg`` in place, each coerced
    to the type of the field's current value (bool from 1/true/yes/on, a
    tuple as comma-separated floats).  Unknown fields raise."""
    names = {f.name for f in fields(cfg)}
    for pair in pairs:
        name, _, raw = pair.partition("=")
        name = name.strip()
        if name not in names:
            raise UnknownField("unknown config field: %s" % name)
        current = getattr(cfg, name)
        if isinstance(current, bool):
            value = raw.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(current, int):
            value = int(raw)
        elif isinstance(current, tuple):
            value = tuple(float(x) for x in raw.split(","))
        elif isinstance(current, str):
            value = raw
        else:
            value = float(raw)
        setattr(cfg, name, value)
    return cfg


def fingerprint(config_class):
    """A short hash of every default of ``config_class``.

    **The config line is a list of differences, which makes it silent about
    the thing the differences are measured against.**  Change a default
    between two batches and the logs of both say the same thing while the
    flights differ -- eighteen spaceplane flights flown with
    ``GLIDE_RESERVE_ON`` defaulted to ``True`` were indistinguishable in the
    log from the controls they were compared with, and the comparison drawn
    from them was backwards.  Two logs with different fingerprints are not
    the same experiment, however alike their difference lists look."""
    base = config_class()
    text = ";".join("%s=%r" % (f.name, getattr(base, f.name))
                    for f in sorted(fields(base), key=lambda f: f.name))
    return hashlib.sha1(text.encode()).hexdigest()[:8]


def differences(cfg):
    """``FIELD=value`` for every field of ``cfg`` off its class's default."""
    base = type(cfg)()
    return ["%s=%s" % (f.name, getattr(cfg, f.name)) for f in fields(cfg)
            if getattr(cfg, f.name) != getattr(base, f.name)]
