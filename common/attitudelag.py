"""The attitude loop's closing rate, measured in flight.

The nose is fitted as a first-order follower of the command it was given:
``d(nose)/dt = rate * error``, with ``error`` the small-angle vector from the
nose to the command (radians, perpendicular to the nose).  Least squares over
the ticks, with exponential forgetting over ``WINDOW`` time constants, so a
change of plant (ignition, a valve, the throttle) is followed within a few of
its own time constants:

    rate = sum(w e . de/dt_nose) / sum(w e . e)

Ticks where a control input is pinned are left out: a saturated turn moves at
the vehicle's angular-acceleration limit, not at the loop's bandwidth, and a
flip read as a linear loop would call a fast loop slow.

Nothing here is fitted to a craft: the window and the confidence asked for are
dimensionless (counts of time constants, a relative standard error).  Until
the fit is first confident, :attr:`rate` is None -- an unknown is not a value;
after that it holds the last confident fit.
"""

import math

from common import vec

WINDOW = 4.0            # time constants of history the fit keeps
CONFIDENCE = 0.25       # relative standard error below which it answers
PINNED = 1.0 - 1e-3     # an input at its limit (kRPC clamps at exactly 1)


def error_vector(nose, cmd):
    """Radians from ``nose`` to ``cmd`` as a vector perpendicular to the
    nose, or None when either is unusable."""
    n = vec.unit(nose)
    c = vec.unit(cmd)
    perp = vec.project_out(c, n)
    s = vec.norm(perp)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = math.atan2(s, vec.dot(c, n))
    return vec.scale(perp, angle / s)


class LagFit:
    """Feed it every tick with :meth:`update`; read :attr:`rate` (1/s)."""

    def __init__(self):
        self.prev = None        # (ut, nose, cmd) of the last usable tick
        self.see = 0.0          # sum w e.e      (rad^2 s)
        self.sed = 0.0          # sum w e.ndot   (rad^2)
        self.sdd = 0.0          # sum w ndot.ndot (rad^2 / s)
        self.weight = 0.0       # sum w dt       (s)
        self.samples = 0
        self.rate = None
        self.fit = None         # the last estimate, confident or not
        self.rel_error = None

    def reset(self):
        self.__init__()

    def gap(self):
        """A tick that is not this plant's: the next one starts afresh,
        with nothing differenced across the gap."""
        self.prev = None

    def update(self, ut, nose, cmd, saturation=None):
        """``cmd`` is the command that was in force since the last tick (the
        one the nose has been following), ``nose`` this tick's nose."""
        if nose is None or cmd is None or vec.norm(nose) < 0.5 \
                or vec.norm(cmd) < 0.5:
            self.prev = None
            return self.rate
        prev, self.prev = self.prev, (ut, vec.unit(nose), vec.unit(cmd))
        if prev is None:
            return self.rate
        dt = ut - prev[0]
        if dt <= 1e-4:
            self.prev = prev
            return self.rate
        if saturation is not None and saturation >= PINNED:
            return self.rate
        # The command the nose was chasing over [prev, now] is prev's cmd.
        e = error_vector(prev[1], prev[2])
        ndot = vec.scale(vec.project_out(vec.sub(self.prev[1], prev[1]),
                                         prev[1]), 1.0 / dt)
        tau = 1.0 / self.fit if self.fit and self.fit > 0.0 else None
        if tau is not None:
            keep = math.exp(-dt / (WINDOW * tau))
            self.see *= keep
            self.sed *= keep
            self.sdd *= keep
            self.weight *= keep
        self.see += vec.dot(e, e) * dt
        self.sed += vec.dot(e, ndot) * dt
        self.sdd += vec.dot(ndot, ndot) * dt
        self.weight += dt
        self.samples += 1
        if self.see <= 1e-12:
            return self.rate
        fit = self.sed / self.see
        if fit <= 0.0:
            self.fit = None
            self.rel_error = None
            return self.rate
        self.fit = fit
        # The slope's standard error over the window's effective sample
        # count (weight/dt).  Once confident the rate is held through a
        # quiet or noisy stretch: the last measurement still stands.
        rss = max(0.0, self.sdd - fit * self.sed)
        n_eff = self.weight / dt
        if n_eff <= 2.0:
            return self.rate
        se = math.sqrt(rss / (n_eff - 1.0) / self.see)
        self.rel_error = se / fit
        if self.rel_error < CONFIDENCE:
            self.rate = fit
        return self.rate
