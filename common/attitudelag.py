"""The attitude loop's closing rate, measured in flight.

The nose is fitted as a *delayed copy* of the command it was given: the lag
``tau`` that minimises the mean squared angle between the nose at ``t`` and
the command ``tau`` earlier, over a window of recent ticks.  The rate is
``1 / tau``.

A first-order fit (nose rate proportional to the error) was the first model
and it read the farm's noses 3-10x slow: kRPC's attitude loop is PI, so it
follows a moving command with almost no standing error -- the nose's rate is
the command's, not the error's -- and a regression of rate on error measures
only the small residual it is closing (LOG17411: err 2.2 deg against a
command moving 6.9 deg/s, a ~0.3 s lag, fitted at 0.38 /s).  A delay is what
the steering laws need anyway: how long after a command the nose is there.

Ticks where a control input is pinned are left out: a saturated turn moves at
the vehicle's angular-acceleration limit, not at the loop's bandwidth.

Nothing here is fitted to a craft: the window is a count of lags, and the fit
answers only when the best delay explains the nose clearly better than no
delay does (a still command carries no information about the lag).  Until
then :attr:`rate` is None; after that it holds the last confident fit.
"""

import math

from common import vec

WINDOW = 8.0            # lags of history the fit keeps (at least 2 s)
MIN_TICKS = 10          # a delay needs a few samples
GAIN = 0.5              # the best delay must halve the no-delay residual
PINNED = 1.0 - 1e-3     # an input at its limit (kRPC clamps at exactly 1)


def error_vector(nose, cmd):
    """Radians from ``nose`` to ``cmd`` as a vector perpendicular to the
    nose (zero when they coincide)."""
    n = vec.unit(nose)
    c = vec.unit(cmd)
    perp = vec.project_out(c, n)
    s = vec.norm(perp)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = math.atan2(s, vec.dot(c, n))
    return vec.scale(perp, angle / s)


def _angle_sq(a, b):
    """~ the squared angle between unit vectors, without an acos."""
    return 2.0 * (1.0 - max(-1.0, min(1.0, vec.dot(a, b))))


class LagFit:
    """Feed it every tick with :meth:`update`; read :attr:`rate` (1/s) and
    :attr:`lag` (s)."""

    def __init__(self):
        self.hist = []          # (ut, nose, cmd in force since last tick)
        self.rate = None
        self.lag = None
        self.samples = 0
        self.rel_error = None   # residual at the best delay / at none
        self.hunt = None        # rad: rms scatter of the nose about the
                                # delayed command (the loop's hunt)
        self.next_fit_ut = None

    def reset(self):
        self.__init__()

    def gap(self):
        """A tick that is not this plant's: nothing is compared across it."""
        self.hist = []

    def update(self, ut, nose, cmd, saturation=None):
        """``cmd`` is the command in force since the last tick, ``nose`` this
        tick's nose."""
        if (nose is None or cmd is None or vec.norm(nose) < 0.5
                or vec.norm(cmd) < 0.5
                or (saturation is not None and saturation >= PINNED)):
            self.gap()
            return self.rate
        if self.hist and ut <= self.hist[-1][0] + 1e-6:
            return self.rate
        self.hist.append((ut, vec.unit(nose), vec.unit(cmd)))
        self.samples += 1
        span = max(2.0, WINDOW * (self.lag or 0.25))
        while len(self.hist) > MIN_TICKS and ut - self.hist[0][0] > span:
            self.hist.pop(0)
        if self.next_fit_ut is not None and ut < self.next_fit_ut:
            return self.rate
        # A few refits per lag: a full search every tick is wasted work.
        self.next_fit_ut = ut + 0.25 * (self.lag or 0.25)
        self._fit()
        return self.rate

    def _fit(self):
        h = self.hist
        n = len(h)
        if n < MIN_TICKS:
            return
        # The nose at tick i against the command in force k ticks before it
        # (h[i - k][2] has been in force since tick i - k - 1): k = 0 is the
        # command the nose has had one tick to follow.
        # Every delay is scored on the same ticks (the newest half): scored
        # on its own n - k, a longer delay dropped the oldest -- largest --
        # errors and always looked better (lag ran away to 3.5 s, gen-1115).
        best = None
        base = None
        top = n // 2
        for k in range(0, top):
            total = 0.0
            for i in range(top, n):
                total += _angle_sq(h[i][1], h[i - k][2])
            mean = total / (n - top)
            if base is None:
                base = mean
            if best is None or mean < best[1]:
                best = (k, mean)
        dt = (h[-1][0] - h[0][0]) / (n - 1)
        if base <= 1e-12:
            # The nose is on its command within a tick: a lag the loop
            # cannot resolve, so one tick -- if the command moved at all.
            moved = any(_angle_sq(h[i][2], h[i - 1][2]) > 1e-12
                        for i in range(1, n))
            if moved and self.rate is None:
                self.lag, self.rate = dt, 1.0 / dt
            if moved:
                self.hunt = 0.0
            return
        self.rel_error = best[1] / base
        if best[0] == 0:
            # Within a tick: the lag is shorter than the loop can resolve.
            if self.rate is None:
                self.lag, self.rate = dt, 1.0 / dt
            return
        if self.rel_error > GAIN:
            return
        # k ticks of command age plus the tick the newest command has had.
        self.lag = (best[0] + 1) * dt
        self.rate = 1.0 / self.lag
        self.hunt = math.sqrt(max(0.0, best[1]))
