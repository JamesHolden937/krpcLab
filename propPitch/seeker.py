"""Extremum seeking on one command: dither it, fit the response, climb.

No kRPC here.  ``PitchSeeker`` owns the collective blade pitch.  Each tick
it is given the game time and the objective ``J`` measured at that moment
(the forward aerodynamic force per unit mass, ``autopilot.py``) and returns
the pitch to command:

    pitch(t) = centre + A sin(w t)

Over the last ``WINDOW_PERIODS`` periods of the dither it fits

    J = a + b (t - t_now) + s sin(w t) + c cos(w t)

by least squares.  The trend term ``b`` absorbs the airspeed changing under
the fit (an accelerating plane sees J fall steadily as drag rises), so it
does not read as a gradient.  ``s / A`` is the slope dJ/dpitch, seen through
the response's phase lag; ``atan2(-c, s)`` is that lag, logged so the
assumed ``PHASE_LAG_DEG`` can be checked against what the rotor delivers.
The demodulation runs against ``sin(w t - lag)`` so the in-phase part is the
slope even when the rotor's spin-up delays the thrust.

The centre then moves up the slope at ``RATE_MAX tanh(slope / SLOPE_SCALE)``
degrees per second, clamped to ``[PITCH_MIN, PITCH_MAX]``.
"""

import math


def solve(m, y):
    """``m x = y`` for a small dense system, Gaussian elimination with
    partial pivoting.  ``None`` if singular."""
    n = len(y)
    a = [list(row) + [y[i]] for i, row in enumerate(m)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            return None
        a[col], a[piv] = a[piv], a[col]
        for r in range(col + 1, n):
            f = a[r][col] / a[col][col]
            if f:
                for k in range(col, n + 1):
                    a[r][k] -= f * a[col][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (a[r][n] - sum(a[r][k] * x[k] for k in range(r + 1, n))) / a[r][r]
    return x


class Fit:
    __slots__ = ("mean", "trend", "in_phase", "quad", "slope", "lag_deg", "n")

    def __init__(self, mean, trend, in_phase, quad, amp, n):
        self.mean, self.trend = mean, trend
        self.in_phase, self.quad = in_phase, quad
        self.slope = in_phase / amp if amp > 0 else 0.0
        self.lag_deg = math.degrees(math.atan2(-quad, in_phase))
        self.n = n


class PitchSeeker:
    def __init__(self, cfg, centre):
        self.cfg = cfg
        self.centre = self.clamp(centre)
        self.samples = []           # (t, J)
        self.last_t = None
        self.fit = None

    def clamp(self, p):
        return min(self.cfg.PITCH_MAX_DEG, max(self.cfg.PITCH_MIN_DEG, p))

    def omega(self):
        return 2.0 * math.pi / self.cfg.DITHER_PERIOD_S

    def command(self, t):
        return self.clamp(self.centre + self.cfg.DITHER_AMP_DEG
                          * math.sin(self.omega() * t))

    def fit_window(self, t_now):
        cfg = self.cfg
        w = self.omega()
        lag = math.radians(cfg.PHASE_LAG_DEG)
        n = len(self.samples)
        if n < 8:
            return None
        rows = []
        for t, j in self.samples:
            ph = w * t - lag
            rows.append(((1.0, t - t_now, math.sin(ph), math.cos(ph)), j))
        m = [[0.0] * 4 for _ in range(4)]
        y = [0.0] * 4
        for x, j in rows:
            for i in range(4):
                y[i] += x[i] * j
                for k in range(4):
                    m[i][k] += x[i] * x[k]
        sol = solve(m, y)
        if sol is None:
            return None
        return Fit(sol[0], sol[1], sol[2], sol[3], cfg.DITHER_AMP_DEG, n)

    def step(self, t, j):
        """Record ``J`` measured at ``t`` (under the command issued for
        ``t``), move the centre, return the next command."""
        cfg = self.cfg
        if j is not None and math.isfinite(j):
            self.samples.append((t, j))
        span = cfg.WINDOW_PERIODS * cfg.DITHER_PERIOD_S
        while self.samples and self.samples[0][0] < t - span:
            self.samples.pop(0)
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        full = (self.samples
                and self.samples[-1][0] - self.samples[0][0]
                >= cfg.DITHER_PERIOD_S * min(1.0, cfg.WINDOW_PERIODS))
        self.fit = self.fit_window(t) if full else None
        if self.fit is not None and dt > 0.0:
            rate = cfg.RATE_MAX_DEG_S * math.tanh(
                self.fit.slope / cfg.SLOPE_SCALE)
            self.centre = self.clamp(self.centre + rate * min(dt, 1.0))
        return self.command(t)
