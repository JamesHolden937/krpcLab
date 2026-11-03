"""Extremum seeking on one command: dither it, fit the response, climb.

No kRPC here.  ``PitchSeeker`` owns the collective blade pitch.  Each tick
it is given the game time and the objective ``J`` measured at that moment
(the forward aerodynamic force per unit mass, ``autopilot.py``) and returns
the pitch to command:

    pitch(t) = centre + A sin(w t)

Over the last ``WINDOW_PERIODS`` periods of the dither it fits

    J = a + b dt + e dt^2 + s sin(w t) + c cos(w t)

by least squares.  The trend terms absorb the airspeed changing under the
fit, so it does not read as a gradient: on the takeoff roll J rose 25 m/s^2
in two seconds and a linear trend over 6 s read a false negative slope that
pulled the pitch from 68 to 23 deg into windmilling (LOG15167, crashed).
``s / A`` is the slope dJ/dpitch, seen through
the response's phase lag; ``atan2(-c, s)`` is that lag, logged so the
assumed ``PHASE_LAG_DEG`` can be checked against what the rotor delivers.
The demodulation runs against ``sin(w t - lag)`` so the in-phase part is the
slope even when the rotor's spin-up delays the thrust.

Where the fit is sure the curve bends down (the second harmonic, whose
coefficient is -J'' A^2/4, more than ``CURVATURE_MIN_T`` standard errors
below zero) feeds a smoothed curvature (``CURVATURE_TAU_S``); once there is
one the centre takes Newton steps: it moves toward ``-J'/J''`` degrees away
over ``NEWTON_TAU_S``.  One window's curvature is too noisy to step on while
the centre moves.  A clean signal alone would otherwise
step at full rate right through the optimum and hunt.  Elsewhere the
centre moves up the slope at ``RATE_MAX tanh(t / SLOPE_T_SCALE)``
degrees per second, ``t`` the in-phase coefficient over its standard error,
clamped to ``[PITCH_MIN, PITCH_MAX]``.  Stepping on the t-statistic rather
than the slope is what makes it scale-free: at rest the true slope is
0.0001-0.002 (m/s^2)/deg but clean, at 100 m/s it is ~1 and noisy, and a
gain on the raw slope sat at pitch 0 for 150 s on the first and slewed on
noise on the second (LOG15163-15167).
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
    __slots__ = ("mean", "trend", "curve", "in_phase", "quad", "slope",
                 "slope_t", "lag_deg", "curvature", "curvature_t", "n")

    def __init__(self, coef, se, amp, n):
        (self.mean, self.trend, self.curve, self.in_phase, self.quad,
         _, cos2) = coef
        self.slope = self.in_phase / amp if amp > 0 else 0.0
        # The in-phase coefficient over its standard error: how sure the fit
        # is of the slope's sign, whatever the slope's size.
        self.slope_t = self.in_phase / se[3] if se[3] > 0 else 0.0
        self.lag_deg = math.degrees(math.atan2(-self.quad, self.in_phase))
        # J(p + A sin) = ... - J'' A^2/4 cos(2 w t): the second harmonic is
        # the curvature.
        self.curvature = -4.0 * cos2 / (amp * amp) if amp > 0 else 0.0
        self.curvature_t = -cos2 / se[6] if se[6] > 0 else 0.0
        self.n = n


class PitchSeeker:
    def __init__(self, cfg, centre):
        self.cfg = cfg
        self.centre = self.clamp(centre)
        self.samples = []           # (t, J)
        self.last_t = None
        self.fit = None
        self.rate = 0.0
        self.newton = False
        self.curvature = None       # smoothed J'' over well-measured fits

    def clamp(self, p):
        return min(self.cfg.PITCH_MAX_DEG, max(self.cfg.PITCH_MIN_DEG, p))

    def omega(self):
        return 2.0 * math.pi / self.cfg.DITHER_PERIOD_S

    def command(self, t):
        return self.clamp(self.centre + self.cfg.DITHER_AMP_DEG
                          * math.sin(self.omega() * t))

    def fit_window(self, t_now):
        """Least squares of J on [1, dt, dt^2, sin, cos, sin2, cos2] over
        the window (dt in periods, so the system stays well scaled)."""
        cfg = self.cfg
        w = self.omega()
        lag = math.radians(cfg.PHASE_LAG_DEG)
        n = len(self.samples)
        k = 7
        if n < 2 * k:
            return None
        rows = []
        for t, j in self.samples:
            ph = w * t - lag
            d = (t - t_now) / cfg.DITHER_PERIOD_S
            rows.append(((1.0, d, d * d, math.sin(ph), math.cos(ph),
                          math.sin(2 * ph), math.cos(2 * ph)), j))
        m = [[0.0] * k for _ in range(k)]
        y = [0.0] * k
        for x, j in rows:
            for i in range(k):
                y[i] += x[i] * j
                for c in range(k):
                    m[i][c] += x[i] * x[c]
        coef = solve(m, y)
        if coef is None:
            return None
        rss = 0.0
        for x, j in rows:
            r = j - sum(a * b for a, b in zip(coef, x))
            rss += r * r
        # A noise floor, so a clean simulated plant does not divide by 0.
        sigma2 = max(rss / max(1, n - k), 1e-12)
        se = []
        for i in range(k):
            e = [0.0] * k
            e[i] = 1.0
            col = solve(m, e)
            se.append(math.sqrt(sigma2 * max(0.0, col[i])) if col else 0.0)
        return Fit(coef, se, cfg.DITHER_AMP_DEG, n)

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
            f = self.fit
            if f.curvature < 0.0 and f.curvature_t <= -cfg.CURVATURE_MIN_T:
                a = min(1.0, dt / cfg.CURVATURE_TAU_S)
                self.curvature = (f.curvature if self.curvature is None else
                                  self.curvature + a * (f.curvature
                                                        - self.curvature))
            dist = (-f.slope / self.curvature if self.curvature is not None
                    else None)
            if dist is not None:
                rate = max(-cfg.RATE_MAX_DEG_S, min(
                    cfg.RATE_MAX_DEG_S, dist / cfg.NEWTON_TAU_S))
            else:
                rate = cfg.RATE_MAX_DEG_S * math.tanh(
                    self.fit.slope_t / cfg.SLOPE_T_SCALE)
            self.rate = rate
            self.newton = dist is not None
            self.centre = self.clamp(self.centre + rate * min(dt, 1.0))
        return self.command(t)
