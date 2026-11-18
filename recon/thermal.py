"""KSP's aerodynamic heating, computed: each part's skin and interior
temperature over the rest of the flight at a candidate density and speed,
and the fastest speed that keeps every part under its limits.

The formulas are KSP's own (``FlightIntegrator`` in Assembly-CSharp, the
constants from ``Physics.cfg``)::

    lerp    = clamp((M - 2) / (5 - 2), 0, 1) ** 3
    T_shock = lerp(V, 21 V^0.75, lerp)
    h       = lerp(4 rho'^0.5 (8.14 + V), 7e-7 rho'^0.5 V^3, lerp)  (kW/m^2/K)
              rho' = rho, its square root only below 1 kg/m^3

and per part, skin: ``C_s dT_s/dt = h A_c (T_shock - T_s) - eA sigma
(T_s^4 - T_env^4) - G (T_s - T_i)``, interior: ``C_i dT_i/dt = G (T_s -
T_i) + Q_cond``.  What KSP does not say from outside -- each part's
convecting area ``A_c`` (wetted/cross-section lerp and the turbulence
factor folded in), its radiating ``eA`` and its skin-to-interior
conductance ``G`` -- is read off kRPC's own flux numbers for that part in
flight (``thermal_convection_flux`` etc.), so nothing here is fitted.
"""

import math

SIGMA = 5.670374e-11        # kW / m^2 / K^4


def mach_lerp(mach):
    x = min(1.0, max(0.0, (mach - 2.0) / 3.0))
    return x * x * x


def _rho_term(rho):
    return rho if rho > 1.0 else math.sqrt(max(rho, 0.0))


def shock_temperature(speed, mach):
    m = mach_lerp(mach)
    low = speed
    high = 21.0 * speed ** 0.75
    return low + (high - low) * m


def coefficient(rho, speed, mach):
    m = mach_lerp(mach)
    r = _rho_term(rho)
    low = r * (8.14 + speed) * 4.0
    high = 1e-7 * 7.0 * r * speed ** 3
    return low + (high - low) * m


class PartHeat:
    """One part's thermal constants, measured, and its state."""

    def __init__(self, name, t_max, s_max):
        self.name = name
        self.t_max = t_max
        self.s_max = s_max
        self.a_c = None          # m^2, convecting
        self.e_a = None          # m^2, emissivity x radiating area
        self.g = None            # kW/K, skin to interior
        self.c_s = None          # kJ/K, skin
        self.c_i = None          # kJ/K, interior
        self.q_cond = 0.0        # kW into the interior from neighbours
        self.t_s = None
        self.t_i = None

    @staticmethod
    def _ema(old, new, a):
        if new is None or not math.isfinite(new):
            return old
        return new if old is None else old + a * (new - old)

    def update(self, t_i, t_s, conv, rad, s2i, cond, c_i, c_s, rho, speed,
               mach, t_env, a):
        """One tick's readings (kW, K, kJ/K)."""
        self.t_i, self.t_s = t_i, t_s
        self.c_i = c_i if c_i else self.c_i
        self.c_s = c_s if c_s else self.c_s
        self.q_cond = self._ema(self.q_cond, cond, a)
        h = coefficient(rho, speed, mach)
        dt_shock = shock_temperature(speed, mach) - t_s
        # Only where the flux is well above noise.
        if conv is not None and h > 0 and abs(dt_shock) > 50.0 and mach > 1.5:
            self.a_c = self._ema(self.a_c, conv / (h * dt_shock), a)
        dr = t_s ** 4 - t_env ** 4
        if rad is not None and dr > 1e9:
            self.e_a = self._ema(self.e_a, abs(rad) / (SIGMA * dr), a)
        if s2i is not None and abs(t_s - t_i) > 20.0:
            self.g = self._ema(self.g, s2i / (t_s - t_i), a)

    def ready(self):
        return None not in (self.a_c, self.e_a, self.g, self.c_s, self.c_i,
                            self.t_s, self.t_i)

    def peak(self, rho, speed, mach, t_env, duration, step=1.0):
        """The highest fraction of either limit this part reaches over
        ``duration`` s at (rho, speed): (frac, t_s, t_i at the end)."""
        t_sh = shock_temperature(speed, mach)
        hc = coefficient(rho, speed, mach) * self.a_c
        ts, ti = self.t_s, self.t_i
        worst = max(ts / self.s_max, ti / self.t_max)
        n = max(1, int(duration / step))
        dt = duration / n
        # Explicit Euler is unstable when the skin's time constant is under
        # the step: substep it.
        k_s = (hc + self.g + 4.0 * self.e_a * SIGMA * ts ** 3) / self.c_s
        sub = max(1, int(math.ceil(dt * k_s / 0.5)))
        ds = dt / sub
        for _ in range(n):
            for _ in range(sub):
                if ts > 3.0 * self.s_max or ti > 3.0 * self.t_max:
                    break
                q_in = hc * (t_sh - ts)
                q_out = self.e_a * SIGMA * (ts ** 4 - t_env ** 4)
                q_si = self.g * (ts - ti)
                ts += ds * (q_in - q_out - q_si) / self.c_s
                ti += ds * (q_si + self.q_cond) / self.c_i
            worst = max(worst, ts / self.s_max, ti / self.t_max)
            if worst > 2.0 or not (math.isfinite(ts) and math.isfinite(ti)):
                # Well past the limit (or diverged): no need to know by how
                # much.
                return max(worst, 2.0), ts, ti
        return worst, ts, ti


def speed_limit(parts, rho, sound, t_env, distance, frac_max, v_lo=200.0,
                v_hi=3000.0, iters=18):
    """The fastest speed at which every measured part stays under
    ``frac_max`` of its limits for the ``distance`` still to fly (at that
    speed: the flight is shorter the faster it goes).  None until a part
    has been measured."""
    ready = [p for p in parts if p.ready()]
    if not ready:
        return None
    try:
        return _speed_limit(ready, rho, sound, t_env, distance, frac_max,
                            v_lo, v_hi, iters)
    except (OverflowError, ValueError, ZeroDivisionError):
        return None


def _speed_limit(ready, rho, sound, t_env, distance, frac_max, v_lo, v_hi,
                 iters):

    def ok(v):
        dur = min(distance / v, 3600.0)
        m = v / sound
        step = max(1.0, dur / 200.0)
        return all(p.peak(rho, v, m, t_env, dur, step)[0] <= frac_max
                   for p in ready)
    if not ok(v_lo):
        return v_lo
    if ok(v_hi):
        return v_hi
    for _ in range(iters):
        mid = 0.5 * (v_lo + v_hi)
        if ok(mid):
            v_lo = mid
        else:
            v_hi = mid
    return v_lo
