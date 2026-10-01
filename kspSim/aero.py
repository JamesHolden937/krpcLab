"""The vessel's aerodynamics, as tables read out of the game.

``kspSim/tools/probe.py`` fills them from ``simulate_aerodynamic_wrench_at``:
force and torque over dynamic pressure, in body axes (x right, y nose, z
belly), about the centre of mass the vessel had when it was probed.  Here
they are interpolated -- linearly in Mach and sideslip, and in angle of
attack around the whole circle.  Control deflection, gear and body rates add
delta tables measured at zero sideslip.
"""
import bisect
import math

from kspSim.curves import FloatCurve


def _bracket(grid, x):
    """(i, w): x lies between grid[i] and grid[i+1] at weight w, clamped."""
    if x <= grid[0]:
        return 0, 0.0
    if x >= grid[-1]:
        return len(grid) - 2, 1.0
    i = bisect.bisect_right(grid, x) - 1
    return i, (x - grid[i]) / (grid[i + 1] - grid[i])


def _lerp6(a, b, w):
    u = 1.0 - w
    return [a[k] * u + b[k] * w for k in range(6)]


def _delta(table, zero, offset=None):
    """table - zero, cell by cell; torques moved from a centre of mass at
    ``offset`` to the origin when the two were measured about different ones."""
    out = []
    for ti, zi in zip(table, zero):
        row = []
        for t, z in zip(ti, zi):
            d = [t[k] - z[k] for k in range(6)]
            if offset:
                f = d[:3]
                d[3] += offset[1] * f[2] - offset[2] * f[1]
                d[4] += offset[2] * f[0] - offset[0] * f[2]
                d[5] += offset[0] * f[1] - offset[1] * f[0]
            row.append(d)
        out.append(row)
    return out


class Surface:
    """One control surface: its wrench delta as a function of the mixed input
    it is deflected to, and its weight on each axis.

    Measured per surface with the others' axes switched off
    (``probe.py``, "solo"): the surfaces add exactly (0.7% over six), each
    weight is -1, 0 or +1, and KSP clamps the mixed input per surface -- the
    model predicts an unseen pitch+roll input to 2%, where adding per-axis
    tables was 119% out.
    """

    def __init__(self, solo, wrap):
        zero = solo["zero"]
        off = solo.get("com_offset")
        resp = {k: wrap(_delta(v, zero, off)) for k, v in solo["responses"].items()}

        def size(t):
            return sum(x * x for r in t for c in r for x in c)

        dom = max(("pitch", "roll", "yaw"), key=lambda ax: size(resp.get(ax + " 1.0", [[[0.0]]])))
        pts = {}
        for key, t in resp.items():
            ax, lv = key.split()
            if ax == dom:
                pts[float(lv)] = t
        first = next(iter(resp.values()))
        pts[0.0] = [[[0.0] * 6 for _ in r] for r in first]
        self.levels = sorted(pts)
        self.tabs = [pts[lv] for lv in self.levels]
        # Tables at other sideslips (``probe.probe_solo_beta``): the same
        # levels of the dominant axis, each a delta against the zero taken at
        # that sideslip.  ``betas`` includes 0 (the tables above); outside
        # the probed range the nearest is held.
        self.betas = [0.0]
        self.tabs_b = [self.tabs]
        for b, sb in sorted((solo.get("beta") or {}).items(), key=lambda kv: float(kv[0])):
            rb = {k: wrap(_delta(v, sb["zero"], off)) for k, v in sb["responses"].items()}
            if not all("%s %s" % (dom, lv) in rb for lv in self.levels if lv != 0.0):
                continue
            self.betas.append(float(b))
            self.tabs_b.append([rb["%s %s" % (dom, lv)] if lv != 0.0 else pts[0.0]
                                for lv in self.levels])
        order = sorted(range(len(self.betas)), key=lambda i: self.betas[i])
        self.betas = [self.betas[i] for i in order]
        self.tabs_b = [self.tabs_b[i] for i in order]
        self.weights = {}
        for ax in ("pitch", "roll", "yaw"):
            t = resp.get(ax + " 1.0")
            if t is None:
                self.weights[ax] = 0
                continue
            best = None
            for w in (-1, 0, 1):
                ref = self.tabs[self.levels.index(float(w))]
                e = sum((x - y) ** 2 for r1, r2 in zip(t, ref) for c1, c2 in zip(r1, r2)
                        for x, y in zip(c1, c2))
                if best is None or e < best[0]:
                    best = (e, w)
            self.weights[ax] = best[1]

    def target(self, pitch, roll, yaw):
        w = self.weights
        return max(-1.0, min(1.0, w["pitch"] * pitch + w["roll"] * roll + w["yaw"] * yaw))


class Aero:
    def __init__(self, tables, physics=None):
        self.mach = tables["mach"]
        alpha = list(tables["alpha"])
        self.beta = tables["beta"]
        base = tables["base"]
        base2 = tables.get("base2")
        # Close the circle: angle of attack wraps at +-180.
        if alpha[0] + 360.0 > alpha[-1]:
            alpha.append(alpha[0] + 360.0)
            base = [row + [row[0]] for row in base]
            if base2:
                base2 = [row + [row[0]] for row in base2]
        self.alpha = alpha
        self.base = base
        self.cube = None
        self.re_curve = None
        curve = ((physics or {}).get("curves") or {}).get("DRAG_PSEUDOREYNOLDS")
        if base2 and curve:
            self._split_reynolds(base2, tables["re"], tables["re2"], FloatCurve(curve))
        self.j0 = self.beta.index(0) if 0 in self.beta else None
        self.omega = tables.get("omega", 0.1)

        def wrap2(t):
            return [row + [row[0]] for row in t] if len(t[0]) + 1 == len(alpha) else t

        def zero_beta(mi, ai):
            return base[mi][ai][self.j0] if self.j0 is not None else base[mi][ai][len(self.beta) // 2]

        # Delta tables (mach x alpha -> 6) relative to the base at beta 0.
        def delta(t):
            t = wrap2(t)
            return [[[t[mi][ai][k] - zero_beta(mi, ai)[k] for k in range(6)]
                     for ai in range(len(alpha))] for mi in range(len(self.mach))]

        self.speed = tables.get("speed")
        self.surfaces = [Surface(solo, wrap2) for solo in tables.get("solo", [])]
        # Airbrakes (ModuleAeroSurface): delta tables against opening angle,
        # 0 degrees being the zero table (``probe.probe_aero_surfaces``).
        self.brakes = {}
        for k, b in enumerate(tables.get("aero_surfaces") or []):
            if not b or not b.get("angles"):
                continue
            deltas = [wrap2(_delta(t, b["zero"], b.get("com_offset"))) for t in b["tables"]]
            zero = [[[0.0] * 6 for _ in r] for r in deltas[0]]
            self.brakes[k] = ([0.0] + list(b["angles"]), [zero] + deltas)
            # An airbrake's control input opens it (one-sided), and in
            # flight it is already open: its solo tables at other sideslips
            # measure the opening from shut -- 5.4 N m/Pa of yaw at beta -15
            # on the booster's grid fins, where the open fin adds a fraction
            # of that -- and stacked on the brake table they flew the booster
            # 355 m median against 70.  Beta 0 only, until the brake tables
            # have a sideslip axis of their own.
            if k < len(self.surfaces):
                self.surfaces[k].betas = [0.0]
                self.surfaces[k].tabs_b = [self.surfaces[k].tabs]
        self.controls = {}
        for axis, levels in ({} if self.surfaces else tables.get("controls", {})).items():
            pts = sorted((float(lv), delta(t)) for lv, t in levels.items())
            zero = [[[0.0] * 6 for _ in alpha] for _ in self.mach]
            pts.append((0.0, zero))
            pts.sort(key=lambda p: p[0])
            self.controls[axis] = ([p[0] for p in pts], [p[1] for p in pts])
        self.damping = {}
        for i, axis in enumerate(("x", "y", "z")):
            if axis in tables.get("damping", {}):
                self.damping[i] = delta(tables["damping"][axis])
        meta = tables.get("controls_meta")
        if "gear_down" in tables and "gear_zero" in tables and meta:
            self.gear = wrap2(_delta(tables["gear_down"], tables["gear_zero"],
                                     meta.get("com_offset")))
        elif "gear_down" in tables and "controls_zero" in tables and meta:
            self.gear = wrap2(_delta(tables["gear_down"], tables["controls_zero"],
                                     meta.get("com_offset")))
        elif "gear_down" in tables:
            self.gear = delta(tables["gear_down"])
        else:
            self.gear = None

    def _split_reynolds(self, base2, re1, re2, curve):
        """base = rest + m(Re) cube, from the same table at two pseudo-Reynolds
        numbers (``probe.probe_reynolds``); a Mach whose two multipliers
        barely differ keeps the whole table as ``rest`` at its own m."""
        rest, cube, m1s = [], [], []
        for mi in range(len(self.mach)):
            m1, m2 = curve(re1[mi]), curve(re2[mi])
            r_rows, c_rows = [], []
            for ai in range(len(self.alpha)):
                r_row, c_row = [], []
                for bi in range(len(self.beta)):
                    b1 = self.base[mi][ai][bi]
                    b2 = base2[mi][ai][bi]
                    if abs(m1 - m2) < 0.05:
                        cb = [0.0] * 6
                        rs = list(b1)
                    else:
                        cb = [(b1[k] - b2[k]) / (m1 - m2) for k in range(6)]
                        rs = [b1[k] - m1 * cb[k] for k in range(6)]
                    r_row.append(rs)
                    c_row.append(cb)
                r_rows.append(r_row)
                c_rows.append(c_row)
            rest.append(r_rows)
            cube.append(c_rows)
            m1s.append(m1)
        self.base = rest
        self.cube = cube
        self.re_curve = curve

    def _plane(self, t, mi, mw, ai, aw):
        """Bilinear in (mach, alpha) of a delta table."""
        a = _lerp6(t[mi][ai], t[mi][ai + 1], aw)
        b = _lerp6(t[mi + 1][ai], t[mi + 1][ai + 1], aw)
        return _lerp6(a, b, mw)

    def _surface(self, surf, mi, mw, ai, aw, level, beta_deg=0.0):
        lv = surf.levels
        li, lw = _bracket(lv, max(lv[0], min(lv[-1], level)))

        def at(tabs):
            return _lerp6(self._plane(tabs[li], mi, mw, ai, aw),
                          self._plane(tabs[li + 1], mi, mw, ai, aw), lw)
        bs = surf.betas
        if len(bs) == 1:
            return at(surf.tabs)
        bi, bw = _bracket(bs, max(bs[0], min(bs[-1], beta_deg)))
        return _lerp6(at(surf.tabs_b[bi]), at(surf.tabs_b[bi + 1]), bw)

    def surface_delta(self, k, mach, alpha_deg, level):
        """Surface k's wrench delta over q at mixed input ``level`` (beta 0)."""
        a = (alpha_deg + 180.0) % 360.0 - 180.0
        if a < self.alpha[0]:
            a += 360.0
        mi, mw = _bracket(self.mach, mach)
        ai, aw = _bracket(self.alpha, a)
        surf = self.surfaces[k]
        lv = surf.levels
        li, lw = _bracket(lv, max(lv[0], min(lv[-1], level)))
        return _lerp6(self._plane(surf.tabs[li], mi, mw, ai, aw),
                      self._plane(surf.tabs[li + 1], mi, mw, ai, aw), lw)

    def brake_delta(self, k, mach, alpha_deg, angle):
        """Airbrake k's wrench delta over q, opened to ``angle`` degrees."""
        angles, tabs = self.brakes[k]
        a = (alpha_deg + 180.0) % 360.0 - 180.0
        if a < self.alpha[0]:
            a += 360.0
        mi, mw = _bracket(self.mach, mach)
        ai, aw = _bracket(self.alpha, a)
        li, lw = _bracket(angles, max(angles[0], min(angles[-1], angle)))
        return _lerp6(self._plane(tabs[li], mi, mw, ai, aw),
                      self._plane(tabs[li + 1], mi, mw, ai, aw), lw)

    def surface_targets(self, pitch, roll, yaw):
        return [s.target(pitch, roll, yaw) for s in self.surfaces]

    def coefficients(self, mach, alpha_deg, beta_deg, controls=None, omega=None,
                     gear=0.0, speed=None, deflections=None, reynolds=None, brakes=None):
        """(Fx, Fy, Fz, Tx, Ty, Tz) / q in body axes.

        ``deflections``: each surface's mixed input, as reached (with lag).
        ``omega``: air-relative body rates, rad/s; the damping table was taken
        at the probe's airspeed and damping goes as rate over airspeed, so it
        is rescaled by ``speed``."""
        a = (alpha_deg + 180.0) % 360.0 - 180.0
        if a < self.alpha[0]:
            a += 360.0
        mi, mw = _bracket(self.mach, mach)
        ai, aw = _bracket(self.alpha, a)
        bi, bw = _bracket(self.beta, beta_deg)
        base = self.base

        def at(t, m):
            lo = _lerp6(t[m][ai][bi], t[m][ai][bi + 1], bw)
            hi = _lerp6(t[m][ai + 1][bi], t[m][ai + 1][bi + 1], bw)
            return _lerp6(lo, hi, aw)
        c = _lerp6(at(base, mi), at(base, mi + 1), mw)
        if self.cube is not None:
            # Tables probed without the density leave the multiplier at 1.
            mult = self.re_curve(reynolds) if reynolds is not None else 1.0
            d = _lerp6(at(self.cube, mi), at(self.cube, mi + 1), mw)
            for k in range(6):
                c[k] += mult * d[k]
        if controls:
            for axis, value in controls.items():
                if not value or axis not in self.controls:
                    continue
                levels, tabs = self.controls[axis]
                li, lw = _bracket(levels, max(levels[0], min(levels[-1], value)))
                d = _lerp6(self._plane(tabs[li], mi, mw, ai, aw),
                           self._plane(tabs[li + 1], mi, mw, ai, aw), lw)
                for k in range(6):
                    c[k] += d[k]
        if deflections and self.surfaces:
            for surf, x in zip(self.surfaces, deflections):
                if not x:
                    continue
                d = self._surface(surf, mi, mw, ai, aw, x, beta_deg)
                for k in range(6):
                    c[k] += d[k]
        if omega is not None and self.damping:
            v_probe = None
            if self.speed and speed:
                v_probe = self.speed[mi] + (self.speed[mi + 1] - self.speed[mi]) * mw
            for i, w in enumerate(omega):
                if w and i in self.damping:
                    d = self._plane(self.damping[i], mi, mw, ai, aw)
                    f = w / self.omega
                    if v_probe:
                        f *= v_probe / max(speed, 1.0)
                    for k in range(6):
                        c[k] += d[k] * f
        if brakes:
            for k, angle in brakes:
                if angle and k in self.brakes:
                    d = self.brake_delta(k, mach, alpha_deg, angle)
                    for j in range(6):
                        c[j] += d[j]
        if gear and self.gear is not None:
            d = self._plane(self.gear, mi, mw, ai, aw)
            for k in range(6):
                c[k] += d[k] * gear
        return c

    @staticmethod
    def angles(v_body):
        """(alpha, beta) in degrees of an air-relative body-axis velocity."""
        speed = math.sqrt(v_body[0] ** 2 + v_body[1] ** 2 + v_body[2] ** 2)
        if speed == 0.0:
            return 0.0, 0.0, 0.0
        beta = math.degrees(math.asin(max(-1.0, min(1.0, v_body[0] / speed))))
        alpha = math.degrees(math.atan2(v_body[2], v_body[1]))
        return alpha, beta, speed
