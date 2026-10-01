"""How KSP spreads pitch/yaw/roll input over the control surfaces.

``ModuleControlSurface`` gives each surface a deflection that is a weighted
sum of the three inputs, clamped to its range, with weights set by where the
surface sits and which way it faces.  Those weights are fitted here from the
game: per-surface wrench tables (``deflection_override``) against per-axis
input tables, and checked against a combined input the fit never saw.
"""
import bisect


def delta_table(table, zero):
    return [[[table[i][j][k] - zero[i][j][k] for k in range(6)]
             for j in range(len(table[0]))] for i in range(len(table))]


def shift_torque(delta, offset):
    """Deltas about a centre of mass at ``offset`` -> about the origin."""
    out = []
    for row in delta:
        r = []
        for d in row:
            f = d[:3]
            c = (offset[1] * f[2] - offset[2] * f[1],
                 offset[2] * f[0] - offset[0] * f[2],
                 offset[0] * f[1] - offset[1] * f[0])
            r.append([d[0], d[1], d[2], d[3] + c[0], d[4] + c[1], d[5] + c[2]])
        out.append(r)
    return out


class Surface:
    """One surface's wrench delta as a function of its deflection."""

    def __init__(self, levels, tables, zero):
        pts = sorted((float(lv), delta_table(t, zero)) for lv, t in tables.items())
        nm = len(zero)
        na = len(zero[0])
        pts.append((0.0, [[[0.0] * 6 for _ in range(na)] for _ in range(nm)]))
        pts.sort(key=lambda p: p[0])
        self.levels = [p[0] for p in pts]
        self.tabs = [p[1] for p in pts]

    def at(self, x, i, j):
        """Delta at deflection ``x`` (fraction of the override range);
        beyond the measured +-1 it is extrapolated from the last segment."""
        lv = self.levels
        k = max(0, min(len(lv) - 2, bisect.bisect_right(lv, x) - 1))
        w = (x - lv[k]) / (lv[k + 1] - lv[k])
        a = self.tabs[k][i][j]
        b = self.tabs[k + 1][i][j]
        return [a[n] + (b[n] - a[n]) * w for n in range(6)]


def predict(surfaces, weights, inputs, i, j, authority=None):
    """Summed delta for inputs {axis: value} at table cell (i, j).

    KSP clamps each surface's mixed input to [-1, 1] and then scales it by
    the authority limiter, which can exceed 100% -- a control input can
    deflect a surface further than ``deflection_override`` reaches."""
    total = [0.0] * 6
    for s, surf in enumerate(surfaces):
        x = sum(weights[s].get(a, 0.0) * v for a, v in inputs.items())
        x = max(-1.0, min(1.0, x)) * (authority[s] if authority else 1.0)
        d = surf.at(x, i, j)
        for n in range(6):
            total[n] += d[n]
    return total


def error(surfaces, weights, inputs, target, authority=None):
    e = 0.0
    for i in range(len(target)):
        for j in range(len(target[0])):
            p = predict(surfaces, weights, inputs, i, j, authority)
            t = target[i][j]
            e += sum((p[n] - t[n]) ** 2 for n in range(6))
    return e


def fit(surfaces, axis_targets, authority=None, grid=None, sweeps=4):
    """axis_targets: {axis: {level: delta table}} -> per-surface weights."""
    grid = grid or [x / 20.0 for x in range(-20, 21)]
    weights = [{} for _ in surfaces]
    for axis, levels in axis_targets.items():
        for _ in range(sweeps):
            for s in range(len(surfaces)):
                best = None
                for g in grid:
                    weights[s][axis] = g
                    e = sum(error(surfaces, weights, {axis: float(lv)}, t, authority)
                            for lv, t in levels.items())
                    if best is None or e < best[0]:
                        best = (e, g)
                weights[s][axis] = best[1]
    return weights
