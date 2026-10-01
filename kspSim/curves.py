"""KSP's FloatCurve: Unity AnimationCurve keys with Hermite tangents."""
import bisect


class FloatCurve:
    def __init__(self, keys):
        """keys: [(time, value, in_tangent, out_tangent)], any order."""
        keys = sorted((tuple(k) + (0.0, 0.0))[:4] for k in keys)
        self.t = [k[0] for k in keys]
        self.keys = keys

    def __call__(self, x):
        keys = self.keys
        if x <= keys[0][0]:
            return keys[0][1]
        if x >= keys[-1][0]:
            return keys[-1][1]
        i = bisect.bisect_right(self.t, x) - 1
        t0, v0, _, out0 = keys[i]
        t1, v1, in1, _ = keys[i + 1]
        dt = t1 - t0
        s = (x - t0) / dt
        s2 = s * s
        s3 = s2 * s
        return ((2 * s3 - 3 * s2 + 1) * v0 + (s3 - 2 * s2 + s) * dt * out0
                + (-2 * s3 + 3 * s2) * v1 + (s3 - s2) * dt * in1)


def parse_cfg_curves(text):
    """{name: [(t, v, in, out)]} for every ``name { key = ... }`` block."""
    curves = {}
    name = None
    for raw in text.splitlines():
        line = raw.split("//")[0].strip()
        if not line:
            continue
        if line == "{" or line == "}":
            if line == "}":
                name = None
            continue
        if line.startswith("key ="):
            vals = [float(x) for x in line[5:].split()]
            if name is not None:
                curves.setdefault(name, []).append(vals)
            continue
        if "=" not in line:
            name = line
    return curves
