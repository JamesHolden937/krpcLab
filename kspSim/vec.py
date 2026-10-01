"""3-vectors as tuples (the same convention as ``common.vec``, kept local so
the simulator has no dependency on an autopilot's helpers)."""
import math


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 0 else (0.0, 0.0, 0.0)


def mat_vec(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def mat_inv(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if det == 0:
        raise ZeroDivisionError("singular matrix")
    k = 1.0 / det
    return (((e * i - f * h) * k, (c * h - b * i) * k, (b * f - c * e) * k),
            ((f * g - d * i) * k, (a * i - c * g) * k, (c * d - a * f) * k),
            ((d * h - e * g) * k, (b * g - a * h) * k, (a * e - b * d) * k))


def angle_deg(a, b):
    na, nb = norm(a), norm(b)
    if na == 0 or nb == 0:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(a, b) / (na * nb)))))
