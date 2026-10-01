"""Quaternions as (x, y, z, w) tuples, Unity's order and kRPC's.

Unity's frames are left-handed, but a quaternion acting on a vector is the
same arithmetic in either; only cross products care, and none are here.
"""
import math


def mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def norm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def rotate(q, v):
    """q * v * q^-1 for a unit q."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def from_axis_angle(axis, angle):
    s = math.sin(angle / 2.0)
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(angle / 2.0))


def from_rotvec(v):
    """Rotation by |v| radians about v."""
    angle = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if angle < 1e-12:
        return norm((v[0] / 2.0, v[1] / 2.0, v[2] / 2.0, 1.0))
    return from_axis_angle(v, angle)


def matrix(q):
    """Rows of the rotation matrix: m[i][j], v' = m v."""
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def from_matrix(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return norm(((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                     (m[1][0] - m[0][1]) / s, 0.25 * s))
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return norm((0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
                     (m[2][1] - m[1][2]) / s))
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return norm(((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
                     (m[0][2] - m[2][0]) / s))
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return norm(((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
                 (m[1][0] - m[0][1]) / s))


def from_to(a, b):
    """Minimum-arc rotation taking direction a to direction b (Unity's
    FromToRotation, including its choice of axis at 180 degrees)."""
    na = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    nb = math.sqrt(b[0] ** 2 + b[1] ** 2 + b[2] ** 2)
    a = (a[0] / na, a[1] / na, a[2] / na)
    b = (b[0] / nb, b[1] / nb, b[2] / nb)
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    if d > 1.0 - 1e-15:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -1.0 + 1e-12:
        # Any perpendicular axis.
        axis = (0.0, -a[2], a[1]) if abs(a[0]) < 0.9 else (-a[1], a[0], 0.0)
        return from_axis_angle(axis, math.pi)
    c = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    return norm((c[0], c[1], c[2], 1.0 + d))


def to_angle_axis(q):
    """(angle in degrees, unit axis) with angle in [0, 360)."""
    q = norm(q)
    w = max(-1.0, min(1.0, q[3]))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-12:
        return 0.0, (float("inf"), float("inf"), float("inf"))
    return math.degrees(angle), (q[0] / s, q[1] / s, q[2] / s)


def angle_between(q1, q2):
    d = abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    return math.degrees(2.0 * math.acos(min(1.0, d)))
