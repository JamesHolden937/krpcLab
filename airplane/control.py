"""The airplane pilot's laws, free of kRPC so they can be tested offline.

**Frames.**  Attitudes come from the vessel's axes in its surface frame
(kRPC ``surface_reference_frame``: x up, y north, z east) and body rates
from its angular velocity in the same frame.  KSP's frames are all
left-handed, and the one cross-product formula here gives
``d(axis)/dt = w x axis`` in every one of them (measured for the
spaceplane, ``spaceplane/allocate.py``), so the rates are read off as the
motion of the axes themselves -- no handedness is assumed.

**Angles and rates** are the aeronautical ones: bank ``phi`` positive right
wing down, pitch ``theta`` nose up, heading ``psi`` clockwise from north;
body rates ``p`` roll right, ``q`` pitch up, ``r`` yaw right.  The stick
inputs have the same senses (KSP: pitch +1 nose up, roll +1 right, yaw +1
nose right).  With them the Euler kinematics are the textbook ones::

    p = phi' - psi' sin(theta)
    q = theta' cos(phi) + psi' cos(theta) sin(phi)
    r = -theta' sin(phi) + psi' cos(theta) cos(phi)

which is how a coordinated turn is flown: a bank and a turn rate
``psi' = g tan(phi) / V`` give a pull ``q`` (the back pressure that keeps
the nose up) and a yaw ``r``; ``theta'`` asks for the pitch the hold wants
on top.
"""

import math

G0 = 9.80665
RPM = 2.0 * math.pi / 60.0              # rad/s per rev/min


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap(rad):
    """An angle into (-pi, pi]."""
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def attitude(nose, right, roof):
    """``(phi, theta, psi)`` in radians from the body axes in the surface
    frame (x up, y north, z east)."""
    theta = math.asin(clamp(nose[0], -1.0, 1.0))
    psi = math.atan2(nose[2], nose[1])
    phi = math.atan2(-right[0], roof[0])
    return phi, theta, psi


def body_rates(omega, nose, right, roof):
    """``(p, q, r)``: roll right, pitch up, yaw right (rad/s), from the
    angular velocity and the body axes in one frame."""
    belly = (-roof[0], -roof[1], -roof[2])
    p = _dot(_cross(omega, right), belly)       # right wing going down
    q = _dot(_cross(omega, nose), roof)         # nose going up
    r = _dot(_cross(omega, nose), right)        # nose going right
    return p, q, r


def sideslip(v_body):
    """Sideslip (rad) from the air velocity in the vessel's axes (x right,
    y nose, z belly): positive with the air coming from the right, which a
    yaw to the right (r > 0) takes out.  None with no airspeed."""
    if v_body is None or math.hypot(v_body[0], v_body[1]) < 1e-3:
        return None
    return math.atan2(v_body[0], v_body[1])


def euler_to_body(phi, theta, dphi, dtheta, dpsi):
    """The body rates ``(p, q, r)`` that fly these Euler rates."""
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    p = dphi - dpsi * st
    q = dtheta * cp + dpsi * ct * sp
    r = -dtheta * sp + dpsi * ct * cp
    return p, q, r


def turn_rate(phi, speed, max_g, min_speed):
    """The heading rate of a coordinated turn at bank ``phi``:
    ``g tan(phi) / V``, the bank's tangent held to what ``max_g`` of load
    allows (``sqrt(n^2 - 1)``), the speed to at least ``min_speed``."""
    t = math.tan(clamp(phi, -math.radians(89.9), math.radians(89.9)))
    cap = math.sqrt(max(0.0, max_g * max_g - 1.0))
    t = clamp(t, -cap, cap)
    return G0 * t / max(speed, min_speed)


def gain_scale(q_dyn, q_ref, lo, hi):
    """How much harder the surfaces must be pushed than at ``q_ref``: their
    moment goes with dynamic pressure."""
    if q_dyn is None or q_dyn <= 0.0:
        return hi
    return clamp(q_ref / q_dyn, lo, hi)


class RateLoop:
    """One axis: stick input from the body-rate error, PI, the gains
    scaled by ``gain_scale``.  The integral (the axis's trim) is kept in
    output units, held at +/- ``i_max``, and stops winding while the output
    is pinned the way it would wind."""

    def __init__(self, kp, ki, i_max):
        self.kp, self.ki, self.i_max = kp, ki, i_max
        self.i = 0.0

    def step(self, want, rate, scale, dt, integrate=True, proportional=True):
        e = want - rate
        p = scale * self.kp * e if proportional else 0.0
        u = p + self.i
        if integrate and dt > 0.0:
            di = scale * self.ki * e * dt
            if not ((u >= 1.0 and di > 0.0) or (u <= -1.0 and di < 0.0)):
                self.i = clamp(self.i + di, -self.i_max, self.i_max)
            u = p + self.i
        return clamp(u, -1.0, 1.0)


def stick(read, written, trim):
    """The player's own input on an axis: kRPC adds a client's input to
    the player's (``PilotAddon.OnFlyByWire``) and KSP folds the trim into
    it (``FlightInputHandler``), so the read-back less ours and the trim.
    None when it cannot be told."""
    if read is None or written is None:
        return None
    return clamp(read - written - (trim or 0.0), -1.0, 1.0)
