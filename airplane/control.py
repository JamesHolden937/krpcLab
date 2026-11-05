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

**Gains come from the craft, not from a table.**  kRPC's
``available_torque`` (wheels, RCS, gimbals and the control surfaces *at the
present dynamic pressure and Mach*) over ``moment_of_inertia`` is the
angular acceleration a full input gives on each axis (``authority``).  A
rate loop meant to close in ``tau`` then needs ``kp = 1 / (accel tau)``
(``rate_gains``), and ``tau`` is set by what the loop can serve -- a few of
its own measured ticks -- not by the craft.  What is left to choose is
design: how many ticks, how much slower the attitude loops are than the
rate loops.  Where kRPC's figure is wrong high (it models the surfaces,
it does not measure them), the loop rings; ``Ringing`` sees the output
reverse and halves that axis's gain, which recovers slowly when quiet.
(Measuring the effectiveness from differenced rates was tried for the
spaceplane, ``SURFACE_OWN`` 2026-10-21: at kRPC's tick it was too noisy to
fly on.)
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


def authority(torque, inertia):
    """Angular acceleration per unit input, (pitch, roll, yaw) in rad/s^2,
    from kRPC's ``available_torque`` (``((+x, +y, +z), (-x, -y, -z))``, N m,
    vessel axes: x pitch, y roll, z yaw) and ``moment_of_inertia`` (kg m^2,
    same axes): the mean of the two directions' magnitudes over the
    inertia.  None for an axis it cannot tell."""
    if torque is None or inertia is None:
        return (None, None, None)
    pos, neg = torque
    out = []
    for i in range(3):
        t = 0.5 * (abs(pos[i]) + abs(neg[i]))
        out.append(t / inertia[i] if inertia[i] and inertia[i] > 0.0
                   else None)
    return tuple(out)


def rate_gains(accel, tau, integral_factor, kp_max):
    """``(kp, ki)`` for a body-rate loop that closes in ``tau`` seconds on
    an axis whose full input gives ``accel`` rad/s^2: the input that would
    take the rate error out in ``tau``, and an integral ``integral_factor``
    times slower.  No authority (None or 0): ``kp_max``, the loop pinned
    rather than divided by zero."""
    if accel is None or accel <= 0.0:
        kp = kp_max
    else:
        kp = min(kp_max, 1.0 / (accel * tau))
    return kp, kp / (integral_factor * tau)


class RateLoop:
    """One axis: stick input from the body-rate error, PI.  The integral
    (the axis's trim) is kept in output units, held at +/- ``i_max``, and
    stops winding while the output is pinned the way it would wind."""

    def __init__(self, i_max):
        self.i_max = i_max
        self.i = 0.0

    def step(self, want, rate, kp, ki, dt, integrate=True, proportional=True):
        e = want - rate
        p = kp * e if proportional else 0.0
        u = p + self.i
        if integrate and dt > 0.0:
            di = ki * e * dt
            if not ((u >= 1.0 and di > 0.0) or (u <= -1.0 and di < 0.0)):
                self.i = clamp(self.i + di, -self.i_max, self.i_max)
            u = p + self.i
        return clamp(u, -1.0, 1.0)


class Ringing:
    """One axis's gain multiplier: halved when the loop's output has
    reversed ``reversals`` times by at least ``swing`` within ``window_s``
    (a loop ringing at its own gain), at most once per window; raised back
    toward 1 by ``recover_per_s`` while it is quiet; never below
    ``floor``."""

    def __init__(self, reversals, swing, window_s, recover_per_s, floor):
        self.reversals, self.swing = reversals, swing
        self.window_s, self.recover = window_s, recover_per_s
        self.floor = floor
        self.mult = 1.0
        self.dir = 0            # +1 rising, -1 falling, 0 not yet moved
        self.hi = self.lo = None
        self.turns = []         # times of the reversals
        self.cut_at = -1e9

    def update(self, t, u, dt):
        """Feed the output ``u`` at time ``t``; True when it backed off."""
        if self.hi is None:
            self.hi = self.lo = u
        self.hi, self.lo = max(self.hi, u), min(self.lo, u)
        if self.dir > 0 and self.hi - u >= self.swing:
            self.turns.append(t)
            self.dir, self.lo = -1, u
        elif self.dir < 0 and u - self.lo >= self.swing:
            self.turns.append(t)
            self.dir, self.hi = 1, u
        elif self.dir == 0:
            if u - self.lo >= self.swing:
                self.dir, self.hi = 1, u
            elif self.hi - u >= self.swing:
                self.dir, self.lo = -1, u
        self.turns = [x for x in self.turns if t - x <= self.window_s]
        if (len(self.turns) >= self.reversals
                and t - self.cut_at >= self.window_s):
            self.mult = max(self.floor, 0.5 * self.mult)
            self.cut_at = t
            self.turns = []
            return True
        if not self.turns and dt > 0.0:
            self.mult = min(1.0, self.mult + self.recover * dt)
        return False


def stick(read, written, trim):
    """The player's own input on an axis: kRPC adds a client's input to
    the player's (``PilotAddon.OnFlyByWire``) and KSP folds the trim into
    it (``FlightInputHandler``), so the read-back less ours and the trim.
    None when it cannot be told."""
    if read is None or written is None:
        return None
    return clamp(read - written - (trim or 0.0), -1.0, 1.0)
