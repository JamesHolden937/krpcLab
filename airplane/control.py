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

**Gains come from the craft, measured in flight.**  kRPC's
``available_torque`` (wheels, RCS, gimbals and the control surfaces *at the
present dynamic pressure and Mach*) over ``moment_of_inertia`` is the
angular acceleration a full input gives on each axis (``authority``) -- as
kRPC models it.  ``Plant`` measures how wrong that is: it fits each axis's
``w' = k accel u - d w + c`` by least squares over the windows after an
input steps, and trusts the fit as far as it explains the rate.
``pi_gains`` then puts both poles of the rate loop at ``-1/tau``, ``tau``
set by what the loop can serve -- a few of its own measured ticks -- and
by its surfaces' travel.  What is left to choose is design: how many
ticks, how much slower the attitude loops are than the rate loops.  Where
the loop still rings, ``Ringing`` sees the output reverse and halves that
axis's gain, which recovers slowly when quiet.  (This is not INDI, which
the spaceplane tried as ``SURFACE_OWN`` on 2026-10-21 and found too noisy
at kRPC's tick: the fit averages hundreds of ticks and sets only the
gains, never the command.)

**The surfaces take time to move.**  KSP slews a control surface's input
by ``actuatorSpeed x dt / ctrlSurfaceRange`` of full travel per physics
step (``Mathf.MoveTowards``; ``Mathf.Lerp`` -- a first-order lag -- when
``useExponentialSpeed``), and its angle the same way
(``ModuleControlSurface``, Assembly-CSharp): stock parts take 0.3-2.4 s
end to end.  ``actuators`` reads that per axis from the game (the window
mod's ``SurfaceActuators``), torque-weighted, wheels and RCS counted as
instant.  ``closing_time`` then keeps each loop no faster than its surfaces
can deliver the deflection a step needs, and ``RateLoop`` never commands
faster than they move.
"""

import math

from common.rateplant import (                          # noqa: F401
    Plant, actuators, authority, clamp, closing_time, pi_gains)

G0 = 9.80665
RPM = 2.0 * math.pi / 60.0              # rad/s per rev/min


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


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


# The PI loop of ``pi_gains`` (both poles at -1/tau, no damping) crosses
# over at omega tau = sqrt(2 + sqrt(5)) with atan(2 sqrt(2 + sqrt(5))) =
# 76.3 deg of phase margin, before any delay.
CROSSOVER = math.sqrt(2.0 + math.sqrt(5.0))
MARGIN_NO_DELAY_DEG = math.degrees(math.atan(2.0 * CROSSOVER))


def tau_ticks(margin_deg, delay_ticks):
    """The rate loop's closing time, in ticks, that keeps ``margin_deg`` of
    phase margin through a delay of ``delay_ticks``: a delay L costs
    ``omega L`` of phase at the crossover ``omega = CROSSOVER / tau``, so
    ``tau = CROSSOVER L / (76.3 deg - margin)``.  The loop reads the state
    a tick before it acts and holds its command for the next tick, so
    about a tick and a half is the delay of any craft at any tick; at 4
    ticks it had 32 deg of margin, and the Warbird's roll rang at +/-130
    deg/s after a stick release (LOG15483)."""
    room = math.radians(MARGIN_NO_DELAY_DEG - margin_deg)
    if room <= 0.0:
        raise ValueError("margin %.1f >= %.1f deg" % (margin_deg,
                                                       MARGIN_NO_DELAY_DEG))
    return CROSSOVER * delay_ticks / room


class RateLoop:
    """One axis: stick input from the body-rate error, PI.  The integral
    (the axis's trim) is kept in output units, held at +/- ``i_max``.  The
    output moves at most ``slew x dt`` a step (what the surfaces can
    follow), and the integral stops winding while the output is pinned --
    at +/-1 or at that slew -- the way it would wind."""

    def __init__(self, i_max):
        self.i_max = i_max
        self.i = 0.0
        self.u = 0.0

    def _limit(self, u, slew, dt):
        u = clamp(u, -1.0, 1.0)
        if slew and dt > 0.0:
            u = clamp(u, self.u - slew * dt, self.u + slew * dt)
        return u

    def step(self, want, rate, kp, ki, dt, integrate=True, proportional=True,
             slew=None, weight=1.0):
        """``weight`` of the rate asked for goes through the proportional
        term (the rest only through the integral): below 1 a step in what
        is asked for no longer kicks the input, so the PI's zero does not
        overshoot it, while a disturbance meets the full gain."""
        e = want - rate
        p = kp * (weight * want - rate) if proportional else 0.0
        raw = p + self.i
        if integrate and dt > 0.0:
            di = ki * e * dt
            u = self._limit(raw, slew, dt)
            pinned_up = u < raw - 1e-12
            pinned_down = u > raw + 1e-12
            if not ((pinned_up and di > 0.0) or (pinned_down and di < 0.0)):
                self.i = clamp(self.i + di, -self.i_max, self.i_max)
            raw = p + self.i
        self.u = self._limit(raw, slew, dt)
        return self.u


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


class Stiffness:
    """How much input holding an attitude costs per radian of angle of
    attack: the airframe's static stability over its control power,
    ``K / E`` (both scale with dynamic pressure, so the ratio is the
    airframe's), measured in flight.  In a quasi-steady hold the input
    flown (ours, the stick and the trim) balances the stability moment:
    ``u = u0 + g alpha``; ``g`` is fitted by least squares forgetting over
    ``forget_s`` on the ticks with the body rate under ``still`` rad/s,
    and is trusted (``ready``) once the angles of attack it has seen
    spread over ``spread`` rad.

    The rate loop's PI is sized on ``w' = E u`` alone; on an airframe whose
    stability moment ``K alpha`` dwarfs what its integral can supply, an
    attitude error is held off by the spring and the integral winds in at
    ``1 / (tau^3 E)`` -- the KL Jet Trainer's nose fell from 8 to -4 deg
    after its rotation with 0.05 of up input against the 0.5 it needed
    (LOG15521).  With the spring the attitude loop's slow pole is ``E ki /
    K``; putting it at the attitude loop's own ``p`` takes an attitude
    integral of ``p K / E = p g`` (input per radian-second)."""

    def __init__(self, forget_s, still, spread, g_max):
        self.forget_s, self.still = forget_s, still
        self.spread, self.g_max = spread, g_max
        self.n = 0.0                # forgetting sums: weight, x, y, xx, xy
        self.sx = self.sy = self.sxx = self.sxy = 0.0
        self.lo = self.hi = None

    def update(self, alpha, u, rate, dt):
        if alpha is None or u is None or dt <= 0.0 or abs(rate) > self.still:
            return
        lam = math.exp(-dt / self.forget_s)
        self.n = lam * self.n + 1.0
        self.sx = lam * self.sx + alpha
        self.sy = lam * self.sy + u
        self.sxx = lam * self.sxx + alpha * alpha
        self.sxy = lam * self.sxy + alpha * u
        # The spread seen, decaying toward the present over forget_s.
        f = 1.0 - lam
        self.lo = alpha if self.lo is None else min(alpha,
                                                     self.lo + f * (alpha - self.lo))
        self.hi = alpha if self.hi is None else max(alpha,
                                                     self.hi + f * (alpha - self.hi))

    @property
    def ready(self):
        return (self.lo is not None and self.hi - self.lo >= self.spread
                and self.n > 5.0)

    @property
    def g(self):
        """Input per radian of angle of attack, in [0, g_max]; 0 until
        ``ready`` (the loop then flies as it would without it)."""
        if not self.ready:
            return 0.0
        var = self.sxx / self.n - (self.sx / self.n) ** 2
        if var <= 1e-9:
            return 0.0
        cov = self.sxy / self.n - (self.sx / self.n) * (self.sy / self.n)
        return clamp(cov / var, 0.0, self.g_max)


GROUND = ("landed", "splashed", "pre_launch")


def on_ground(situation):
    """Whether a kRPC ``VesselSituation`` (or its name) has the craft on
    the ground or the water: the wheels, skids or floats carry it, so they
    and not the air hold its pitch and bank."""
    if situation is None:
        return False
    name = getattr(situation, "name", None) or str(situation).split(".")[-1]
    return name in GROUND


def tick_tau(tau, dt, tau_ticks):
    """A rate loop's closing time for a tick of ``dt``: at least
    ``tau_ticks`` of that tick.  The gains are sized on the mean tick; a
    longer one gives each of the PI terms more of the rate error than the
    loop was designed for (``kp effect dt`` = ``2 dt / tau``, ``ki effect
    dt^2`` = ``(dt / tau)^2``), and near one tick per tau the loop
    overshoots every tick and the rate flips sign tick to tick."""
    return max(tau, tau_ticks * dt) if dt > 0.0 else tau


class StickGate:
    """Whether the player is flying an axis, from the stick read back.  The
    read-back is the game's input less ours and the trim, and in the live
    game it flickers past a small deadband for a tick at a time with the
    stick untouched (LOG15411: 834 one-tick crossings in 400 s against 3-6
    a minute on the farm); every crossing let the hold go and re-took its
    target where the plane had drifted to.  So the axis is the player's
    once the stick has stayed past ``deadband`` for ``confirm_s`` (at once
    past ``firm``), and the hold's again once it has stayed inside
    ``deadband`` for ``confirm_s``."""

    def __init__(self, deadband, firm, confirm_s):
        self.deadband, self.firm, self.confirm_s = deadband, firm, confirm_s
        self.active = False
        self.held = 0.0             # how long the stick has disagreed

    def update(self, s, dt):
        if s is None:
            return self.active
        a = abs(s)
        out = a > self.deadband
        if out == self.active:
            self.held = 0.0
            return self.active
        self.held += dt
        if self.held >= self.confirm_s or (out and a > self.firm):
            self.active, self.held = out, 0.0
        return self.active


def stick(read, written, trim):
    """The player's own input on an axis: kRPC adds a client's input to
    the player's (``PilotAddon.OnFlyByWire``) and KSP folds the trim into
    it (``FlightInputHandler``), so the read-back less ours and the trim.
    None when it cannot be told."""
    if read is None or written is None:
        return None
    return clamp(read - written - (trim or 0.0), -1.0, 1.0)
