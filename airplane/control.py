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


def actuators(lines):
    """Per axis (pitch, roll, yaw): ``(slew, lag)`` from the window mod's
    ``SurfaceActuators`` lines.  ``slew`` is how fast the axis's input can
    move, in full travel per second: the surfaces' ``actuatorSpeed /
    ctrlSurfaceRange``, combined harmonically by torque, with the torque
    that is not on a slewed surface (wheels, RCS, gimbals) counted as
    instant; None when no slewed surface acts on the axis.  ``lag`` is the
    torque-weighted first-order lag (s) of exponential surfaces (input
    ``range / speed`` plus angle ``1 / speed``), 0 with none.  None when the
    lines cannot be read."""
    if not lines or not lines[0].startswith("total"):
        return None
    tot = [float(x) for x in lines[0].split()[1:7]]
    total = [0.5 * (tot[i] + tot[i + 3]) for i in range(3)]
    slow = [0.0, 0.0, 0.0]          # sum of torque / slew
    lagged = [0.0, 0.0, 0.0]        # sum of torque x lag
    for line in lines[1:]:
        f = [float(x) for x in line.split()]
        speed, rng, expo = f[0], f[1], f[2] > 0.5
        if speed <= 0.0 or rng <= 0.0:
            continue
        for i in range(3):
            t = 0.5 * (f[3 + i] + f[6 + i])
            if expo:
                lagged[i] += t * (rng / speed + 1.0 / speed)
            else:
                slow[i] += t / (speed / rng)
    out = []
    for i in range(3):
        if total[i] <= 0.0:
            out.append((None, 0.0))
            continue
        out.append((total[i] / slow[i] if slow[i] > 0.0 else None,
                    lagged[i] / total[i]))
    return tuple(out)


def closing_time(tick_tau, accel, slew, lag, step, slew_factor, lag_factor):
    """A rate loop's closing time on one axis: at least ``tick_tau`` (what
    the loop can serve); long enough that its surfaces reach the input a
    rate step of ``step`` rad/s needs while the step is still being taken --
    the input is ``step / (accel tau)`` and it has to get there in about
    ``tau / slew_factor`` at ``slew``, so ``tau >= sqrt(slew_factor step /
    (accel slew))``; and ``lag_factor`` times a lagged surface's lag."""
    tau = tick_tau
    if slew and accel and accel > 0.0 and step > 0.0:
        tau = max(tau, math.sqrt(slew_factor * step / (accel * slew)))
    if lag:
        tau = max(tau, lag_factor * lag)
    return tau


class Plant:
    """One axis's rate response, measured in flight: ``w' = k accel u - d w
    + c`` -- ``accel`` kRPC's angular acceleration per unit input (or 1
    rad/s^2 where it has none), ``k`` how wrong that figure is, ``d`` the
    airframe's own rate damping (1/s), ``c`` the out-of-trim acceleration
    -- fitted by recursive least squares with forgetting (``forget_s``) on
    every tick: the input flown (ours, the player's and the trim) against
    the change in rate over the next tick.

    kRPC models the surfaces; on the Electric Warbird at 200 m/s its roll
    figure was 3.7x low and its pitch figure 4.4x high (2026-11-05,
    LOG15265), so gains from it rang in roll and sat 10 deg off in pitch.
    The fit starts at kRPC's figure (``k`` 1, no damping) and its
    covariance is capped (``p_max``) so the parameters an unexcited stretch
    cannot see stop drifting rather than wind up; only the ``window_s``
    after an input step of ``excite`` is fitted at all; and what is used is
    pulled back toward kRPC's figure as far as the fit fails to explain the
    rate (``quality``).  ``k`` is held in
    [``k_min``, ``k_max``], ``d`` in [0, ``d_max``]."""

    def __init__(self, forget_s, p_max, k_min, k_max, d_max, excite,
                 window_s):
        self.forget_s, self.p_max, self.excite = forget_s, p_max, excite
        self.window_s = window_s
        self.window = 0.0
        self.k_min, self.k_max, self.d_max = k_min, k_max, d_max
        self.theta = [1.0, 0.0, 0.0]            # k, -d, c
        self.p = [[p_max if i == j else 0.0 for j in range(3)]
                  for i in range(3)]
        self.n = 0
        self.u_last = None
        self.e2 = 0.0               # the fit's squared error, forgetting
        self.ym = 0.0               # the change in rate: its mean ...
        self.y2 = 0.0               # ... and its spread

    @property
    def quality(self):
        """How much of the rate's change the fit explains, 0..1: one less
        its forgetting mean squared error (taken before each update) over
        the spread of what it fitted.  0 until it has fitted anything."""
        if self.y2 <= 0.0:
            return 0.0
        return clamp(1.0 - self.e2 / self.y2, 0.0, 1.0)

    @property
    def k(self):
        """The effect relative to kRPC's figure, trusted as far as the fit
        explains the data: ``k^quality`` (geometric, between kRPC's 1 and
        the fit).  The Warbird's yaw in HOLD, cross-controlled against a
        held bank, fits at k 0.1-0.3 explaining little of the rate; taken
        at face value it set the yaw gain 3-10x high and the rudder
        jittered (2026-11-05)."""
        k = clamp(self.theta[0], self.k_min, self.k_max)
        return math.exp(self.quality * math.log(k))

    @property
    def d(self):
        """The damping, likewise trusted as far as the fit explains."""
        return self.quality * clamp(-self.theta[1], 0.0, self.d_max)

    def update(self, accel, u, w0, w1, dt):
        """Feed one tick: input ``u`` flown from rate ``w0`` to ``w1``
        over ``dt``.  Ignored without a usable step."""
        if dt <= 0.0 or dt > 0.5:
            return
        # Only an input that stepped, and the rate's answer to it, say
        # anything about the input's effect: in a steady hold the input
        # moves only as feedback on the rate, and fitted there the effect
        # slides toward nothing (the Warbird's yaw: k 1 -> 0.06 in 10 s,
        # its gain blown up, 2026-11-05).  A step opens a window_s window.
        last, self.u_last = self.u_last, u
        if last is not None and abs(u - last) >= self.excite:
            self.window = self.window_s
        else:
            self.window -= dt
        if self.window <= 0.0:
            return
        a = accel if accel and accel > 0.0 else 1.0
        # The tick's mean rate, not its first: Euler on w0 reads a damped
        # axis's effect low by (1 - exp(-d dt)) / (d dt), 18% at d 5.
        phi = (a * u, 0.5 * (w0 + w1), 1.0)
        y = (w1 - w0) / dt
        lam = math.exp(-dt / self.forget_s)
        p = self.p
        pphi = [sum(p[i][j] * phi[j] for j in range(3)) for i in range(3)]
        den = lam + sum(phi[i] * pphi[i] for i in range(3))
        gain = [x / den for x in pphi]
        err = y - sum(self.theta[i] * phi[i] for i in range(3))
        f = 1.0 - lam
        self.e2 += f * (err * err - self.e2)
        self.ym += f * (y - self.ym)
        self.y2 += f * ((y - self.ym) ** 2 - self.y2)
        self.theta = [self.theta[i] + gain[i] * err for i in range(3)]
        self.p = [[(p[i][j] - gain[i] * pphi[j]) / lam for j in range(3)]
                  for i in range(3)]
        tr = sum(self.p[i][i] for i in range(3))
        if tr > 3.0 * self.p_max:
            s = 3.0 * self.p_max / tr
            self.p = [[x * s for x in row] for row in self.p]
        self.n += 1


def pi_gains(effect, damping, tau, kp_max):
    """``(kp, ki)`` for a rate loop on ``w' = effect u - damping w``
    (``effect`` rad/s^2 per unit input): PI with both closed-loop poles at
    ``-1/tau``, ``s^2 + (damping + effect kp) s + effect ki = (s +
    1/tau)^2``.  A plant damped faster than ``2/tau`` gets no proportional
    term (it is already quick); no effectiveness, ``kp_max``.

    ``kp_max`` is the axis's own: full input for an error of the axis's
    rate cap (``1 / cap``).  An axis with almost no authority (the runway
    at walking pace) would otherwise ask for ``2 / (tau effect)`` -- 20x
    full input per rad/s on the Warbird's roll at rest, the ailerons and
    rudder slamming side to side.  Held there, the poles go where that
    gain puts them: ``ki`` places both at ``-effect kp / 2`` (critical)."""
    if effect is None or effect <= 0.0:
        return kp_max, kp_max / tau
    wn = 1.0 / tau
    kp = (2.0 * wn - damping) / effect
    if kp > kp_max:
        return kp_max, min(wn * wn, 0.25 * (effect * kp_max) ** 2) / effect
    return max(0.0, kp), wn * wn / effect


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
             slew=None):
        e = want - rate
        p = kp * e if proportional else 0.0
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
