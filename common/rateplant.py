"""One rate axis of a vessel, measured in flight, and the PI gains that
close it -- shared by every pilot that sets its own rate-loop gains (the
airplane runs its own loop on them, the booster hands them to kRPC's).

kRPC's ``available_torque`` over ``moment_of_inertia`` is the angular
acceleration a full input gives on each axis as the game models it
(``authority``).  The game is often wrong about its control surfaces:
the Electric Warbird's roll 3.7x low at 200 m/s, its pitch 4.4x high
(LOG15265); the Heavy Shuttle booster's six tail fins, falling tail first,
credited a couple of hundred times the roll they give (LOG16185/16186).
``Plant`` fits how wrong (``k``) from the input flown and the rate's
answer to it.  ``actuators`` reads how fast the surfaces move (the window
mod's ``SurfaceActuators``), ``closing_time`` keeps a loop no faster than
they can serve, and ``pi_gains`` puts both closed-loop poles at -1/tau.
"""

import math


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


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
    that is not on a slewed surface (wheels, RCS, instant gimbals) counted
    as instant; None when no slewed surface acts on the axis.  ``lag`` is the
    torque-weighted first-order lag (s) of exponential surfaces (input
    ``range / speed`` plus angle ``1 / speed``) and of engine gimbals with a
    response speed (``1 / speed``, kind 2), 0 with none.  None when the
    lines cannot be read."""
    if not lines or not lines[0].startswith("total"):
        return None
    tot = [float(x) for x in lines[0].split()[1:7]]
    total = [0.5 * (tot[i] + tot[i + 3]) for i in range(3)]
    slow = [0.0, 0.0, 0.0]          # sum of torque / slew
    lagged = [0.0, 0.0, 0.0]        # sum of torque x lag
    for line in lines[1:]:
        f = [float(x) for x in line.split()]
        speed, rng, kind = f[0], f[1], f[2]
        if speed <= 0.0 or rng <= 0.0:
            continue
        for i in range(3):
            t = 0.5 * (f[3 + i] + f[6 + i])
            if kind > 1.5:
                # an engine gimbal: Mathf.Lerp by speed x dt a step
                lagged[i] += t / speed
            elif kind > 0.5:
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
