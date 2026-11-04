"""The game's own blade aerodynamics, evaluated off-line: the thrust and
rotor torque at any collective Deploy Angle, and the angle that gives the
most thrust.  No kRPC here.

What KSP does with a rotor blade (``ModuleControlSurface.FixedUpdate`` and
``ModuleLiftingSurface``, read from Assembly-CSharp):

* The blade is one lifting surface, evaluated at one point.  The airflow is
  the velocity of the point ``velocityOffset`` from the part's origin (in
  its own axes; -5.8 m along the span for the R-25, so ~6 m from the hub)
  -- ``displaceVelocity``.  Dynamic pressure and Mach are of that point's
  speed ``W``: ``q = rho W^2 / 2``, ``M = W / a``.
* The lift normal is the part's forward (z) axis, turned by the deflection
  ``s * Deploy Angle`` about the part's right (x) axis (``s`` = the variant's
  deploy direction).  With ``u`` the flow direction and ``c = u . n``:

      lift = -n sign(c) Cl(|c|) Ml(M) k q LIFT_MULTIPLIER
      drag = -u         Cd(|c|) Md(M) k q LIFT_DRAG_MULTIPLIER

  ``k`` = deflectionLiftCoeff x ctrlSurfaceArea.  Lift is along the normal,
  not perpendicular to the flow (``perpendicularOnly`` is off).  Lift
  acts at the part's CoL offset, drag at its origin.
* No induced flow: a blade's inflow is the vessel's airspeed plus its own
  rotation, nothing else.

So for a measured airspeed vector, rotor speed and atmosphere, the thrust
along the nose at every Deploy Angle is a sum over the blades, and the best
angle is a one-dimensional search.  Each rotor's blades sit at evenly spaced
azimuths, so the blades' geometry frozen at any one moment is already an
average around the disc.

Blade load slows a rotor once its motor runs out of torque.  KSP's rotor
is a velocity drive (``ModuleRoboticServoRotor``: damper 1000, capped at
``maxTorque`` x the torque limit; no torque at all without power), so the
rotor holds its target rpm until the blades' torque reaches the motor's,
then slows until the two balance.  The search scores every angle at the
rpm the rotor would settle at.  The motor's torque is *learned*
(``PropModel.observe``): whenever a rotor runs below its target and is not
spinning up, the torque the model says its blades are putting on it at the
measured rpm is all the motor has (a power-starved motor reads near 0), and
while it holds its target the estimate recovers toward the game's cap.
"""

import math

from . import bladeTable


class Curve:
    """A KSP FloatCurve: cubic Hermite between keys (time, value, in-tangent,
    out-tangent), clamped outside them (Unity's AnimationCurve)."""

    __slots__ = ("t", "v", "tin", "tout")

    def __init__(self, keys):
        keys = sorted(keys)
        self.t = [k[0] for k in keys]
        self.v = [k[1] for k in keys]
        self.tin = [k[2] for k in keys]
        self.tout = [k[3] for k in keys]

    def __call__(self, x):
        t = self.t
        if x <= t[0]:
            return self.v[0]
        if x >= t[-1]:
            return self.v[-1]
        i = 1
        while t[i] < x:
            i += 1
        t0, t1 = t[i - 1], t[i]
        dt = t1 - t0
        s = (x - t0) / dt
        s2 = s * s
        s3 = s2 * s
        return ((2 * s3 - 3 * s2 + 1) * self.v[i - 1]
                + (s3 - 2 * s2 + s) * self.tout[i - 1] * dt
                + (-2 * s3 + 3 * s2) * self.v[i]
                + (s3 - s2) * self.tin[i] * dt)


class BladeType:
    def __init__(self, name, entry, lift_mult=None, drag_mult=None):
        self.name = name
        k = entry["dlc"] * entry["area"]
        self.kl = k * (bladeTable.LIFT_MULTIPLIER if lift_mult is None
                       else lift_mult)
        self.kd = k * (bladeTable.LIFT_DRAG_MULTIPLIER if drag_mult is None
                       else drag_mult)
        self.offset = entry["offset"] if entry["displace"] else (0.0, 0.0, 0.0)
        self.col = entry.get("col", (0.0, 0.0, 0.0))
        self.lift = Curve(entry["liftCurve"])
        self.lift_mach = Curve(entry["liftMachCurve"])
        self.drag = Curve(entry["dragCurve"])
        self.drag_mach = Curve(entry["dragMachCurve"])


def blade_type(part_name):
    """The aero data of a stock blade by part name (``Part.name``; KSP
    writes '.' for '_'), or None."""
    for name in (part_name, part_name.replace(".", "_")):
        if name in bladeTable.BLADES:
            return BladeType(name, bladeTable.BLADES[name])
    return None


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _add(a, b, s=1.0):
    return (a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2])


def _unit(a):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n) if n > 0 else a


class Blade:
    """One blade's geometry in the vessel frame, relative to its rotor's
    axis point: ``origin`` (the part's origin), ``right`` and ``normal`` (its
    x and z axes), ``point`` (the airflow point), ``col`` (where lift acts).
    ``sign`` is the deflection sense of +Deploy Angle."""

    def __init__(self, kind, origin, right, normal, sign=1.0, up=None):
        self.kind = kind
        self.origin = origin
        self.right = _unit(right)
        self.normal = _unit(normal)
        up = _unit(_cross(self.normal, self.right)) if up is None else up
        o = kind.offset
        self.point = _add(_add(_add(origin, self.right, o[0]), up, o[1]),
                          self.normal, o[2])
        c = kind.col
        self.col = _add(_add(_add(origin, self.right, c[0]), up, c[1]),
                        self.normal, c[2])
        self.sign = sign

    def normal_at(self, deploy_deg):
        """The lift normal at ``deploy_deg``: ``normal`` turned by
        ``sign * deploy`` about ``right`` (Rodrigues; Unity's AngleAxis)."""
        th = math.radians(self.sign * deploy_deg)
        c, s = math.cos(th), math.sin(th)
        k, v = self.right, self.normal
        kxv = _cross(k, v)
        kv = _dot(k, v) * (1.0 - c)
        return (v[0] * c + kxv[0] * s + k[0] * kv,
                v[1] * c + kxv[1] * s + k[1] * kv,
                v[2] * c + kxv[2] * s + k[2] * kv)

    def force(self, deploy_deg, omega, v_air, rho, sound):
        """Aero force (kN, vessel frame) and its point of action, with the
        rotor turning at ``omega`` (rad/s vector about the axis point) and
        the vessel moving at ``v_air`` through the air."""
        vel = _add(v_air, _cross(omega, self.point))
        w2 = _dot(vel, vel)
        if w2 < 1e-6:
            return (0.0, 0.0, 0.0), None
        w = math.sqrt(w2)
        u = (vel[0] / w, vel[1] / w, vel[2] / w)
        n = self.normal_at(deploy_deg)
        c = _dot(u, n)
        ac = min(1.0, abs(c))
        q = 0.5 * rho * w2
        mach = w / sound if sound > 0 else 0.0
        k = self.kind
        lift = -math.copysign(1.0, c) * k.lift(ac) * k.lift_mach(mach) * k.kl * q
        drag = -k.drag(ac) * k.drag_mach(mach) * k.kd * q
        return (n[0] * lift + u[0] * drag, n[1] * lift + u[1] * drag,
                n[2] * lift + u[2] * drag), (n, lift, u, drag)


class Rotor:
    """A rotor and its blades.  ``axis`` is the rotor's spin axis (unit,
    vessel frame), ``spin`` +1/-1 the sense of rotation about it."""

    def __init__(self, axis, blades, max_torque=None, spin=1.0):
        self.axis = _unit(axis)
        self.blades = blades
        self.cap = max_torque           # the game's motor limit
        self.max_torque = max_torque    # what the motor delivers (learned)
        self.spin = spin
        self.history = []
        self.clock = 0.0

    def thrust_torque(self, deploy_deg, rpm, v_air, rho, sound,
                      forward=(0.0, 1.0, 0.0)):
        """(thrust along ``forward`` in kN, torque the blades put on the rotor
        against its spin in kN m) at ``rpm``."""
        om = self.spin * rpm * math.pi / 30.0
        omega = (self.axis[0] * om, self.axis[1] * om, self.axis[2] * om)
        thrust = torque = 0.0
        sense = (self.axis[0] * self.spin, self.axis[1] * self.spin,
                 self.axis[2] * self.spin)
        for b in self.blades:
            f, parts = b.force(deploy_deg, omega, v_air, rho, sound)
            thrust += _dot(f, forward)
            if parts is None:
                continue
            n, lift, u, drag = parts
            fl = (n[0] * lift, n[1] * lift, n[2] * lift)
            fd = (u[0] * drag, u[1] * drag, u[2] * drag)
            torque -= _dot(_add(_cross(b.col, fl), _cross(b.origin, fd)), sense)
        return thrust, torque

    def settled(self, deploy_deg, rpm_target, v_air, rho, sound, forward):
        """(thrust, torque, rpm) where the rotor settles: at ``rpm_target``
        if the motor holds it, else where the blades' torque equals
        ``max_torque``."""
        t, q = self.thrust_torque(deploy_deg, rpm_target, v_air, rho, sound,
                                  forward)
        if self.max_torque is None or q <= self.max_torque:
            return t, q, rpm_target
        lo, hi = 0.0, rpm_target
        if self.thrust_torque(deploy_deg, 0.0, v_air, rho, sound,
                              forward)[1] >= self.max_torque:
            # Even stopped the blades load it past the motor: they drive it
            # backwards; call it stopped.
            t, q = self.thrust_torque(deploy_deg, 0.0, v_air, rho, sound,
                                      forward)
            return t, q, 0.0
        for _ in range(18):
            mid = 0.5 * (lo + hi)
            _, qm = self.thrust_torque(deploy_deg, mid, v_air, rho, sound,
                                       forward)
            if qm > self.max_torque:
                hi = mid
            else:
                lo = mid
        t, q = self.thrust_torque(deploy_deg, lo, v_air, rho, sound, forward)
        return t, q, lo


    def observe(self, rpm, rpm_target, deploy, v_air, rho, sound, dt,
                forward=(0.0, 1.0, 0.0), sag=0.02, rise_rpm=2.0, window_s=0.6,
                tau_s=0.5, recover_s=5.0):
        """Learn the motor's torque from its measured ``rpm`` with the blades
        at ``deploy``."""
        if self.cap is None or rpm is None or dt <= 0.0:
            return
        # KSP refreshes a rotor's rpm readout every 0.3 s
        # (rpmRefreshSeconds), so a rotor spinning up reads flat between
        # refreshes: it is "rising" if it rose at all over the last window.
        # Stamped after the clock advances: a tick longer than the window
        # (a slow START) must not empty the window it was just added to.
        self.clock += dt
        self.history.append((self.clock, rpm))
        while self.history and self.history[0][0] < self.clock - window_s:
            self.history.pop(0)
        # Learn only from a full window of flat readings: a rotor still at
        # rest as its motor engages reads like a dead one for a moment.
        steady = (self.history[-1][0] - self.history[0][0] >= 0.9 * window_s
                  and rpm - min(r for _, r in self.history) <= rise_rpm)
        _, load = self.thrust_torque(deploy, rpm, v_air, rho, sound, forward)
        load = max(0.0, load)
        a = min(1.0, dt / tau_s)
        if rpm < rpm_target * (1.0 - sag):
            if steady:
                self.max_torque += a * (min(load, self.cap) - self.max_torque)
        else:
            # Holding the target: the motor has at least the load, and the
            # cap is the most it can have.
            self.max_torque = max(self.max_torque, min(load, self.cap))
            self.max_torque += min(1.0, dt / recover_s) * (self.cap
                                                           - self.max_torque)


class Optimum:
    __slots__ = ("deploy", "thrust", "torques", "rpms")

    def __init__(self, deploy, thrust, torques, rpms):
        self.deploy = deploy
        self.thrust = thrust
        self.torques = torques
        self.rpms = rpms


class PropModel:
    def __init__(self, rotors, forward=(0.0, 1.0, 0.0)):
        self.rotors = rotors
        self.forward = forward

    def observe(self, rpms, rpm_targets, deploy, v_air, rho, sound, dt):
        for r, rpm, tgt in zip(self.rotors, rpms, rpm_targets):
            r.observe(rpm, tgt, deploy, v_air, rho, sound, dt, self.forward)

    def evaluate(self, deploy, rpm_targets, v_air, rho, sound):
        """Total thrust (kN) with each rotor where it settles."""
        thrust, torques, rpms = 0.0, [], []
        for r, rpm in zip(self.rotors, rpm_targets):
            t, q, w = r.settled(deploy, rpm, v_air, rho, sound, self.forward)
            thrust += t
            torques.append(q)
            rpms.append(w)
        return Optimum(deploy, thrust, torques, rpms)

    def optimum(self, rpm_targets, v_air, rho, sound, lo=0.0, hi=90.0,
                step=1.0, tol=0.05):
        """The Deploy Angle in [lo, hi] with the most thrust: a grid at
        ``step``, then golden-section on the best cell's neighbours."""
        n = max(2, int(math.ceil((hi - lo) / step)))
        grid = [lo + (hi - lo) * i / n for i in range(n + 1)]
        vals = [self.evaluate(d, rpm_targets, v_air, rho, sound).thrust
                for d in grid]
        i = max(range(len(grid)), key=vals.__getitem__)
        a = grid[max(0, i - 1)]
        b = grid[min(n, i + 1)]
        g = (math.sqrt(5.0) - 1.0) / 2.0
        c = b - g * (b - a)
        d = a + g * (b - a)
        fc = self.evaluate(c, rpm_targets, v_air, rho, sound).thrust
        fd = self.evaluate(d, rpm_targets, v_air, rho, sound).thrust
        while b - a > tol:
            if fc >= fd:
                b, d, fd = d, c, fc
                c = b - g * (b - a)
                fc = self.evaluate(c, rpm_targets, v_air, rho, sound).thrust
            else:
                a, c, fc = c, d, fd
                d = a + g * (b - a)
                fd = self.evaluate(d, rpm_targets, v_air, rho, sound).thrust
        best = 0.5 * (a + b)
        if vals[i] > max(fc, fd):          # the grid point itself (an edge)
            best = grid[i]
        return self.evaluate(best, rpm_targets, v_air, rho, sound)


def choose_signs(rotors, rpm, rho=1.2, sound=340.0, deploys=(60.0, 75.0, 85.0),
                 forward=(0.0, 1.0, 0.0)):
    """Each blade's deflection sense (and each rotor's spin sense, which the
    thrust only sees through their product): the one that pushes along
    ``forward`` at rest.  kRPC exposes neither the blade's variant (which
    sets ``deflectionDirection``) nor which way a rotor turns, and a plane
    whose blades push backwards at the craft's own pitch range is not one
    anyone built.  Returns the per-blade thrust sign margin, for the log."""
    out = []
    for r in rotors:
        r.spin = 1.0
        om = rpm * math.pi / 30.0
        omega = (r.axis[0] * om, r.axis[1] * om, r.axis[2] * om)
        for b in r.blades:
            score = {}
            for s in (1.0, -1.0):
                b.sign = s
                score[s] = sum(_dot(b.force(d, omega, (0.0, 0.0, 0.0), rho,
                                            sound)[0], forward)
                               for d in deploys)
            b.sign = 1.0 if score[1.0] >= score[-1.0] else -1.0
            out.append((b.sign, score[1.0], score[-1.0]))
    return out
