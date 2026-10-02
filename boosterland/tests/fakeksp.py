"""A tiny stand-in for kRPC plus a point-mass flight model.

It is deliberately crude -- a point mass, optional atmosphere -- but it
exercises every line of the real control loop
(streams, phase machine, GUI wiring, shutdown) and closes the loop on the
guidance: thrust commanded by the script actually moves the vehicle, so a test
can assert the booster ends up on the pad.
"""

import math
import sys
import types

G0 = 9.80665
MU = 3.5316000e12
RADIUS = 600000.0


# --- generic fakes --------------------------------------------------------
class Frame:
    def __init__(self, name):
        self.name = name


class Stream:
    """kRPC streams push values; re-evaluating on read is close enough."""

    def __init__(self, func, args):
        self.func, self.args = func, args

    def __call__(self):
        return self.func(*self.args)

    def remove(self):
        pass


class RectTransform:
    def __init__(self):
        self.size = (800, 600)
        self.position = (0, 0)


class Widget:
    def __init__(self, content=""):
        self.rect_transform = RectTransform()
        self.content = content
        self.size = 12
        self.color = (1, 1, 1)
        self.alignment = None
        self.line_spacing = 1.0
        self.visible = True

    def remove(self):
        pass


class Button(Widget):
    def __init__(self, label):
        super().__init__()
        self.text = Widget(label)
        self.clicked = False


class Panel:
    def __init__(self):
        self.rect_transform = RectTransform()
        self.texts, self.buttons, self.panels = [], [], []
        self.visible = True

    def add_panel(self):
        panel = Panel()
        self.panels.append(panel)
        return panel

    def add_text(self, content=""):
        widget = Widget(content)
        self.texts.append(widget)
        return widget

    def add_button(self, label):
        button = Button(label)
        self.buttons.append(button)
        return button

    def remove(self):
        pass


class Canvas:
    def __init__(self):
        self.rect_transform = RectTransform()
        self.panels = []

    def add_panel(self):
        panel = Panel()
        self.panels.append(panel)
        return panel


class UI:
    class TextAnchor:
        middle_center = "middle_center"
        upper_left = "upper_left"

    def __init__(self):
        self.stock_canvas = Canvas()


# --- the flight model -----------------------------------------------------
SCALE_HEIGHT = 5600.0
SOUND_SPEED = 340.0


class Body:
    name = "Kerbin"
    gravitational_parameter = MU
    equatorial_radius = RADIUS
    rotational_speed = 0.0

    def __init__(self, atmosphere=False):
        self.reference_frame = Frame("body")
        self.non_rotating_reference_frame = Frame("inertial")
        self.has_atmosphere = atmosphere
        self.atmosphere_depth = 70000.0 if atmosphere else 0.0

    def surface_position(self, lat, lon, frame):
        return (RADIUS, 0.0, 0.0)

    def density_at(self, altitude):
        if not self.has_atmosphere or altitude >= self.atmosphere_depth:
            return 0.0
        return 1.225 * math.exp(-max(0.0, altitude) / SCALE_HEIGHT)

    def pressure_at(self, altitude):
        """An isothermal atmosphere, i.e. a constant speed of sound.

        The script derives ``c = sqrt(1.4 P / rho)`` from this so it can key
        its drag curve on Mach; keeping ``c`` constant here means the fake's
        transonic hump sits at one Mach number the whole flight, which is the
        assumption the real curve does *not* get to make.
        """
        return self.density_at(altitude) * SOUND_SPEED ** 2 / 1.4


class Control:
    def __init__(self):
        self.throttle = 0.0
        self.rcs = False
        self.gear = False
        self.sas = False
        self.action_groups = {}

    def set_action_group(self, group, state):
        self.action_groups[group] = state


class AutoPilot:
    def __init__(self, vessel):
        self.vessel = vessel
        self.reference_frame = None
        self.target_direction = (1.0, 0.0, 0.0)
        self.engaged = False
        self.up_reference = None
        self.target_roll = float("nan")
        self.attitude_history = []      # (nose, up) as commanded, per call
        self.engage_failures = 0        # ticks KSP refuses to engage, LOG10
        self.engage_attempts = 0

    def __setattr__(self, name, value):
        """``engaged = True`` can throw from inside KSP.

        LOG10: engaging makes kRPC total the vessel's available torque, and
        ModuleGimbal.GetPotentialTorque raises IndexOutOfRange while the
        gimbals are still settling after staging.  kRPC surfaces the
        server-side exception as a plain ValueError, which is what killed the
        flight on its first tick.
        """
        if name == "engaged" and value:
            self.engage_attempts = getattr(self, "engage_attempts", 0) + 1
            if getattr(self, "engage_failures", 0) >= self.engage_attempts:
                raise ValueError(
                    "Index was out of range. Must be non-negative and less "
                    "than the size of the collection.\r\nParameter name: index"
                    "\nServer stack trace:\n  at ModuleGimbal."
                    "GetPotentialTorque")
        object.__setattr__(self, name, value)

    def set_direction_and_up(self, direction, up, roll=0.0):
        """kRPC's singularity-free attitude command.

        The fake does not fly roll -- it is a point mass -- so this records
        what was commanded and lets the tests check the roll *law*: that the
        reference stays perpendicular to the nose, stays in the trajectory
        plane, and never snaps to the other side between ticks.
        """
        self.target_direction = tuple(direction)
        self.up_reference = tuple(up)
        self.target_roll = roll
        self.attitude_history.append((tuple(direction), tuple(up)))


class Flight:
    def __init__(self, vessel):
        self.vessel = vessel

    @property
    def atmosphere_density(self):
        return self.vessel.body.density_at(_norm(self.vessel.r) - RADIUS)

    @property
    def surface_altitude(self):
        """Height above the ground -- or the garbage KSP sometimes returns.

        LOG4 and LOG5 both flew with this pinned at about -7e17 m, because
        KSP's terrain height under the vessel was nonsense.  The script has to
        notice and fall back rather than trust it.
        """
        if self.vessel.broken_terrain_sensor:
            return -7.56016086360505e17
        return _norm(self.vessel.r) - RADIUS

    @property
    def vertical_speed(self):
        return _dot(self.vessel.v, _unit(self.vessel.r))

    @property
    def horizontal_speed(self):
        up = _unit(self.vessel.r)
        radial = _dot(self.vessel.v, up)
        return math.sqrt(max(0.0, _dot(self.vessel.v, self.vessel.v) - radial ** 2))

    def simulate_aerodynamic_force_at(self, body, position, velocity, rotation):
        """The game's answer -- true drag, plus the noise a real probe has.

        KSP evaluates this at whatever attitude the vessel currently holds, so
        the number the script gets back jitters from tick to tick even when the
        physics has not changed.  The wobble here keeps the guidance honest:
        it has to smooth a noisy estimate rather than trust one sample.

        The attitude is read when the vessel has a ``lift_per_rad``: the
        probe's whole point is to ask about an attitude the vessel is *not*
        holding, so a fake that ignores the quaternion cannot exercise the
        steering loop at all.  The vessel's own rotation is the identity here,
        so ``vec.rotation_onto`` hands back a quaternion that carries the
        current nose onto the attitude being asked about -- applying it to the
        nose recovers exactly that attitude.
        """
        wobble = 1.0 + 0.25 * math.sin(self.vessel.aero_probes * 1.7)
        self.vessel.aero_probes += 1
        nose = None
        if self.vessel.lift_per_rad:
            nose = _quat_rotate(rotation, self.vessel.pointing)
        return self.vessel.true_aero(position, velocity, wobble, nose)


class Situation:
    landed = "landed"
    splashed = "splashed"
    flying = "flying"
    sub_orbital = "sub_orbital"


class Vessel:
    name = "Test Booster"

    def __init__(self, r, v, mass, thrust, isp, bottom=11.0, atmosphere=False,
                 slew_deg_s=None, min_throttle=0.0, gimbal_deg_s=0.0,
                 lift_per_rad=0.0):
        self.lift_per_rad = lift_per_rad    # Cl*A per radian; 0 = no wing
        self.r, self.v = tuple(r), tuple(v)
        self.bottom = bottom               # centre of mass to the leg feet
        self.broken_terrain_sensor = False
        self.broken_bounding_box = False
        # None => the old instant-pointing autopilot.  A number is how fast the
        # vehicle can actually rotate, which is the difference between a
        # steering command being obeyed and merely being issued: LOG7's
        # CORRECTION burn spent 36 s commanding an attitude 45-72 deg away and
        # never got there, and an instant autopilot cannot show that.
        self.slew_deg_s = slew_deg_s
        # Extra turn rate available *while the engines are burning*, in
        # proportion to the throttle: the gimbal.  A booster falling through
        # thick air has very little authority without it, and with
        # ``gimbal_deg_s`` at 0 (the default) the fake models only the cost of
        # thrusting off-target and none of the control it buys -- which is the
        # entire argument for CORRECTION_GIMBAL_THROTTLE.
        self.gimbal_deg_s = gimbal_deg_s
        # Real engines have a floor: a Merlin-alike will not run below ~40%,
        # and a command under it either does nothing or snaps up to the floor.
        self.min_throttle = min_throttle
        self.pointing = _unit(v)
        # Other craft that coast alongside this one -- the stage just dropped.
        # They ride on this vessel's step() so every existing test loop keeps
        # advancing the whole neighbourhood without knowing about them.
        self.companions = []
        self.max_align_error = 0.0
        self.impact_speed = None
        self.impact_horizontal = None
        self.aero_probes = 0
        self.body = Body(atmosphere)
        self.mass = mass
        self.dry_mass = mass * 0.35
        self.thrust = thrust
        self.vacuum_specific_impulse = isp
        self.specific_impulse = isp
        self.situation = Situation.sub_orbital
        self.control = Control()
        self.auto_pilot = AutoPilot(self)
        self.orbit = types.SimpleNamespace(body=self.body)
        self._flight = Flight(self)

    # -- kRPC surface --
    def flight(self, frame):
        return self._flight

    def position(self, frame):
        return self.r

    def velocity(self, frame):
        return self.v

    def direction(self, frame):
        if self.slew_deg_s is None:
            return self.auto_pilot.target_direction     # instant pointing
        return self.pointing

    def rotation(self, frame):
        return (0.0, 0.0, 0.0, 1.0)

    @property
    def reference_frame(self):
        return Frame("vessel")

    def bounding_box(self, frame):
        """The vessel's extent -- or the nonsense KSP returns through staging.

        LOG4, LOG5 and LOG6 all connected mid-separation and got a lower
        corner of about -7e17 m, which became every landing height for the
        rest of the flight.
        """
        if self.broken_bounding_box:
            return ((-1.8, -6.998429e17, -1.8), (1.8, self.bottom, 1.8))
        # +y is through the nose, so the lower corner is the tail.
        return ((-1.8, -self.bottom, -1.8), (1.8, self.bottom, 1.8))

    @property
    def available_thrust(self):
        return self.thrust if self.mass > self.dry_mass else 0.0

    @property
    def max_thrust(self):
        return self.thrust

    def drag_area(self, speed):
        """Cd*A with a transonic hump, so the estimate cannot be a constant."""
        return 25.0 * (1.0 + 0.8 * math.exp(-((speed - 320.0) / 120.0) ** 2))

    def true_drag(self, position, velocity, wobble=1.0):
        rho = self.body.density_at(_norm(position) - RADIUS)
        speed = _norm(velocity)
        if rho <= 0.0 or speed <= 0.0:
            return (0.0, 0.0, 0.0)
        q = 0.5 * rho * self.drag_area(speed) * wobble * speed
        return _scale(velocity, -q)

    def true_aero(self, position, velocity, wobble=1.0, nose=None):
        """Drag, plus the sideforce of an angle of attack when one is modelled.

        ``lift_per_rad`` is a Cl*A per radian *squared*, and it defaults to 0 -- which is
        every test written before aerodynamic steering existed, and also the
        honest default, since a point mass has no wing.  With a value set, a
        nose held ``alpha`` off retrograde produces a force perpendicular to
        the airflow, **opposite** the side the nose is tilted toward.  That
        sign is arbitrary here on purpose: the flight code measures the sign
        rather than assuming it, so a fake that guessed the other way would
        still fly.
        """
        drag = self.true_drag(position, velocity, wobble)
        if not self.lift_per_rad or nose is None:
            return drag
        rho = self.body.density_at(_norm(position) - RADIUS)
        speed = _norm(velocity)
        if rho <= 0.0 or speed <= 0.0:
            return drag
        retrograde = _scale(_unit(velocity), -1.0)
        offset = _sub(nose, _scale(retrograde, _dot(nose, retrograde)))
        if _norm(offset) < 1e-9:
            return drag
        alpha = _angle_between(nose, retrograde)
        if alpha > math.pi / 4.0:           # far off retrograde: no wing
            return drag
        side = _unit(offset)
        # alpha * |alpha|, which is the shape the real vehicle measured:
        # a cylinder in crossflow, not a wing with a linear lift curve.
        lift = 0.5 * rho * speed * speed * self.lift_per_rad * alpha * alpha
        return _add(drag, _scale(side, -lift))

    # -- physics --
    def _slew(self, dt):
        """Rotate toward the commanded attitude at a finite rate."""
        target = _unit(self.auto_pilot.target_direction)
        if self.slew_deg_s is None:
            self.pointing = target
            return
        self.max_align_error = max(self.max_align_error,
                                   _angle_between(self.pointing, target))
        rate = self.slew_deg_s + self.gimbal_deg_s * self._applied_throttle()
        self.pointing = _rotate_toward(self.pointing, target,
                                       math.radians(rate) * dt)

    def _applied_throttle(self):
        """What the engines actually run at, floor included."""
        throttle = max(0.0, min(1.0, self.control.throttle))
        if self.min_throttle > 0.0 and 0.0 < throttle < self.min_throttle:
            return self.min_throttle          # cannot throttle that deep
        return throttle

    def step(self, dt):
        self._step(dt)
        # After the booster, so a companion that tracks it sees where it is
        # now rather than where it was a step ago.
        for companion in self.companions:
            companion.step(dt)

    def _step(self, dt):
        if self.situation == Situation.landed:
            return
        self._slew(dt)
        throttle = self._applied_throttle()
        force = self.thrust * throttle
        if self.mass <= self.dry_mass:
            force = 0.0
        a = _scale(self.r, -MU / _norm(self.r) ** 3)
        nose = _unit(self.pointing) if self.lift_per_rad else None
        a = _add(a, _scale(self.true_aero(self.r, self.v, 1.0, nose),
                           1.0 / self.mass))
        if force > 0.0:
            # Thrust goes where the vehicle points, not where it was told to.
            direction = _unit(self.pointing)
            a = _add(a, _scale(direction, force / self.mass))
            self.mass = max(self.dry_mass,
                            self.mass - force / (self.vacuum_specific_impulse * G0) * dt)
        self.v = _add(self.v, _scale(a, dt))
        self.r = _add(self.r, _scale(self.v, dt))
        # The legs, not the centre of mass, are what touches the ground.
        if _norm(self.r) - RADIUS <= self.bottom:
            self.impact_speed = _norm(self.v)
            # The sideways half on its own: a booster arrives on its legs or
            # on its side, and total speed cannot tell those apart.
            up = _unit(self.r)
            self.impact_horizontal = _norm(
                _sub(self.v, _scale(up, _dot(self.v, up))))
            self.r = _scale(_unit(self.r), RADIUS + self.bottom)
            self.v = (0.0, 0.0, 0.0)
            self.situation = Situation.landed


class OtherCraft:
    """A second vessel in the neighbourhood -- the stage just dropped.

    Only the surface ``ProximityScan`` reads: where it is, how fast it is
    going, and how big it is.  It coasts on its own velocity, which is what
    the scan assumes of it too.
    """

    def __init__(self, r, v, size=10.0, name="Upper Stage", follow=None,
                 offset=(0.0, 0.0, 0.0)):
        self.r, self.v = tuple(r), tuple(v)
        self.size = size
        self.name = name
        self.broken_bounding_box = False
        # A stage that matches the booster's motion exactly and so never
        # separates from it: what the SEPARATION_MAX_COAST_S guard is for.
        self.follow = follow
        self.offset = tuple(offset)

    def position(self, frame):
        return self.r

    def velocity(self, frame):
        return self.v

    @property
    def reference_frame(self):
        return Frame("other")

    def bounding_box(self, frame):
        if self.broken_bounding_box:
            return ((-1.8, -6.998429e17, -1.8), (1.8, self.size, 1.8))
        return ((-1.8, -self.size, -1.8), (1.8, self.size, 1.8))

    def step(self, dt):
        if self.follow is not None:
            self.r = _add(self.follow.r, self.offset)
            self.v = self.follow.v
            return
        # Gravity only: no engines, and no drag model worth the trouble.  It
        # has to fall, though -- a companion that hovers drifts away from the
        # booster at 8 m/s^2 and clears in a second for entirely fake reasons.
        a = _scale(self.r, -MU / _norm(self.r) ** 3)
        self.v = _add(self.v, _scale(a, dt))
        self.r = _add(self.r, _scale(self.v, dt))


class SpaceCenter:
    VesselSituation = Situation

    def __init__(self, vessel):
        self.active_vessel = vessel
        self.vessels = [vessel]
        self.ut = 1000.0

    def transform_direction(self, direction, from_frame, to_frame):
        return direction        # the fake body does not rotate


class Connection:
    def __init__(self, vessel):
        self.space_center = SpaceCenter(vessel)
        self.ui = UI()
        self.closed = False

    def add_stream(self, func, *args):
        return Stream(func, args)

    def close(self):
        self.closed = True


def add_craft(conn, vessel, other):
    """Put ``other`` in the sky next to ``vessel``, and keep it moving."""
    conn.space_center.vessels.append(other)
    vessel.companions.append(other)
    return other


def install(vessel):
    """Put a fake ``krpc`` module in ``sys.modules`` and return the connection."""
    conn = Connection(vessel)
    module = types.ModuleType("krpc")
    module.connect = lambda **kwargs: conn
    sys.modules["krpc"] = module
    return conn


# --- vector helpers (kept local so the fake stays self-contained) ----------
def _angle_between(a, b):
    c = max(-1.0, min(1.0, _dot(_unit(a), _unit(b))))
    return math.acos(c)


def _rotate_toward(a, b, max_rad):
    """``a`` turned toward ``b`` by at most ``max_rad``, as a slerp step."""
    a, b = _unit(a), _unit(b)
    angle = _angle_between(a, b)
    if angle <= max_rad or angle < 1e-9:
        return b
    if math.pi - angle < 1e-6:          # antiparallel: any path is as good
        b = _unit(_add(b, (1e-3, 1e-3, 1e-3)))
        angle = _angle_between(a, b)
    t = max_rad / angle
    s = math.sin(angle)
    return _unit(_add(_scale(a, math.sin((1.0 - t) * angle) / s),
                      _scale(b, math.sin(t * angle) / s)))


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _quat_rotate(q, v):
    """Rotate ``v`` by the quaternion ``(x, y, z, w)``.

    The probe hands the game an attitude to evaluate at, and a fake that
    cannot read one cannot model an angle of attack.  This is the same
    convention ``common.vec.quat_rotate`` uses -- and the flight code
    verifies its own quaternions against the result rather than trusting the
    convention, so the two agreeing here is a convenience, not a contract.
    """
    x, y, z, w = q
    t = _scale(_cross((x, y, z), v), 2.0)
    return _add(_add(v, _scale(t, w)), _cross((x, y, z), t))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return a if n == 0 else _scale(a, 1.0 / n)
