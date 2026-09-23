"""Streamed vessel state.

Everything read every tick goes through a kRPC stream: streams are pushed by
the server, so a tick costs no round trips no matter how many values it reads.
All vectors are in the body's rotating reference frame, matching
:mod:`boosterland.trajectory`.
"""

import math
from dataclasses import dataclass

from . import vec


@dataclass
class Snapshot:
    ut: float
    position: tuple
    velocity: tuple
    direction: tuple
    rotation: tuple
    mass: float
    available_thrust: float
    max_thrust: float
    isp: float
    surface_altitude: float
    vertical_speed: float
    horizontal_speed: float
    throttle: float
    situation: object
    height_above_pad: float
    mean_altitude: float
    leg_clearance: float     # COM to the bottom of the vessel, metres
    surface_altitude_ok: bool = True   # False => the sensor was not believed
    terrain_max: float = 7000.0        # highest ground the body can have
    pad_above_datum: float = 0.0       # how high the pad sits over the body's datum
    terrain_below_sea: float = 100.0   # slack for ground under the datum

    @property
    def speed(self):
        return vec.norm(self.velocity)

    @property
    def legs_altitude(self):
        """Height of the landing legs above the terrain directly below.

        ``surface_altitude`` is measured at the centre of mass, so the bottom
        of a booster is a good few metres lower; cutting the engines on the
        COM height drops the vehicle that distance.  When the terrain sensor
        is not believed (see :func:`surface_altitude_sane`) the value has
        already been replaced with the height above the pad radius, so this
        stays a real height rather than nonsense.
        """
        return self.surface_altitude - self.leg_clearance

    @property
    def landing_height(self):
        """The height to plan the landing burn against, believed within reason.

        The terrain sensor says where the ground actually is; the pad radius
        says where it would be if the booster were coming down on the pad.
        The sensor is the one to fly, because the booster is only ever over
        the pad on the flight that has already worked -- but a sensor can be
        poisoned, and three flights have been lost to one that was (LOG4/5/6),
        so it is clamped to heights the ground can physically be at.  The pad
        radius is the one number here no sensor can break, and terrain around
        it is bounded on both sides: it can rise ``terrain_max`` above the pad
        and it can fall no further than the pad's own height over the datum
        plus ``terrain_below_sea``.

        Both bounds are load-bearing and each has cost a flight.  Taking the
        smaller of the two heights outright -- which is what this did -- is
        only conservative when the ground is *higher* than the pad.  LOG2 and
        LOG3 came down 1.6 km out over terrain 58 m *below* the pad radius,
        the pad-relative height reached zero while the legs were still 48 m
        up, and the engines were cut there: the booster arrived at 21 m/s
        having flown a textbook burn to ``vs=-2.0``.
        """
        floor = self.height_above_pad - self.terrain_max
        ceiling = (self.height_above_pad + self.pad_above_datum
                   + self.terrain_below_sea)
        return min(max(self.legs_altitude, floor), ceiling)

    @property
    def max_accel(self):
        """Best available thrust-to-weight.

        ``available_thrust`` drops to zero when the engines are not active, so
        fall back to ``max_thrust`` -- the landing burn still has to be planned
        for engines that are relit on the way down.
        """
        if self.mass <= 0.0:
            return 0.0
        return max(self.available_thrust, self.max_thrust) / self.mass


class Telemetry:
    def __init__(self, conn, vessel, frame, cfg, logbook=None):
        self.conn = conn
        self.vessel = vessel
        self.cfg = cfg
        self.log = logbook
        self.surface_altitude_ok = True
        self.leg_clearance = None       # measured once the game will answer
        self._leg_clearance_attempts = 0
        self._refresh_leg_clearance(0.0)
        flight = vessel.flight(frame)
        control = vessel.control
        add = conn.add_stream
        self._streams = {
            "ut": add(getattr, conn.space_center, "ut"),
            "position": add(vessel.position, frame),
            "velocity": add(vessel.velocity, frame),
            "direction": add(vessel.direction, frame),
            "rotation": add(vessel.rotation, frame),
            "mass": add(getattr, vessel, "mass"),
            "available_thrust": add(getattr, vessel, "available_thrust"),
            "max_thrust": add(getattr, vessel, "max_thrust"),
            "isp": add(getattr, vessel, "specific_impulse"),
            "situation": add(getattr, vessel, "situation"),
            "surface_altitude": add(getattr, flight, "surface_altitude"),
            "vertical_speed": add(getattr, flight, "vertical_speed"),
            "horizontal_speed": add(getattr, flight, "horizontal_speed"),
            "throttle": add(getattr, control, "throttle"),
        }

    def sample(self, env):
        s = {k: stream() for k, stream in self._streams.items()}
        position = tuple(s["position"])
        isp = s["isp"]
        if isp <= 0.0:
            isp = self.vessel.vacuum_specific_impulse
        self._refresh_leg_clearance(s["ut"])
        radius = vec.norm(position)
        mean_altitude = radius - env.equatorial_radius
        height_above_pad = radius - env.target_radius
        surface_altitude = self._surface_altitude(
            s["ut"], s["surface_altitude"], mean_altitude, height_above_pad)
        return Snapshot(
            ut=s["ut"],
            position=position,
            velocity=tuple(s["velocity"]),
            direction=tuple(s["direction"]),
            rotation=tuple(s["rotation"]),
            mass=s["mass"],
            available_thrust=s["available_thrust"],
            max_thrust=s["max_thrust"],
            isp=isp,
            surface_altitude=surface_altitude,
            vertical_speed=s["vertical_speed"],
            horizontal_speed=s["horizontal_speed"],
            throttle=s["throttle"],
            situation=s["situation"],
            height_above_pad=height_above_pad,
            mean_altitude=mean_altitude,
            leg_clearance=(self.leg_clearance if self.leg_clearance is not None
                           else self.cfg.LEG_CLEARANCE_FALLBACK_M),
            surface_altitude_ok=self.surface_altitude_ok,
            terrain_max=self.cfg.TERRAIN_MAX_M,
            pad_above_datum=max(0.0, env.target_radius - env.equatorial_radius),
            terrain_below_sea=self.cfg.TERRAIN_BELOW_SEA_M,
        )

    def _refresh_leg_clearance(self, ut):
        """Measure the COM-to-feet offset, retrying until the game answers.

        ``Vessel.bounding_box`` is not always answerable.  Through staging --
        exactly when this script connects -- KSP has returned a lower corner of
        about -7e17 m, and because the offset was measured once at startup that
        one number became every landing height for the whole flight (LOG4,
        LOG5, LOG6).  So it is checked against a length a booster could have,
        and re-measured each tick until it is one.
        """
        if self.leg_clearance is not None:
            return self.leg_clearance
        first = self._leg_clearance_attempts == 0
        self._leg_clearance_attempts += 1
        try:
            lower, _upper = self.vessel.bounding_box(self.vessel.reference_frame)
            # +y is through the nose, so the lower corner is the tail.  Deployed
            # legs are not in the part bounding box, hence the extra margin.
            clearance = max(0.0, -lower[1]) + self.cfg.LEG_CLEARANCE_MARGIN_M
        except Exception:
            clearance = None
        if clearance is None or not math.isfinite(clearance) \
                or clearance > self.cfg.LEG_CLEARANCE_MAX_M:
            if first and self.log is not None:
                # A booster this long does not exist; flying on the fallback is
                # worse than the truth but survivable, and it is retried below.
                self.log.event(ut, "bounding box not usable (%s); leg clearance "
                                   "falling back to %.1f m"
                               % (clearance, self.cfg.LEG_CLEARANCE_FALLBACK_M))
            return None
        self.leg_clearance = clearance
        if self.log is not None:
            self.log.event(ut, "leg clearance %.2f m (COM to feet)" % clearance)
        return clearance

    def _surface_altitude(self, ut, raw, mean_altitude, height_above_pad):
        """The terrain sensor, or the pad radius when it is talking nonsense.

        KSP's terrain height under the vessel is not always a number: LOG4 and
        LOG5 both flew a whole boostback with ``surface_altitude`` pinned at
        about -7e17 m, which made the landing-burn trigger fire the instant
        boostback ended and then declared touchdown at 24 km and 386 m/s.  A
        single bad sensor must not be able to do that, so it is checked against
        a height terrain can actually have and dropped when it is not one.
        """
        ok = surface_altitude_sane(raw, mean_altitude, self.cfg)
        if ok != self.surface_altitude_ok:
            self.surface_altitude_ok = ok
            if self.log is not None:
                self.log.event(ut, "surface_altitude %s: raw=%.4g terrain=%.4g"
                               % ("believed again" if ok
                                  else "REJECTED, using pad radius",
                                  raw, mean_altitude - raw))
        return raw if ok else height_above_pad

    def close(self):
        for stream in self._streams.values():
            try:
                stream.remove()
            except Exception:
                pass


def surface_altitude_sane(value, mean_altitude, cfg):
    """Could ``value`` be a real height above the ground at this altitude?

    ``surface_altitude`` is the mean altitude minus the terrain height under
    the vessel, so the terrain height it implies has to be one the body can
    have: never above ``TERRAIN_MAX_M``, and no further below sea level than
    ``TERRAIN_BELOW_SEA_M``.
    """
    if not math.isfinite(value):
        return False
    terrain = mean_altitude - value
    return -cfg.TERRAIN_BELOW_SEA_M <= terrain <= cfg.TERRAIN_MAX_M

