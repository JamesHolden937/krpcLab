"""An offline world built from the *measured* aerodynamic tables.

Not a physics model of a spaceplane -- a replay of what
``test_instances/planeprobe.py`` actually got back from the game, wrapped in
the interface ``spaceplane.trajectory`` and ``spaceplane.guidance`` expect.
The point is to exercise the real propagator and the real solve against real
coefficients without spending a three-minute flight on a sign error.

Its limits are worth stating, because the sibling project has a whole section
on sim results that did not survive contact with the game: the density profile
here is an exponential fit rather than KSP's layered one, there is no attitude
dynamics at all (the propagator is *given* the angle of attack, so nothing
can fail to hold it), and the coefficients are interpolated between the six
Mach/altitude pairs the probe happened to sample.  So this can show that the
guidance converges and that the signs are right.  It cannot show that the
vehicle lands.
"""
import math
import os
import re

from boosterland import vec
from spaceplane.environment import Table

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TABLE = os.path.join(os.path.dirname(HERE), "test_instances",
                             "planeprobe-gearup.txt")

MU = 3.5316000e12
R_KERBIN = 600000.0
ATMOSPHERE = 70000.0


def parse_probe(path=DEFAULT_TABLE):
    """``[(altitude, speed, {alpha: (ClA, CdA)})]`` out of a probe report."""
    rows = []
    header = re.compile(r"^\s+(\d+) m\s+(\d+) m/s")
    current = None
    alphas = None
    with open(path) as fh:
        for line in fh:
            m = header.match(line)
            if m:
                current = (float(m.group(1)), float(m.group(2)), {})
                rows.append(current)
                alphas = None
                continue
            if current is None:
                continue
            stripped = line.strip()
            if stripped.startswith("aoa "):
                # The approach section has a column header that also begins
                # "aoa" -- "aoa speed sink L/D ClA CdA" -- so the test is
                # whether the rest of the line is numbers, not what it starts
                # with.
                try:
                    alphas = [float(x) for x in stripped.split()[1:]]
                except ValueError:
                    alphas = None
                    current = None
            elif stripped.startswith("ClA ") and alphas:
                for a, v in zip(alphas, [float(x) for x in
                                         stripped.split()[1:]]):
                    current[2][a] = [v, None]
            elif stripped.startswith("CdA ") and alphas:
                for a, v in zip(alphas, [float(x) for x in
                                         stripped.split()[1:]]):
                    if a in current[2]:
                        current[2][a][1] = v
                current = current
    # The approach section carries the only *subsonic* measurement in the
    # file, as one line per angle at the speed that carries the weight, and
    # without it the offline world has no idea that Cl*A is nine times larger
    # down there (85 m^2 against 9).  Fold it in as one low-Mach row.
    low = {}
    in_glide = False
    with open(path) as fh:
        for line in fh:
            if "glide at" in line:
                in_glide = True
                continue
            if in_glide:
                if line.strip().startswith(("stall", "best glide", "flare",
                                            "min sink")):
                    in_glide = False
                    continue
                bits = line.split()
                if len(bits) == 6:
                    try:
                        alpha, _, _, _, cla, cda = (float(x) for x in bits)
                    except ValueError:
                        continue
                    low[alpha] = (cla, cda)
    if low:
        rows.append((100.0, 60.0, {a: list(v) for a, v in low.items()}))

    # Drop anything the parse did not complete.
    out = []
    for altitude, speed, table in rows:
        clean = {a: (v[0], v[1]) for a, v in table.items()
                 if v[1] is not None}
        if not clean:
            continue
        if clean:
            out.append((altitude, speed, clean))
    return out


class Runway:
    def __init__(self, cfg, radius):
        self.cfg = cfg
        self.ends = {}
        for name, lat_deg, lon, heading in (
                ("09", cfg.RUNWAY_09_LAT, cfg.RUNWAY_09_LON, 90.0),
                ("27", cfg.RUNWAY_27_LAT, cfg.RUNWAY_27_LON, 270.0)):
            lat = math.radians(lat_deg)
            lo = math.radians(lon)
            # A right-handed lat/lon sphere; the handedness never matters
            # because every answer here is a difference of two points on it.
            threshold = (radius * math.cos(lat) * math.cos(lo),
                         radius * math.sin(lat),
                         radius * math.cos(lat) * math.sin(lo))
            self.ends[name] = {"name": name, "threshold": threshold,
                               "heading": heading, "radius": radius}
        a = self.ends["09"]["threshold"]
        b = self.ends["27"]["threshold"]
        along = vec.unit(vec.sub(b, a))
        self.ends["09"]["along"] = along
        self.ends["27"]["along"] = vec.scale(along, -1.0)
        self.radius = radius
        self.midpoint = vec.scale(vec.unit(vec.add(a, b)), radius)

    def horizontal(self, end, direction):
        up = vec.unit(end["threshold"])
        return vec.unit(vec.project_out(direction, up))

    def low_gate(self, end):
        along = self.horizontal(end, end["along"])
        out = vec.add(end["threshold"],
                      vec.scale(along, -self.cfg.GATE_DIST_M))
        return vec.scale(vec.unit(out), end["radius"] + self.cfg.GATE_ALT_M)

    def high_gate(self, end):
        along = self.horizontal(end, end["along"])
        reach = (self.cfg.HAC_ALT_M - self.cfg.GATE_ALT_M) * self.cfg.HAC_LD
        out = vec.add(end["threshold"],
                      vec.scale(along, -(self.cfg.GATE_DIST_M
                                         + max(0.0, reach))))
        return vec.scale(vec.unit(out), end["radius"] + self.cfg.HAC_ALT_M)

    def gate(self, end):
        if getattr(self.cfg, "HAC_ON", False):
            return self.high_gate(end)
        return self.low_gate(end)

    def choose(self, position, velocity):
        if not self.cfg.RUNWAY_BOTH_ENDS:
            return self.ends["09"]
        # The *arrival* bearing, not the current heading.  Comparing the
        # vehicle's velocity with a runway direction as plain 3D vectors is
        # meaningless once they are far apart on a sphere: 165 degrees out,
        # local east at the vehicle is nearly opposite local east at the
        # runway, and the first offline entry picked the wrong end of the
        # runway for the whole flight because of it.  The chord from here to
        # the threshold, with the threshold's own vertical removed, is the
        # direction the vehicle will be travelling when it gets there.
        reference = self.ends["09"]["threshold"]
        chord = vec.sub(reference, position)
        track = vec.project_out(chord, vec.unit(reference))
        if vec.norm(track) < 1.0:
            track = vec.project_out(velocity, vec.unit(position))
        if vec.norm(track) < 1.0:
            return self.ends["09"]
        track = vec.unit(track)
        return max(self.ends.values(),
                   key=lambda e: vec.dot(track, self.horizontal(e,
                                                                e["along"])))


class FakeEnv:
    """The measured tables, an exponential atmosphere, and a runway."""

    def __init__(self, cfg, rows=None, spin=True, runway_alt=70.0):
        self.cfg = cfg
        self.mu = MU
        self.equatorial_radius = R_KERBIN
        self.atmosphere_depth = ATMOSPHERE
        self.rows = rows if rows is not None else parse_probe()
        self.runway = Runway(cfg, R_KERBIN + runway_alt)
        self.target_radius = self.runway.radius
        self.target = self.runway.gate(self.runway.ends["09"])
        period = 21549.425
        w = (2.0 * math.pi / period) if spin else 0.0
        self.omega = (0.0, w, 0.0)
        self._build_tables(cfg)
        self.probe_calls = 0

    # -- air ---------------------------------------------------------------
    # The densities the probe itself reported, at the altitudes it reported
    # them.  An exponential fit was tried first and is 27% low at 10 km and
    # 80% *high* at 45 km -- which for a term that goes as rho v^2 through the
    # part of the entry that does all the braking is not a detail.  Kerbin's
    # atmosphere is layered and a single scale height cannot represent it, so
    # interpolate the measurement instead of modelling it.
    DENSITY_ANCHORS = ((0.0, 1.225), (10000.0, 0.282), (20000.0, 0.04),
                       (25000.0, 0.0151), (30000.0, 0.00571),
                       (35000.0, 0.0023), (45000.0, 0.00048),
                       (55000.0, 9.63e-5), (65000.0, 1.15e-5))

    def density(self, altitude):
        if altitude >= self.atmosphere_depth:
            return 0.0
        anchors = self.DENSITY_ANCHORS
        if altitude <= anchors[0][0]:
            return anchors[0][1]
        for (h0, d0), (h1, d1) in zip(anchors, anchors[1:]):
            if h0 <= altitude <= h1:
                # Log-linear: density is exponential *within* a layer even
                # when it is not across them.
                f = (altitude - h0) / (h1 - h0)
                return d0 * math.exp(f * math.log(d1 / d0))
        # Above the last anchor, fall off on the last layer's scale height to
        # the top of the atmosphere rather than extrapolating to nothing.
        (h0, d0), (h1, d1) = anchors[-2], anchors[-1]
        scale = (h1 - h0) / math.log(d0 / d1)
        return d1 * math.exp(-(altitude - h1) / scale)

    def speed_of_sound(self, altitude):
        return max(180.0, 340.0 - 0.0015 * min(altitude, 30000.0))

    def mach(self, speed, altitude):
        return speed / self.speed_of_sound(altitude)

    # -- the table ---------------------------------------------------------
    def _build_tables(self, cfg):
        """Fold the measured rows onto the config's own bins.

        Deliberately through ``spaceplane.environment.Table``, the same class
        the flight code reads, so an offline run exercises the real
        interpolation rather than a convenient stand-in.
        """
        alphas = tuple(float(a) for a in cfg.ALPHA_BINS)
        machs = tuple(float(m) for m in cfg.MACH_BINS)
        self.lift = Table(alphas, machs)
        self.drag = Table(alphas, machs)
        for altitude, speed, table in self.rows:
            mach = self.mach(speed, altitude)
            row = min(range(len(machs)), key=lambda i: abs(machs[i] - mach))
            for column, alpha in enumerate(alphas):
                cla, cda = self._row_value(table, alpha)
                self.lift.set(row, column, cla)
                self.drag.set(row, column, cda)
        self._alphas = alphas

    def coefficients(self, alpha_deg, speed, altitude):
        if not self.drag.filled():
            return 0.0, 0.0
        mach = self.mach(speed, altitude)
        return (self.lift.lookup(alpha_deg, mach),
                self.drag.lookup(alpha_deg, mach))

    @staticmethod
    def _row_value(table, alpha):
        keys = sorted(table)
        if alpha <= keys[0]:
            return table[keys[0]]
        if alpha >= keys[-1]:
            return table[keys[-1]]
        for k0, k1 in zip(keys, keys[1:]):
            if k0 <= alpha <= k1:
                f = (alpha - k0) / (k1 - k0)
                v0, v1 = table[k0], table[k1]
                return (v0[0] + f * (v1[0] - v0[0]),
                        v0[1] + f * (v1[1] - v0[1]))
        return table[keys[-1]]

    def ready(self):
        return self.drag.filled()

    def refresh(self, *args, **kwargs):
        self.probe_calls += 1

    def set_profile(self, profile):
        pass


def circular_state(env, altitude, longitude_deg, latitude_deg=0.0):
    """A prograde circular orbit state in the *rotating* frame.

    The rotating frame is what everything in this project works in, so the
    surface rotation has to come out of the inertial speed -- forgetting that
    is worth 175 m/s at Kerbin's equator and would make every offline range
    answer wrong in the same direction.
    """
    radius = env.equatorial_radius + altitude
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    r = (radius * math.cos(lat) * math.cos(lon),
         radius * math.sin(lat),
         radius * math.cos(lat) * math.sin(lon))
    # East is +d/dlon in this file's own lat/lon mapping, which puts it at
    # cross(r, north) and *not* cross(north, r).  The first version had it the
    # other way round and flew every offline entry westward, which read as the
    # guidance refusing to converge.  The flight code never sees this: it gets
    # its directions from the game.
    east = vec.unit(vec.cross(vec.unit(r), (0.0, 1.0, 0.0)))
    inertial = math.sqrt(env.mu / radius)
    v_inertial = vec.scale(east, inertial)
    return r, vec.sub(v_inertial, vec.cross(env.omega, r))
