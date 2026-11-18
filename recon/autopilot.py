"""Recon: the QUDRAC flies from the runway over a point the player picks on
the map and back to the runway, as fast as its heat and 15 g allow.

Phase machine::

    STANDBY -> TAKEOFF -> OUT -> HOME -> FINAL -> FLARE -> ROLLOUT -> DONE

(START in the air goes straight to OUT.)

* **The target** is the last custom waypoint on the body (stock map view's
  waypoint tool), read at START; ``TARGET_LAT``/``LON`` without one.
* **The attitude** is the airplane pilot's CRUISE hold, run headless inside
  this one: this pilot sets its bank and pitch targets every tick and the
  airplane loops (rates measured in flight, sideslip nulled, turns
  coordinated) fly them.  The player's stick still goes on top.
* **Heat** governs the speed: the throttle is proportional on where the
  hottest part (internal or skin, each against its own limit) will be
  ``HEAT_LOOKAHEAD_S`` from now (``guidance.heat_throttle``).  At the heat
  limit with the throttle shut it stops descending.
* **Load**: total g (thrust's push included) under ``G_LIMIT``: the wings
  get what the axial load leaves (``guidance.normal_g_budget``) as the
  airplane hold's turn load and pitch-rate cap; measured g over the line
  shuts the throttle.
* **Navigation**: great-circle course to the goal; heading error into a
  coordinated turn's bank.  OUT flies to the target at an altitude it
  searches for (``seek_altitude``: up while heat limits, down while thrust
  does); past it, it carries on round the planet on the same great circle
  (a turn at Mach 5 costs more than it saves), and HOME descends to the final gate of whichever runway the
  return comes in on, slowing on the measured drag
  (``guidance.coast_speed``); FINAL flies localizer and glide slope at the
  approach speed; FLARE eases the sink; ROLLOUT brakes on the centreline.
"""

import math
import sys
import time

from airplane.autopilot import AirplanePilot
from airplane.config import Config as PlaneConfig
from common import cli
from common.krpcbatch import ask
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from common.panel import ControlPanel
from common.rpccount import RpcCounter
from . import guidance as gd
from . import thermal as th
from .config import Config, apply_overrides, defaults_fingerprint, differences

ACCENT_RECON = (1.0, 0.45, 0.35)
PER_PART = 10

STANDBY = "STANDBY"
TAKEOFF = "TAKEOFF"
OUT = "OUT"
HOME = "HOME"
FINAL = "FINAL"
FLARE = "FLARE"
ROLLOUT = "ROLLOUT"
DONE = "DONE"


def pick_waypoint(sc, body):
    """The last custom (no contract) waypoint on ``body``: (name, lat,
    lon), or None."""
    try:
        wps = list(sc.waypoint_manager.waypoints)
    except Exception:                                   # noqa: BLE001
        return None
    found = None
    for w in wps:
        try:
            if w.has_contract or w.body.name != body.name:
                continue
            found = (w.name, w.latitude, w.longitude)
        except Exception:                               # noqa: BLE001
            continue
    return found


class _EventsOnly:
    """The inner hold's log: its events go into ours, its telemetry does
    not (one logbook's interval gate would let only one of the two FLY
    lines through)."""

    def __init__(self, log):
        self.log = log

    def event(self, ut, text):
        self.log.event(ut, "plane: " + text)

    def telemetry(self, ut, text):
        return False


class ReconPilot:
    def __init__(self, conn, cfg, log):
        self.conn = conn
        self.cfg = cfg
        self.log = log
        self.sc = conn.space_center
        self.vessel = self.sc.active_vessel
        self.body = self.vessel.orbit.body
        self.radius = self.body.equatorial_radius
        self.state = STANDBY
        self.finished = False
        self.exit_reason = None
        self.autostart = False
        self.rpc = None
        self.loop_rate = LoopRate()
        self.lines = []
        ut = self.sc.ut
        log.event(ut, "vessel=%s parts=%d" % (self.vessel.name,
                                               len(self.vessel.parts.all)))
        log.event(ut, "config: %s defaults_fingerprint=%s"
                  % (" ".join(differences(cfg)) or "(defaults)",
                     defaults_fingerprint()))
        # The attitude hold: the airplane pilot, CRUISE, headless.
        pcfg = PlaneConfig()
        pcfg.MODE = 0
        self.plane = AirplanePilot(conn, pcfg, _EventsOnly(log),
                                   headless=True)
        self.airborne = False
        self.flight = self.vessel.flight(self.body.reference_frame)
        # Pitch and heading against the horizon (the body frame's read odd).
        self.sflight = self.vessel.flight(self.vessel.surface_reference_frame)
        self.gui = ControlPanel(conn, "RECON", "START", ACCENT_RECON)
        self.parts = list(self.vessel.parts.all)
        self.part_names = [pt.name for pt in self.parts]
        self.engine = None
        self.target = None
        self.runway = None          # (lat, lon, course) of the chosen threshold
        self.closest = None
        self.heat = None            # filtered (frac, rate)
        self.heat_name = ""
        self.last = None            # (ut, speed) for the drag measurement
        # Drag: rho CdA / 2m per metre, measured below Mach 0.9 (the
        # approach's regime).  Probe 1's climb: CdA ~0.87 m^2 at Mach 0.65.
        self.cda = 0.87
        self.start_ut = None
        self.throttle = 0.0
        self.h_seek = None          # the searched cruise altitude
        self.accel = 0.0            # dV/dt, filtered
        self.fuel_rate = None       # LiquidFuel per game second, filtered
        self.last_fuel = None
        self.around = False         # going home the long way round
        self.res = self.vessel.resources
        self.cla = None             # [(alpha rad, Cl*A m^2)], low speed
        self.alpha_rot = 0.0
        self.v_liftoff = 0.0
        self.rotating = False
        self.heat_models = None
        self.v_heat = None          # the computed fastest safe speed
        self.next_plan = 0.0

    # -- reads ------------------------------------------------------------------
    def read(self):
        fl, v = self.flight, self.vessel
        calls = [(getattr, (fl, "latitude")), (getattr, (fl, "longitude")),
                 (getattr, (fl, "speed")), (getattr, (fl, "vertical_speed")),
                 (getattr, (fl, "mean_altitude")),
                 (getattr, (fl, "surface_altitude")),
                 (getattr, (self.sflight, "heading")),
                 (getattr, (fl, "g_force")),
                 (getattr, (fl, "mach")), (getattr, (fl, "atmosphere_density")),
                 (getattr, (v, "thrust")), (getattr, (v, "mass")),
                 (getattr, (v, "situation")),
                 (getattr, (self.sflight, "pitch")),
                 (self.res.amount, ("LiquidFuel",))]
        calls += [(getattr, (fl, "static_air_temperature")),
                  (getattr, (fl, "speed_of_sound"))]
        for pt in self.parts:
            calls += [(getattr, (pt, "temperature")),
                      (getattr, (pt, "max_temperature")),
                      (getattr, (pt, "skin_temperature")),
                      (getattr, (pt, "max_skin_temperature")),
                      (getattr, (pt, "thermal_convection_flux")),
                      (getattr, (pt, "thermal_radiation_flux")),
                      (getattr, (pt, "thermal_skin_to_internal_flux")),
                      (getattr, (pt, "thermal_conduction_flux")),
                      (getattr, (pt, "thermal_mass")),
                      (getattr, (pt, "thermal_skin_mass"))]
        return ask(self.conn, calls)

    def hottest(self, got):
        frac, name, lost = 0.0, "", []
        for i, pt in enumerate(self.parts):
            t, tm, s, sm = got[PER_PART * i:PER_PART * i + 4]
            if None in (t, tm, s, sm) or tm <= 0 or sm <= 0:
                lost.append(self.part_names[i])
                continue
            f = max(t / tm, s / sm)
            if f > frac:
                frac, name = f, self.part_names[i]
        return frac, name, lost

    # -- takeover ---------------------------------------------------------------
    def take_over(self, ut):
        cfg = self.cfg
        wp = pick_waypoint(self.sc, self.body)
        if wp is None and not (math.isnan(cfg.TARGET_LAT)
                               or math.isnan(cfg.TARGET_LON)):
            wp = ("config", cfg.TARGET_LAT, cfg.TARGET_LON)
        if wp is None:
            return False
        self.target = wp
        engines = self.vessel.parts.engines
        self.engine = engines[0] if engines else None
        if self.engine is not None:
            # Air-breathing only: the craft carries no oxidizer.
            try:
                self.engine.auto_mode_switch = False
                if "air" not in str(self.engine.mode).lower():
                    self.engine.toggle_mode()
                self.engine.active = True
            except Exception as exc:                    # noqa: BLE001
                self.log.event(ut, "engine: %r" % exc)
        self.rho_ground = self.flight.atmosphere_density or 1.2
        self.plan_takeoff(ut)
        self.plane.take_over(ut)
        c = self.vessel.control
        c.brakes = False
        self.start_ut = ut
        lat, lon = self.flight.latitude, self.flight.longitude
        self.log.event(ut, "target %s at %.4f, %.4f: %.1f km, course %.1f"
                       % (wp[0], wp[1], wp[2],
                          gd.distance(lat, lon, wp[1], wp[2], self.radius) / 1e3,
                          math.degrees(gd.bearing(lat, lon, wp[1], wp[2]))))
        return True

    # -- the loop ---------------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s%s" % (self.state, state,
                                                 (": " + note) if note else ""))
        self.state = state

    def finish(self, reason):
        self.exit_reason = reason
        self.finished = True

    def lift_curve(self, ut):
        """Cl*A over alpha 0..LIFT_SWEEP_MAX_DEG, from KSP's own
        aerodynamics at the vessel's place and attitude (wings level), the
        airflow at alpha below the nose in its plane of symmetry."""
        cfg = self.cfg
        ref = self.body.reference_frame
        vf = self.vessel.reference_frame
        sc = self.sc
        pos = self.vessel.position(ref)
        rot = self.vessel.rotation(ref)
        nose = sc.transform_direction((0.0, 1.0, 0.0), vf, ref)
        belly = sc.transform_direction((0.0, 0.0, 1.0), vf, ref)
        rho = self.flight.atmosphere_density or 1.2
        v = 100.0
        q = 0.5 * rho * v * v
        out = []
        for deg in range(0, int(cfg.LIFT_SWEEP_MAX_DEG) + 1):
            a = math.radians(deg)
            d = tuple(math.cos(a) * n + math.sin(a) * b
                      for n, b in zip(nose, belly))
            try:
                f = self.flight.simulate_aerodynamic_force_at(
                    self.body, pos, tuple(v * x for x in d), rot)
            except Exception:                           # noqa: BLE001
                continue
            # Lift: the force's part against the belly, square to the flow.
            up = tuple(-b for b in belly)
            lift_dir = tuple(u - d_ * sum(uu * dd for uu, dd in zip(up, d))
                             for u, d_ in zip(up, d))
            norm = math.sqrt(sum(x * x for x in lift_dir)) or 1.0
            lift = sum(fx * lx / norm for fx, lx in zip(f, lift_dir))
            out.append((a, lift / q))
        self.cla = out or None
        if self.cla:
            peak = max(self.cla, key=lambda x: x[1])
            self.log.event(ut, "lift curve (rho %.3f): Cl*A %s; peak %.2f m^2 "
                           "at %.0f deg" % (rho, " ".join(
                               "%d:%.2f" % (round(math.degrees(a)), c)
                               for a, c in self.cla[::5]), peak[1],
                               math.degrees(peak[0])))

    def cla_at(self, alpha):
        """Cl*A at ``alpha`` off the swept curve (linear between)."""
        pts = self.cla or [(0.0, 0.0)]
        if alpha <= pts[0][0]:
            return pts[0][1]
        for (a0, c0), (a1, c1) in zip(pts, pts[1:]):
            if alpha <= a1:
                return c0 + (c1 - c0) * (alpha - a0) / (a1 - a0)
        return pts[-1][1]

    def cla_max(self):
        return max(c for _, c in self.cla) if self.cla else None

    def tail_strike(self, ut):
        """The pitch at which the aft-most low corner of any part touches
        the runway, pivoting on the main wheels (vessel frame: y forward,
        z down)."""
        vf = self.vessel.reference_frame
        wheels = set()
        try:
            wheels = {w.part for w in self.vessel.parts.wheels}
        except Exception:                               # noqa: BLE001
            pass
        boxes = []
        for pt in self.parts:
            try:
                lo, hi = pt.bounding_box(vf)
            except Exception:                           # noqa: BLE001
                continue
            boxes.append((pt in wheels, lo, hi))
        gear = [b for b in boxes if b[0]]
        if not gear:
            return None
        ground_z = max(hi[2] for _, lo, hi in gear)
        # The main wheels: the aft-most ones.
        main_y = min((lo[1] + hi[1]) / 2 for _, lo, hi in gear)
        best = None
        for is_wheel, lo, hi in boxes:
            if is_wheel:
                continue
            y, z = lo[1], hi[2]         # aft end, bottom
            if y >= main_y:
                continue
            ang = math.atan2(ground_z - z, main_y - y)
            if best is None or ang < best:
                best = ang
        if best is not None:
            self.log.event(ut, "tail strike at %.1f deg of pitch"
                           % math.degrees(best))
        return best

    def plan_takeoff(self, ut):
        """The rotation angle and the lift-off speed it gives."""
        cfg = self.cfg
        self.lift_curve(ut)
        tail = self.tail_strike(ut)
        if not self.cla:
            self.alpha_rot, self.v_liftoff = math.radians(10.0), 100.0
            self.log.event(ut, "no lift curve: rotating at 100 m/s to 10 deg")
            return
        peak_a = max(self.cla, key=lambda x: x[1])[0]
        a = peak_a
        if tail is not None:
            a = min(a, tail - math.radians(cfg.TAIL_CLEARANCE_DEG))
        a = max(a, 0.0)
        mass = self.vessel.mass
        rho = self.flight.atmosphere_density or 1.2
        thrust = 0.0
        try:
            thrust = self.vessel.available_thrust
        except Exception:                               # noqa: BLE001
            pass
        weight = mass * gd.G0 - thrust * math.sin(a)
        cla = self.cla_at(a)
        self.alpha_rot = a
        self.v_liftoff = math.sqrt(max(weight, 0.0) / (0.5 * rho * cla)) \
            if cla > 0 else 100.0
        self.log.event(ut, "take-off: rotate to %.1f deg (lift peak %.0f, "
                       "tail %s), Cl*A %.2f, lift-off %.0f m/s at %.0f kg"
                       % (math.degrees(a), math.degrees(peak_a),
                          "%.1f" % math.degrees(tail) if tail else "?",
                          cla, self.v_liftoff, mass))

    def approach_speed(self, mass, rho):
        """APPROACH_STALL_RATIO times the stall speed at this mass in
        sea-level-ish air (the runway's: ``rho`` taken only once it is
        thicker, so the speed is never planned for thinner air than the
        landing's)."""
        cmax = self.cla_max() or 1.0
        r = max(rho or 0.0, self.rho_ground)
        return self.cfg.APPROACH_STALL_RATIO * math.sqrt(
            mass * gd.G0 / (0.5 * r * cmax))

    def update_thermal(self, parts_got, rho, speed, mach, t_env, dt):
        """Measure each part's thermal constants (thermal.PartHeat)."""
        if self.heat_models is None:
            self.heat_models = []
            for i, name in enumerate(self.part_names):
                r = parts_got[PER_PART * i:PER_PART * i + PER_PART]
                self.heat_models.append(th.PartHeat(name, r[1] or 1e9,
                                                    r[3] or 1e9))
        if dt <= 0.0 or None in (rho, mach, t_env):
            return
        a = min(1.0, dt / 3.0)
        for i, m in enumerate(self.heat_models):
            r = parts_got[PER_PART * i:PER_PART * i + PER_PART]
            if None in r[:4]:
                continue
            m.update(r[0], r[2], r[4], r[5], r[6], r[7], r[8], r[9], rho,
                     speed, mach, t_env, a)

    def plan_speed(self, ut, rho, sound, t_env, remaining):
        """The fastest speed every part survives for the rest of the
        flight at this altitude's air (thermal.speed_limit), every
        THERMAL_PLAN_S."""
        cfg = self.cfg
        if ut < self.next_plan or not self.heat_models or not sound:
            return
        self.next_plan = ut + cfg.THERMAL_PLAN_S
        v = th.speed_limit(self.heat_models, rho, sound, t_env or 250.0,
                           remaining, cfg.HEAT_MAX)
        if v is not None and (self.v_heat is None
                              or abs(v - self.v_heat) > 20.0):
            hot = max((p for p in self.heat_models if p.ready()),
                      key=lambda p: p.peak(rho, v, v / sound, t_env or 250.0,
                                           min(remaining / v, 3600.0))[0])
            self.log.event(ut, "thermal: %.0f m/s safe for the %.0f km left "
                           "(rho %.4f; binding %s: A_c %.3f eA %.3f G %.4f)"
                           % (v, remaining / 1e3, rho, hot.name, hot.a_c,
                              hot.e_a, hot.g))
        self.v_heat = v

    def seek_altitude(self, alt, agl, heat_thr, dt):
        """The cruise altitude, searched for: up while heat holds the
        throttle back, down while the throttle is full and the speed has
        stopped growing (thrust-limited); held within 2 km of the craft."""
        cfg = self.cfg
        if self.h_seek is None:
            self.h_seek = alt
        step = cfg.ALT_SEEK_RATE_M_S * dt
        at_heat = self.v_heat is not None and \
            self.speed >= self.v_heat - cfg.SPEED_BAND_M_S
        if heat_thr < 0.98 or at_heat:
            self.h_seek += step
        elif self.throttle >= 0.99 and self.accel < cfg.SEEK_ACCEL_M_S2:
            self.h_seek -= step
        floor = alt - agl + cfg.ALT_FLOOR_AGL_M
        self.h_seek = gd.clamp(self.h_seek, max(floor, alt - 2000.0),
                               alt + 2000.0)

    def choose_route(self, ut, lat, lon, speed, fuel):
        """Home the long way: on round the planet on the same great circle
        (the user, 2026-11-17: at this speed a turn back costs more than it
        saves).  Logs whether the fuel left covers it at the burn
        measured."""
        cfg = self.cfg
        a = (cfg.RUNWAY_09_LAT, cfg.RUNWAY_09_LON)
        b = (cfg.RUNWAY_27_LAT, cfg.RUNWAY_27_LON)
        home = gd.distance(lat, lon, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2,
                           self.radius)
        rest = 2.0 * math.pi * self.radius - home
        self.around = True
        need = None
        if self.fuel_rate and self.fuel_rate > 0.0:
            need = self.fuel_rate * rest / max(speed, 1.0)
        self.log.event(ut, "route: on round, %.0f km to go; fuel %.0f, needs "
                       "%s at the measured burn%s"
                       % (rest / 1e3, fuel or 0.0,
                          "%.0f" % need if need else "?",
                          " -- SHORT" if need and fuel is not None
                          and need > fuel else ""))

    def runway_for_heading(self, heading):
        """The threshold whose course is nearest ``heading``."""
        cfg = self.cfg
        a = (cfg.RUNWAY_09_LAT, cfg.RUNWAY_09_LON)
        b = (cfg.RUNWAY_27_LAT, cfg.RUNWAY_27_LON)
        c09 = gd.bearing(a[0], a[1], b[0], b[1])
        c27 = gd.bearing(b[0], b[1], a[0], a[1])
        if abs(gd.wrap(heading - c09)) <= abs(gd.wrap(heading - c27)):
            return (a[0], a[1], c09, "09")
        return (b[0], b[1], c27, "27")

    def choose_runway(self, lat, lon):
        """The threshold whose landing course best matches the way home."""
        cfg = self.cfg
        a = (cfg.RUNWAY_09_LAT, cfg.RUNWAY_09_LON)
        b = (cfg.RUNWAY_27_LAT, cfg.RUNWAY_27_LON)
        c09 = gd.bearing(a[0], a[1], b[0], b[1])
        c27 = gd.bearing(b[0], b[1], a[0], a[1])
        home = gd.bearing(lat, lon, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        if abs(gd.wrap(home - c09)) <= abs(gd.wrap(home - c27)):
            return (a[0], a[1], c09, "09")
        return (b[0], b[1], c27, "27")

    def run(self, ut):
        cfg = self.cfg
        got = self.read()
        (lat, lon, speed, vs, alt, agl, hdg, gforce, mach, rho, thrust, mass,
         situation, pitch, fuel, t_env, sound) = got[:17]
        if None in (lat, lon, speed, alt):
            return
        speed = speed or 0.0
        self.speed = speed
        vs = vs or 0.0
        agl = agl if agl is not None else alt
        gforce = gforce or 0.0
        mass = mass or 1.0
        thrust = thrust or 0.0
        frac, name, lost = self.hottest(got[17:])
        if lost:
            self.log.event(ut, "parts lost: %s" % ", ".join(lost))
            self.finish("craft broken: %s" % ", ".join(lost))
            return
        sit = str(situation).lower()
        ground = "landed" in sit or "pre_launch" in sit
        dt = 0.0
        if self.last is not None:
            dt = max(0.0, ut - self.last[0])
        gamma = math.asin(gd.clamp(vs / max(speed, 1.0), -1.0, 1.0))
        # Heat, filtered over ~1 s (rate from the filtered fraction).
        if self.heat is None:
            self.heat = (frac, 0.0)
        elif dt > 0.0:
            a = min(1.0, dt / 1.0)
            f0, r0 = self.heat
            f1 = f0 + a * (frac - f0)
            self.heat = (f1, r0 + a * ((f1 - f0) / dt - r0))
        self.heat_name = name
        hfrac, hrate = self.heat
        self.update_thermal(got[17:], rho, speed, mach, t_env, dt)
        # Drag, from the speed's change: what thrust and gravity do not
        # explain.  Kept below Mach 0.9 (the approach's air) on level-ish
        # flight and real q.
        if (dt > 0.0 and self.last is not None and mach is not None
                and mach < 0.9 and rho and speed > 60.0 and not ground):
            acc = (speed - self.last[1]) / dt
            drag = thrust / mass - acc - gd.G0 * math.sin(gamma)
            cda = 2.0 * mass * drag / (rho * speed * speed)
            if 0.05 < cda < 10.0:
                self.cda += min(1.0, dt / 5.0) * (cda - self.cda)
        if dt > 0.0 and self.last is not None:
            a = min(1.0, dt / 2.0)
            self.accel += a * ((speed - self.last[1]) / dt - self.accel)
        if fuel is not None:
            if self.last_fuel is not None and ut > self.last_fuel[0] + 1.0:
                r = (self.last_fuel[1] - fuel) / (ut - self.last_fuel[0])
                self.fuel_rate = r if self.fuel_rate is None else \
                    self.fuel_rate + 0.1 * (r - self.fuel_rate)
                self.last_fuel = (ut, fuel)
            elif self.last_fuel is None:
                self.last_fuel = (ut, fuel)
        self.last = (ut, speed)
        # Load: the axial g the thrust puts on the pilot, and what is left
        # for the wings.
        axial = thrust / (mass * gd.G0)
        g_line = cfg.G_LIMIT - cfg.G_MARGIN
        normal = max(1.0, gd.normal_g_budget(g_line, axial))
        if gforce > g_line:
            normal = max(1.0, normal * g_line / gforce)
        pcfg = self.plane.cfg
        pcfg.TURN_MAX_G = normal
        pcfg.MAX_PITCH_RATE_DEG_S = min(
            15.0, math.degrees((normal - 1.0) * gd.G0 / max(speed, 50.0))
            + 1.0)

        heat_thr = gd.heat_throttle(hfrac, hrate, cfg.HEAT_MAX,
                                    cfg.HEAT_LOOKAHEAD_S, cfg.HEAT_BAND)
        g_thr = 0.0 if gforce > g_line else 1.0
        throttle = min(heat_thr, g_thr)
        course = None               # where to point, rad
        gamma_cmd = 0.0
        bank_cap = math.radians(cfg.BANK_MAX_DEG)
        pitch_hold = None           # an attitude, on the ground
        brakes = False
        note = ""

        if self.state == TAKEOFF:
            rlat, rlon, rc, _ = self.takeoff_runway
            along, cross = gd.along_cross(lat, lon, rlat, rlon, rc,
                                          self.radius)
            course = gd.localizer_course(rc, cross, max(speed, 30.0),
                                         cfg.LOC_TAU_S,
                                         math.radians(cfg.LOC_CUT_DEG))
            if ground:
                # On the wheels: no more than a few degrees of cut.
                course = gd.clamp(course, rc - math.radians(3.0),
                                  rc + math.radians(3.0))
            if not ground and agl > 3.0:
                self.airborne = True
            if not self.airborne:
                # Nose up to the rotation angle as early as it can be: the
                # rotation (at the pitch-rate cap) starts so that it ends as
                # the speed reaches lift-off.
                lead = self.alpha_rot / math.radians(
                    pcfg.MAX_PITCH_RATE_DEG_S) * max(self.accel, 0.0)
                pitch_hold = (self.alpha_rot
                              if speed >= self.v_liftoff - lead else 0.0)
                self.plane.ground_pitch_integral = bool(pitch_hold)
                if pitch_hold and not self.rotating:
                    self.rotating = True
                    self.log.event(ut, "rotate at %.0f m/s (lift-off %.0f, "
                                   "lead %.0f)" % (speed, self.v_liftoff, lead))
            else:
                gamma_cmd = math.radians(cfg.CLIMB_GAMMA_DEG) * min(
                    1.0, agl / 300.0)
                gamma_cmd = max(gamma_cmd, math.radians(5.0))
                bank_cap = math.radians(5.0) if agl < 100 else bank_cap
                self.plane.ground_pitch_integral = False
                if agl > cfg.GEAR_UP_AGL_M:
                    self.vessel.control.gear = False
                    self.enter(OUT, ut, "gear up at %.0f m, %.0f m/s"
                               % (agl, speed))
        if self.state == OUT:
            tlat, tlon = self.target[1], self.target[2]
            dist = gd.distance(lat, lon, tlat, tlon, self.radius)
            course = gd.bearing(lat, lon, tlat, tlon)
            back = gd.distance(tlat, tlon, cfg.RUNWAY_09_LAT,
                               cfg.RUNWAY_09_LON, self.radius)
            self.plan_speed(ut, rho, sound, t_env, dist + back)
            if self.v_heat is not None:
                throttle = min(throttle, gd.clamp(
                    (self.v_heat - speed) / cfg.SPEED_BAND_M_S, 0.0, 1.0))
            self.seek_altitude(alt, agl, heat_thr, dt)
            gamma_cmd = gd.flight_path_command(
                self.h_seek, alt, speed, cfg.ALT_TAU_S,
                math.radians(cfg.CLIMB_GAMMA_DEG),
                math.radians(cfg.DESCENT_GAMMA_DEG))
            if self.closest is None or dist < self.closest[0]:
                self.closest = (dist, ut, alt, speed)
            off = abs(gd.wrap(course - math.radians(hdg or 0.0)))
            radius = gd.turn_radius(speed, normal)
            if dist < cfg.PASS_RADIUS_M or (off > math.pi / 2
                                            and dist < 2.5 * radius):
                self.enter(HOME, ut, "over the target: closest %.0f m at "
                           "%.0f m, %.0f m/s, %.0f s after START"
                           % (self.closest[0], self.closest[2],
                              self.closest[3], self.closest[1] - self.start_ut))
                self.choose_route(ut, lat, lon, speed, fuel)
            note = "tgt %.1f km h_seek %.0f" % (dist / 1e3, self.h_seek)
        if self.state == HOME:
            if self.runway is None:
                self.runway = self.choose_runway(lat, lon)
                self.log.event(ut, "home: runway %s" % self.runway[3])
            rlat, rlon, rc, _ = self.runway
            glat, glon = gd.offset(rlat, rlon, rc + math.pi, cfg.FINAL_M,
                                   self.radius)
            gate_alt = cfg.RUNWAY_ALT_M + cfg.FINAL_M * cfg.FINAL_SLOPE
            dist = gd.distance(lat, lon, glat, glon, self.radius)
            course = gd.bearing(lat, lon, glat, glon)
            if self.around:
                # The long way: the same great circle, the other way.
                if abs(gd.wrap(course - math.radians(hdg or 0.0))) < math.pi / 2:
                    self.around = False
                    self.log.event(ut, "round the far side: home %.0f km "
                                   "ahead" % (dist / 1e3))
                else:
                    course = (course + math.pi) % (2.0 * math.pi)
                    dist = 2.0 * math.pi * self.radius - dist
            self.plan_speed(ut, rho, sound, t_env, dist + cfg.FINAL_M)
            if self.v_heat is not None:
                throttle = min(throttle, gd.clamp(
                    (self.v_heat - speed) / cfg.SPEED_BAND_M_S, 0.0, 1.0))
            self.seek_altitude(alt, agl, heat_thr, dt)
            h_cmd = gd.glide_altitude(dist, gate_alt, cfg.DESCENT_SLOPE,
                                      self.h_seek)
            gamma_cmd = gd.flight_path_command(
                h_cmd, alt, speed, cfg.ALT_TAU_S,
                math.radians(cfg.CLIMB_GAMMA_DEG),
                math.radians(cfg.DESCENT_GAMMA_DEG))
            # Slow on the drag measured in the approach's air.
            k = cfg.COAST_DRAG_SHARE * 1.2 * self.cda / (2.0 * mass)
            v_cmd = gd.coast_speed(self.approach_speed(mass, rho), dist, k)
            throttle = min(throttle, gd.clamp(
                (v_cmd - speed) / cfg.SPEED_BAND_M_S, 0.0, 1.0))
            along, cross = gd.along_cross(lat, lon, rlat, rlon, rc,
                                          self.radius)
            if dist < max(1500.0, 0.5 * gd.turn_radius(speed, normal)) or (
                    along > -cfg.FINAL_M and abs(cross) < 1500.0
                    and abs(gd.wrap(math.radians(hdg or 0.0) - rc)) < 0.5):
                self.vessel.control.gear = True
                self.enter(FINAL, ut, "gate: %.0f m off, alt %.0f, %.0f m/s,"
                           " along %.0f cross %+.0f" % (dist, alt, speed,
                                                         along, cross))
            note = "gate %.1f km v_cmd %.0f" % (dist / 1e3, v_cmd)
        if self.state in (FINAL, FLARE):
            rlat, rlon, rc, _ = self.runway
            along, cross = gd.along_cross(lat, lon, rlat, rlon, rc,
                                          self.radius)
            course = gd.localizer_course(rc, cross, speed, cfg.LOC_TAU_S,
                                         math.radians(cfg.LOC_CUT_DEG))
            to_aim = cfg.AIM_M - along
            h_cmd = cfg.RUNWAY_ALT_M + max(0.0, to_aim) * cfg.FINAL_SLOPE
            gamma_cmd = gd.flight_path_command(
                h_cmd, alt, speed, 4.0, math.radians(10.0),
                -math.atan(cfg.FINAL_SLOPE) - math.radians(4.0))
            throttle = gd.clamp((self.approach_speed(mass, rho) - speed)
                                / cfg.SPEED_BAND_M_S, 0.0, 1.0)
            bank_cap = math.radians(30.0)
            if self.state == FINAL and agl < cfg.FLARE_AGL_M:
                self.enter(FLARE, ut, "agl %.0f, %.0f m/s, sink %.1f, "
                           "cross %+.0f" % (agl, speed, -vs, cross))
            if self.state == FLARE:
                # From the glide's sink at the flare's top to the
                # touchdown's at the wheels.
                glide = speed * cfg.FINAL_SLOPE
                sink = cfg.TOUCHDOWN_SINK_M_S + (glide - cfg.TOUCHDOWN_SINK_M_S) \
                    * gd.clamp(agl / cfg.FLARE_AGL_M, 0.0, 1.0)
                gamma_cmd = -math.asin(gd.clamp(sink / max(speed, 1.0),
                                                0.0, 1.0))
                throttle = 0.0
                bank_cap = math.radians(5.0)
            if ground:
                self.enter(ROLLOUT, ut, "touchdown %.0f m/s, sink %.1f, "
                           "along %.0f, cross %+.0f, g %.1f"
                           % (speed, -vs, along, cross, gforce))
            note = "along %.0f cross %+.0f" % (along, cross)
        if self.state == ROLLOUT:
            rlat, rlon, rc, _ = self.runway
            course = rc
            pitch_hold = 0.0
            throttle = 0.0
            brakes = True
            along, cross = gd.along_cross(lat, lon, rlat, rlon, rc,
                                          self.radius)
            if speed < 0.5:
                self.enter(DONE, ut, "stopped: along %.0f, cross %+.0f, "
                           "%.0f s after START" % (along, cross,
                                                    ut - self.start_ut))
                self.finish("landed")
            note = "along %.0f cross %+.0f" % (along, cross)

        # -- into the attitude hold --------------------------------------------
        hdg_r = math.radians(hdg or 0.0)
        if course is None:
            course = hdg_r
        err = gd.wrap(course - hdg_r)
        bank = gd.bank_for_turn(err / cfg.HEADING_TAU_S, speed, normal)
        bank = gd.clamp(bank, -bank_cap, bank_cap)
        # Heat-limited with the throttle shut: no descent.
        if hfrac >= cfg.HEAT_NO_DESCENT and throttle <= 0.0 and \
                self.state in (OUT, HOME):
            gamma_cmd = max(gamma_cmd, 0.0)
        plane = self.plane
        theta = math.radians(pitch or 0.0)
        if pitch_hold is not None:
            pitch_cmd = pitch_hold
        else:
            step = math.radians(cfg.PITCH_STEP_DEG)
            pitch_cmd = theta + gd.clamp(gamma_cmd - gamma, -step, step)
        plane.target = [bank, pitch_cmd, course]
        c = self.vessel.control
        if abs(throttle - self.throttle) > 0.005 or (
                throttle in (0.0, 1.0) and throttle != self.throttle):
            c.throttle = throttle
            self.throttle = throttle
        if brakes:
            c.brakes = True
        plane.run_fly(ut)
        if cfg.RUN_TIME_S > 0 and ut - self.start_ut >= cfg.RUN_TIME_S:
            self.finish("run time")
        deg = math.degrees
        self.log.telemetry(ut, (
            "%s spd=%6.1f M=%4.2f alt=%6.0f agl=%6.0f vs=%+6.1f hdg=%5.1f "
            "crs=%5.1f bank=%+5.1f gam=%+5.1f/%+5.1f pitch=%+5.1f/%+5.1f "
            "thr=%.2f vheat=%.0f hseek=%.0f heat=%.3f%+.4f/s %s g=%4.1f ax=%.1f nrm=%.1f cda=%.2f "
            "lat=%.4f lon=%.4f %s"
            % (self.state, speed, mach or 0.0, alt, agl, vs, hdg or 0.0,
               deg(course) % 360, deg(bank), deg(gamma), deg(gamma_cmd),
               deg(theta), deg(pitch_cmd), throttle, self.v_heat or 0.0,
               self.h_seek or 0.0, hfrac, hrate, name,
               gforce, axial, normal, self.cda, lat, lon, note)))
        self.lines = [
            "phase   : %s %s" % (self.state, note),
            "target  : %s" % (self.target[0] if self.target else "-"),
            "speed   : %.0f m/s  M%.2f  alt %.0f m" % (speed, mach or 0.0,
                                                       alt),
            "heat    : %.0f%% %s" % (100 * hfrac, name),
            "load    : %.1f g (cap %.0f)" % (gforce, cfg.G_LIMIT),
            "throttle: %.2f" % throttle,
        ]

    def tick(self):
        ut = self.sc.ut
        if self.rpc is not None:
            self.rpc.tick(self.state, ut)
        started = time.monotonic()
        if self.gui.terminate_pressed():
            self.finish("terminated")
            return ut
        if self.state == STANDBY:
            if self.autostart or self.gui.start_pressed():
                if self.take_over(ut):
                    self.gui.hide_start()
                    self.takeoff_runway = self.runway_for_heading(
                        math.radians(self.sflight.heading))
                    self.log.event(ut, "take-off runway %s, course %.2f"
                                   % (self.takeoff_runway[3], math.degrees(
                                       self.takeoff_runway[2])))
                    landed = str(self.vessel.situation).lower()
                    if "landed" in landed or "pre_launch" in landed:
                        self.vessel.control.throttle = 1.0
                        self.throttle = 1.0
                        self.enter(TAKEOFF, ut)
                    else:
                        self.enter(OUT, ut, "started in the air")
                else:
                    self.lines = ["no target: place a waypoint on the map",
                                  "then press START"]
                    self.autostart = False
            else:
                self.lines = ["phase   : STANDBY",
                              "place a waypoint on the map, press START"]
        elif self.state != DONE:
            self.run(ut)
        self.loop_rate.sample(self.state, ut, time.monotonic() - started)
        try:
            self.gui.update(self.lines)
        except Exception:                               # noqa: BLE001
            pass
        return ut

    def shutdown(self, reason):
        ut = 0.0
        try:
            ut = self.sc.ut
        except Exception:                               # noqa: BLE001
            pass
        if self.state not in (STANDBY,):
            try:
                self.plane.hand_back(ut)
                self.vessel.control.throttle = 0.0
            except Exception:                           # noqa: BLE001
                pass
        if self.closest is not None:
            self.log.event(ut, "closest approach %.0f m" % self.closest[0])
        self.log.event(ut, "loop rate, game-s per tick / wall-s of work: %s"
                       % self.loop_rate.report())
        if self.rpc is not None:
            self.log.event(ut, self.rpc.report())
        self.log.event(ut, "shutdown: %s" % reason)
        for g in (self.gui, self.plane.gui):
            try:
                g.close()
            except Exception:                           # noqa: BLE001
                pass


def run(conn, cfg, log, autostart=False):
    pilot = ReconPilot(conn, cfg, log)
    pilot.rpc = RpcCounter()
    pilot.rpc.install(conn)
    pilot.autostart = autostart
    if autostart:
        pilot.gui.hide_start()
    wait = sleeper(cfg, lambda: conn.space_center.ut, conn)
    reason = "complete"
    try:
        while not pilot.finished:
            ut = pilot.tick()
            wait(cfg.LOOP_SLEEP_S, ut)
        reason = pilot.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
        raise
    except Exception as exc:                            # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        reason = "error: %r" % exc
    finally:
        pilot.shutdown(reason)
    return pilot, reason


def main(argv=None):
    args = cli.parser("recon", "QUDRAC: over a map waypoint and back to the "
                      "runway, fast.").parse_args(
        sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    status = 0
    try:
        conn = cli.connect(args)
        _, reason = run(conn, cfg, log, args.autostart)
        if reason.startswith("error"):
            status = 1
    except KeyboardInterrupt:
        pass
    except Exception as exc:                            # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        status = 1
    finally:
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
