"""Recon: the QUDRAC flies from the runway over a point the player picks on
the map and back to the runway, as fast as its heat and 15 g allow.

Phase machine::

    STANDBY -> TAKEOFF -> OUT -> HOME -> FINAL -> FLARE -> ROLLOUT -> DONE

(START in the air goes straight to OUT.)

* **The target** is the last custom waypoint on the body (stock map view's
  waypoint tool), read at START; ``TARGET_LAT``/``LON`` without one.
* **The attitude** is kRPC's autopilot: this pilot sets its pitch, heading
  and roll every tick (pitch never further than a second of the load's
  pitch rate from where the nose is).
* **Heat** governs the speed: the throttle is proportional on where the
  hottest part (internal or skin, each against its own limit) will be
  ``HEAT_LOOKAHEAD_S`` from now (``guidance.heat_throttle``).  At the heat
  limit with the throttle shut it stops descending.
* **Load**: total g (thrust's push included) under ``G_LIMIT``: the wings
  get what the axial load leaves (``guidance.normal_g_budget``): the bank
  and the pitch command's rate are held to it; measured g over the line
  shuts the throttle.
* **Navigation**: great-circle course to the goal; heading error into a
  turn's bank.  OUT flies to the target at an altitude it
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

from common import cli
from common.krpcpilot import set_autopilot_attitude, set_autopilot_engaged
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
        # The attitude: kRPC's autopilot, pitch/heading/roll against the
        # horizon (the airplane pilot's hold was tried first and let the
        # nose sag into the runway after lift-off, LOG17744).
        self.ap = self.vessel.auto_pilot
        self.ap.reference_frame = self.vessel.surface_reference_frame
        self.airborne = False
        self.pitch_tgt = 0.0
        self.flight = self.vessel.flight(self.body.reference_frame)
        # Pitch and heading against the horizon (the body frame's read odd).
        self.sflight = self.vessel.flight(self.vessel.surface_reference_frame)
        # The velocity through the air in the surface frame's axes.
        self.hflight = self.vessel.flight(
            self.sc.ReferenceFrame.create_hybrid(
                position=self.body.reference_frame,
                rotation=self.vessel.surface_reference_frame))
        self.path_i = 0.0
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
        self.sfc = None             # LiquidFuel per newton-second, measured
        self.fuel = None
        self.v_fuel_logged = None
        self.level = False
        self.v_pred = None
        self.alpha_peak = math.radians(15.0)   # re-read off the lift curve
        self.d_max = None           # the short way home's farthest, going round
        self.a_clear = None         # the pitch the tail clearance allows
        self.derot = None           # the rollout's lowering pitch
        self.braking = False
        self.go_arounds = 0
        self.clear_logged = False
        self.last_theta = None
        self.rot_pitch = None
        self.tail_arm = 2.0         # m, re-read from the parts at START
        try:
            wheels = {w.part for w in self.vessel.parts.wheels}
        except Exception:                               # noqa: BLE001
            wheels = set()
        self.low_parts = [pt for pt in self.parts if pt not in wheels]

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
        calls += [(getattr, (self.sflight, "angle_of_attack")),
                  (getattr, (self.sflight, "roll")),
                  (getattr, (self.hflight, "velocity"))]
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
        c = self.vessel.control
        c.sas = False
        self.ap.target_pitch_and_heading(float(self.sflight.pitch),
                                         float(self.sflight.heading))
        self.ap.target_roll = 0.0
        exc = set_autopilot_engaged(self.ap, True)
        if exc is not None:
            self.log.event(ut, "autopilot: %r" % exc)
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

    def alpha_for(self, cla):
        """The angle of attack giving ``cla`` (m^2) off the swept curve, up
        to its peak."""
        pts = self.cla or [(0.0, 0.0), (self.alpha_peak, 1.0)]
        peak = max(pts, key=lambda x: x[1])
        if cla >= peak[1]:
            return peak[0]
        for (a0, c0), (a1, c1) in zip(pts, pts[1:]):
            if c0 <= cla <= c1 and c1 > c0:
                return a0 + (a1 - a0) * (cla - c0) / (c1 - c0)
            if a1 >= peak[0]:
                break
        return 0.0 if cla <= pts[0][1] else peak[0]

    def cla_max(self):
        return max(c for _, c in self.cla) if self.cla else None

    def rotate(self, ut, target, theta, agl, dt):
        """The rotation: the pitch target raised at ROTATE_RATE_MAX_DEG_S
        toward ``target``, held where it is once any non-wheel part is
        within TAIL_CLEARANCE_M of the ground (measured each tick).
        Returns the pitch to command."""
        cfg = self.cfg
        if not target:
            self.rot_pitch = None
            return 0.0
        if self.rot_pitch is None:
            self.rot_pitch = theta
        clear = self.clearance(agl)
        if clear is not None and clear < cfg.TAIL_CLEARANCE_M:
            if not self.clear_logged:
                self.log.event(ut, "tail clearance %.2f m at %.1f deg of "
                               "pitch: the nose stops" % (clear,
                                                          math.degrees(theta)))
                self.clear_logged = True
            self.rot_pitch = min(self.rot_pitch, theta)
        else:
            self.rot_pitch = min(target, self.rot_pitch + math.radians(
                cfg.ROTATE_RATE_MAX_DEG_S) * dt)
        return self.rot_pitch

    def clearance(self, agl):
        """The lowest non-wheel part's height above the ground (m): its
        bounding box in the surface frame (x up, origin the vessel's centre
        of mass) against the ground, ``agl`` below the centre of mass."""
        sf = self.vessel.surface_reference_frame
        calls = [(pt.bounding_box, (sf,)) for pt in self.low_parts]
        got = ask(self.conn, calls)
        lows = [b[0][0] for b in got if b is not None]
        if not lows:
            return None
        return min(lows) + agl

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
                self.tail_arm = max(0.5, main_y - y)
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
        self.alpha_peak = peak_a
        a = peak_a
        if tail is not None:
            a = min(a, tail - math.radians(cfg.TAIL_CLEARANCE_DEG))
        # The angle the tail clearance measured now allows (the parts'
        # boxes against the ground, about the main wheels): the bounding-box
        # estimate above said 23.9 deg and the nozzle struck at 9 (LOG17738).
        # Rotating there lifts the craft off as the nose comes up: a
        # nose-high roll on the wheels at low speed is directionally
        # unstable on this airframe (heading diverged at 44 m/s with the
        # nose at 7 deg, roll and yaw inputs off, LOG17743).
        clear0 = self.clearance(self.flight.surface_altitude or 0.0)
        if clear0 is not None:
            a_clear = math.atan2(max(clear0 - cfg.TAIL_CLEARANCE_M, 0.0),
                                 self.tail_arm)
            self.log.event(ut, "tail clearance at rest %.2f m, arm %.2f m: "
                           "%.1f deg of rotation" % (clear0, self.tail_arm,
                                                     math.degrees(a_clear)))
            a = min(a, a_clear)
            self.a_clear = a_clear
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
        v_q = math.sqrt(2.0 * cfg.ROTATE_MIN_Q_PA / rho)
        if v_q > self.v_liftoff:
            self.log.event(ut, "take-off: lift-off at %.0f m/s would do; the "
                           "fin needs %.0f" % (self.v_liftoff, v_q))
            self.v_liftoff = v_q
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
        stall = self.cfg.APPROACH_STALL_RATIO * math.sqrt(
            mass * gd.G0 / (0.5 * r * cmax))
        # ... and never under the speed the fin needs on the wheels (the
        # rotation's ROTATE_MIN_Q_PA): at 61 m/s the touchdown snapped the
        # heading 60 deg as the mains met the runway (LOG17756), the take-
        # off's departure at the other end of the flight.
        return max(stall, math.sqrt(2.0 * self.cfg.ROTATE_MIN_Q_PA / r))

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
        """The speed to hold, every THERMAL_PLAN_S: the fastest the fuel
        left covers home (in level cruise), and -- with THERMAL_FLY -- the
        fastest every part survives for the rest of the flight
        (thermal.speed_limit).  Without THERMAL_FLY the thermal prediction
        is logged (vpred=), not flown: its constants from kRPC's flux
        numbers came out ~1000x small and it asked for 200 m/s at Mach 3
        with the cockpit at 0.42 of its limit (LOG17752)."""
        cfg = self.cfg
        if ut < self.next_plan or not self.heat_models or not sound:
            return
        self.next_plan = ut + cfg.THERMAL_PLAN_S
        vp = th.speed_limit(self.heat_models, rho, sound, t_env or 250.0,
                            remaining, cfg.HEAT_MAX)
        self.v_pred = vp
        v = vp if cfg.THERMAL_FLY else None
        # The fuel's speed is a cruise's: drag at this attitude in a climb
        # (30 deg off the airflow at 140 m/s) asked for less speed, which
        # steepened the climb, which asked for less (LOG17748).
        vf = self.fuel_speed(remaining, v) if self.level else None
        if vf is not None and (v is None or vf < v):
            if self.v_fuel_logged is None or abs(vf - self.v_fuel_logged) > 20:
                self.log.event(ut, "fuel: %.0f m/s is the fastest the %.0f LF "
                               "left covers for %.0f km (%.2e LF per N s)"
                               % (vf, self.fuel or 0.0, remaining / 1e3,
                                  self.sfc))
                self.v_fuel_logged = vf
            v = vf
        self.v_heat = v

    def drag_at(self, speed):
        """Drag (N) at ``speed`` along the present airflow, at this
        place and attitude: KSP's own aerodynamics."""
        ref = self.body.reference_frame
        try:
            vel = self.vessel.velocity(ref)
            n = math.sqrt(sum(x * x for x in vel)) or 1.0
            d = tuple(x / n for x in vel)
            f = self.flight.simulate_aerodynamic_force_at(
                self.body, self.vessel.position(ref),
                tuple(speed * x for x in d), self.vessel.rotation(ref))
        except Exception:                               # noqa: BLE001
            return None
        return -sum(a * b for a, b in zip(f, d))

    def fuel_speed(self, remaining, v_hi):
        """The fastest speed whose fuel for ``remaining`` metres (drag x
        LF per N s x time) fits in the tank: fuel per metre grows with the
        drag, so faster costs more.  None until the burn is measured."""
        if not self.sfc or self.fuel is None or remaining <= 0.0:
            return None

        def need(v):
            d = self.drag_at(v)
            if d is None:
                return None
            return self.sfc * max(d, 0.0) * remaining / v
        lo, hi = 150.0, v_hi or 3000.0
        n = need(hi)
        if n is None:
            return None
        if n <= self.fuel:
            return hi
        n = need(lo)
        if n is None or n > self.fuel:
            return lo
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            n = need(mid)
            if n is None:
                return lo
            if n <= self.fuel:
                lo = mid
            else:
                hi = mid
        return lo

    def seek_altitude(self, alt, agl, heat_thr, dt):
        """The cruise altitude, searched for: down while thrust is the
        limit (the throttle full and the speed no longer growing by
        SEEK_ACCEL_M_S2: thicker air gives the RAPIER more), up otherwise --
        while heat or a computed speed holds the throttle back (thinner air:
        more speed for the same heat or fuel) and while the craft still
        accelerates (spare thrust: thinner air is where its top speed is).
        Held within an altitude-loop time constant of climb of the craft.  (Climbing only on a binding limit
        left it at Mach 3.3 at 1.7 km, LOG17749.)"""
        cfg = self.cfg
        if self.h_seek is None:
            self.h_seek = alt
        # As fast as the craft can climb or sink: its own speed along the
        # steepest flight path allowed (a fixed 150 m/s let it reach Mach
        # 4.5 at 4.4 km and meet the heat limit there, LOG17750).
        up = self.speed * math.sin(math.radians(cfg.CLIMB_GAMMA_DEG))
        down = self.speed * math.sin(math.radians(-cfg.DESCENT_GAMMA_DEG))
        gentle = self.speed * math.sin(math.radians(cfg.HEAT_SEEK_GAMMA_DEG))
        at_cap = self.v_heat is not None and \
            self.speed >= self.v_heat - cfg.SPEED_BAND_M_S
        top = alt + up * cfg.ALT_TAU_S
        if self.throttle >= 0.99 and self.accel < cfg.SEEK_ACCEL_M_S2:
            # Thrust is the limit (the engine starving up high too).
            self.h_seek -= down * dt
        elif self.throttle >= 0.99 and self.accel >= cfg.SEEK_ACCEL_M_S2:
            # Spare thrust: up on the excess power, the path whose climb
            # holds the speed (sin gamma = a / g), no steeper than allowed.
            # At the full 30 deg on any acceleration it climbed to 36 km at
            # Mach 4.5, past where the engine breathes (LOG17754).
            spare = self.speed * gd.clamp(self.accel / gd.G0, 0.0, math.sin(
                math.radians(cfg.CLIMB_GAMMA_DEG)))
            self.h_seek = min(self.h_seek, alt + spare * cfg.ALT_TAU_S)
            self.h_seek += spare * dt
            top = alt + max(spare, gentle) * cfg.ALT_TAU_S
        elif heat_thr < 0.98 or at_cap:
            # Heat or fuel binds: up gently, never far ahead -- the heat
            # answers in tens of seconds, and climbing at 24 deg on it at
            # Mach 4.6 carried the craft to 49 km, past where the engine
            # breathes (LOG17753).
            self.h_seek = min(self.h_seek, alt + gentle * cfg.ALT_TAU_S)
            self.h_seek += gentle * dt
            top = alt + gentle * cfg.ALT_TAU_S
        # (Throttled back for no limit: hold.  Climbing on every
        # not-thrust-limited tick took it unpowered to 42 km, LOG17752.)
        floor = alt - agl + cfg.ALT_FLOOR_AGL_M
        self.h_seek = gd.clamp(self.h_seek, max(floor, alt - down
                                                * cfg.ALT_TAU_S), top)

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
        rate = self.fuel_rate
        if self.fuel_rate and self.fuel_rate > 0.0:
            need = self.fuel_rate * rest / max(speed, 1.0)
        # The most economical speed's need for the lap (the fuel planner's
        # own law at its slowest): if even that is more than is left, the
        # short way home.
        econ = None
        if self.sfc and fuel is not None:
            econ = min((self.sfc * max(self.drag_at(v) or 0.0, 0.0) * rest / v
                        for v in (300.0, 500.0, 800.0, 1200.0)), default=None)
        if econ is not None and econ > fuel:
            self.around = False
        self.log.event(ut, "route: %s, %.0f km to go round; fuel %.0f, needs "
                       "%s at the measured burn, %s at the most economical "
                       "speed" % ("ON ROUND" if self.around else
                                  "BACK the short way (fuel)", rest / 1e3,
                                  fuel or 0.0,
                                  "%.0f" % need if need else "?",
                                  "%.0f" % econ if econ is not None else "?"))

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
         situation, pitch, fuel, aoa, roll, vel_s, t_env, sound) = got[:20]
        if None in (lat, lon, speed, alt):
            return
        speed = speed or 0.0
        self.speed = speed
        vs = vs or 0.0
        agl = agl if agl is not None else alt
        gforce = gforce or 0.0
        mass = mass or 1.0
        thrust = thrust or 0.0
        frac, name, lost = self.hottest(got[20:])
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
        self.update_thermal(got[20:], rho, speed, mach, t_env, dt)
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
            self.fuel = fuel
            if self.fuel_rate is not None and thrust > 1000.0:
                s = self.fuel_rate / thrust
                self.sfc = s if self.sfc is None else \
                    self.sfc + min(1.0, dt / 10.0) * (s - self.sfc)
        self.last = (ut, speed)
        # Load: the axial g the thrust puts on the pilot, and what is left
        # for the wings.
        axial = thrust / (mass * gd.G0)
        g_line = cfg.G_LIMIT - cfg.G_MARGIN
        normal = max(1.0, gd.normal_g_budget(g_line, axial))
        if gforce > g_line:
            normal = max(1.0, normal * g_line / gforce)

        self.level = abs(gamma) < math.radians(cfg.CRUISE_LEVEL_DEG)
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

        theta_now = math.radians(pitch or 0.0)
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
                # Nose up to the rotation angle: the rotation (at its rate)
                # starts so that it ends as the speed reaches lift-off.
                lead = self.alpha_rot / math.radians(
                    cfg.ROTATE_RATE_MAX_DEG_S) * max(self.accel, 0.0)
                want = (self.alpha_rot
                        if speed >= self.v_liftoff - lead else 0.0)
                pitch_hold = self.rotate(ut, want, theta_now, agl, dt)
                if want and not self.rotating:
                    self.rotating = True
                    self.log.event(ut, "rotate at %.0f m/s (lift-off %.0f, "
                                   "lead %.0f)" % (speed, self.v_liftoff, lead))
            else:
                gamma_cmd = math.radians(cfg.CLIMB_GAMMA_DEG) * min(
                    1.0, agl / 300.0)
                gamma_cmd = max(gamma_cmd, math.radians(5.0))
                bank_cap = math.radians(5.0) if agl < 100 else bank_cap
                if agl < cfg.CLEAR_UNTIL_AGL_M and self.rot_pitch is not None:
                    # Just off: the same clearance watch.
                    pitch_hold = self.rotate(ut, max(self.rot_pitch,
                                                     theta_now), theta_now,
                                             agl, dt)
                if agl > cfg.GEAR_UP_AGL_M:
                    self.vessel.control.gear = False
                    self.enter(OUT, ut, "gear up at %.0f m, %.0f m/s"
                               % (agl, speed))
        if self.state == OUT:
            tlat, tlon = self.target[1], self.target[2]
            dist = gd.distance(lat, lon, tlat, tlon, self.radius)
            course = gd.bearing(lat, lon, tlat, tlon)
            # The rest of the flight: on to the target, then on round the
            # same great circle home.
            back = gd.distance(tlat, tlon, cfg.RUNWAY_09_LAT,
                               cfg.RUNWAY_09_LON, self.radius)
            self.plan_speed(ut, rho, sound, t_env,
                            dist + 2.0 * math.pi * self.radius - back)
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
                # The long way: the same great circle, the other way, until
                # the short way home is the shorter (past the runway's
                # antipode).  By distance, not heading: a tumble at 41 km
                # swung the heading through the bearing home and turned it
                # back 900 km out (LOG17753).
                self.d_max = max(self.d_max or 0.0, dist)
                if self.d_max > 0.9 * math.pi * self.radius and \
                        dist < self.d_max - 1000.0:
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
            # Slow on the drag measured, in the air the craft is in now: the
            # descent only thickens it, so the coast this plans is the
            # longest one.  (Planned in sea-level air it began slowing 25 km
            # out and reached the gate at 1328 m/s from 12.7 km, LOG17755.)
            k = cfg.COAST_DRAG_SHARE * (rho or 0.0) * self.cda / (2.0 * mass)
            v_cmd = gd.coast_speed(self.approach_speed(mass, rho), dist, k)
            throttle = min(throttle, gd.clamp(
                (v_cmd - speed) / cfg.SPEED_BAND_M_S, 0.0, 1.0))
            along, cross = gd.along_cross(lat, lon, rlat, rlon, rc,
                                          self.radius)
            v_app = self.approach_speed(mass, rho)
            # Onto the final only slow enough to fly it: otherwise on round
            # the gate (steering at it circles it) while the coast slows
            # (the gate at 656 m/s landed 2626 m along a 2400 m runway,
            # LOG17759).
            slow = speed <= cfg.FINAL_SPEED_RATIO * v_app
            if slow and (dist < max(1500.0, 0.5 * gd.turn_radius(speed, normal)) or (
                    along > -cfg.FINAL_M and abs(cross) < 1500.0
                    and abs(gd.wrap(math.radians(hdg or 0.0) - rc)) < 0.5)):
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
            if not ground and along > cfg.GO_AROUND_SHARE * cfg.RUNWAY_LENGTH_M:
                # Still flying past this share of the runway: around again.
                self.vessel.control.gear = False
                self.enter(HOME, ut, "go-around: %.0f m along at %.0f m, "
                           "%.0f m/s" % (along, agl, speed))
                self.go_arounds += 1
            if ground:
                self.enter(ROLLOUT, ut, "touchdown %.0f m/s, sink %.1f, "
                           "along %.0f, cross %+.0f, g %.1f"
                           % (speed, -vs, along, cross, gforce))
            note = "along %.0f cross %+.0f" % (along, cross)
        if self.state == ROLLOUT:
            rlat, rlon, rc, _ = self.runway
            course = rc
            # The nose lowered at DEROTATE_RATE_DEG_S from where the
            # touchdown left it: commanded straight to level, kRPC's
            # autopilot slammed it from 12.5 deg and the intake struck the
            # runway at 73 m/s (LOG17757).
            if self.derot is None:
                self.derot = theta_now
            self.derot = max(0.0, self.derot - math.radians(
                cfg.DEROTATE_RATE_DEG_S) * dt)
            pitch_hold = self.derot
            throttle = 0.0
            # The brakes only once the nose is on its wheel: braking on the
            # mains alone at 80 m/s (below the centre of mass) threw the
            # nose to -16 deg onto the intake and the craft tumbled
            # (LOG17758).
            brakes = self.derot <= 0.0 and theta_now < math.radians(1.0)
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
        # Heat-limited with the throttle shut: no descent.
        if hfrac >= cfg.HEAT_NO_DESCENT and throttle <= 0.0 and \
                self.state in (OUT, HOME):
            gamma_cmd = max(gamma_cmd, 0.0)
        theta = math.radians(pitch or 0.0)
        ap = self.ap
        alpha_cmd = None
        if pitch_hold is not None or vel_s is None or ground:
            # On the wheels (and the rotation): pitch, heading, wings level.
            err = gd.wrap(course - hdg_r)
            bank = 0.0
            pitch_cmd = pitch_hold if pitch_hold is not None else theta
            if self.state in (FINAL, FLARE, ROLLOUT) and self.a_clear:
                pitch_cmd = min(pitch_cmd, self.a_clear)
            ap.target_pitch_and_heading(math.degrees(pitch_cmd),
                                        math.degrees(course) % 360.0)
            ap.target_roll = 0.0
            self.path_i = 0.0
        else:
            # In the air the path is flown, not an attitude: the
            # acceleration square to the velocity that closes the path angle
            # (against gravity) and turns to the course, held to the load
            # budget, vertical first; the lift along it, at the angle of
            # attack the lift curve gives for it at this q; the nose that
            # angle above the velocity toward the lift, the roof along it.
            # (Pitch and heading targets with a bank set apart from them
            # gave a turn no lift where it was wanted: it zoomed to 40 km,
            # LOG17755, and sank into the ground at 80 deg of bank,
            # LOG17760.)
            track = math.atan2(vel_s[2], vel_s[1])
            err = gd.wrap(course - track)
            a_max = normal * gd.G0
            a_up, a_right = gd.normal_accel(
                speed, gamma, gamma_cmd, err, cfg.PATH_TAU_S,
                cfg.HEADING_TAU_S, a_max, bank_cap)
            # The path's integral: what the lift curve (taken at low speed)
            # does not know of the craft at this Mach.
            if dt > 0.0:
                self.path_i = gd.clamp(
                    self.path_i + speed * (gamma_cmd - gamma)
                    / (cfg.PATH_TAU_S * cfg.PATH_I_TAU_S) * dt,
                    -0.5 * a_max, 0.5 * a_max)
            a_up += self.path_i
            q_dyn = 0.5 * (rho or 0.0) * speed * speed
            need = mass * math.hypot(a_up, a_right)
            alpha_cmd = self.alpha_for(need / q_dyn if q_dyn > 1.0 else 1e9)
            if self.state in (FINAL, FLARE) and self.a_clear:
                # The nose never past the tail clearance's pitch on the
                # runway (the flare at 53 m/s set the nozzle down, LOG17754).
                alpha_cmd = min(alpha_cmd, max(0.0, self.a_clear - gamma))
            nose, roof, bank = gd.path_attitude(vel_s, a_up, a_right,
                                                alpha_cmd)
            set_autopilot_attitude(ap, nose, roof)
            pitch_cmd = math.asin(gd.clamp(nose[0], -1.0, 1.0))
        c = self.vessel.control
        if abs(throttle - self.throttle) > 0.005 or (
                throttle in (0.0, 1.0) and throttle != self.throttle):
            c.throttle = throttle
            self.throttle = throttle
        if brakes != self.braking:
            c.brakes = brakes
            self.braking = brakes
            if brakes:
                self.log.event(ut, "brakes on at %.0f m/s" % speed)
        if cfg.RUN_TIME_S > 0 and ut - self.start_ut >= cfg.RUN_TIME_S:
            self.finish("run time")
        deg = math.degrees
        self.log.telemetry(ut, (
            "%s spd=%6.1f M=%4.2f alt=%6.0f agl=%6.0f vs=%+6.1f hdg=%5.1f "
            "crs=%5.1f bank=%+5.1f gam=%+5.1f/%+5.1f pitch=%+5.1f/%+5.1f a=%+5.1f/%s pi=%+.1f "
            "thr=%.2f vheat=%.0f vpred=%.0f hseek=%.0f heat=%.3f%+.4f/s %s g=%4.1f ax=%.1f nrm=%.1f cda=%.2f "
            "lat=%.4f lon=%.4f %s"
            % (self.state, speed, mach or 0.0, alt, agl, vs, hdg or 0.0,
               deg(course) % 360, deg(bank), deg(gamma), deg(gamma_cmd),
               deg(theta), deg(pitch_cmd), aoa or 0.0,
               "%.1f" % deg(alpha_cmd) if alpha_cmd is not None else "-",
               self.path_i, throttle, self.v_heat or 0.0,
               self.v_pred or 0.0,
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
                set_autopilot_engaged(self.ap, False)
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
        try:
            self.gui.close()
        except Exception:                               # noqa: BLE001
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
        import traceback
        log.event(0.0, "FATAL %r\n%s" % (exc, traceback.format_exc()))
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
