// A whole transfer: within one SOI, to a moon, or to another planet.  The
// phase machine here is the one the game flies; only the propagation is the
// stand-in's (Run) or the game's (the plugin's Controller).
//
//     within SOI:     FINAL
//     to a moon:      PUMP -> WAIT -> PUSH -> COAST -> APPROACH -> CAPTURE -> FINAL
//     to a planet:    PARK -> WAIT -> DEPART -> ESCAPE -> CRUISE -> APPROACH
//                     -> CAPTURE -> FINAL
//
// - PUMP / PARK / FINAL fly Guidance (Hohmann in small burns, plane change at
//   the nodes) to an orbit about the current body.
// - WAIT coasts.  To a moon: at each push point it asks whether a prograde
//   push there puts the encounter on the aim (Transfer.PushDv) and starts the
//   push half its duration early.  To a planet: it waits for the periapsis
//   pass nearest the window's departure.
// - PUSH re-solves the push every tick; DEPART burns prograde at periapsis
//   until the hyperbolic excess is the window's.
// - COAST / CRUISE correct the B-plane miss (Transfer.Correction), or
//   re-target by Lambert when the miss is too big; MATCH sheds an arrival
//   too fast to capture about the star.
// - APPROACH nudges the periapsis onto the capture altitude; CAPTURE only
//   has to make the orbit bound; FINAL shapes it.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace LowThrust
{
    public class Maneuver
    {
        public string phase, body;
        public double t0, t1, dv, prograde, normal, radial;
    }

    public class Event
    {
        public double t;
        public string text;
        public Event(double t, string text) { this.t = t; this.text = text; }
    }

    public class MissionResult
    {
        public bool done;
        public string reason;
        public double t0, t, dv, propUsed;
        public List<Maneuver> maneuvers;
        public List<Event> events;
        public Body body;
        public Conic final;
    }

    public class Craft
    {
        public Body body;
        public V3 r, v;
        public double m, t;
        public Craft(Body body, V3 r, V3 v, double m, double t)
        {
            this.body = body; this.r = r; this.v = v; this.m = m; this.t = t;
        }
    }

    /// <summary>A phase's answer: a command (throttle, direction, tick), or
    /// an end ("done", or why it failed).</summary>
    public class Cmd
    {
        public double thr, tick;
        public V3? dir;
        public string end;
        public static Cmd Of(double thr, V3? dir, double tick) { return new Cmd { thr = thr, dir = dir, tick = tick }; }
        public static Cmd End(string why) { return new Cmd { end = why }; }
    }

    /// <summary>Mission.Step's answer: a command to hold, or the result.</summary>
    public class Step
    {
        public MissionResult result;
        public double throttle, tick;
        public V3 direction;
    }

    /// <summary>The mission's working state between ticks (null: not set).</summary>
    public class PlanState
    {
        public double? rc, ra_cap, rp_wait, ra_wait, vcap, match_vs, rp_park, ra_park, vinf;
        public LaunchWindow window;
        public long? pass_, trim_checked, checked_;
        public double? horizon, next_fix, push, push_t, trim_left, trim_sign, trim_t, eta, blocked_until,
                       cap, arc_end, skip_t, push_left, ca_t, t_go, arrive_t, lambert_t, dodge_next,
                       match_end_dv;
        public int? skips;
        public string resume, lifting, evading;
        public bool trim_now;
        public bool? rebinding, match_done;
        public V3? vrel, dodge, dodge_dir;
    }

    public class Mission
    {
        static readonly CultureInfo IC = CultureInfo.InvariantCulture;
        public Config cfg;
        public Dictionary<string, Body> sys;
        public Ship vessel;
        public Craft craft;
        public double t0;
        public Body dest, home;
        public Target finalTarget, target;
        public bool record;
        public double dv;
        public List<Maneuver> maneuvers = new List<Maneuver>();
        public List<Event> events = new List<Event>();
        class OpenBurn { public string phase, body; public double t0, dv, pro, nor, rad; }
        OpenBurn burn;
        public double lastDv;
        public string phase;
        public PlanState plan = new PlanState();
        public string abort;
        public string kind;

        public Mission(Dictionary<string, Body> system, Ship vessel, Body body, V3 r, V3 v, double t0,
                       Body dest, Target target, Config cfg, bool record = true)
        {
            this.cfg = cfg;
            sys = system;
            this.vessel = vessel;
            craft = new Craft(body, r, v, vessel.massKg, t0);
            this.t0 = t0;
            this.dest = dest;
            home = body;
            finalTarget = target;
            this.record = record;
            Route();
        }

        static string F(double x, int d) { return x.ToString("F" + d, IC); }

        // --- bookkeeping ------------------------------------------------------
        public void Event(string text) { events.Add(new Event(craft.t, text)); }

        public void SetPhase(string p, string text = "")
        {
            CloseBurn();
            lastDv = 0.0;
            phase = p;
            Event(p + (text != "" ? ": " + text : ""));
        }

        void CloseBurn()
        {
            OpenBurn b = burn;
            if (b != null && record)
            {
                double n = Math.Max(b.dv, 1e-12);
                maneuvers.Add(new Maneuver
                {
                    phase = b.phase, body = b.body, t0 = b.t0, t1 = craft.t, dv = b.dv,
                    prograde = b.pro / n, normal = b.nor / n, radial = b.rad / n
                });
            }
            burn = null;
        }

        public bool Burning { get { return burn != null; } }

        // --- routing ------------------------------------------------------------
        void Route()
        {
            Body here = craft.body;
            if (dest == here) { kind = "soi"; FinalStart(); }
            else if (dest.parent == here) { kind = "moon"; PlanMoon(); }
            else if (dest.parent == here.parent && here.parent != null) { kind = "planet"; PlanPlanet(); }
            else
                throw new ArgumentException("no route from " + here.name + " to " + dest.name
                                            + " (within an SOI, to a moon, or to a sibling)");
        }

        double CaptureRp()
        {
            Body d = dest;
            Target tg = finalTarget;
            double? alt = tg.rp.HasValue ? tg.rp : tg.ra;
            double floor = d.radius + d.atmosphere + cfg.SAFE_ALT_M;
            return Math.Max(alt.HasValue ? alt.Value : floor, floor);
        }

        double CaptureRa()
        {
            Body d = dest;
            Target tg = finalTarget;
            double ra = Math.Min(cfg.CAPTURE_AP_FRACTION * d.Soi, Ceiling(d));
            if (tg.ra.HasValue) ra = Math.Max(ra, tg.ra.Value);
            return Math.Max(ra, CaptureRp());
        }

        public double Accel() { return vessel.thrustN / craft.m; }

        void PlanMoon()
        {
            Body here = craft.body, moon = dest;
            Conic c = Orbit.Conic(craft.r, craft.v, here.mu);
            double rc = CaptureRp(), raCap = CaptureRa();
            double raWait, rpWait;
            bool ok;
            WaitingOrbit(here, moon, c.rp, rc, raCap, Accel(), cfg, false, out raWait, out rpWait, out rc,
                         out raCap, out ok);
            if (!ok) Event("warning: no waiting orbit lets this engine capture the arrival");
            else if (rc > CaptureRp() * 1.001)
                Event("capture high and loose for this engine: " + Km(raCap, moon) + " x " + Km(rc, moon));
            plan.rc = rc; plan.ra_cap = raCap; plan.rp_wait = rpWait; plan.ra_wait = raWait;
            double loose = cfg.CAPTURE_LOOSE_FRACTION * moon.Soi;
            plan.vcap = Math.Max(Math.Max(
                Transfer.CapturableVinf(moon, Accel(), rc, raCap, Py.Radians(cfg.CAPTURE_ARC_DEG),
                                        cfg.CAPTURE_FILL, loose),
                LongCapturableHere()),
                0.25 * Math.Sqrt(2 * moon.mu / moon.Soi));
            plan.match_vs = cfg.ARRIVE_MARGIN * Math.Max(plan.vcap.Value, 1.0);
            target = new Target(raWait, rpWait, moon.inc, moon.lan);
            SetPhase("PUMP", "waiting orbit " + Km(raWait, here) + " x " + Km(rpWait, here) + ", in "
                             + moon.name + "'s plane");
        }

        void PlanPlanet()
        {
            Body here = craft.body, star = here.parent;
            double rp = Math.Max(Orbit.Conic(craft.r, craft.v, here.mu).rp,
                                 here.radius + here.atmosphere + cfg.SAFE_ALT_M);
            double ra = cfg.PARK_AP_FRACTION * Ceiling(craft.body);
            SimResult est = Sim.Fly(new SimBody(here), vessel, craft.r, craft.v, new Target(ra, rp),
                                    cfg.ETA_DEFAULT, cfg, false);
            double ready = craft.t + 1.3 * est.time + cfg.WINDOW_LEAD_S;
            double syn = 1 / Math.Abs(1 / here.Period - 1 / dest.Period);
            LaunchWindow w = Transfer.FindWindow(here, dest, ready, star.mu, syn);
            if (w == null) throw new ArgumentException("no Lambert window found");
            V3 nNow = V3.Cross(craft.r, craft.v).Unit;
            var park = Transfer.ParkingOrientation(here.mu, rp, ra, w.vinfDep, nNow);
            V3 normal = park.Item1, apse = park.Item2;
            double planeDv = park.Item3;
            double rc = CaptureRp(), raCap = CaptureRa();
            double cRc, cRa;
            bool cap = Transfer.CaptureOrbit(dest, w.vinfArr.Norm, Accel(), rc, raCap,
                                             Py.Radians(cfg.CAPTURE_ARC_DEG), cfg.CAPTURE_FILL,
                                             Math.Max(raCap, Ceiling(dest)), out cRc, out cRa);
            if (!cap)
            {
                plan.rc = rc; plan.ra_cap = raCap;
                double vArr = w.vinfArr.Norm;
                double vmax = LongCapturableHere();
                double[] early = vArr <= vmax ? Transfer.CaptureStart(CapSpecHere(), rc, vArr, Accel()) : null;
                double[] tight = null;
                if (early != null && dest.children.Count > 0)
                {
                    tight = early;
                    plan.ra_cap = 0.5 * dest.Soi;
                    double[] lo = Transfer.CaptureStart(CapSpecHere(), rc, vArr, Accel());
                    plan.ra_cap = raCap;
                    if (lo != null && lo[2] <= cfg.ARRIVE_MARGIN * dest.Soi)
                    {
                        double mu = dest.mu;
                        double lower = Math.Sqrt(mu * (2 / rc - 2 / (rc + 0.5 * dest.Soi)))
                                       - Math.Sqrt(mu * (2 / rc - 2 / (rc + raCap)));
                        if (lo[1] + lower < tight[1]) early = null;
                    }
                }
                if (early == null && dest.children.Count > 0)
                {
                    double loose = 0.5 * dest.Soi;
                    plan.ra_cap = loose;
                    early = Transfer.CaptureStart(CapSpecHere(), rc, vArr, Accel());
                    if (early != null && early[2] <= cfg.ARRIVE_MARGIN * dest.Soi) raCap = loose;
                    else { early = null; plan.ra_cap = raCap; }
                }
                if (early != null)
                {
                    double tau = early[0], dvCap = early[1], outD = early[2];
                    Event("long capture: ~" + F(dvCap, 0) + " m/s into " + Km(raCap, dest) + " x " + Km(rc, dest)
                          + ", starting " + F(tau / 3600, 1) + " h before periapsis, " + F(outD / 1e6, 1)
                          + " Mm out (" + F(outD / dest.Soi, 1) + " SOI)");
                }
                else
                {
                    vmax = Math.Min(vmax, 0.9 * vArr);
                    double arc = Py.Radians(cfg.CAPTURE_ARC_DEG);
                    double loose = Math.Max(raCap, 0.5 * dest.Soi);
                    double v1 = Transfer.CapturableVinf(dest, Accel(), rc, raCap, arc, cfg.CAPTURE_FILL, loose);
                    if (v1 > vmax)
                    {
                        double r1, a1;
                        Transfer.CaptureOrbit(dest, cfg.ARRIVE_MARGIN * v1, Accel(), rc, raCap, arc,
                                              cfg.CAPTURE_FILL, loose, out r1, out a1);
                        rc = r1; raCap = a1;
                    }
                    double vs = cfg.ARRIVE_MARGIN * Math.Max(v1, vmax);
                    if (vs <= 0)
                    {
                        abort = "this engine cannot capture at " + dest.name + " even arriving at rest";
                        return;
                    }
                    plan.match_vs = vs;
                    Event("match " + dest.name + " in the cruise: shed " + F(vArr - vs, 0)
                          + " m/s about the star to arrive at its SOI at " + F(vs, 0)
                          + " m/s, then capture into " + Km(raCap, dest) + " x " + Km(rc, dest));
                }
            }
            else if (cRc > rc * 1.001)
            {
                rc = cRc; raCap = cRa;
                Event("capture high and loose for this engine: " + Km(raCap, dest) + " x " + Km(rc, dest));
            }
            plan.window = w; plan.rp_park = rp; plan.ra_park = ra; plan.vinf = w.vinfDep.Norm;
            plan.rc = rc; plan.ra_cap = raCap;
            if (abort != null) return;
            target = new Target(ra, rp, Math.Acos(normal.z), null, apse, normal, true);
            Event("window: depart " + FmtUt(w.tDep) + ", " + F(w.tof / 21600, 0) + " d flight, v_inf out "
                  + F(w.vinfDep.Norm, 0) + " m/s, in " + F(w.vinfArr.Norm, 0) + " m/s");
            SetPhase("PARK", "ellipse " + Km(ra, here) + " x " + Km(rp, here)
                             + " on the departure asymptote (plane change ~" + F(planeDv, 0) + " m/s)");
        }

        /// <summary>Highest apoapsis about body that stays clear of its moons
        /// and well inside its SOI.</summary>
        public double Ceiling(Body body)
        {
            double top = 0.5 * body.Soi;
            foreach (Body ch in body.children)
                top = Math.Min(top, ch.sma * (1 - ch.ecc) - cfg.WAIT_SOI_MARGIN * ch.Soi);
            return top;
        }

        public static string Km(double r, Body body) { return F((r - body.radius) / 1e3, 0) + " km"; }

        // --- the loop -----------------------------------------------------------
        /// <summary>One decision from the current state (craft): the result
        /// when the mission has ended, else the command to hold for its tick.
        /// The caller then reports what was flown (Account) and any SOI change
        /// (SoiChange).</summary>
        public Step DoStep()
        {
            Craft cr = craft;
            if (abort != null) return Ended(false, abort);
            Conic c = Orbit.Conic(cr.r, cr.v, cr.body.mu);
            double floor = cr.body.radius + cr.body.atmosphere;
            if (c.r < floor) return Ended(false, "hit " + cr.body.name + "'s atmosphere/surface");
            if (c.e < 1 && c.rp < floor && (phase == "WAIT" || phase == "COAST" || phase == "ESCAPE")
                && cr.body != dest)
                Repump("periapsis inside the atmosphere");
            if (cr.t > t0 + cfg.MISSION_MAX_DAYS * 21600) return Ended(false, "time limit");

            Cmd step = Evade(c) ?? FloorGuard(c) ?? PhaseStep(c);
            if (step.end == "done")
            {
                if (cr.body != dest) return Ended(false, "finished about " + cr.body.name + ", not " + dest.name);
                return Ended(true, "arrived");
            }
            if (step.end != null) return Ended(false, step.end);
            double throttle = step.thr, tick = step.tick;
            if (throttle > 0)
                tick = Math.Min(tick, Math.Max(0.1, cfg.THRUST_DV_SHARE * cr.v.Norm / (Accel() * throttle)));
            return new Step { throttle = throttle, direction = step.dir ?? V3.Zero, tick = tick };
        }

        Cmd PhaseStep(Conic c)
        {
            switch (phase)
            {
                case "FINAL": return SoiLaw(c, () => Cmd.End("done"));
                case "PUMP": return Pump(c);
                case "PARK": return Park(c);
                case "WAIT": return Wait(c);
                case "TRIM": return Trim(c);
                case "PUSH": return Push(c);
                case "DEPART": return Depart(c);
                case "ESCAPE": return Cmd.Of(0.0, null, Tick(c));
                case "COAST": return Correct(c, dest.parent);
                case "CRUISE": return Correct(c, dest.parent);
                case "MATCH": return Match(c);
                case "APPROACH": return ApproachStep(c);
                case "CAPTURE": return CaptureStep(c);
            }
            throw new InvalidOperationException("unknown phase " + phase);
        }

        /// <summary>Fly the mission in the stand-in, to its end.</summary>
        public MissionResult Run()
        {
            while (true)
            {
                Step s = DoStep();
                if (s.result != null) return s.result;
                if (s.throttle > 0) Thrust(s.direction, s.throttle, s.tick);
                else
                {
                    CloseBurn();
                    var hit = Coast(craft, s.tick);
                    if (hit != null) SoiChange(hit.Item1, hit.Item2);
                }
            }
        }

        Step Ended(bool done, string reason)
        {
            CloseBurn();
            Craft cr = craft;
            return new Step
            {
                result = new MissionResult
                {
                    done = done, reason = reason, t0 = t0, t = cr.t, dv = dv, propUsed = Sim.XenonKg(vessel, dv),
                    maneuvers = maneuvers, events = events, body = cr.body,
                    final = Orbit.Conic(cr.r, cr.v, cr.body.mu)
                }
            };
        }

        void Thrust(V3 direction, double throttle, double tick)
        {
            Craft cr = craft;
            V3 force = direction * (vessel.thrustN * throttle);
            V3 r0 = cr.r, v0 = cr.v;
            double h = tick / cfg.THRUST_SUBSTEPS;
            V3 r = cr.r, v = cr.v;
            for (int i = 0; i < cfg.THRUST_SUBSTEPS; i++) Sim.Rk4(ref r, ref v, cr.m, cr.body.mu, force, h);
            cr.r = r; cr.v = v;
            cr.t += tick;
            Account(direction, vessel.thrustN * throttle / cr.m * tick, tick, r0, v0);
            var hit = Crossing(cr, cr.r, cr.t);
            if (hit != null)
            {
                Switch(cr, hit.Item1, hit.Item2);
                SoiChange(hit.Item1, hit.Item2 ?? cr.body);
            }
        }

        /// <summary>Book dv m/s of thrust along direction over the tick just
        /// flown from (r0, v0).  dv 0 closes the burn.</summary>
        public void Account(V3 direction, double dvTick, double tick, V3 r0, V3 v0)
        {
            if (dvTick <= 0) { CloseBurn(); return; }
            Craft cr = craft;
            dv += dvTick;
            lastDv = dvTick;
            V3 R, T, N;
            Orbit.Rtn(r0, v0, out R, out T, out N);
            if (burn == null || burn.phase != phase)
            {
                CloseBurn();
                burn = new OpenBurn { phase = phase, body = cr.body.name, t0 = cr.t - tick };
            }
            burn.dv += dvTick;
            burn.pro += dvTick * V3.Dot(direction, T);
            burn.nor += dvTick * V3.Dot(direction, N);
            burn.rad += dvTick * V3.Dot(direction, R);
        }

        void Repump(string why)
        {
            plan.pass_ = null; plan.horizon = null; plan.next_fix = null; plan.push = null; plan.push_t = null;
            if (kind == "moon")
            {
                target = new Target(plan.ra_wait, plan.rp_wait, dest.inc, dest.lan);
                SetPhase("PUMP", why + ": re-pumping");
            }
            else if (kind == "planet")
            {
                target = target.Copy();
                SetPhase("PARK", why + ": re-parking");
            }
        }

        /// <summary>The craft has changed SOI ("exit" or "enter"); body: the
        /// body it crossed.</summary>
        public void SoiChange(string how, Body body)
        {
            Craft cr = craft;
            Event("SOI " + (how == "exit" ? "exit" : "entry") + ": now orbiting " + cr.body.name);
            if (how == "exit" && (phase == "COAST" || phase == "WAIT" || phase == "PUSH")
                && cr.body != dest && cr.body == home)
            {
                Repump("after a " + (body != null ? body.name : "") + " flyby");
                return;
            }
            string[] approachFrom = { "COAST", "CRUISE", "MATCH", "ESCAPE", "PUSH", "WAIT", "PUMP", "TRIM", "PARK" };
            string[] cruiseFrom = { "ESCAPE", "WAIT", "DEPART", "PARK", "TRIM" };
            if (cr.body == dest && approachFrom.Contains(phase))
                SetPhase("APPROACH", "periapsis target " + Km(plan.rc.Value, cr.body));
            else if (how == "exit" && kind == "planet" && cr.body == dest.parent && cruiseFrom.Contains(phase))
                SetPhase("CRUISE", "to " + dest.name);
        }

        // --- SOI geometry -----------------------------------------------------------
        static bool ChildHit(Body child, double t, V3 r) { return (r - child.Position(t)).Norm < child.Soi; }

        /// <summary>The SOI boundary (r, t) has crossed: ("exit", null),
        /// ("enter", child), or null.</summary>
        public static Tuple<string, Body> Crossing(Craft craft, V3 r, double t)
        {
            if (r.Norm > craft.body.Soi) return Tuple.Create("exit", (Body)null);
            foreach (Body c in craft.body.children)
                if (ChildHit(c, t, r)) return Tuple.Create("enter", c);
            return null;
        }

        public static void Switch(Craft craft, string how, Body child)
        {
            V3 rb, vb;
            if (how == "exit")
            {
                Body b = craft.body;
                b.State(craft.t, out rb, out vb);
                craft.r = craft.r + rb; craft.v = craft.v + vb;
                craft.body = b.parent;
            }
            else
            {
                child.State(craft.t, out rb, out vb);
                craft.r = craft.r - rb; craft.v = craft.v - vb;
                craft.body = child;
            }
        }

        /// <summary>Coast dt exactly, stopping at an SOI boundary (bisected).
        /// Returns the crossing kind and the body crossed into (the parent on
        /// an exit), or null.</summary>
        public static Tuple<string, Body> Coast(Craft craft, double dt)
        {
            double mu = craft.body.mu;
            V3 r1, v1;
            Orbit.Propagate(craft.r, craft.v, mu, dt, out r1, out v1);
            var hit = Crossing(craft, r1, craft.t + dt);
            if (hit == null) { craft.r = r1; craft.v = v1; craft.t = craft.t + dt; return null; }
            double lo = 0.0, hi = dt;
            for (int i = 0; i < 50; i++)
            {
                double mid = (lo + hi) / 2;
                V3 rm, vm;
                Orbit.Propagate(craft.r, craft.v, mu, mid, out rm, out vm);
                if (Crossing(craft, rm, craft.t + mid) == null) lo = mid; else hi = mid;
            }
            Orbit.Propagate(craft.r, craft.v, mu, hi, out r1, out v1);
            craft.r = r1; craft.v = v1;
            craft.t += hi;
            var h2 = Crossing(craft, craft.r, craft.t);
            Switch(craft, h2.Item1, h2.Item2);
            return Tuple.Create(h2.Item1, h2.Item2 ?? craft.body);
        }

        /// <summary>A tick short enough not to jump an SOI boundary.</summary>
        public static double BoundaryTick(Craft craft)
        {
            double r = craft.r.Norm, spd = craft.v.Norm;
            double lim = Py.Inf;
            if (craft.body.Soi < Py.Inf) lim = Math.Max(10.0, 0.5 * (craft.body.Soi - r) / Math.Max(spd, 1e-3));
            foreach (Body c in craft.body.children)
            {
                V3 rc, vc;
                c.State(craft.t, out rc, out vc);
                double gap = (craft.r - rc).Norm - c.Soi;
                double rel = (craft.v - vc).Norm;
                lim = Math.Min(lim, Math.Max(10.0, 0.5 * gap / Math.Max(rel, 1e-3)));
            }
            return lim;
        }

        static bool SafeUntil(Craft craft, Conic c)
        {
            if (c.e >= 1 || c.ra >= craft.body.Soi) return false;
            foreach (Body ch in craft.body.children)
            {
                double lo = ch.sma * (1 - ch.ecc) - ch.Soi;
                double hi = ch.sma * (1 + ch.ecc) + ch.Soi;
                if (!(c.ra < lo || c.rp > hi)) return false;
            }
            return true;
        }

        // --- tick sizes ---------------------------------------------------------------
        public double Tick(Conic c)
        {
            Craft cr = craft;
            double t;
            if (c.e < 1) t = Sim.TickFor(Orbit.Elements(cr.r, cr.v, cr.body.mu), cfg);
            else t = c.r / Math.Max(cr.v.Norm, 1e-3) * 2 * Math.PI / cfg.TICKS_PER_ORBIT;
            return Math.Max(1.0, Math.Min(t, BoundaryTick(cr)));
        }

        // --- phases -----------------------------------------------------------------------
        Cmd SoiLaw(Conic c, Func<Cmd> nextPhase)
        {
            Craft cr = craft;
            Body h = phase == "FINAL" ? dest : home;
            if (cr.body != h)
            {
                if (cr.body == h.parent) return Cmd.End("thrown out of " + h.name + "'s SOI by a flyby");
                return Cmd.Of(0.0, null, Tick(c));
            }
            if (burn == null && c.e < 1)
            {
                double? trim = TrimCheck(c);
                if (trim.HasValue)
                {
                    plan.trim_left = Math.Abs(trim.Value);
                    plan.trim_sign = trim.Value > 0 ? 1.0 : -1.0;
                    plan.resume = phase;
                    SetPhase("TRIM", trim.Value.ToString("+0.00;-0.00", IC) + " m/s "
                                     + (plan.trim_now ? "now" : "at periapsis") + " to miss a moon");
                    return Trim(c);
                }
            }
            Elements el = Orbit.Elements(cr.r, cr.v, cr.body.mu);
            if (el == null || el.ra > cfg.REBIND_SOI_FRACTION * cr.body.Soi)
            {
                if (plan.rebinding != true)
                {
                    plan.rebinding = true;
                    Event((el == null ? "unbound from" : "apoapsis high in") + " " + cr.body.name
                          + "'s SOI: braking");
                }
                double tk = Tick(c);
                return Cmd.Of(1.0, cr.v.Unit * -1.0, tk);
            }
            plan.rebinding = false;
            bool coasting = burn == null;
            target = Guidance.Lock(target, el, cfg, coasting);
            double tick = Tick(c);
            if (burn == null) plan.eta = ArcEta(el);
            Command cmd = Guidance.Command(el, target, Accel(), tick, plan.eta ?? cfg.ETA_DEFAULT, cfg, cr.body.mu);
            if (cmd.done) return nextPhase();
            if (cmd.throttle <= 0 || cr.t < (plan.blocked_until ?? -1)) return Cmd.Of(0.0, null, tick);
            V3 R, T, N;
            Orbit.Rtn(cr.r, cr.v, out R, out T, out N);
            V3 d = (T * cmd.prograde + N * cmd.normal).Unit;
            if (burn == null) GuardPass(c, d, el, cmd.need);
            double? cap = plan.cap;
            if (cap.HasValue)
            {
                double left = cap.Value - (burn != null ? burn.dv : 0.0);
                if (left <= 1e-3)
                {
                    plan.blocked_until = plan.arc_end;
                    plan.cap = null;
                    return Cmd.Of(0.0, null, tick);
                }
                return Cmd.Of(Math.Min(cmd.throttle, left / (Accel() * tick)), d, tick);
            }
            return Cmd.Of(cmd.throttle, d, tick);
        }

        double ArcEta(Elements el)
        {
            double mu = craft.body.mu;
            Target tg = target;
            double dvl = 0.0;
            if (tg.ra.HasValue)
                dvl += Math.Abs(Math.Sqrt(mu * (2 / el.rp - 2 / (el.rp + tg.ra.Value)))
                                - Math.Sqrt(mu * (2 / el.rp - 2 / (el.rp + el.ra))));
            if (tg.rp.HasValue)
                dvl += Math.Abs(Math.Sqrt(mu * (2 / el.ra - 2 / (el.ra + tg.rp.Value)))
                                - Math.Sqrt(mu * (2 / el.ra - 2 / (el.ra + el.rp))));
            foreach (Body b in Others())
            {
                double[] band = Transfer.Band(b, 1.0);
                if (!(el.ra < band[0] || el.rp > band[1])) return cfg.ETA_WIDE_MIN;
            }
            double half = Math.Acos(cfg.ETA_DEFAULT);
            double arcS = 2 * half * el.rp * el.rp / el.h;
            double passes = dvl / Accel() / Math.Max(arcS, 1e-6);
            if (passes <= cfg.ARC_PASSES_MAX) return cfg.ETA_DEFAULT;
            half = Math.Min(Math.Acos(cfg.ETA_WIDE_MIN), half * passes / cfg.ARC_PASSES_MAX);
            return Math.Cos(half);
        }

        /// <summary>The bodies a coast about the current body must stay clear
        /// of.</summary>
        List<Body> Others()
        {
            return craft.body.children.Where(b => b != dest).ToList();
        }

        void GuardPass(Conic c, V3 d, Elements el, double need)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            var others = Others();
            plan.cap = null;
            double arcS = 2 * Math.Acos(plan.eta ?? cfg.ETA_DEFAULT) * el.r * el.r / el.h;
            plan.arc_end = cr.t + arcS;
            if (others.Count == 0) return;
            double full = FlyArc(Py.Inf).dv;
            Tuple<double, double> latest = null;
            foreach (double f in new[] { 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0 })
            {
                var arc = FlyArc(f * full);
                Conic c1 = Orbit.Conic(arc.r, arc.v, mu);
                if (c1.e >= 1) continue;
                var hit = Transfer.FirstHazardWhen(arc.r, arc.v, arc.t, mu, others, cr.t + 1.05 * c1.period);
                if (hit == null)
                {
                    if (f != 1.0)
                    {
                        plan.cap = f * full;
                        Event("burn capped at " + F(f * full, 1) + " m/s: the full pass would meet a moon next revolution");
                    }
                    return;
                }
                if (latest == null || hit.Item1 > latest.Item1) latest = Tuple.Create(hit.Item1, f);
            }
            if (latest != null && latest.Item2 != 1.0) plan.cap = latest.Item2 * full;
            Event("warning: no burn this pass avoids a moon next revolution; the latest encounter kept");
        }

        class ArcEnd { public V3 r, v; public double t, dv; }

        ArcEnd FlyArc(double cap)
        {
            Craft cr = craft;
            double mu = cr.body.mu, accel = Accel();
            V3 r = cr.r, v = cr.v;
            double t = cr.t, dvs = 0.0;
            double end = t + 0.5 * Orbit.Conic(r, v, mu).period;
            double eta = plan.eta ?? cfg.ETA_DEFAULT;
            while (t < end && dvs < cap)
            {
                Elements el = Orbit.Elements(r, v, mu);
                if (el == null) break;
                double h = Math.Max(1.0, Math.Min(Sim.TickFor(el, cfg), end - t));
                Command cmd = Guidance.Command(el, target, accel, h, eta, cfg, mu);
                if (cmd.done || cmd.throttle <= 0) break;
                V3 R, T, N;
                Orbit.Rtn(r, v, out R, out T, out N);
                V3 d = (T * cmd.prograde + N * cmd.normal).Unit;
                double thr = Math.Min(cmd.throttle, (cap - dvs) / (accel * h));
                Sim.Rk4(ref r, ref v, 1.0, mu, d * (accel * thr), h);
                t += h;
                dvs += accel * thr * h;
            }
            return new ArcEnd { r = r, v = v, t = t, dv = dvs };
        }

        double? TrimCheck(Conic c)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            var others = Others();
            if (others.Count == 0) return null;
            double tPe = Orbit.TimeToPeriapsis(c, mu);
            long key = Py.Round(cr.t + tPe);
            if (plan.trim_checked == key || cr.t - (plan.trim_t ?? -Py.Inf) < 0.5 * c.period) return null;
            plan.trim_checked = key; plan.trim_t = cr.t;
            double t1 = cr.t + tPe;
            plan.trim_now = false;
            if (Transfer.FirstHazard(cr.r, cr.v, cr.t, mu, others, t1) != null)
            {
                double? dvNow = Phasing(cr.r, cr.v, cr.t, c, others, t1 + 1.05 * c.period);
                if (dvNow.HasValue) { plan.trim_now = true; return dvNow; }
            }
            V3 rp_, vp_;
            Orbit.Propagate(cr.r, cr.v, mu, tPe, out rp_, out vp_);
            if (Transfer.FirstHazard(rp_, vp_, t1, mu, others, t1 + 1.05 * c.period) == null) return null;
            double? dvp = Phasing(rp_, vp_, t1, c, others, null);
            if (!dvp.HasValue) Event("warning: no phasing trim clears the moons");
            return dvp;
        }

        double? Phasing(V3 r, V3 v, double t, Conic c, List<Body> others, double? tEnd)
        {
            double mu = craft.body.mu;
            V3 u = v.Unit;
            double speed = v.Norm;
            double dpdv = 3 * c.a * speed * c.period / mu;
            double slip = others.Max(b => 2 * cfg.WAIT_SOI_MARGIN * b.Soi / Math.Sqrt(mu / b.sma));
            double unit = slip / Math.Max(dpdv, 1e-9);
            var steps = new SortedSet<double> { 0.5, 1.0, 2.0, 4.0, 8.0 };
            foreach (double x in new[] { 0.25, 0.5, 1.0, 2.0 })
                if (0.1 <= x * unit && x * unit <= 0.02 * speed) steps.Add(x * unit);
            foreach (double x in steps)
                foreach (double s in new[] { 1.0, -1.0 })
                {
                    double dvt = s * x;
                    V3 v1 = v + u * dvt;
                    Conic c1 = Orbit.Conic(r, v1, mu);
                    if (c1.e >= 1) continue;
                    double end = tEnd ?? t + 2.05 * c1.period;
                    if (Transfer.FirstHazard(r, v1, t, mu, others, end) == null) return dvt;
                }
            return null;
        }

        void FinalStart()
        {
            target = finalTarget;
            SetPhase("FINAL", "to the target orbit");
        }

        Cmd Pump(Conic c)
        {
            return SoiLaw(c, () =>
            {
                SetPhase("WAIT", "for the pass whose push meets " + dest.name);
                return Cmd.Of(0.0, null, Tick(c));
            });
        }

        Cmd Park(Conic c)
        {
            return SoiLaw(c, () =>
            {
                SetPhase("WAIT", "for the window, " + FmtUt(plan.window.tDep));
                return Cmd.Of(0.0, null, Tick(c));
            });
        }

        Cmd Wait(Conic c)
        {
            double? trim = c.e < 1 ? TrimCheck(c) : null;
            if (trim.HasValue)
            {
                plan.push = null;
                plan.trim_left = Math.Abs(trim.Value);
                plan.trim_sign = trim.Value > 0 ? 1.0 : -1.0;
                plan.resume = "WAIT";
                SetPhase("TRIM", trim.Value.ToString("+0.00;-0.00", IC) + " m/s "
                                 + (plan.trim_now ? "now" : "at periapsis") + " to miss a moon");
                return Trim(c);
            }
            return kind == "moon" ? WaitMoon(c) : WaitPlanet(c);
        }

        Cmd Trim(Conic c)
        {
            Craft cr = craft;
            double tick = Tick(c);
            double left = plan.trim_left.Value - lastDv;
            lastDv = 0.0;
            plan.trim_left = left;
            if (left <= 0.01)
            {
                SetPhase(plan.resume, "trim done");
                return Cmd.Of(0.0, null, tick);
            }
            double tPe = Orbit.TimeToPeriapsis(c, cr.body.mu);
            if (burn == null && !plan.trim_now && tPe > left / Accel() / 2 && tPe < 0.9 * c.period)
                return Hold(c, tPe - left / Accel() / 2);
            return Cmd.Of(Math.Min(1.0, left / (Accel() * tick)), cr.v.Unit * plan.trim_sign.Value, tick);
        }

        double Horizon(V3 r, V3 v)
        {
            Conic c = Orbit.Conic(r, v, craft.body.mu);
            return Math.Min(c.e < 1 ? cfg.PUSH_REVOLUTION * c.period : 1e7, cfg.PUSH_HORIZON_S);
        }

        Cmd Hold(Conic c, double dt)
        {
            if (!SafeUntil(craft, c)) dt = Math.Min(dt, Tick(c));
            return Cmd.Of(0.0, null, Math.Max(1.0, dt));
        }

        /// <summary>(seconds to the next push point, whether it is the
        /// periapsis, how far ahead it is decided).</summary>
        double[] PushPoint(Conic c, Elements el, double lead)
        {
            double mu = craft.body.mu;
            double[] best = { Orbit.TimeTo(el, 0.0, mu), 1, lead };
            double early = 0.5 * cfg.PUSH_OFF_PERIAPSIS * PushMax(c) / cfg.PUSH_DV_FACTOR / Accel();
            for (int k = 1; k < 12; k++)
            {
                double t = Orbit.TimeTo(el, k * Math.PI / 6, mu);
                if (early <= t && t < best[0]) best = new[] { t, 0, early + el.period / 12 };
            }
            return best;
        }

        Cmd WaitMoon(Conic c)
        {
            Craft cr = craft;
            Body moon = dest;
            double mu = cr.body.mu;
            Elements el = Orbit.Elements(cr.r, cr.v, mu);
            double accel = Accel();
            double lead = Math.Min(0.25 * c.period, PushMax(c) / accel);
            if (plan.push.HasValue)
            {
                double dvp = plan.push.Value, tp = plan.push_t.Value - cr.t;
                if (tp <= dvp / accel / 2) return StartPush(c, dvp);
                return Hold(c, tp - dvp / accel / 2);
            }
            double tPe, ahead;
            bool atPe;
            if (el != null)
            {
                double[] pp = PushPoint(c, el, lead);
                tPe = pp[0]; atPe = pp[1] > 0; ahead = pp[2];
            }
            else { tPe = Orbit.TimeToPeriapsis(c, mu); atPe = true; ahead = lead; }
            if (tPe > ahead) return Hold(c, tPe - ahead);
            long key = Py.Round(cr.t + tPe);
            if (plan.pass_ != key)
            {
                plan.pass_ = key;
                V3 rp_, vp_;
                Orbit.Propagate(cr.r, cr.v, mu, tPe, out rp_, out vp_);
                double reference = PushMax(c) / cfg.PUSH_DV_FACTOR;
                double? dvp = Transfer.PushDv(rp_, vp_, cr.t + tPe, mu, moon, plan.rc.Value, moon.Normal,
                                              atPe ? PushMax(c) : cfg.PUSH_OFF_PERIAPSIS * reference, Horizon,
                                              accel: accel);
                if (dvp.HasValue)
                {
                    V3 r1 = rp_, v1 = vp_;
                    double t1 = cr.t + tPe;
                    Transfer.ProgradeBurn(ref r1, ref v1, ref t1, mu, dvp.Value, accel, true);
                    Approach ca = Transfer.ClosestApproach(r1, v1, t1, mu, moon, Horizon(r1, v1));
                    var others = cr.body.children.Where(b => b != moon).ToList();
                    Body bad = Transfer.FirstHazard(r1, v1, t1, mu, others, ca.t);
                    if (bad != null)
                    {
                        Event("pass skipped: its coast would meet " + bad.name);
                        dvp = null;
                    }
                    else if (ca.vrel.Norm > plan.vcap.Value * Math.Pow(1.25, plan.skips ?? 0))
                    {
                        if (cr.t - (plan.skip_t ?? -Py.Inf) >= Synodic(c))
                        {
                            plan.skips = (plan.skips ?? 0) + 1;
                            plan.skip_t = cr.t;
                        }
                        Event("pass skipped: it arrives at " + F(ca.vrel.Norm, 0) + " m/s, faster than the "
                              + F(plan.vcap.Value, 0) + " this engine captures");
                        dvp = null;
                    }
                }
                plan.push = dvp; plan.push_t = cr.t + tPe;
                if (dvp.HasValue) return WaitMoon(c);
            }
            return Hold(c, tPe + 0.01 * c.period);
        }

        Cmd StartPush(Conic c, double dvp)
        {
            SetPhase("PUSH", F(dvp, 1) + " m/s prograde");
            plan.push_left = dvp;
            plan.ca_t = null; plan.vrel = null; plan.match_done = null; plan.horizon = null; plan.skips = null;
            plan.skip_t = null; plan.push = null; plan.push_t = null;
            return Push(c);
        }

        double Synodic(Conic c)
        {
            return 1 / Math.Max(Math.Abs(1 / c.period - 1 / dest.Period), 1e-12);
        }

        double PushMax(Conic c)
        {
            double mu = craft.body.mu;
            double rM = dest.sma * (1 + dest.ecc);
            double v0 = c.e < 1 ? Math.Sqrt(mu * (2 / c.rp - 1 / c.a)) : 0.0;
            double v1 = Math.Sqrt(mu * (2 / c.rp - 2 / (c.rp + rM)));
            return cfg.PUSH_DV_FACTOR * Math.Max(v1 - v0, 1.0);
        }

        Cmd Push(Conic c)
        {
            Craft cr = craft;
            Body moon = dest;
            double mu = cr.body.mu;
            double tick = Tick(c);
            double left = plan.push_left.Value - lastDv;
            double? dvp = Transfer.PushDv(cr.r, cr.v, cr.t, mu, moon, plan.rc.Value, moon.Normal,
                                          Math.Min(PushMax(c), 1.5 * left + 2), Horizon, 12,
                                          Math.Max(0.0, 0.5 * left - 2), Accel(), false);
            if (dvp.HasValue) left = dvp.Value;
            plan.push_left = left;
            lastDv = 0.0;
            if (left < cfg.CORRECT_DV)
            {
                SetPhase("COAST", "to " + moon.name);
                return Cmd.Of(0.0, null, tick);
            }
            return Cmd.Of(Math.Min(1.0, left / (Accel() * tick)), cr.v.Unit, tick);
        }

        Cmd WaitPlanet(Conic c)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            LaunchWindow w = plan.window;
            double tPe = Orbit.TimeToPeriapsis(c, mu);
            double vinf = plan.vinf.Value;
            double need = (Math.Sqrt(Math.Pow(vinf, 2) + 2 * mu / c.rp) - Math.Sqrt(mu * (2 / c.rp - 1 / c.a)))
                          / Accel();
            double tGo = plan.t_go ?? w.tDep;
            if (cr.t + tPe < tGo - c.period / 2)
                return Hold(c, Math.Max(tPe + 1.0, tGo - 1.5 * c.period - cr.t));
            long key = Py.Round(cr.t + tPe);
            if (plan.checked_ != key)
            {
                plan.checked_ = key;
                V3 r1, v1;
                double t1;
                PredictDepart(Math.Max(0.0, tPe - need / 2), out r1, out v1, out t1);
                Body bad = Transfer.FirstHazard(r1, v1, t1, mu, cr.body.children, t1 + cfg.ESCAPE_CHECK_S);
                if (bad != null)
                {
                    Event("departure pass skipped: its escape would meet " + bad.name);
                    plan.t_go = cr.t + tPe + c.period;
                    return Hold(c, tPe + 1.0);
                }
            }
            if (tPe <= need / 2)
            {
                SetPhase("DEPART", F(need * Accel(), 0) + " m/s at periapsis");
                return Depart(c);
            }
            return Hold(c, tPe - need / 2);
        }

        void PredictDepart(double wait, out V3 r, out V3 v, out double t)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            Orbit.Propagate(cr.r, cr.v, mu, wait, out r, out v);
            t = cr.t + wait;
            double accel = Accel();
            double arc = Py.Radians(cfg.DEPART_ARC_DEG);
            for (int i = 0; i < 100000; i++)
            {
                Conic c = Orbit.Conic(r, v, mu);
                double rem = Math.Sqrt(Math.Pow(plan.vinf.Value, 2) + 2 * mu / c.r) - v.Norm;
                double nu = c.nu <= Math.PI ? c.nu : c.nu - 2 * Math.PI;
                if (rem <= cfg.CORRECT_DV || nu > arc) break;
                double h = Math.Max(1.0, Math.Min(rem / accel, 0.005 * c.r / Math.Max(v.Norm, 1.0)));
                Sim.Rk4(ref r, ref v, 1.0, mu, v.Unit * accel, h);
                t += h;
            }
        }

        Cmd Depart(Conic c)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            double tick = Tick(c);
            double rem = Math.Sqrt(Math.Pow(plan.vinf.Value, 2) + 2 * mu / c.r) - cr.v.Norm;
            double nu = c.nu <= Math.PI ? c.nu : c.nu - 2 * Math.PI;
            if (rem <= cfg.CORRECT_DV || nu > Py.Radians(cfg.DEPART_ARC_DEG))
            {
                if (c.e < 1)
                {
                    plan.t_go = cr.t;
                    plan.checked_ = null;
                    SetPhase("WAIT", "still bound after this pass (apoapsis " + Km(c.ra, cr.body) + "): another");
                    return Cmd.Of(0.0, null, tick);
                }
                SetPhase("ESCAPE", "v_inf " + F(c.vinf, 0) + " m/s (window " + F(plan.vinf.Value, 0) + ")");
                return Cmd.Of(0.0, null, tick);
            }
            return Cmd.Of(Math.Min(1.0, rem / (Accel() * tick)), cr.v.Unit, tick);
        }

        Cmd Correct(Conic c, Body frame)
        {
            Craft cr = craft;
            Body d = dest;
            double tick = Tick(c);
            double dt;
            if (CaptureDue(tick, out dt)) return StartCapture(c);
            tick = Math.Min(tick, dt);
            if (plan.match_vs.HasValue && cr.body == frame)
            {
                double dt2;
                if (MatchDue(tick, out dt2)) return Match(c);
                tick = Math.Min(tick, dt2);
            }
            if (cr.body != frame) return Cmd.Of(0.0, null, tick);
            double due = plan.next_fix ?? -Py.Inf;
            if (burn == null && cr.t < due) return Cmd.Of(0.0, null, Math.Min(tick, due - cr.t));
            if (kind == "planet")
            {
                Cmd big = Retarget(c, tick, false);
                if (big != null) return big;
            }
            double horizon;
            if (plan.horizon.HasValue) horizon = plan.horizon.Value;
            else if (kind == "planet")
            {
                LaunchWindow w = plan.window;
                horizon = 1.5 * Math.Max((plan.arrive_t ?? w.tDep + w.tof) - cr.t, 86400.0);
            }
            else horizon = Horizon(cr.r, cr.v);
            double? guess = kind == "planet" ? plan.arrive_t : null;
            Approach ca;
            V3 dvc = Transfer.Correction(cr.r, cr.v, cr.t, cr.body.mu, d, horizon, plan.rc.Value, d.Normal,
                                         out ca, 0.01, guess, Polar());
            if (guess.HasValue) plan.arrive_t = ca.t;
            plan.vrel = ca.vrel; plan.ca_t = ca.t;
            plan.horizon = Math.Max(ca.t - cr.t, 0) * 1.5 + 3600;
            double mag = dvc.Norm;
            double toCa = ca.t - cr.t;
            if (kind == "planet" && mag > cfg.CORRECT_MAX_DV)
            {
                Cmd big = Retarget(c, tick, true);
                if (big != null) return big;
            }
            if (kind == "moon" && (ca.d.Norm > cfg.MISS_REPLAN * d.Soi || mag > cfg.CORRECT_MAX_DV))
            {
                if (Orbit.Conic(cr.r, cr.v, cr.body.mu).e < 1)
                {
                    plan.pass_ = null;
                    plan.horizon = null;
                    SetPhase("WAIT", "no encounter (" + F(ca.d.Norm / 1e3, 0) + " km from " + d.name + "): a new push");
                    return Cmd.Of(0.0, null, tick);
                }
            }
            if (mag < cfg.CORRECT_DV || toCa < cfg.CORRECT_MIN_LEAD_S)
            {
                plan.next_fix = cr.t + Math.Max(toCa * cfg.CORRECT_EVERY, 600);
                return Cmd.Of(0.0, null, tick);
            }
            return Cmd.Of(Math.Min(1.0, mag / (Accel() * tick)), dvc.Unit, tick);
        }

        Cmd Retarget(Conic c, double tick, bool force)
        {
            Craft cr = craft;
            Body d = dest;
            double mu = cr.body.mu;
            V3 n = V3.Cross(cr.r, cr.v).Unit;
            double? taN = plan.lambert_t;
            double ta;
            if (!taN.HasValue)
            {
                LaunchWindow w = plan.window;
                double left = Math.Max((plan.arrive_t ?? w.tDep + w.tof) - cr.t, 10 * 86400.0);
                Approach ca = Transfer.ClosestApproach(cr.r, cr.v, cr.t, mu, d, 1.6 * left);
                double limit = Math.Min(cfg.RETARGET_MISS * d.Soi, cfg.RETARGET_FRACTION * d.sma);
                if (!force && ca.d.Norm < limit) return null;
                V3 gdv;
                double gta;
                if (!Transfer.Retarget(cr.r, cr.v, cr.t, mu, d, cr.t + 0.6 * left, cr.t + 1.6 * left, n,
                                       out gdv, out gta, 24, plan.rc, Polar()))
                    return null;
                plan.lambert_t = gta; plan.arrive_t = gta;
                plan.horizon = null;
                Event("re-targeted: " + F(gdv.Norm, 0) + " m/s onto a Lambert arc arriving " + FmtUt(gta)
                      + " (the miss was " + F(ca.d.Norm / 1e9, 1) + " Gm)");
                ta = gta;
            }
            else ta = taN.Value;
            V3? dvl = Transfer.LambertDv(cr.r, cr.v, cr.t, mu, d, ta, n, plan.rc, Polar());
            if (!dvl.HasValue || dvl.Value.Norm < cfg.CORRECT_DV)
            {
                plan.lambert_t = null;
                plan.next_fix = cr.t + 600;
                return null;
            }
            return Cmd.Of(Math.Min(1.0, dvl.Value.Norm / (Accel() * tick)), dvl.Value.Unit, tick);
        }

        Cmd ApproachStep(Conic c)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            double tick = Tick(c);
            if (cr.body != dest) return Cmd.Of(0.0, null, tick);
            double tPe = Orbit.TimeToPeriapsis(c, mu);
            double dt;
            bool due = CaptureDue(tick, out dt);
            if (due || tPe < 0) return StartCapture(c);
            tick = Math.Min(tick, dt);
            Cmd dodge = Dodge(c, tPe);
            if (dodge != null) return dodge;
            double need = Transfer.CaptureNeed(c, mu, plan.ra_cap.Value);
            double err = c.rp - plan.rc.Value;
            if (Math.Abs(err) < cfg.APPROACH_TOL * plan.rc.Value || tPe < cfg.APPROACH_MIN_LEAD_S)
                return Cmd.Of(0.0, null, Math.Min(tick, Math.Max(1.0, tPe - need / Accel() / 2)));
            V3 g = RpGradientHere();
            double gg = g.x * g.x + g.y * g.y + g.z * g.z;
            if (gg == 0) return Cmd.Of(0.0, null, tick);
            V3 dvv = new V3(-err * g.x / gg, -err * g.y / gg, -err * g.z / gg);
            double mag = dvv.Norm;
            if (mag < cfg.CORRECT_DV) return Cmd.Of(0.0, null, tick);
            return Cmd.Of(Math.Min(1.0, mag / (Accel() * tick)), dvv.Unit, tick);
        }

        Cmd Dodge(Conic c, double tPe)
        {
            Craft cr = craft;
            double mu = cr.body.mu;
            var moons = cr.body.children;
            if (plan.dodge.HasValue)
            {
                V3 left = plan.dodge.Value - plan.dodge_dir.Value * lastDv;
                lastDv = 0.0;
                if (V3.Dot(left, plan.dodge_dir.Value) <= 0.01) plan.dodge = null;
                else
                {
                    plan.dodge = left;
                    double tick = Tick(c);
                    return Cmd.Of(Math.Min(1.0, left.Norm / (Accel() * tick)), plan.dodge_dir.Value, tick);
                }
            }
            if (moons.Count == 0 || cr.t < (plan.dodge_next ?? -Py.Inf)) return null;
            plan.dodge_next = cr.t + 0.1 * tPe;
            if (Transfer.FirstHazard(cr.r, cr.v, cr.t, mu, moons, cr.t + tPe) == null || !FlybyHarms(cr.t + tPe))
                return null;
            Capture cap = CapSpecHere();
            V3 g = RpGradientHere();
            V3 u = cr.v.Unit;
            V3 e1 = g - u * V3.Dot(g, u);
            if (e1.Norm > 0) e1 = e1.Unit;
            else { V3 R, T, N; Orbit.Rtn(cr.r, cr.v, out R, out T, out N); e1 = R; }
            V3 e2 = V3.Cross(u, e1);
            double reach = 0.5 * Accel() * tPe;
            Target tg = finalTarget;
            V3? nT = null;
            if (tg.inc.HasValue)
            {
                double lanNow, incNow;
                Orbit.InclinationLan(cr.r, cr.v, out incNow, out lanNow);
                nT = Orbit.PlaneNormal(tg.inc.Value, tg.lan ?? lanNow);
            }
            double vC = Math.Sqrt(mu / plan.rc.Value);
            Func<V3, double> tilt = v_ => Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(V3.Cross(cr.r, v_).Unit, nT.Value))));
            Func<V3, V3, double> cost = (dvv, v1) =>
            {
                if (!nT.HasValue) return dvv.Norm;
                double extra = Math.Max(0.0, tilt(v1) - tilt(cr.v));
                return dvv.Norm + 2 * vC * Math.Sin(extra / 2);
            };
            V3? best = null;
            double bestC = Py.Inf, m = 0.5;
            while (m <= reach && m < bestC)
            {
                for (int k = 0; k < 8; k++)
                {
                    double a = k * Math.PI / 4;
                    V3 dvv = (e1 * Math.Cos(a) + e2 * Math.Sin(a)) * m;
                    for (int i = 0; i < 2; i++)
                    {
                        V3 vv = cr.v + dvv;
                        Conic ck;
                        V3 g1 = Transfer.RpGradient(cr.r, vv, mu, out ck);
                        double gg1 = V3.Dot(g1, g1);
                        if (gg1 > 0) dvv = dvv + g1 * ((plan.rc.Value - ck.rp) / gg1);
                    }
                    V3 v1 = cr.v + dvv;
                    Conic c1 = Orbit.Conic(cr.r, v1, mu);
                    if (!(cap.floor <= c1.rp && c1.rp <= cap.rpMax) || dvv.Norm > reach) continue;
                    double t1 = Orbit.TimeToPeriapsis(c1, mu);
                    double cv = cost(dvv, v1);
                    if (cv < bestC && t1 > 0 && Transfer.FirstHazard(cr.r, v1, cr.t, mu, moons, cr.t + t1) == null)
                    {
                        best = dvv; bestC = cv;
                    }
                }
                m *= 2;
            }
            if (!best.HasValue) return null;
            plan.dodge = best; plan.dodge_dir = best.Value.Unit;
            lastDv = 0.0;
            Event("dodging a moon on the way in: " + F(best.Value.Norm, 1) + " m/s");
            return Dodge(c, tPe);
        }

        bool FlybyHarms(double tEnd)
        {
            Craft cr = craft;
            Body d = dest;
            var cp = new Craft(cr.body, cr.r, cr.v, cr.m, cr.t);
            double floor = d.radius + d.atmosphere + cfg.SAFE_ALT_M;
            bool flew = false;
            while (cp.t < tEnd)
            {
                Body b = cp.body;
                Conic c = Orbit.Conic(cp.r, cp.v, b.mu);
                if (b != d && c.rp < b.radius + b.atmosphere && Orbit.TimeToPeriapsis(c, b.mu) > 0) return true;
                if (b == d && flew) return c.rp < floor;
                var hit = Coast(cp, Math.Max(1.0, Math.Min(tEnd - cp.t, BoundaryTick(cp))));
                if (hit != null && hit.Item1 == "exit" && cp.body == d) flew = true;
                if (hit != null && hit.Item1 == "exit" && cp.body != d) return true;
            }
            return false;
        }

        Cmd FloorGuard(Conic c)
        {
            Craft cr = craft;
            Body b = cr.body;
            if (phase == "CAPTURE" || phase == "DEPART" || c.e >= 1) return null;
            double crash = b.radius + b.atmosphere;
            if (c.rp >= crash || Orbit.TimeToPeriapsis(c, b.mu) < 0) return null;
            V3 g = RpGradientHere();
            double gg = V3.Dot(g, g);
            if (gg == 0) return null;
            if (plan.lifting != b.name)
            {
                plan.lifting = b.name;
                Event("periapsis " + Km(c.rp, b) + " inside " + b.name + ": lifting it");
            }
            double tick = Tick(c);
            V3 want = g * ((crash + cfg.SAFE_ALT_M - c.rp) / gg);
            return Cmd.Of(Math.Min(1.0, want.Norm / (Accel() * tick)), want.Unit, tick);
        }

        Cmd Evade(Conic c)
        {
            Craft cr = craft;
            Body b = cr.body;
            if (b == dest || b == home || b == dest.parent) return null;
            double floor = b.radius + b.atmosphere + cfg.SAFE_ALT_M;
            if (c.rp >= floor || Orbit.TimeToPeriapsis(c, b.mu) < 0) return null;
            V3 g = RpGradientHere();
            double gg = V3.Dot(g, g);
            if (gg == 0) return null;
            if (plan.evading != b.name)
            {
                plan.evading = b.name;
                Event("evasive burn: periapsis " + Km(c.rp, b) + " over " + b.name);
            }
            double tick = Tick(c);
            V3 want = g * ((floor * 1.3 - c.rp) / gg);
            return Cmd.Of(Math.Min(1.0, want.Norm / (Accel() * tick)), want.Unit, tick);
        }

        // --- match: shed the arrival speed about the star ------------------------
        void MatchAim(V3 u, out V3 P, out V3 V, out double back)
        {
            Body d = dest;
            double vs = plan.match_vs.Value;
            V3 w = V3.Cross(u, Transfer.AimNormal(u, d.Normal, Polar())).Unit;
            double b = AimB();
            back = Math.Sqrt(Math.Max(Math.Pow(cfg.MATCH_STANDOFF * d.Soi, 2) - b * b, 0.0));
            P = u * -back + w * b;
            V = u * vs;
        }

        bool Polar() { return kind == "planet" && dest.children.Count > 0 && !finalTarget.inc.HasValue; }

        double AimB() { return Transfer.AimParameter(plan.rc.Value, plan.match_vs.Value, dest); }

        Approach MatchEncounter()
        {
            Craft cr = craft;
            Approach ca = Transfer.ClosestApproach(cr.r, cr.v, cr.t, cr.body.mu, dest,
                                                   plan.horizon ?? 1.5 * Math.Max(plan.ca_t.Value - cr.t, 86400.0),
                                                   400, plan.ca_t);
            plan.ca_t = ca.t; plan.vrel = ca.vrel;
            return ca;
        }

        double MatchLead(double v)
        {
            double vs = plan.match_vs.Value;
            double a = cfg.MATCH_SHARE * Accel();
            double back = cfg.MATCH_STANDOFF * dest.Soi;
            return ((v * v - vs * vs) / (2 * a) + back) / v;
        }

        bool MatchDue(double tick, out double dt)
        {
            Craft cr = craft;
            dt = tick;
            if (plan.match_done == true || !plan.ca_t.HasValue || plan.ca_t.Value <= cr.t) return false;
            double v = plan.vrel.Value.Norm;
            if (v <= plan.match_vs.Value) return false;
            double lead = MatchLead(v);
            double left = plan.ca_t.Value - cr.t;
            if (left > 1.5 * lead) { dt = Math.Min(tick, Math.Max(60.0, 0.5 * (left - 1.5 * lead))); return false; }
            Approach ca = MatchEncounter();
            v = ca.vrel.Norm;
            lead = MatchLead(v); left = ca.t - cr.t;
            if (left <= lead) return true;
            dt = Math.Min(tick, Math.Max(60.0, 0.5 * (left - lead)));
            return false;
        }

        Cmd Match(Conic c)
        {
            Craft cr = craft;
            Body d = dest;
            double tick = Tick(c);
            if (cr.body != d.parent) return Cmd.Of(0.0, null, tick);
            Approach ca = MatchEncounter();
            double v = ca.vrel.Norm;
            if (phase != "MATCH")
            {
                SetPhase("MATCH", F(v, 0) + " m/s to shed about " + d.parent.name + ", "
                                  + F((ca.t - cr.t) / 3600, 0) + " h out");
                plan.match_end_dv = dv + 2 * (v - plan.match_vs.Value) + 20.0;
            }
            V3 u = ca.vrel.Unit;
            V3 P, V;
            double back;
            MatchAim(u, out P, out V, out back);
            double vs = plan.match_vs.Value;
            double tGo = 2 * Math.Max(v * (ca.t - cr.t) - back, 0.0) / (v + vs);
            bool spent = dv > plan.match_end_dv.Value;
            if (v <= 1.05 * vs || tGo < cfg.MATCH_END_S || spent)
            {
                V3 r, vr;
                DestRel(out r, out vr);
                plan.match_done = true;
                plan.next_fix = null;
                SetPhase(kind == "moon" ? "COAST" : "CRUISE",
                         (spent ? "given up: " : "matched: ") + "arriving at " + F(v, 0) + " m/s, "
                         + F(r.Norm / 1e6, 1) + " Mm from " + d.name);
                return Cmd.Of(0.0, null, tick);
            }
            tick = Math.Min(tick, Math.Max(1.0, 0.02 * tGo));
            double mu = cr.body.mu;
            V3 rc_, vc_, rd, vd;
            Orbit.Propagate(cr.r, cr.v, mu, tGo, out rc_, out vc_);
            d.State(cr.t + tGo, out rd, out vd);
            V3 zem = (rd + P) - rc_;
            V3 zev = (vd + V) - vc_;
            V3 acc = zem * (6 / Math.Pow(tGo, 2)) - zev * (2 / tGo);
            return Cmd.Of(Math.Min(1.0, acc.Norm / Accel()), acc.Unit, tick);
        }

        // --- capture -----------------------------------------------------------------
        /// <summary>(r, v) relative to the destination, from its SOI or its
        /// parent's; false from anywhere else.</summary>
        bool DestRel(out V3 r, out V3 v)
        {
            Craft cr = craft;
            Body d = dest;
            if (cr.body == d) { r = cr.r; v = cr.v; return true; }
            if (cr.body == d.parent)
            {
                V3 rb, vb;
                d.State(cr.t, out rb, out vb);
                r = cr.r - rb; v = cr.v - vb;
                return true;
            }
            r = v = V3.Zero;
            return false;
        }

        Capture CapSpecHere() { return CapSpec(dest, plan.rc.Value, plan.ra_cap.Value, Ceiling(dest), cfg); }

        double LongCapturableHere() { return LongCapturable(CapSpecHere(), dest, Accel(), cfg); }

        /// <summary>The capture law's decision now: burn (true, throttle in
        /// x) or coast (false, dt in x).</summary>
        bool CaptureDecide(double tick, bool started, out double x)
        {
            Craft cr = craft;
            V3 r, v;
            x = tick;
            if (!DestRel(out r, out v)) return false;
            Capture cap = CapSpecHere();
            Conic cc = Orbit.Conic(r, v, cap.mu);
            if (!started && cr.body != dest) return false;
            double wait;
            double rho = Transfer.CaptureStep(r, v, cc, cap, Accel(), started, out wait);
            if (rho <= 0) { x = Math.Min(tick, Math.Max(1.0, wait)); return false; }
            x = rho;
            return true;
        }

        bool CaptureDue(double tick, out double dt)
        {
            double x;
            bool burnNow = CaptureDecide(tick, false, out x);
            dt = burnNow ? tick : x;
            return burnNow;
        }

        Cmd StartCapture(Conic c)
        {
            V3 r, v;
            DestRel(out r, out v);
            Capture cap = CapSpecHere();
            Conic cc = Orbit.Conic(r, v, cap.mu);
            double outD = r.Norm;
            string where = craft.body == dest ? "" : ", " + F(outD / 1e6, 1) + " Mm out (" + F(outD / dest.Soi, 1) + " SOI)";
            SetPhase("CAPTURE", F(Transfer.CaptureNeed(cc, cap.mu, cap.raCap), 0) + " m/s at periapsis, starting "
                                + F(Orbit.TimeToPeriapsis(cc, cap.mu) / 3600, 1) + " h before it" + where);
            return CaptureStep(c);
        }

        Cmd CaptureStep(Conic c)
        {
            Craft cr = craft;
            V3 r, v;
            if (!DestRel(out r, out v)) return Cmd.Of(0.0, null, Tick(c));
            Capture cap = CapSpecHere();
            Conic cc = Orbit.Conic(r, v, cap.mu);
            double tick = Math.Min(Math.Min(Tick(c), Math.Max(1.0, cc.r / Math.Max(v.Norm, 1e-3) * 2 * Math.PI
                                                                 / cfg.TICKS_PER_ORBIT)),
                                   Transfer.CaptureStepS(v, Accel()));
            if (cr.body == dest && Transfer.Captured(r, v, cc, cap))
            {
                target = finalTarget;
                SetPhase("FINAL", "captured, " + Km(cc.ra, cr.body) + " x " + Km(cc.rp, cr.body));
                return Cmd.Of(0.0, null, tick);
            }
            if (Transfer.CaptureLost(r, v, cc, cap))
                return Cmd.End("capture failed: the engine cannot shed the excess speed in one periapsis pass");
            double x;
            if (!CaptureDecide(tick, true, out x)) return Cmd.Of(0.0, null, x);
            double full = Accel() * tick;
            V3 want = Transfer.CaptureCommand(r, v, cc, cap, x * full);
            return Cmd.Of(Math.Min(1.0, want.Norm / full), want.Unit, tick);
        }

        V3 RpGradientHere()
        {
            Conic c;
            return Transfer.RpGradient(craft.r, craft.v, craft.body.mu, out c);
        }

        // --- module functions -------------------------------------------------------------
        /// <summary>UT as 'y1 d230 03:12' in Kerbin years (426 d) and 6 h days.</summary>
        public static string FmtUt(double t)
        {
            const double day = 21600, year = 426 * day;
            double y = Math.Floor(t / year), rem = t - y * year;
            double d = Math.Floor(rem / day); rem -= d * day;
            double h = Math.Floor(rem / 3600); rem -= h * 3600;
            return string.Format(IC, "y{0} d{1} {2:00}:{3:00}", (long)y + 1, (long)d + 1, (long)h,
                                 (long)Math.Floor(rem / 60));
        }

        public static Capture CapSpec(Body d, double rc, double raCap, double ceiling, Config cfg)
        {
            double floor = d.radius + d.atmosphere + cfg.SAFE_ALT_M;
            return new Capture
            {
                mu = d.mu, raCap = raCap, raOk = Math.Max(raCap, ceiling), floor = floor,
                rpMax = Math.Max(cfg.CAPTURE_RP_BAND * rc, 1.2 * floor), surface = d.radius + d.atmosphere,
                arc = Py.Radians(cfg.CAPTURE_ARC_DEG), fill = cfg.CAPTURE_FILL, k = cfg.CAPTURE_PATH_SHARE, rc = rc
            };
        }

        public static double LongCapturable(Capture cap, Body d, double accel, Config cfg)
        {
            Func<double, bool> ok = vinf =>
            {
                double[] got = Transfer.CaptureStart(cap, cap.rc.Value, vinf, accel);
                return got != null && got[2] <= cfg.ARRIVE_MARGIN * d.Soi;
            };
            double lo = 0.0, hi = 8000.0;
            if (ok(hi)) return hi;
            for (int i = 0; i < 16; i++)
            {
                double mid = (lo + hi) / 2;
                if (ok(mid)) lo = mid; else hi = mid;
            }
            return lo;
        }

        public static double MoonVcapLong(Body moon, double accel, double rc, double raCap, Config cfg)
        {
            return Math.Max(LongCapturable(CapSpec(moon, rc, raCap, 0.5 * moon.Soi, cfg), moon, accel, cfg),
                            0.25 * Math.Sqrt(2 * moon.mu / moon.Soi));
        }

        /// <summary>A moon transfer's waiting orbit about here and the capture
        /// it allows: apoapsis, periapsis, capture periapsis, capture
        /// apoapsis, and whether this engine can capture the arrival.  The
        /// apoapsis well short of the moon, the periapsis as low as allowed
        /// but high enough that the engine captures the arrival; never in
        /// another moon's band where it can be avoided.</summary>
        public static void WaitingOrbit(Body here, Body moon, double rpNow, double rcMin, double raCap,
                                        double accel, Config cfg, bool fromAbove, out double raWait,
                                        out double rpWait, out double rcOut, out double raCOut, out bool ok)
        {
            double rpMin = Math.Max(rpNow, here.radius + here.atmosphere + cfg.SAFE_ALT_M);
            double peri = moon.sma * (1 - moon.ecc);
            double top = peri - cfg.WAIT_SOI_MARGIN * moon.Soi;
            var others = here.children.Where(b => b != moon).ToList();
            var inner = others.Where(b => b.sma < moon.sma).ToList();
            double arc = Py.Radians(cfg.CAPTURE_ARC_DEG);
            double vM = Math.Sqrt(here.mu / moon.sma);
            double vLong = MoonVcapLong(moon, accel, rcMin, raCap, cfg);

            // (rp, ra, rc, raC) if this waiting periapsis works, else null;
            // clear: its orbit is clear of the other moons
            Func<double, double, bool, bool, Tuple<double[], bool>> candidate = (rp, ceiling, onePassPush, span) =>
            {
                double ra = Math.Min(Math.Max(cfg.WAIT_AP_FRACTION * peri, 1.05 * rp), ceiling);
                bool clear = Transfer.ClearOf(rp, others, cfg.WAIT_SOI_MARGIN)
                             && (!span || new[] { rp, ra, (rp + ra) / 2 }.All(x => Transfer.ClearOf(x, others, cfg.WAIT_SOI_MARGIN)));
                if (!clear) return Tuple.Create((double[])null, false);
                double vinf = Transfer.MoonArrivalVinf(here.mu, rp, moon.sma, vM);
                double crc, cra;
                bool cap = Transfer.CaptureOrbit(moon, vinf, accel, rcMin, raCap, arc, cfg.CAPTURE_FILL,
                                                 cfg.CAPTURE_LOOSE_FRACTION * moon.Soi, out crc, out cra);
                if (!cap && vinf <= vLong) { cap = true; crc = rcMin; cra = raCap; }
                if (cap && (!onePassPush || Transfer.PushFits(here.mu, rp, ra, moon.sma, accel, arc, cfg.CAPTURE_FILL)))
                    return Tuple.Create(new[] { rp, ra, crc, cra }, true);
                return Tuple.Create((double[])null, true);
            };
            Func<double, bool, bool, double[]> search = (ceiling, onePassPush, span) =>
            {
                double rp = rpMin;
                while (rp <= ceiling)
                {
                    var got = candidate(rp, ceiling, onePassPush, span);
                    if (got.Item1 != null) return got.Item1;
                    rp += Math.Max(1e3, 0.01 * (ceiling - rpMin));
                }
                return null;
            };
            Func<double, double[]> searchGap = ceiling =>
            {
                double step = Math.Max(1e3, 0.01 * (ceiling - rpMin));
                double rp = ceiling;
                double[] best = null;
                while (rp >= rpMin)
                {
                    var got = candidate(rp, ceiling, false, true);
                    if (got.Item1 != null) best = got.Item1;
                    else if (best != null && !got.Item2) break;
                    rp -= step;
                }
                return best;
            };

            double[] found = null;
            if (inner.Count > 0 && !fromAbove)
            {
                double wall = inner.Min(b => b.sma * (1 - b.ecc) - cfg.WAIT_SOI_MARGIN * b.Soi);
                if (wall > rpMin) found = search(Math.Min(top, wall), true, false);
            }
            if (found == null && fromAbove) found = searchGap(top);
            if (found == null) found = search(top, false, false);
            ok = found != null;
            if (ok)
            {
                rpWait = found[0]; raWait = found[1]; rcOut = found[2]; raCOut = found[3];
            }
            else
            {
                rpWait = rpMin; rcOut = rcMin; raCOut = raCap;
                double rp = top;
                while (rp > rpMin)
                {
                    if (Transfer.ClearOf(rp, others, cfg.WAIT_SOI_MARGIN)) { rpWait = rp; break; }
                    rp -= Math.Max(1e3, 0.01 * (top - rpMin));
                }
                raWait = Math.Min(Math.Max(cfg.WAIT_AP_FRACTION * peri, 1.05 * rpWait), top);
            }
            rpWait = Math.Min(rpWait, raWait);
        }

        /// <summary>"body: ap= pe= [i= lan= argpe=]" (km and degrees) as
        /// (body, fields).  Throws ArgumentException.</summary>
        public static Body ParseOrbit(string text, Dictionary<string, Body> system, out Dictionary<string, double> f)
        {
            int colon = text.IndexOf(':');
            string name = (colon < 0 ? text : text.Substring(0, colon)).Trim();
            string rest = colon < 0 ? "" : text.Substring(colon + 1);
            string cap = name.Length == 0 ? name : char.ToUpperInvariant(name[0]) + name.Substring(1).ToLowerInvariant();
            Body body;
            if (!system.TryGetValue(cap, out body)) throw new ArgumentException("unknown body '" + name + "'");
            string[] allowed = { "ap", "pe", "i", "lan", "argpe" };
            f = new Dictionary<string, double>();
            foreach (string item in rest.Replace(",", " ").Split((char[])null, StringSplitOptions.RemoveEmptyEntries))
            {
                int eq = item.IndexOf('=');
                string k = eq < 0 ? item : item.Substring(0, eq);
                string val = eq < 0 ? "" : item.Substring(eq + 1);
                if (Array.IndexOf(allowed, k) < 0)
                    throw new ArgumentException("unknown field '" + k + "' (" + string.Join(", ", allowed) + ")");
                double x;
                if (!double.TryParse(val, NumberStyles.Float, IC, out x))
                    throw new ArgumentException("not a number: " + item);
                f[k] = x;
            }
            return body;
        }

        /// <summary>The Target about body for parsed fields f: anything left
        /// out is free; an argument of periapsis on a circular target is
        /// dropped.</summary>
        public static Target TargetFrom(Body body, Dictionary<string, double> f)
        {
            bool circular = !f.ContainsKey("ap") || !f.ContainsKey("pe") || Math.Abs(f["ap"] - f["pe"]) < 1.0;
            Func<string, double?> km = k => f.ContainsKey(k) ? body.radius + f[k] * 1e3 : (double?)null;
            Func<string, double?> rad = k => f.ContainsKey(k) ? Py.Radians(f[k]) : (double?)null;
            return new Target(km("ap"), km("pe"), rad("i"), rad("lan"),
                              argpe: f.ContainsKey("argpe") && !circular ? rad("argpe") : null);
        }

        /// <summary>The legs from here to dest: each within an SOI, to a moon,
        /// or to a sibling planet.</summary>
        public static List<Body> RouteTo(Body here, Body dest)
        {
            if (dest == here || dest.parent == here || (dest.parent == here.parent && here.parent != null))
                return new List<Body> { dest };
            if (dest.parent != null && dest.parent.parent == here.parent) return new List<Body> { dest.parent, dest };
            throw new ArgumentException("no route from " + here.name + " to " + dest.name);
        }

        /// <summary>The target of leg i of legs (a planet leg that a moon leg
        /// follows ends in that moon leg's waiting orbit).</summary>
        public static Target LegTarget(List<Body> legs, int i, Target target, double accel, Config cfg)
        {
            if (i + 1 >= legs.Count) return target;
            Body leg = legs[i], moon = legs[i + 1];
            double rc = Math.Max(target.rp ?? target.ra ?? 0.0, moon.radius + moon.atmosphere + cfg.SAFE_ALT_M);
            double raW, rpW, x1, x2;
            bool ok;
            WaitingOrbit(leg, moon, leg.radius + leg.atmosphere + cfg.SAFE_ALT_M, rc,
                         cfg.CAPTURE_AP_FRACTION * moon.Soi, accel, cfg, true, out raW, out rpW, out x1, out x2, out ok);
            return new Target(raW, rpW, moon.inc, moon.lan);
        }

        /// <summary>Fly from (body, r, v, t0) to target about dest in the
        /// stand-in, in as many legs as RouteTo needs.</summary>
        public static MissionResult Fly(Dictionary<string, Body> system, Ship vessel, Body body, V3 r, V3 v,
                                        double t0, Body dest, Target target, Config cfg, bool record = true)
        {
            var legs = RouteTo(body, dest);
            double t = t0;
            var results = new List<MissionResult>();
            for (int i = 0; i < legs.Count; i++)
            {
                Target tgt = LegTarget(legs, i, target, vessel.thrustN / vessel.massKg, cfg);
                MissionResult res = new Mission(system, vessel, body, r, v, t, legs[i], tgt, cfg, record).Run();
                results.Add(res);
                if (!res.done) break;
                body = res.body; t = res.t;
                r = res.final.rvec; v = res.final.vvec;
            }
            MissionResult last = results[results.Count - 1];
            return new MissionResult
            {
                done = last.done, reason = last.reason, t0 = t0, t = last.t, dv = results.Sum(x => x.dv),
                propUsed = results.Sum(x => x.propUsed), maneuvers = results.SelectMany(x => x.maneuvers).ToList(),
                events = results.SelectMany(x => x.events).ToList(), body = last.body, final = last.final
            };
        }
    }
}
