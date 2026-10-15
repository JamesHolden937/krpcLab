// The autopilot in the game: flies Mission's phase machine against the
// vessel, in the game's own physics step.
//
// Every step on rails it brings the vessel from where the last step left it
// to the game's time now: commands are decided from the vessel's state as
// they fall due (several in one step at high warp, none in many at low), a
// thrusting command is integrated (RK4, gravity plus the engines' thrust
// along the axis as the vessel points it, propellant and charge drawn) and
// the orbit written back to the game through its elements; a coast is left
// to the game.  Nothing waits on anything: no other process, no hold, no
// change to Unity's time scale.  The warp rate is the speed and nothing
// else: the highest the frame budget, the propellant on hand for one step and
// a vessel still turning allow.
//
// A coast is run ahead in prediction (exact two-body) until the mission next
// wants thrust, so the vessel turns to that burn while it coasts.  A thrust
// command fires only if the thrust axis is within ALIGN_DEG of it when it
// starts; it then thrusts along the axis as the attitude leaves it each step
// (on the command once the wheels hold it there, behind it while they still
// turn), and its dv is booked from what was integrated.  What the axis is off
// the command costs is logged: the thrust dv spent, and the part of it lost
// to the angle (dv x (1 - cos)).

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using UnityEngine;

namespace LowThrust
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class Controller : MonoBehaviour
    {
        static readonly CultureInfo IC = CultureInfo.InvariantCulture;
        public enum Mode { Idle, Planning, Flying }

        // --- what the window and the bridge see -----------------------------------
        public static Controller Instance;
        public static Mode mode = Mode.Idle;
        public static List<string> status = new List<string>();
        public static string result = "";
        public static string logPath = "";
        public static int flights;                 // flights ended since load
        public static List<string> overrides = new List<string>();

        // --- the flight ----------------------------------------------------------
        Config cfg;
        Vessel vessel;
        Dictionary<string, Body> sys;
        Engines engines;
        FlightLog log;
        List<Body> legs;
        int leg;
        Target finalTarget;
        Mission mis;
        Thread planner;
        Mission planned;
        Exception planError;
        double t0;

        class Command
        {
            public double thr, tick, t0, tEnd, dvDone, elapsed;
            public bool coast, fired;
            public V3 d, rtnCmd, r0, v0;
            public double err;
        }
        Command cur;
        Step pending;
        V3 coastRtn = new V3(0.0, 1.0, 0.0);
        double tSim;
        // what the attitude holds: components against the orbit (R, T, N)
        V3 holdVec;
        bool holding;

        // --- measures --------------------------------------------------------------
        readonly Stopwatch frameWatch = new Stopwatch();
        readonly Stopwatch wall = new Stopwatch();
        double emaMs;
        int cpuIndex = 1;
        class PhaseStat { public double wall, game, maxStepMs, frames, frameMs, maxFrameMs; public int steps; }
        readonly Dictionary<string, PhaseStat> stats = new Dictionary<string, PhaseStat>();
        int starved, unfired, fired;
        // the thrust axis this step, against the orbit (R, T, N)
        V3 axisRtn;
        // steering: thrust dv flown, its part lost to the axis being off the
        // command, and the dv flown more than STEER_OFF_DEG off it
        double steerDv, steerLoss, steerOffDv, steerMaxDeg;
        double shareSum, shareN;
        double lastShown;
        string lastPhase = "";

        // --- Persistent Thrust, if installed, must not thrust as well ---------------
        static FieldInfo ptThrottle;
        static MethodInfo ptSet;

        void Awake() { Instance = this; }

        void OnDestroy()
        {
            if (mode != Mode.Idle) Stop("the flight scene ended", false);
            if (Instance == this) Instance = null;
        }

        static Config LoadConfig(IEnumerable<string> extra)
        {
            var cfg = new Config();
            string file = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/LowThrust/PluginData/settings.cfg");
            if (File.Exists(file))
                cfg.Apply(File.ReadAllLines(file).Select(l => l.Trim())
                              .Where(l => l.Length > 0 && !l.StartsWith("#") && !l.StartsWith("//")));
            return cfg.Apply(extra);
        }

        void Show(params string[] lines) { status = new List<string>(lines); }

        // --- the window's buttons ------------------------------------------------------
        /// <summary>The target from the window: body and fields; a blank
        /// field is free, one apsis alone a circular orbit there.</summary>
        static string TargetText()
        {
            string[] names = { "pe", "ap", "i", "lan", "argpe" };
            var f = new Dictionary<string, string>();
            for (int i = 0; i < names.Length; i++) f[names[i]] = PanelState.fields[i].Trim();
            if (f["pe"] != "" && f["ap"] == "") f["ap"] = f["pe"];
            else if (f["ap"] != "" && f["pe"] == "") f["pe"] = f["ap"];
            return PanelState.body.ToLowerInvariant() + ": "
                   + string.Join(" ", names.Where(k => f[k] != "").Select(k => k + "=" + f[k]).ToArray());
        }

        /// <summary>The stand-in's flight from here (constant mass), on a
        /// worker thread; its summary to the window.</summary>
        public void Plan()
        {
            if (mode != Mode.Idle) return;
            Vessel v = FlightGlobals.ActiveVessel;
            try
            {
                cfg = LoadConfig(overrides);
                var system = GameSystem.Build();
                Dictionary<string, double> f;
                Body dest = Mission.ParseOrbit(TargetText(), system, out f);
                Target target = Mission.TargetFrom(dest, f);
                Body here = system[v.orbit.referenceBody.bodyName];
                double ut = Planetarium.GetUniversalTime();
                V3[] vel = new V3[1];
                V3 r = GameSystem.StateOf(v.orbit, ut, vel);
                var eng = new Engines(v);
                double thrust = eng.thrustN > 0 ? eng.thrustN : Engines.FullThrust(v);
                var ship = new Ship(Engines.Mass(v), thrust, eng.ispS > 0 ? eng.ispS : 4200.0);
                Config c = cfg;
                double? dvMax, daysMax;
                string why = Budget(c, here, r, vel[0], dest, target, out dvMax, out daysMax);
                PanelState.options = new List<TradeOption>();
                PanelState.picked = null;
                if (why != null) { Show(why); return; }
                Show("planning: the mission flown at " + c.PLAN_GRID + " arc widths...");
                new Thread(() =>
                {
                    try
                    {
                        var watch = Stopwatch.StartNew();
                        List<TradeOption> front;
                        TradeOption pick = Pick(system, ship, here, r, vel[0], ut, dest, target, c, dvMax, daysMax, out why,
                                                out front);
                        PanelState.options = Tradeoff.Thin(front, 10, pick);
                        var lines = new List<string>();
                        if (pick == null) lines.Add(why);
                        else
                        {
                            MissionResult res = pick.result;
                            lines.Add("PLAN: " + pick.days.ToString("F1", IC) + " days, " + pick.dv.ToString("F0", IC)
                                      + " m/s, " + res.propUsed.ToString("F0", IC) + " kg propellant (arcs "
                                      + pick.eta.ToString("F3", IC) + (pick.share < 1 ? ", transfer " + pick.share.ToString("F2", IC) : "") + ")");
                            var by = new List<KeyValuePair<string, double>>();
                            foreach (Maneuver m in res.maneuvers)
                            {
                                string k = m.body + " " + m.phase;
                                int i = by.FindIndex(x => x.Key == k);
                                if (i < 0) by.Add(new KeyValuePair<string, double>(k, m.dv));
                                else by[i] = new KeyValuePair<string, double>(k, by[i].Value + m.dv);
                            }
                            foreach (var kv in by.Take(10)) lines.Add("  " + kv.Key + "\t" + kv.Value.ToString("F0", IC) + " m/s");
                        }
                        lines.Add("(" + watch.Elapsed.TotalSeconds.ToString("F1", IC) + " s to plan)");
                        PanelState.picked = pick;
                        status = lines;
                    }
                    catch (Exception e) { status = new List<string> { "PLAN failed:", e.Message }; }
                }) { IsBackground = true }.Start();
            }
            catch (Exception e) { Show("PLAN failed:", e.Message); }
        }

        /// <summary>The budget from the window (null: none); a refusal at
        /// once if the dv asked for is far below even impulsive burns, or a
        /// field is not a number.</summary>
        static string Budget(Config c, Body here, V3 r, V3 v, Body dest, Target target, out double? dvMax,
                             out double? daysMax)
        {
            dvMax = daysMax = null;
            double x;
            if (PanelState.maxDv.Trim() != "")
            {
                if (!double.TryParse(PanelState.maxDv.Trim(), NumberStyles.Float, IC, out x) || x <= 0)
                    return "max \u0394v: not a positive number";
                dvMax = x;
            }
            if (PanelState.maxDays.Trim() != "")
            {
                if (!double.TryParse(PanelState.maxDays.Trim(), NumberStyles.Float, IC, out x) || x <= 0)
                    return "max time: not a positive number";
                daysMax = x;
            }
            double imp = Tradeoff.Impulsive(here, r, v, dest, target, c);
            if (dvMax.HasValue && dvMax.Value < c.IMPULSIVE_MARGIN * imp)
                return "impossible: " + dvMax.Value.ToString("F0", IC) + " m/s is far below what even ideal "
                       + "impulsive burns need (~" + imp.ToString("F0", IC) + " m/s)";
            return null;
        }

        /// <summary>The mission flown at every arc width, the frontier, and the
        /// option the budget picks (null and why if none).</summary>
        static TradeOption Pick(Dictionary<string, Body> system, Ship ship, Body here, V3 r, V3 v, double ut, Body dest,
                                Target target, Config c, double? dvMax, double? daysMax, out string why,
                                out List<TradeOption> front)
        {
            var scanned = Tradeoff.Scan(system, ship, here, r, v, ut, dest, target, c,
                                        dvMax.HasValue ? dvMax.Value * c.BUDGET_SCAN_FACTOR : Py.Inf,
                                        daysMax.HasValue ? daysMax.Value * Tradeoff.DAY * c.BUDGET_SCAN_FACTOR : Py.Inf);
            front = Tradeoff.Frontier(scanned);
            return Tradeoff.Choose(scanned, dvMax, daysMax, c, out why);
        }

        /// <summary>Start flying to the window's target.</summary>
        public void Begin()
        {
            if (mode != Mode.Idle) return;
            vessel = FlightGlobals.ActiveVessel;
            if (vessel == null) return;
            cfg = LoadConfig(overrides);
            log = new FlightLog(cfg.LOG_INTERVAL_UT);
            logPath = log.path;
            result = "";
            try
            {
                double ut = Planetarium.GetUniversalTime();
                log.Event(ut, cfg.Line());
                sys = GameSystem.Build();
                log.Event(ut, "rails check: " + GameSystem.Check(sys, ut, 1e6));
                Dictionary<string, double> f;
                string text = TargetText();
                Body dest = Mission.ParseOrbit(text, sys, out f);
                finalTarget = Mission.TargetFrom(dest, f);
                Body here = sys[vessel.orbit.referenceBody.bodyName];
                {
                    V3[] vv = new V3[1];
                    V3 rr = GameSystem.StateOf(vessel.orbit, ut, vv);
                    string why = Budget(cfg, here, rr, vv[0], dest, finalTarget, out dvMax, out daysMax);
                    if (why != null) throw new ArgumentException(why);
                }
                Prepare();
                log.Event(ut, "target: " + here.name + " -> " + dest.name + " " + text);
                log.Event(ut, string.Format(IC, "vessel: {0:F0} kg, {1:F0} N from {2} engine(s), Isp {3:F0} s, {4:F4} m/s^2; "
                                                + "wheels {5:F2} {6:F2} {7:F2} deg/s^2",
                                            Engines.Mass(vessel), engines.thrustN, engines.list.Count, engines.ispS,
                                            engines.thrustN / Engines.Mass(vessel), Py.Degrees(Attitude.alpha.x),
                                            Py.Degrees(Attitude.alpha.y), Py.Degrees(Attitude.alpha.z)));
                log.Event(ut, FrameCheck(vessel));
                log.Event(ut, engines.Describe());
                if (engines.thrustN <= 0) throw new ArgumentException("no engine lit on this vessel");
                legs = Mission.RouteTo(here, dest);
                leg = 0;
                t0 = ut;
                stats.Clear();
                starved = unfired = fired = 0;
                steerDv = steerLoss = steerOffDv = steerMaxDeg = 0;
                shareSum = shareN = 0;
                overBudget = false;
                plan = null;
                wall.Reset(); wall.Start();
                StartLeg();
            }
            catch (Exception e)
            {
                Stop("error: " + e.Message, false);
            }
        }

        double? dvMax, daysMax;
        bool overBudget;
        TradeOption plan;

        public void Terminate() { if (mode != Mode.Idle) Stop("terminated from the window", false); }

        /// <summary>Engines lit, SAS off (the wheels are steered here),
        /// throttle shut (off rails the game would thrust), warp limits
        /// relaxed.</summary>
        void Prepare()
        {
            foreach (Part p in vessel.parts)
                foreach (PartModule pm in p.Modules)
                {
                    var e = pm as ModuleEngines;
                    if (e != null && !e.EngineIgnited && e.isEnabled) e.Activate();
                }
            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            FlightInputHandler.state.mainThrottle = 0f;
            vessel.ctrlState.mainThrottle = 0f;
            engines = new Engines(vessel);
            Attitude.axis = v => engines.AxisWorld();
            if (cfg.WARP_BELOW_LIMITS) Warp.Relax(cfg.TERRAIN_MARGIN_M);
            cpuIndex = 1;
            emaMs = 0;
        }

        /// <summary>Whether the planner's frame and the game's agree: the
        /// orbit normal from the elements, against the world's normal and the
        /// body's pole (logged at the start of every flight).</summary>
        static string FrameCheck(Vessel v)
        {
            double ut = Planetarium.GetUniversalTime();
            V3[] vel = new V3[1];
            V3 r = GameSystem.StateOf(v.orbit, ut, vel);
            double inc = Py.Degrees(LowThrust.Orbit.Inclination(r, vel[0]));
            Vector3d R, T, N;
            GameSystem.WorldRtn(v, out R, out T, out N);
            double worldInc = Vector3d.Angle(N, v.mainBody.transform.up);
            double speed = vel[0].Norm, wspeed = v.obt_velocity.magnitude;
            return string.Format(IC, "frame check: inclination {0:F3} deg from the elements, {1:F3} from the world; "
                                     + "speed {2:F2} v {3:F2} m/s; radius {4:F0} v {5:F0} m", inc, worldInc, speed,
                                 wspeed, r.Norm, (v.CoMD - v.mainBody.position).magnitude);
        }

        void StartLeg()
        {
            double ut = Planetarium.GetUniversalTime();
            Body here = sys[vessel.orbit.referenceBody.bodyName];
            V3[] vel = new V3[1];
            V3 r = GameSystem.StateOf(vessel.orbit, ut, vel);
            engines.Refresh();
            var ship = new Ship(Engines.Mass(vessel), engines.thrustN, engines.ispS);
            Target tgt = Mission.LegTarget(legs, leg, finalTarget, ship.thrustN / ship.massKg, cfg);
            Body dest = legs[leg];
            Dictionary<string, Body> s = sys;
            Config c = cfg;
            planned = null;
            planError = null;
            mode = Mode.Planning;
            Show("planning the " + (legs.Count > 1 ? "leg to " : "transfer to ") + dest.name + "...");
            bool first = leg == 0;
            TradeOption chosen = PanelState.picked;
            List<Body> route = legs;
            Target whole = finalTarget;
            planner = new Thread(() =>
            {
                try
                {
                    if (first)
                    {
                        // the dv/time trade: a clicked option, else the budget's
                        string why = null;
                        List<TradeOption> front;
                        if (chosen == null)
                            chosen = Pick(s, ship, here, r, vel[0], ut, route[route.Count - 1], whole, c, dvMax, daysMax,
                                          out why, out front);
                        if (chosen == null) throw new ArgumentException(why);
                        plan = chosen;
                        c.ETA_DEFAULT = chosen.eta;
                        c.TRANSFER_TIME_SHARE = chosen.share;
                    }
                    planned = new Mission(s, ship, here, r, vel[0], ut, dest, tgt, c);
                }
                catch (Exception e) { planError = e; }
            }) { IsBackground = true };
            planner.Start();
        }

        void Planned()
        {
            if (planner.IsAlive) return;
            if (planError != null) { Stop("error: " + planError.Message, false); return; }
            mis = planned;
            seen = 0;
            if (leg == 0 && plan != null)
                log.Event(Planetarium.GetUniversalTime(), string.Format(IC,
                    "plan: arcs {0:F3}, transfer share {5:F2}, {1:F0} m/s in {2:F1} days (budget: {3} m/s, {4} days)", plan.eta, plan.dv,
                    plan.days, dvMax.HasValue ? dvMax.Value.ToString("F0", IC) : "none",
                    daysMax.HasValue ? daysMax.Value.ToString("F1", IC) : "none", plan.share));
            Drain();
            cur = null;
            pending = null;
            tSim = Planetarium.GetUniversalTime();
            holding = false;
            mode = Mode.Flying;
        }

        int seen;
        void Drain()
        {
            for (; seen < mis.events.Count; seen++) log.Event(mis.events[seen].t, mis.events[seen].text);
        }

        // --- the step -----------------------------------------------------------------
        void FixedUpdate()
        {
            frameWatch.Reset();
            frameWatch.Start();
            Vessel v = FlightGlobals.ActiveVessel;
            Attitude.Step(v);
            if (mode == Mode.Idle || v == null) return;
            if (v != vessel) { Stop("the active vessel changed", false); return; }
            if (mode == Mode.Planning) { Planned(); if (mode != Mode.Flying) return; }
            if (FlightDriver.Pause) return;
            QuietPT(v);
            if (!Warp.OnRails(v))
            {
                if (v.packed && TimeWarp.WarpMode == TimeWarp.Modes.HIGH && TimeWarp.CurrentRateIndex == 0
                    || !v.packed)
                {
                    int idx = Warp.Cap(v, Math.Max(1, cpuIndex));
                    if (idx < 1) { Stop("the game will not rails-warp here (an atmosphere, or too low)", false); return; }
                    Warp.Set(idx);
                }
                return;
            }
            try { Fly(v); }
            catch (Exception e)
            {
                log.Event(Planetarium.GetUniversalTime(), "FATAL " + e);
                Stop("error: " + e.Message, false);
                return;
            }
            if (mode != Mode.Flying) return;
            frameWatch.Stop();
            double ms = frameWatch.Elapsed.TotalMilliseconds;
            Account(ms);
            Govern(v, ms);
        }

        void Fly(Vessel v)
        {
            double ut = Planetarium.GetUniversalTime();
            Craft cr = mis.craft;
            CelestialBody kb = v.orbit.referenceBody;
            V3[] vel = new V3[1];
            if (kb.bodyName != cr.body.name)
            {
                // the game crossed an SOI the planner did not: take its state
                Body nb = sys[kb.bodyName];
                string how = nb == cr.body.parent ? "exit" : "enter";
                if (cur != null && !cur.coast) EndThrust(cur, cur.elapsed);
                cr.body = nb;
                cr.r = GameSystem.StateOf(v.orbit, ut, vel);
                cr.v = vel[0];
                cr.t = ut;
                cur = null;
                pending = null;
                tSim = ut;
                log.Event(ut, "SOI change by the game: now about " + nb.name);
                mis.SoiChange(how, how == "exit" ? nb : nb);
                Drain();
                return;
            }
            cr.r = GameSystem.StateOf(v.orbit, tSim, vel);
            cr.v = vel[0];
            cr.t = tSim;
            cr.m = Engines.Mass(v);
            Advance(v, ut);
        }

        /// <summary>From tSim to ut: commands decided as they fall due,
        /// flown, the orbit written.  False if a decision was sent aside (game
        /// time is then held, and Update calls this again once it is made, until
        /// the autopilot has caught up with the game's clock).</summary>
        bool Advance(Vessel v, double ut)
        {
            Craft cr = mis.craft;
            bool write = false;
            int guard = 0;
            axisRtn = AxisNow(v);
            while (tSim < ut - 1e-9 && mode == Mode.Flying)
            {
                if (++guard > 100000) throw new InvalidOperationException("no progress in one step");
                if (cur == null)
                {
                    Decide(v);
                    if (mode != Mode.Flying || cur == null) return true;
                    continue;
                }
                double end = Math.Min(cur.tEnd, ut);
                double seg = end - tSim;
                if (seg > 0)
                {
                    if (cur.coast || !cur.fired)
                    {
                        var hit = Mission.Coast(cr, seg);
                        tSim = cr.t;
                        if (hit != null)
                        {
                            write = true;
                            if (!cur.coast) EndThrust(cur, tSim - cur.t0);
                            cur = null;
                            pending = null;
                            mis.SoiChange(hit.Item1, hit.Item2);
                            Drain();
                            continue;
                        }
                    }
                    else
                    {
                        write = true;
                        if (Thrust(cur, seg)) continue;     // an SOI crossed: cur ended
                    }
                    tSim = end;
                }
                if (tSim >= cur.tEnd - 1e-9)
                {
                    if (!cur.coast) EndThrust(cur, cur.tick);
                    cur = null;
                }
            }
            if (write) WriteOrbit(v);
            Point(v);
            Shown(v, ut);
            return true;
        }

        void WriteOrbit(Vessel v)
        {
            Craft cr = mis.craft;
            GameSystem.SetOrbit(v.orbit, GameSystem.Find(cr.body.name), cr.r, cr.v, cr.t);
            v.orbitDriver.updateFromParameters();
        }

        /// <summary>A thrusting command flown for seg seconds from the craft's
        /// state: RK4 substeps, the thrust along the axis held against the
        /// orbit, the propellant drawn.  True if it crossed an SOI (then the
        /// command is ended and the mission told).</summary>
        bool Thrust(Command c, double seg)
        {
            Craft cr = mis.craft;
            double hMax = c.tick / cfg.THRUST_SUBSTEPS;
            int n = Math.Max(1, (int)Math.Ceiling(seg / hMax - 1e-9));
            double h = seg / n;
            for (int i = 0; i < n; i++)
            {
                double share = engines.Draw(c.thr, h);
                if (share < 0.999) starved++;
                shareSum += share; shareN++;
                double f = engines.thrustN * c.thr * share;
                V3 R, T, N;
                LowThrust.Orbit.Rtn(cr.r, cr.v, out R, out T, out N);
                V3 d = R * axisRtn.x + T * axisRtn.y + N * axisRtn.z;
                V3 r = cr.r, v = cr.v;
                Sim.Rk4(ref r, ref v, cr.m, cr.body.mu, d * f, h);
                double dv = f / cr.m * h;
                c.dvDone += dv;
                double cos = Py.Clamp(V3.Dot(axisRtn, c.rtnCmd.Unit), -1, 1);
                steerDv += dv;
                steerLoss += dv * (1 - cos);
                double off = Py.Degrees(Math.Acos(cos));
                if (off > cfg.STEER_OFF_DEG) steerOffDv += dv;
                if (dv > 0) steerMaxDeg = Math.Max(steerMaxDeg, off);
                cr.m -= engines.massFlowKg * c.thr * share * h;
                cr.r = r; cr.v = v;
                cr.t += h;
                c.elapsed += h;
                tSim = cr.t;
                var hit = Mission.Crossing(cr, cr.r, cr.t);
                if (hit != null)
                {
                    Mission.Switch(cr, hit.Item1, hit.Item2);
                    EndThrust(c, c.elapsed);
                    cur = null;
                    pending = null;
                    mis.SoiChange(hit.Item1, hit.Item2 ?? cr.body);
                    Drain();
                    return true;
                }
            }
            return false;
        }

        void EndThrust(Command c, double flown)
        {
            mis.Account(c.d, c.fired ? c.dvDone : 0.0, flown, c.r0, c.v0);
        }

        /// <summary>The next command from the craft's state now.</summary>
        void Decide(Vessel v)
        {
            Prime();
            Act(v, Think());
        }

        /// <summary>The engines and the mass the mission plans with (main
        /// thread: they read the game).</summary>
        void Prime()
        {
            engines.Refresh();
            mis.vessel.massKg = mis.craft.m;
            mis.vessel.thrustN = engines.thrustN;
        }

        class Thought
        {
            public Step s;
            public double span, ms;
            public V3? aimRtn;
        }

        /// <summary>The mission's decision from the craft's state: planner
        /// objects only, so it may run on a worker thread.</summary>
        Thought Think()
        {
            Craft cr = mis.craft;
            var watch = Stopwatch.StartNew();
            var th = new Thought { s = pending ?? mis.DoStep() };
            pending = null;
            if (th.s.result == null && th.s.throttle <= 0)
            {
                mis.Account(V3.Zero, 0.0, th.s.tick, cr.r, cr.v);
                Body b0 = cr.body;
                V3 r0 = cr.r, v0 = cr.v;
                double t0c = cr.t;
                th.span = CoastAhead(th.s.tick, out pending, out th.aimRtn);
                cr.body = b0; cr.r = r0; cr.v = v0; cr.t = t0c;
            }
            th.ms = watch.Elapsed.TotalMilliseconds;
            return th;
        }

        /// <summary>The decision made a command (main thread).</summary>
        void Act(Vessel v, Thought th)
        {
            Craft cr = mis.craft;
            Step s = th.s;
            Drain();
            Note(th.ms);
            if (s.result != null)
            {
                LegEnded(s.result);
                return;
            }
            V3 R, T, N;
            LowThrust.Orbit.Rtn(cr.r, cr.v, out R, out T, out N);
            var c = new Command { thr = s.throttle, tick = s.tick, t0 = tSim, tEnd = tSim + s.tick, r0 = cr.r, v0 = cr.v };
            if (s.throttle > 0)
            {
                c.d = s.direction;
                c.rtnCmd = new V3(V3.Dot(c.d, R), V3.Dot(c.d, T), V3.Dot(c.d, N));
                coastRtn = c.rtnCmd;
                V3 axis = AxisPlanner(v, R, T, N);
                c.err = Py.Degrees(Math.Acos(Py.Clamp(V3.Dot(axis.Unit, c.d.Unit), -1, 1)));
                c.fired = c.err <= cfg.ALIGN_DEG && engines.thrustN > 0;
                if (c.fired) fired++;
                else unfired++;
                HoldRtn(c.rtnCmd);
            }
            else
            {
                c.coast = true;
                c.tick = th.span;
                c.tEnd = tSim + th.span;
                HoldRtn(th.aimRtn ?? coastRtn);
            }
            cur = c;
            if (dvMax.HasValue && mis.dv > dvMax.Value && !overBudget)
            {
                overBudget = true;
                log.Event(cr.t, string.Format(IC, "over the dv budget: {0:F0} m/s spent of {1:F0} (plan {2:F0})",
                                              mis.dv, dvMax.Value, plan != null ? plan.dv : 0));
            }
            Telemetry(cr, c);
        }

        /// <summary>The mission run ahead along the exact coast from the tick
        /// it just decided to coast: until it decides to thrust or ends, an
        /// SOI boundary is near, or COAST_AHEAD of a revolution has gone.</summary>
        double CoastAhead(double tick, out Step next, out V3? aimRtn)
        {
            Craft cr = mis.craft;
            next = null;
            aimRtn = null;
            Conic c = LowThrust.Orbit.Conic(cr.r, cr.v, cr.body.mu);
            double limit = cfg.COAST_AHEAD * (c.e < 1 ? c.period : 100 * tick);
            double span = 0.0;
            while (true)
            {
                Body sb = cr.body;
                V3 sr = cr.r, sv = cr.v;
                double st = cr.t;
                if (Mission.Coast(cr, tick) != null)
                {
                    cr.body = sb; cr.r = sr; cr.v = sv; cr.t = st;
                    return Math.Max(span, 1.0);
                }
                span += tick;
                if (span >= limit) return span;
                Step s = mis.DoStep();
                if (s.result != null || s.throttle > 0)
                {
                    next = s;
                    if (s.result == null)
                    {
                        // the burn's direction against the orbit where it starts
                        V3 R, T, N;
                        LowThrust.Orbit.Rtn(cr.r, cr.v, out R, out T, out N);
                        aimRtn = new V3(V3.Dot(s.direction, R), V3.Dot(s.direction, T), V3.Dot(s.direction, N));
                    }
                    return span;
                }
                tick = s.tick;
            }
        }

        // --- pointing -----------------------------------------------------------------
        // The attitude target is always held fixed against the orbit (radial,
        // along-track, normal components): a burn's direction as the guidance
        // holds it through a tick, and during a coast the next burn's
        // direction as it will be where the coast ends, so the vessel arrives
        // pointed.  (World space turns with the body below its inverse-rotation
        // altitude; components against the orbit do not care.)

        /// <summary>The thrust axis in the planner's frame: where the vessel
        /// holds it if it is on its target, else where it points now.</summary>
        V3 AxisPlanner(Vessel v, V3 R, V3 T, V3 N)
        {
            if (holding && Attitude.settled) return R * holdVec.x + T * holdVec.y + N * holdVec.z;
            Vector3d wR, wT, wN;
            GameSystem.WorldRtn(v, out wR, out wT, out wN);
            Vector3d a = engines.AxisWorld();
            return R * Vector3d.Dot(a, wR) + T * Vector3d.Dot(a, wT) + N * Vector3d.Dot(a, wN);
        }

        /// <summary>The thrust axis as this step's attitude leaves it, against
        /// the orbit: the held target exactly while the wheels hold it, else
        /// where the vessel points.</summary>
        V3 AxisNow(Vessel v)
        {
            if (holding && Attitude.settled) return holdVec;
            Vector3d wR, wT, wN;
            GameSystem.WorldRtn(v, out wR, out wT, out wN);
            Vector3d a = engines.AxisWorld();
            return new V3(Vector3d.Dot(a, wR), Vector3d.Dot(a, wT), Vector3d.Dot(a, wN)).Unit;
        }

        void HoldRtn(V3 rtn)
        {
            rtn = rtn.Unit;
            if (!holding || Py.Degrees(Math.Acos(Py.Clamp(V3.Dot(holdVec, rtn), -1, 1))) > 0.05)
                Attitude.Unsettle();
            holdVec = rtn;
            holding = true;
        }

        /// <summary>The attitude target for the coming steps, in world space.</summary>
        void Point(Vessel v)
        {
            if (!holding) { Attitude.target = null; return; }
            V3 rtn = holdVec;
            Attitude.target = () =>
            {
                Vector3d wR, wT, wN;
                GameSystem.WorldRtn(vessel, out wR, out wT, out wN);
                return (wR * rtn.x + wT * rtn.y + wN * rtn.z).normalized;
            };
        }

        // --- the warp rate ------------------------------------------------------------------
        /// <summary>The rate for the coming steps: the highest within the
        /// frame budget (stepped one index at a time on its average), within
        /// the propellant one step may draw, a few seconds a step while the
        /// vessel turns, the altitude limits, and MAX_WARP_INDEX.</summary>
        void Govern(Vessel v, double ms)
        {
            float[] rates = Warp.Rates;
            int top = rates.Length - 1;
            if (cfg.MAX_WARP_INDEX >= 0) top = Math.Min(top, cfg.MAX_WARP_INDEX);
            if (PanelState.maxWarp >= 1) top = Math.Min(top, PanelState.maxWarp);
            if (ms > 0) emaMs = emaMs == 0 ? ms : 0.8 * emaMs + 0.2 * ms;
            int now = TimeWarp.CurrentRateIndex;
            if (emaMs > cfg.FRAME_BUDGET_MS) cpuIndex = Math.Max(1, Math.Min(cpuIndex, now) - 1);
            else if (emaMs < cfg.FRAME_BUDGET_MS / 2) cpuIndex = Math.Min(top, Math.Max(cpuIndex, now) + 1);
            double span = double.PositiveInfinity;
            if (!Attitude.settled) span = 5.0;
            if (cur != null)
            {
                double fuel = engines.SpanFor(cfg.STEP_RESOURCE_SHARE);
                if (cur.coast) span = Math.Min(span, Math.Max(0, cur.tEnd - tSim) + fuel);
                else span = Math.Min(span, fuel / Math.Max(cur.thr, 1e-3));
            }
            int idx = Math.Min(cpuIndex, top);
            while (idx > 1 && Time.fixedDeltaTime * rates[idx] > span) idx--;
            idx = Warp.Cap(v, idx);
            if (idx >= 1) Warp.Set(idx);
        }

        // --- the end -------------------------------------------------------------------------
        void LegEnded(MissionResult res)
        {
            if (res.done && leg + 1 < legs.Count)
            {
                leg++;
                log.Event(res.t, "leg done: " + res.reason + "; next leg to " + legs[leg].name);
                cur = null;
                holding = false;
                Attitude.target = null;
                StartLeg();
                return;
            }
            Conic c = res.final;
            string why = (res.done ? "ARRIVED: " : "FAILED: ") + res.reason;
            log.Event(res.t, string.Format(IC, "{0}; {1} ap {2:F1} km pe {3:F1} km i {4:F2} deg; {5:F1} m/s, {6:F1} days",
                                          why, res.body.name, (c.ra - res.body.radius) / 1e3,
                                          (c.rp - res.body.radius) / 1e3,
                                          Py.Degrees(LowThrust.Orbit.Inclination(c.rvec, c.vvec)), mis.dv,
                                          (res.t - t0) / 21600));
            Stop(why, true);
        }

        void Stop(string reason, bool ended)
        {
            double ut = Planetarium.GetUniversalTime();
            if (log != null)
            {
                foreach (string line in Report()) log.Event(ut, line);
                log.Event(ut, "shutdown: " + reason);
                log.Dispose();
                log = null;
            }
            Attitude.target = null;
            holding = false;
            cur = null;
            pending = null;
            mode = Mode.Idle;
            result = reason;
            flights++;
            Warp.Restore();
            try { if (FlightGlobals.ActiveVessel != null && TimeWarp.CurrentRateIndex > 0) TimeWarp.SetRate(0, true, false); }
            catch (Exception) { }
            var lines = new List<string> { reason };
            if (mis != null)
                lines.Add("dv used\t" + mis.dv.ToString("F0", IC) + " m/s, " + ((ut - t0) / 21600).ToString("F1", IC) + " days");
            if (logPath != "") lines.Add("log\t" + Path.GetFileName(logPath));
            Show(lines.ToArray());
        }

        // --- measures and display ---------------------------------------------------------
        void Note(double ms)
        {
            PhaseStat p = Stat(mis.phase ?? "-");
            p.steps++;
            p.maxStepMs = Math.Max(p.maxStepMs, ms);

        }

        PhaseStat Stat(string phase)
        {
            PhaseStat p;
            if (!stats.TryGetValue(phase, out p)) stats[phase] = p = new PhaseStat();
            return p;
        }

        double lastWall, lastUt;
        void Account(double ms)
        {
            double w = wall.Elapsed.TotalSeconds, ut = Planetarium.GetUniversalTime();
            PhaseStat p = Stat(mis.phase ?? "-");
            if (lastWall > 0) { p.wall += w - lastWall; p.game += ut - lastUt; }
            lastWall = w; lastUt = ut;
            p.frames++;
            p.frameMs += ms;
            p.maxFrameMs = Math.Max(p.maxFrameMs, ms);
        }

        List<string> Report()
        {
            var r = new List<string>();
            double tw = stats.Values.Sum(p => p.wall);
            r.Add(string.Format(IC, "wall per phase (wall s / game s = speed): total {0:F0}s {1}", tw,
                                string.Join(" ", stats.Where(kv => kv.Value.wall > 0).Select(kv => string.Format(IC,
                                    "{0} {1:F0}/{2:F0}={3:F1}x", kv.Key, kv.Value.wall, kv.Value.game,
                                    kv.Value.game / kv.Value.wall)).ToArray())));
            r.Add("autopilot per physics step (mean/max ms, steps; slowest decision ms): " + string.Join(" ",
                stats.Where(kv => kv.Value.frames > 0).Select(kv => string.Format(IC, "{0} {1:F2}/{2:F1} {3:F0}; {4:F1}",
                    kv.Key, kv.Value.frameMs / kv.Value.frames, kv.Value.maxFrameMs, kv.Value.frames,
                    kv.Value.maxStepMs)).ToArray()));
            r.Add(string.Format(IC, "thrust commands: {0} fired, {1} not aligned; {2} substeps short of propellant "
                                    + "(mean thrust share {3:F3}); wheels drew {4:F1} EC ({5:F1} holding; {6} holds "
                                    + "the wheels could not give)", fired, unfired, starved,
                                shareN > 0 ? shareSum / shareN : 1.0, Attitude.ecUsed, Attitude.ecHold,
                                Attitude.holdsLost));
            r.Add(string.Format(IC, "steering: {0:F1} m/s thrust, {1:F2} m/s of it lost to the axis off the command "
                                    + "({2:F3}%); {3:F1} m/s flown more than {4:F1} deg off; worst {5:F2} deg",
                                steerDv, steerLoss, steerDv > 0 ? 100 * steerLoss / steerDv : 0.0, steerOffDv,
                                cfg.STEER_OFF_DEG, steerMaxDeg));
            return r;
        }

        void Telemetry(Craft cr, Command c)
        {
            Conic k = LowThrust.Orbit.Conic(cr.r, cr.v, cr.body.mu);
            Func<double, double> alt = x => (x - cr.body.radius) / 1e3;
            string ap = k.e < 1 ? alt(k.ra).ToString("F0", IC) : "hyp";
            log.Telemetry(cr.t, string.Format(IC,
                "{0,-8} {1,-7} ap={2}k pe={3:F0}k i={4:F2} nu={5:F0} thr={6:F2}/{7:F2} err={8:F1} tick={9:F0}s "
                + "warp={10:F0}x m={11:F0}kg dv={12:F1} step={13:F2}ms",
                mis.phase, cr.body.name, ap, alt(k.rp), Py.Degrees(LowThrust.Orbit.Inclination(cr.r, cr.v)),
                Py.Degrees(k.nu), c.fired ? c.thr : 0.0, c.thr, c.err, c.tick, TimeWarp.CurrentRate, cr.m, mis.dv,
                emaMs));
            if (mis.phase != lastPhase) lastPhase = mis.phase;
        }

        void Shown(Vessel v, double ut)
        {
            if (Time.realtimeSinceStartup - lastShown < 0.5) return;
            lastShown = Time.realtimeSinceStartup;
            Craft cr = mis.craft;
            Conic k = LowThrust.Orbit.Conic(cr.r, cr.v, cr.body.mu);
            Func<double, string> km = x => ((x - cr.body.radius) / 1e3).ToString("F0", IC);
            double speed = 0;
            PhaseStat p;
            if (stats.TryGetValue(mis.phase ?? "-", out p) && p.wall > 0) speed = p.game / p.wall;
            Show("phase\t" + mis.phase, "body\t" + cr.body.name,
                 "ap / pe\t" + (k.e < 1 ? km(k.ra) : "hyp") + " / " + km(k.rp) + " km",
                 "inclination\t" + Py.Degrees(LowThrust.Orbit.Inclination(cr.r, cr.v)).ToString("F2", IC) + " deg",
                 "throttle\t" + (cur != null && cur.fired ? cur.thr : 0).ToString("F2", IC)
                 + (cur != null && !cur.coast && !cur.fired ? " (turning)" : ""),
                 "pointing\t" + (Attitude.lastError < 0 ? "-" : Attitude.lastError.ToString("F1", IC) + " deg off"),
                 "warp\t" + TimeWarp.CurrentRate.ToString("F0", IC) + "x, " + speed.ToString("F0", IC) + "x this phase",
                 "autopilot\t" + emaMs.ToString("F1", IC) + " ms a step",
                 "dv used\t" + mis.dv.ToString("F0", IC) + " m/s" + (plan != null ? " (plan "
                     + plan.dv.ToString("F0", IC) + (dvMax.HasValue ? ", budget " + dvMax.Value.ToString("F0", IC) : "")
                     + ")" : "") + (dvMax.HasValue && mis.dv > dvMax.Value ? "  OVER BUDGET" : ""),
                 "days\t" + ((ut - t0) / 21600).ToString("F1", IC) + (plan != null ? " (plan " + plan.days.ToString("F1", IC)
                     + (daysMax.HasValue ? ", budget " + daysMax.Value.ToString("F1", IC) : "") + ")" : ""),
                 "UT\t" + Mission.FmtUt(ut));
        }

        // --- Persistent Thrust -----------------------------------------------------------
        static void QuietPT(Vessel v)
        {
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                {
                    if (m.GetType().Name != "PersistentEngine") continue;
                    if (ptThrottle == null)
                    {
                        const BindingFlags ALL = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
                        ptThrottle = m.GetType().GetField("persistentThrottle", ALL);
                        ptSet = m.GetType().GetMethod("SetThrottleAfterKey", ALL);
                    }
                    if (ptThrottle != null && ptSet != null && (float)ptThrottle.GetValue(m) != 0f)
                        ptSet.Invoke(m, new object[] { 0f, false });
                }
        }
    }
}
