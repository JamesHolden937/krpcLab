// Offline checks of the solver (no KSP): the two-body maths, the within-SOI
// guidance and its planner, the transfer geometry, and whole missions in the
// stand-in solar system.  Run: lowThrust/mod/build.sh test (or
// mono lowThrust/mod/bin/tests.exe [name-filter]).

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;

namespace LowThrust.Tests
{
    [AttributeUsage(AttributeTargets.Method)]
    public class TestAttribute : Attribute { }

    public class Fail : Exception { public Fail(string m) : base(m) { } }

    public static class A
    {
        public static void True(bool c, string why = "expected true") { if (!c) throw new Fail(why); }
        public static void Near(double got, double want, double delta, string what = "")
        {
            if (!(Math.Abs(got - want) <= delta))
                throw new Fail(string.Format("{0} {1} not within {2} of {3}", what, got, delta, want));
        }
        public static void Less(double a, double b, string what = "")
        {
            if (!(a < b)) throw new Fail(string.Format("{0} {1} not < {2}", what, a, b));
        }
    }

    public static class Runner
    {
        public static int Main(string[] args)
        {
            string filter = args.Length > 0 ? args[0] : "";
            int pass = 0, fail = 0;
            var watch = Stopwatch.StartNew();
            foreach (Type t in typeof(Runner).Assembly.GetTypes().OrderBy(x => x.Name))
                foreach (MethodInfo m in t.GetMethods(BindingFlags.Public | BindingFlags.Static))
                {
                    if (m.GetCustomAttributes(typeof(TestAttribute), false).Length == 0) continue;
                    string name = t.Name + "." + m.Name;
                    if (!name.Contains(filter)) continue;
                    var w = Stopwatch.StartNew();
                    try
                    {
                        m.Invoke(null, null);
                        pass++;
                        Console.WriteLine("ok    {0,-60} {1,6:F1} s", name, w.Elapsed.TotalSeconds);
                    }
                    catch (TargetInvocationException e)
                    {
                        fail++;
                        Exception x = e.InnerException;
                        Console.WriteLine("FAIL  {0,-60} {1}", name, x is Fail ? x.Message : x.ToString());
                    }
                }
            Console.WriteLine("{0} passed, {1} failed, {2:F1} s", pass, fail, watch.Elapsed.TotalSeconds);
            return fail == 0 ? 0 : 1;
        }
    }

    public static class OrbitTests
    {
        static readonly double MU = Sim.KERBIN.mu, R0 = Sim.KERBIN.radius;

        static void Ellipse(double ap, double pe, out V3 r, out V3 v, double i = 0, double lan = 0, double w = 0)
        {
            Sim.StateAtPeriapsis(Sim.KERBIN, ap, pe, i, lan, w, out r, out v);
        }

        [Test] public static void ElementsOfAnEllipse()
        {
            V3 r, v;
            Ellipse(2000e3, 100e3, out r, out v);
            Elements el = Orbit.Elements(r, v, MU);
            A.Near(el.ra, R0 + 2000e3, 1e-3);
            A.Near(el.rp, R0 + 100e3, 1e-3);
            A.Near(el.nu, 0.0, 1e-9);
        }

        [Test] public static void UnboundIsNull()
        {
            var r = new V3(R0 + 100e3, 0, 0);
            var v = new V3(0, 1.01 * Math.Sqrt(2 * MU / r.x), 0);
            A.True(Orbit.Elements(r, v, MU) == null);
        }

        [Test] public static void CoastIsExact()
        {
            V3 r, v, r1, v1, r2, v2;
            Ellipse(2000e3, 100e3, out r, out v);
            Elements el = Orbit.Elements(r, v, MU);
            Orbit.Coast(r, v, MU, el.period, out r1, out v1);
            A.Less((r1 - r).Norm, 1e-3);
            Orbit.Coast(r, v, MU, el.period / 2, out r2, out v2);
            A.Near(r2.Norm, el.ra, 1e-3);
        }

        [Test] public static void TimeTo()
        {
            V3 r, v, r2, v2;
            Ellipse(2000e3, 100e3, out r, out v);
            Elements el = Orbit.Elements(r, v, MU);
            A.Near(Orbit.TimeTo(el, Math.PI, MU), el.period / 2, 1e-6);
            Orbit.Coast(r, v, MU, 1000.0, out r2, out v2);
            A.Near(Orbit.TimeTo(Orbit.Elements(r2, v2, MU), 0.0, MU), el.period - 1000.0, 1e-3);
        }

        [Test] public static void RadiusTowardTheApsides()
        {
            V3 r, v;
            Ellipse(2000e3, 100e3, out r, out v);
            Elements el = Orbit.Elements(r, v, MU);
            V3 pe = el.evec.Unit;
            A.Near(Orbit.RadiusToward(el.p, el.evec, pe), el.rp, 1e-3);
            A.Near(Orbit.RadiusToward(el.p, el.evec, pe * -1.0), el.ra, 1e-3);
        }

        [Test] public static void ImpulsiveDvIsHohmann()
        {
            double r1 = R0 + 100e3, r2 = R0 + 500e3, at = (r1 + r2) / 2;
            double hohmann = Math.Sqrt(MU / r1) * (Math.Sqrt(r2 / at) - 1) + Math.Sqrt(MU / r2) * (1 - Math.Sqrt(r1 / at));
            A.Near(Orbit.ImpulsiveDv(MU, r1, r1, r2, r2), hohmann, 1e-6);
            A.Near(Orbit.ImpulsiveDv(MU, r2, r2, r1, r1), hohmann, 1e-6);
        }

        [Test] public static void PropagateMatchesTheEllipticCoast()
        {
            V3 r, v;
            Ellipse(2000e3, 100e3, out r, out v, 0.3, 1.0, 0.5);
            foreach (double dt in new[] { 100.0, 3000.0, 20000.0 })
            {
                V3 a, av, b, bv;
                Orbit.Coast(r, v, MU, dt, out a, out av);
                Orbit.Propagate(r, v, MU, dt, out b, out bv);
                A.Less((a - b).Norm, 1e-3);
            }
        }

        [Test] public static void HyperbolaRoundTripAndPeriapsisTime()
        {
            var r = new V3(700e3, 0, 0);
            var v = new V3(0, 3500.0, 0);
            V3 r2, v2, r3, v3, r4, v4;
            Orbit.Propagate(r, v, MU, 50000.0, out r2, out v2);
            Orbit.Propagate(r2, v2, MU, -50000.0, out r3, out v3);
            A.Less((r3 - r).Norm, 1e-2);
            Conic c = Orbit.Conic(r2, v2, MU);
            A.True(c.e > 1);
            A.Near(Orbit.TimeToPeriapsis(c, MU), -50000.0, 1e-3);
            Orbit.Propagate(r, v, MU, -20000.0, out r4, out v4);
            A.Near(Orbit.TimeToPeriapsis(Orbit.Conic(r4, v4, MU), MU), 20000.0, 1e-3);
        }

        [Test] public static void ShortCoastFarOutOnAHyperbola()
        {
            double mu = 1.686e11, rc = 280e3;
            double vp = Math.Sqrt(3497.0 * 3497.0 + 2 * mu / rc);
            V3 r, v;
            Orbit.Propagate(new V3(rc, 0, 0), new V3(0, vp, 0), mu, -20000.0, out r, out v);
            foreach (double dt in new[] { 1.0, 63.0, 600.0, -63.0 })
            {
                V3 r2, v2;
                Orbit.Propagate(r, v, mu, dt, out r2, out v2);
                A.Less(Math.Abs((r2 - r).Norm - 3497 * Math.Abs(dt)), 0.02 * 3497 * Math.Abs(dt));
            }
        }

        [Test] public static void Lambert()
        {
            var r1 = new V3(7e5, 0, 0);
            var cases = new[]
            {
                Tuple.Create(new V3(9e5 * Math.Cos(2.1), 9e5 * Math.Sin(2.1), 0), 2500.0),
                Tuple.Create(new V3(9e5 * Math.Cos(4.4), 9e5 * Math.Sin(4.4), 0), 4000.0),
                Tuple.Create(new V3(0, 9e5, 3e5), 3000.0), Tuple.Create(new V3(0, 5e7, 0), 20000.0)
            };
            foreach (var cs in cases)
            {
                V3 v1, v2, rr, vv;
                A.True(Orbit.Lambert(r1, cs.Item1, cs.Item2, MU, new V3(0, 0, 1), out v1, out v2));
                Orbit.Propagate(r1, v1, MU, cs.Item2, out rr, out vv);
                A.Less((rr - cs.Item1).Norm, 1e-2);
                A.Less((vv - v2).Norm, 1e-6);
            }
        }

        [Test] public static void AnglesRoundTrip()
        {
            var rng = new Random(1);
            double mu = 3.5316e12;
            for (int i = 0; i < 500; i++)
            {
                double a = 7e5 + rng.NextDouble() * (5e7 - 7e5);
                double e = rng.Next(2) == 0 ? 0.0 : rng.NextDouble() * 0.9;
                double inc = rng.Next(2) == 0 ? 0.0 : rng.NextDouble() * Math.PI;
                double[] el = { a, e, inc, rng.NextDouble() * 2 * Math.PI, rng.NextDouble() * 2 * Math.PI,
                                rng.NextDouble() * 2 * Math.PI };
                V3 r, v, r2, v2;
                Orbit.StateFromElements(mu, el[0], el[1], el[2], el[3], el[4], el[5], out r, out v);
                double[] back = Bodies.Angles(r, v, mu);
                Orbit.StateFromElements(mu, back[0], back[1], back[2], back[3], back[4], back[5], out r2, out v2);
                A.Less((r - r2).Norm, 1e-6 * a * 1.8);
                A.Less((v - v2).Norm, 1e-6 * a * 1.8);
            }
        }

        [Test] public static void BodyRailsFromOneState()
        {
            var sys = Bodies.Stock();
            Body duna = sys["Duna"];
            double t = 4.0e6;
            V3 r, v;
            duna.State(t, out r, out v);
            var parent = new Body("Sun2", duna.parent.mu, 1);
            Body copy = Bodies.FromState("x", duna.mu, duna.radius, 0, parent, r, v, t);
            foreach (double dt in new[] { 0.0, 1e5, 3e7 })
                A.Less((duna.Position(t + dt) - copy.Position(t + dt)).Norm, 1.0);
        }
    }

    public static class GuidanceTests
    {
        static readonly SimBody BODY = Sim.KERBIN;
        static readonly Config CFG = new Config();

        /// <summary>Fly start (ap, pe km [, inc, lan, argpe deg]) to target
        /// (ap, pe km [, inc, lan deg]; NaN free).</summary>
        static SimResult Check(double[] start, double[] target, double worst, Ship vessel = null)
        {
            vessel = vessel ?? Sim.IonProbe();
            Func<int, double> sa = i => i < start.Length ? Py.Radians(start[i]) : 0.0;
            V3 r, v;
            Sim.StateAtPeriapsis(BODY, start[0] * 1e3, start[1] * 1e3, sa(2), sa(3), sa(4), out r, out v);
            Func<int, double?> rad = i => i < target.Length && !double.IsNaN(target[i]) ? BODY.radius + target[i] * 1e3 : (double?)null;
            Func<int, double?> ang = i => i < target.Length && !double.IsNaN(target[i]) ? Py.Radians(target[i]) : (double?)null;
            var tgt = new Target(rad(0), rad(1), ang(2), ang(3));
            SimResult res = Sim.Fly(BODY, vessel, r, v, tgt, CFG.ETA_DEFAULT, CFG);
            Elements el0 = Orbit.Elements(r, v, BODY.mu);
            Target locked = Guidance.Lock(tgt, el0, CFG);
            double reference;
            if (!tgt.ra.HasValue && !tgt.rp.HasValue && tgt.inc.HasValue)
                reference = Orbit.PlaneChangeDv(el0, V3.Cross(r, v).Unit, locked.normal.Value);
            else
                reference = Orbit.ImpulsiveDv(BODY.mu, el0.ra, el0.rp, tgt.ra ?? res.el.ra, tgt.rp ?? res.el.rp,
                                              Guidance.PlaneError(el0, locked));
            A.True(res.done, res.reason);
            if (tgt.ra.HasValue) A.Less(Math.Abs(res.el.ra - tgt.ra.Value), CFG.TOLERANCE_M, "ra");
            if (tgt.rp.HasValue) A.Less(Math.Abs(res.el.rp - tgt.rp.Value), CFG.TOLERANCE_M, "rp");
            if (tgt.inc.HasValue)
            {
                double inc, lan;
                Orbit.InclinationLan(res.el.rvec, res.el.vvec, out inc, out lan);
                A.Less(Math.Abs(Py.Degrees(inc - tgt.inc.Value)), 2 * CFG.PLANE_TOLERANCE_DEG, "inc");
                if (tgt.lan.HasValue) A.Less(Math.Abs(Py.Degrees(lan - tgt.lan.Value)), 0.05, "lan");
            }
            A.Less(res.dv / reference - 1, worst, "cost over impulsive");
            return res;
        }

        static readonly double N = double.NaN;

        [Test] public static void RaiseCircular() { Check(new[] { 100.0, 100 }, new[] { 500.0, 500 }, 0.02); }
        [Test] public static void LowerCircular() { Check(new[] { 500.0, 500 }, new[] { 100.0, 100 }, 0.02); }
        [Test] public static void CirculariseAtApoapsis() { Check(new[] { 2000.0, 100 }, new[] { 2000.0, 2000 }, 0.03); }
        [Test] public static void OneApsisFree()
        {
            SimResult res = Check(new[] { 1000.0, 80 }, new[] { N, 300 }, 0.02);
            A.Less(Math.Abs(res.el.ra - (BODY.radius + 1000e3)), 5e3);
        }
        [Test] public static void CrossOver() { Check(new[] { 800.0, 300 }, new[] { 600.0, 200 }, 0.02); }
        [Test] public static void WeakEngine()
        {
            Check(new[] { 100.0, 100 }, new[] { 300.0, 300 }, 0.02, new Ship(5000.0, 200.0, 4200.0));
        }
        [Test] public static void ApsidesBurnsAreProgradeOrRetrogradeOnly()
        {
            V3 r, v;
            Sim.StateAtPeriapsis(BODY, 100e3, 100e3, 0, 0, 0, out r, out v);
            Elements el = Orbit.Elements(r, v, BODY.mu);
            Target tgt = Guidance.Lock(new Target(BODY.radius + 500e3, BODY.radius + 500e3), el, CFG);
            Command cmd = Guidance.Command(el, tgt, 0.4, 10.0, CFG.ETA_DEFAULT, CFG, BODY.mu);
            A.True(cmd.prograde == 1.0 && cmd.normal == 0.0);
            A.True(cmd.throttle > 0);
        }
        [Test] public static void InclinationOnACircularOrbit() { Check(new[] { 500.0, 500 }, new[] { N, N, 10 }, 0.02); }
        [Test] public static void PlaneChangeAtApoapsis() { Check(new[] { 2000.0, 100 }, new[] { N, N, 5 }, 0.01); }
        [Test] public static void NodeLongitude() { Check(new[] { 500.0, 500, 30, 0 }, new[] { N, N, 30, 40 }, 0.02); }
        [Test] public static void RaiseWithPlaneChange() { Check(new[] { 100.0, 100 }, new[] { 500.0, 500, 5 }, 0.03); }
        [Test] public static void LowerWithPlaneChange() { Check(new[] { 3000.0, 3000 }, new[] { 100.0, 100, 20 }, 0.10); }

        [Test] public static void PlannerMeetsADeadlineAndTradesDvForIt()
        {
            V3 r, v;
            Sim.StateAtPeriapsis(BODY, 100e3, 100e3, 0, 0, 0, out r, out v);
            var tgt = new Target(BODY.radius + 300e3, BODY.radius + 300e3);
            var weak = new Ship(5000.0, 200.0, 4200.0);
            Planner.Plan free = Planner.Make(BODY, weak, r, v, tgt, CFG);
            double budget = free.result.time / 3;
            Planner.Plan p = Planner.Make(BODY, weak, r, v, tgt, CFG, budget);
            A.True(p.feasible, p.reason);
            A.True(p.result.time <= budget);
            A.True(p.result.dv > free.result.dv);
        }

        [Test] public static void PlannerAnImpossibleDeadlineSaysSo()
        {
            V3 r, v;
            Sim.StateAtPeriapsis(BODY, 100e3, 100e3, 0, 0, 0, out r, out v);
            var tgt = new Target(BODY.radius + 300e3, BODY.radius + 300e3);
            Planner.Plan p = Planner.Make(BODY, new Ship(5000.0, 200.0, 4200.0), r, v, tgt, CFG, 600.0);
            A.True(!p.feasible);
            A.True(p.result.done);
            A.True(p.result.time > 600.0);
        }
    }

    public static class MissionTests
    {
        static readonly Dictionary<string, Body> SYS = Bodies.Stock();
        static readonly Config CFG = new Config();
        const double DAY = 21600.0;

        static MissionResult Fly(string dest, double altKm, double thrust = 2000.0, double? inc = null)
        {
            Body k = SYS["Kerbin"], d = SYS[dest];
            V3 r, v;
            Orbit.StateFromElements(k.mu, k.radius + 100e3, 0, 0, 0, 0, 0, out r, out v);
            var tgt = new Target(d.radius + altKm * 1e3, d.radius + altKm * 1e3,
                                 inc.HasValue ? Py.Radians(inc.Value) : (double?)null);
            return Mission.Fly(SYS, new Ship(5000.0, thrust, 4200.0), k, r, v, 0.0, d, tgt, CFG);
        }

        static void Check(MissionResult res, string dest, double altKm, double dvMax)
        {
            Body d = SYS[dest];
            A.True(res.done, res.reason);
            A.True(res.body == d, "ended about " + res.body.name);
            A.Less(Math.Abs(res.final.ra - d.radius - altKm * 1e3), CFG.TOLERANCE_M, "ra");
            A.Less(Math.Abs(res.final.rp - d.radius - altKm * 1e3), CFG.TOLERANCE_M, "rp");
            A.Less(res.dv, dvMax, "dv");
        }

        [Test] public static void SoiAndPeriodsAreStock()
        {
            A.Near(SYS["Kerbin"].Soi / 1e3, 84159, 5);
            A.Near(SYS["Mun"].Soi / 1e3, 2430, 2);
            A.Near(SYS["Duna"].Period / DAY, 801.6, 0.5);
        }

        [Test] public static void FirstDunaWindow()
        {
            LaunchWindow w = Transfer.FindWindow(SYS["Kerbin"], SYS["Duna"], 0.0, SYS["Sun"].mu, 300 * DAY);
            A.Near(w.tDep / DAY, 230, 10);
            A.Near(w.vinfDep.Norm, 860, 60);
        }

        [Test] public static void ParkingEllipseLeavesOnTheAsymptote()
        {
            Body k = SYS["Kerbin"];
            var vinf = new V3(600.0, -500.0, 80.0);
            double rp = k.radius + 100e3, ra = k.radius + 7000e3;
            var po = Transfer.ParkingOrientation(k.mu, rp, ra, vinf, new V3(0, 0, 1));
            double nu = Transfer.DepartureNuInf(k.mu, rp, vinf.Norm);
            V3 Q = V3.Cross(po.Item1, po.Item2);
            V3 outDir = po.Item2 * Math.Cos(nu) + Q * Math.Sin(nu);
            A.True(V3.Dot(outDir, vinf.Unit) > 0.9999);
            A.True(po.Item1.z > 0);
        }

        [Test] public static void Routes()
        {
            Body k = SYS["Kerbin"];
            A.True(string.Join(",", Mission.RouteTo(k, SYS["Ike"]).Select(b => b.name)) == "Duna,Ike");
            A.True(string.Join(",", Mission.RouteTo(k, SYS["Mun"]).Select(b => b.name)) == "Mun");
            bool threw = false;
            try { Mission.RouteTo(SYS["Mun"], SYS["Ike"]); } catch (ArgumentException) { threw = true; }
            A.True(threw);
        }

        [Test] public static void WaitingOrbitInTheGapBelowTheMoon()
        {
            Body j = SYS["Jool"], vall = SYS["Vall"];
            double rc = vall.radius + CFG.SAFE_ALT_M;
            double ra, rp, x1, x2;
            bool ok;
            Mission.WaitingOrbit(j, vall, j.radius + j.atmosphere + CFG.SAFE_ALT_M, rc,
                                 CFG.CAPTURE_AP_FRACTION * vall.Soi, 0.04, CFG, true, out ra, out rp, out x1, out x2, out ok);
            A.True(ok);
            A.True(rp > Transfer.Band(SYS["Laythe"], 1.2)[1]);
            A.True(ra < Transfer.Band(vall, 1.2)[0]);
        }

        [Test] public static void AimIsMeasuredAtTheSoiEntry()
        {
            Body k = SYS["Kerbin"], mun = SYS["Mun"];
            double rc = mun.radius + 300e3;
            foreach (double v in new[] { 150.0, 600.0 })
            {
                V3 rm, vm;
                mun.State(0.0, out rm, out vm);
                V3 u = vm.Unit;
                V3 side = V3.Cross(u, mun.Normal).Unit;
                double lo = 0.0, hi = 0.9 * mun.Soi;
                V3 r0 = V3.Zero, v0 = V3.Zero;
                Approach ca = null;
                for (int i = 0; i < 50; i++)
                {
                    double b = (lo + hi) / 2;
                    r0 = rm - u * (3 * mun.Soi) + side * b;
                    v0 = vm + u * v;
                    ca = Transfer.ClosestApproach(r0, v0, 0.0, k.mu, mun, 6 * mun.Soi / v);
                    double s = Transfer.AimError(r0, v0, 0.0, k.mu, mun, ca, rc, mun.Normal).s;
                    if (s > 0) hi = b; else lo = b;
                }
                Approach e = Transfer.SoiEntry(r0, v0, 0.0, k.mu, mun, ca);
                Conic c = Orbit.Conic(e.d, e.vrel, mun.mu);
                A.Near(c.rp / rc, 1.0, 0.02);
            }
        }

        const double MOHO_MU = 1.686e11, MOHO_R = 250e3, MOHO_SOI = 9646663.0;

        static Capture MohoCap()
        {
            return new Capture
            {
                mu = MOHO_MU, raCap = 0.3 * MOHO_SOI, raOk = 0.5 * MOHO_SOI, floor = MOHO_R + 30e3,
                rpMax = 1.5 * (MOHO_R + 50e3), surface = MOHO_R, arc = Py.Radians(60), fill = 0.8, k = 0.8
            };
        }

        [Test] public static void LongCaptureEveryEngineCaptures()
        {
            double rc = MOHO_R + 50e3;
            double impulsive = Transfer.CaptureDv(MOHO_MU, rc, 0.3 * MOHO_SOI, 3497.0);
            var starts = new List<double>();
            foreach (double thrust in new[] { 4000.0, 2000.0, 500.0, 100.0 })
            {
                double[] got = Transfer.CaptureStart(MohoCap(), rc, 3497.0, thrust / 5000.0);
                A.True(got != null, "no capture at " + thrust);
                A.True(got[1] > impulsive);
                A.Less(got[1], 3497.0);
                starts.Add(got[2]);
            }
            A.True(starts[1] > MOHO_SOI, "starts outside the SOI");
            A.True(starts.SequenceEqual(starts.OrderBy(x => x)), "weaker starts earlier");
        }

        [Test] public static void LongCaptureCoastsUntilNeededThenThrottlesSteadily()
        {
            Capture cap = MohoCap();
            double rc = MOHO_R + 50e3;
            double vp = Math.Sqrt(3497.0 * 3497.0 + 2 * MOHO_MU / rc);
            V3 r, v;
            Orbit.Propagate(new V3(rc, 0, 0), new V3(0, vp, 0), MOHO_MU, -30000.0, out r, out v);
            double wait;
            double rho = Transfer.CaptureStep(r, v, Orbit.Conic(r, v, MOHO_MU), cap, 0.4, false, out wait);
            A.True(rho == 0.0);
            A.True(wait > 1000.0);
            Orbit.Propagate(new V3(rc, 0, 0), new V3(0, vp, 0), MOHO_MU, -3000.0, out r, out v);
            rho = Transfer.CaptureStep(r, v, Orbit.Conic(r, v, MOHO_MU), cap, 0.4, true, out wait);
            A.True(rho > 0.5);
        }

        [Test] public static void ToTheMun() { Check(Fly("Mun", 50), "Mun", 50, 1250); }
        [Test] public static void ToMinmusOutOfPlane() { Check(Fly("Minmus", 30), "Minmus", 30, 1300); }
        [Test] public static void ToDunaWithAPlaneChange()
        {
            MissionResult res = Fly("Duna", 200, inc: 10);
            Check(res, "Duna", 200, 2000);
            A.Near(Py.Degrees(Orbit.Inclination(res.final.rvec, res.final.vvec)), 10, 0.05);
        }
        [Test] public static void ToIkeThroughDuna() { Check(Fly("Ike", 50), "Ike", 50, 2100); }
        [Test] public static void ToMohoMatchingInTheCruise()
        {
            MissionResult res = Fly("Moho", 50);
            Check(res, "Moho", 50, 6700);
            A.True(res.events.Any(e => e.text.StartsWith("MATCH")), "no MATCH");
        }
        [Test] public static void WeakEngineToMoho() { Check(Fly("Moho", 50, 200.0), "Moho", 50, 7500); }
        [Test] public static void WeakEngineToTheMun() { Check(Fly("Mun", 50, 200.0), "Mun", 50, 1700); }
        [Test] public static void WeakEngineToVall() { Check(Fly("Vall", 50, 200.0), "Vall", 50, 7500); }
    }

    public static class AeroTests
    {
        static readonly Dictionary<string, Body> SYS = Bodies.Stock();
        static readonly Config CFG = new Config();

        [Test] public static void DensityInterpolatesAndEndsAtTheTop()
        {
            Atmosphere a = SYS["Duna"].air;
            A.Near(a.Density(0), 0.1333, 1e-4);
            A.True(a.Density(30500) < a.Density(30000) && a.Density(30500) > a.Density(31000), "monotone");
            A.True(a.Density(50000) == 0 && a.Density(49999) > 0, "top");
        }

        [Test] public static void PassAboveTheAirShedsNothing()
        {
            Body d = SYS["Duna"];
            double rp = d.radius + 60e3, ra = d.radius + 5000e3;
            PassResult p = Aero.PassAt(d, rp, -d.mu / (ra + rp), new V3(1, 0, 0), new V3(0, 0, 1), 0.0, 1e-4, 1.0);
            A.True(p.exited, "exited");
            A.Less(Math.Abs(p.dE), 1.0, "dE");
        }

        [Test] public static void DeeperSheddsMoreAndHeatsMore()
        {
            Body d = SYS["Duna"];
            double ra = d.radius + 5000e3;
            Func<double, PassResult> at = alt => Aero.PassAt(d, d.radius + alt, -d.mu / (ra + d.radius + alt),
                                                             new V3(1, 0, 0), new V3(0, 0, 1), 0.0, 2.0 / 30000, 1.0);
            PassResult a = at(35e3), b = at(25e3);
            A.True(a.exited && b.exited);
            A.True(b.dE > a.dE && b.peakH > a.peakH && b.load > a.load, "monotone in depth");
            A.True(a.minAlt < 35e3 + 100 && a.minAlt > 34e3, "periapsis where planned: " + a.minAlt);
        }

        [Test] public static void TimeToRadiusMatchesThePropagation()
        {
            Body d = SYS["Duna"];
            V3 r, v;
            Orbit.StateFromElements(d.mu, 2820e3, 0.88, 0.2, 0, 0, 2.5, out r, out v);
            Conic c = Orbit.Conic(r, v, d.mu);
            double R = d.radius + 50e3;
            double t = Aero.TimeToRadius(c, d.mu, R);
            V3 r1, v1;
            Orbit.Propagate(r, v, d.mu, t, out r1, out v1);
            A.Near(r1.Norm, R, 1.0, "radius");
            A.True(V3.Dot(r1, v1) < 0, "inbound");
        }

        [Test] public static void DeepestKeepsTheLimits()
        {
            Body d = SYS["Duna"];
            double ra = d.radius + 3000e3, dpm = 2.0 / 30000;
            var lim = new PassLimits { altMin = 10e3, hMax = 60, loadMax = double.PositiveInfinity, accMax = 20,
                                       raMin = d.radius + 80e3, margin = 1.5 };
            Func<double, double> E = rp => -d.mu / (ra + rp);
            double? rp0 = Aero.Deepest(d, E, new V3(1, 0, 0), new V3(0, 0, 1), 0.0, dpm, lim);
            A.True(rp0.HasValue, "a pass");
            PassResult hi = Aero.PassAt(d, rp0.Value, E(rp0.Value), new V3(1, 0, 0), new V3(0, 0, 1), 0.0, dpm * 1.5, 1.5);
            A.Less(hi.peakH, 60.0001, "heating at the margin");
            A.Near(hi.peakH, 60, 2, "the heating limit binds");
        }

        [Test] public static void CaptureDirectionNeverJumps()
        {
            // the capture law's direction between consecutive ticks of a
            // long capture at Duna (weak engine)
            Body d = SYS["Duna"];
            Capture cap = Mission.CapSpec(d, d.radius + 80e3, 0.3 * d.Soi, 0.3 * d.Soi, CFG);
            double accel = 0.067, vinf = 890;
            double rc = d.radius + 80e3, vp = Math.Sqrt(vinf * vinf + 2 * d.mu / rc);
            V3 r, v;
            Orbit.Propagate(new V3(rc, 0, 0), new V3(0, vp, 0), d.mu, -3 * 3600, out r, out v);
            double? last = null;
            double worst = 0;
            for (int i = 0; i < 4000; i++)
            {
                Conic c = Orbit.Conic(r, v, d.mu);
                if (Transfer.Captured(r, v, c, cap) || V3.Dot(r, v) > 0) break;
                double wait;
                double rho = Transfer.CaptureStep(r, v, c, cap, accel, last.HasValue, out wait);
                if (rho <= 0) { V3 r1, v1; Orbit.Propagate(r, v, d.mu, Math.Min(wait, 60), out r1, out v1); r = r1; v = v1; continue; }
                double h = 60;
                V3 want = Transfer.CaptureCommand(r, v, c, cap, rho * accel * h, h);
                // the law's own turn: its angle off retrograde, tick to tick
                // (retrograde itself turns fast near periapsis)
                double off = Py.Degrees(Math.Acos(Py.Clamp(V3.Dot(want.Unit, v.Unit * -1.0), -1, 1)));
                if (last.HasValue) worst = Math.Max(worst, Math.Abs(off - last.Value));
                last = off;
                Sim.Rk4(ref r, ref v, 1.0, d.mu, want.Unit * Math.Min(accel, want.Norm / h), h);
            }
            A.True(last.HasValue, "burned");
            A.Less(worst, 10.0, "direction step (deg off retrograde, tick to tick)");
        }

        [Test] public static void DunaWithAerobrakingIsCheaper()
        {
            Body k = SYS["Kerbin"], d = SYS["Duna"];
            V3 r, v;
            Orbit.StateFromElements(k.mu, k.radius + 100e3, 0, 0, 0, 0, 0, out r, out v);
            var tgt = new Target(d.radius + 200e3, d.radius + 200e3);
            MissionResult dry = Mission.Fly(SYS, new Ship(5000.0, 2000.0), k, r, v, 0.0, d, tgt, CFG);
            MissionResult air = Mission.Fly(SYS, new Ship(5000.0, 2000.0) { cdA = 1.0 }, k, r, v, 0.0, d, tgt, CFG);
            A.True(dry.done && air.done, dry.reason + " / " + air.reason);
            A.True(air.events.Any(e => e.text.StartsWith("AEROBRAKE")), "no AEROBRAKE");
            A.Less(air.dv, dry.dv - 50, "dv " + air.dv.ToString("F0") + " v " + dry.dv.ToString("F0"));
            A.Near(air.final.rp - d.radius, 200e3, CFG.TOLERANCE_M, "rp");
        }
    }
}
