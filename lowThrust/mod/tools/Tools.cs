// The command-line tools, no KSP: plan and fly a transfer in the stand-in.
//
//   mono lowThrust/mod/bin/lowthrust.exe mission --from "kerbin: ap=100 pe=100" --to "mun: ap=50 pe=50"
//   mono lowThrust/mod/bin/lowthrust.exe mission ... --thrust 200 --mass 5000 --isp 4200 --ut 0
//   mono lowThrust/mod/bin/lowthrust.exe fly --start "ap=100 pe=100" --target "ap=500 pe=500 i=5" --within 2d
//
// (lowThrust/tools/simmission and simfly run these.)  mission: every leg of a
// transfer between bodies, its timeline, burns and totals.  fly: one SOI,
// optionally within a time budget, against the impulsive reference.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace LowThrust
{
    public static class Tools
    {
        static readonly CultureInfo IC = CultureInfo.InvariantCulture;

        static string F(double x, int d) { return x.ToString("F" + d, IC); }
        static string S(double x, int d)
        {
            bool neg = x < 0 || (x == 0 && 1 / x < 0);
            return (neg ? "-" : "+") + F(Math.Abs(x), d);
        }
        static string R(string s, int w) { return s.PadLeft(w); }
        static string L(string s, int w) { return s.PadRight(w); }

        public static int Main(string[] argv)
        {
            if (argv.Length == 0)
            {
                Console.Error.WriteLine("usage: lowthrust.exe mission|fly ...");
                return 2;
            }
            var args = new Args(argv.Skip(1).ToArray());
            try
            {
                if (argv[0] == "mission") return MissionTool(args);
                if (argv[0] == "fly") return FlyTool(args);
            }
            catch (ArgumentException e)
            {
                Console.Error.WriteLine(e.Message);
                return 2;
            }
            Console.Error.WriteLine("unknown tool " + argv[0]);
            return 2;
        }

        class Args
        {
            readonly Dictionary<string, string> one = new Dictionary<string, string>();
            public readonly List<string> sets = new List<string>();
            public readonly HashSet<string> flags = new HashSet<string>();
            public Args(string[] a)
            {
                for (int i = 0; i < a.Length; i++)
                {
                    if (a[i] == "--table" || a[i] == "--frontier") { flags.Add(a[i].Substring(2)); continue; }
                    if (!a[i].StartsWith("--") || i + 1 >= a.Length)
                        throw new ArgumentException("bad argument " + a[i]);
                    if (a[i] == "--set") sets.Add(a[++i]);
                    else one[a[i].Substring(2)] = a[++i];
                }
            }
            public string Get(string k, string d = null) { string v; return one.TryGetValue(k, out v) ? v : d; }
            public double Num(string k, double d)
            {
                string v = Get(k);
                return v == null ? d : double.Parse(v, IC);
            }
        }

        static string Dur(double s)
        {
            double d = Math.Floor(s / 21600); s -= d * 21600;
            double h = Math.Floor(s / 3600); s -= h * 3600;
            double m = Math.Floor(s / 60); s -= m * 60;
            return (d != 0 ? (long)d + "d " : "") + string.Format(IC, "{0}h{1:00}m{2:00}s", (long)h, (long)m, (long)s);
        }

        static int MissionTool(Args a)
        {
            Config cfg = new Config().Apply(a.sets);
            var system = Bodies.Stock();
            string from = a.Get("from"), to = a.Get("to");
            if (from == null || to == null) throw new ArgumentException("--from and --to are required");
            Dictionary<string, double> s, t;
            Body b0 = Mission.ParseOrbit(from, system, out s);
            Body b1 = Mission.ParseOrbit(to, system, out t);
            Func<string, double, double> get = (k, d) => s.ContainsKey(k) ? s[k] : d;
            double sAp = get("ap", get("pe", 100.0)) * 1e3;
            double sPe = get("pe", get("ap", 100.0)) * 1e3;
            double ra = b0.radius + Math.Max(sAp, sPe), rp = b0.radius + Math.Min(sAp, sPe);
            V3 r, v;
            Orbit.StateFromElements(b0.mu, (ra + rp) / 2, (ra - rp) / (ra + rp), Py.Radians(get("i", 0)),
                                    Py.Radians(get("lan", 0)), Py.Radians(get("argpe", 0)), 0.0, out r, out v);
            Target target = Mission.TargetFrom(b1, t);
            Ship probe = Sim.IonProbe();
            double mass = a.Num("mass", probe.massKg), thrust = a.Num("thrust", probe.thrustN);
            double isp = a.Num("isp", probe.ispS), ut = a.Num("ut", 0.0);
            int nBurns = (int)a.Num("burns", 40);
            var vessel = new Ship(mass, thrust, isp);

            Console.WriteLine(cfg.Line());
            Console.WriteLine("vessel " + F(mass, 0) + " kg, " + F(thrust, 0) + " N (" + F(thrust / mass, 3)
                              + " m/s^2, TWR " + F(thrust / mass / 9.81, 4) + "), Isp " + F(isp, 0)
                              + " s (mass constant, propellant unlimited)");
            Console.WriteLine("route: " + b0.name + " -> " + string.Join(" -> ", Mission.RouteTo(b0, b1).Select(x => x.name)));
            MissionResult res;
            string maxDv = a.Get("max-dv"), maxDays = a.Get("max-days");
            if (maxDv != null || maxDays != null || a.flags.Contains("frontier"))
            {
                // the dv/time trade: every cutoff flown, the budget picks one
                double? dvMax = maxDv != null ? double.Parse(maxDv, IC) : (double?)null;
                double? daysMax = maxDays != null ? double.Parse(maxDays, IC) : (double?)null;
                double floorDv = Tradeoff.Impulsive(b0, r, v, b1, target, cfg);
                Console.WriteLine("\nimpulsive estimate " + F(floorDv, 0) + " m/s");
                if (dvMax.HasValue && dvMax.Value < cfg.IMPULSIVE_MARGIN * floorDv)
                {
                    Console.WriteLine("impossible: " + F(dvMax.Value, 0) + " m/s is far below what even ideal impulsive "
                                      + "burns need (~" + F(floorDv, 0) + " m/s)");
                    return 1;
                }
                var watch = System.Diagnostics.Stopwatch.StartNew();
                var scanned = Tradeoff.Scan(system, vessel, b0, r, v, ut, b1, target, cfg,
                                            dvMax.HasValue ? dvMax.Value * cfg.BUDGET_SCAN_FACTOR : Py.Inf,
                                            daysMax.HasValue ? daysMax.Value * Tradeoff.DAY * cfg.BUDGET_SCAN_FACTOR : Py.Inf);
                string why;
                TradeOption pick = Tradeoff.Choose(scanned, dvMax, daysMax, cfg, out why);
                Console.WriteLine("\ntrade-off (" + scanned.Count + " plans in " + F(watch.Elapsed.TotalSeconds, 1)
                                  + " s; the frontier, fastest first):");
                foreach (string line in Tradeoff.Table(scanned, pick)) Console.WriteLine("  " + line);
                if (pick == null)
                {
                    Console.WriteLine("\n" + why);
                    return 1;
                }
                Console.WriteLine("chosen: arcs " + F(pick.eta, 3));
                res = pick.result;
            }
            else res = Mission.Fly(system, vessel, b0, r, v, ut, b1, target, cfg);

            Console.WriteLine("\ntimeline");
            foreach (Event e in res.events) Console.WriteLine("  " + R(Mission.FmtUt(e.t), 16) + "  " + e.text);
            Console.WriteLine("\nburns (" + res.maneuvers.Count + "; direction = prograde/normal/radial share of the dv)");
            foreach (Maneuver m in res.maneuvers.Take(nBurns))
                Console.WriteLine("  " + L(m.phase, 9) + " " + L(m.body, 7) + " " + R(Mission.FmtUt(m.t0), 16) + "  "
                                  + R(Dur(m.t1 - m.t0), 12) + "  " + R(F(m.dv, 2), 8) + " m/s  (" + S(m.prograde, 2)
                                  + " " + S(m.normal, 2) + " " + S(m.radial, 2) + ")");
            if (res.maneuvers.Count > nBurns)
            {
                var rest = res.maneuvers.Skip(nBurns).ToList();
                Console.WriteLine("  ... " + rest.Count + " more, " + F(rest.Sum(m => m.dv), 1) + " m/s");
            }
            var by = new List<KeyValuePair<string, double>>();
            var idx = new Dictionary<string, int>();
            foreach (Maneuver m in res.maneuvers)
            {
                string k = L(m.body, 7) + " " + L(m.phase, 9);
                int i;
                if (!idx.TryGetValue(k, out i)) { idx[k] = by.Count; by.Add(new KeyValuePair<string, double>(k, m.dv)); }
                else by[i] = new KeyValuePair<string, double>(k, by[i].Value + m.dv);
            }
            Console.WriteLine("\ndv by phase");
            foreach (var kv in by) Console.WriteLine("  " + kv.Key + " " + R(F(kv.Value, 1), 8) + " m/s");
            Conic c = res.final;
            Console.WriteLine("\n" + (res.done ? "ARRIVED" : "FAILED") + ": " + res.reason);
            Console.WriteLine("  " + res.body.name + ": ap " + F((c.ra - res.body.radius) / 1e3, 1) + " km  pe "
                              + F((c.rp - res.body.radius) / 1e3, 1) + " km  i "
                              + F(Py.Degrees(Orbit.Inclination(c.rvec, c.vvec)), 2) + " deg");
            Console.WriteLine("  total " + F(res.dv, 1) + " m/s, " + F(res.propUsed, 1) + " kg xenon, "
                              + Dur(res.t - res.t0) + " (" + Mission.FmtUt(res.t0) + " -> " + Mission.FmtUt(res.t) + ")");
            return 0;
        }

        static Dictionary<string, double> ParseFields(string text)
        {
            var f = new Dictionary<string, double>();
            string[] allowed = { "ap", "pe", "i", "lan", "argpe" };
            foreach (string item in text.Replace(",", " ").Split((char[])null, StringSplitOptions.RemoveEmptyEntries))
            {
                int eq = item.IndexOf('=');
                string k = eq < 0 ? item : item.Substring(0, eq);
                if (Array.IndexOf(allowed, k) < 0)
                    throw new ArgumentException("unknown orbit field '" + k + "' (" + string.Join(", ", allowed) + ")");
                double x = double.Parse(item.Substring(eq + 1), IC);
                f[k] = k == "ap" || k == "pe" ? x * 1000 : Py.Radians(x);
            }
            return f;
        }

        static double ParseDuration(string text)
        {
            var units = new Dictionary<char, double> { { 's', 1 }, { 'm', 60 }, { 'h', 3600 }, { 'd', 6 * 3600 } };
            char u = text[text.Length - 1];
            if (units.ContainsKey(u)) return double.Parse(text.Substring(0, text.Length - 1), IC) * units[u];
            return double.Parse(text, IC);
        }

        static int FlyTool(Args a)
        {
            Config cfg = new Config().Apply(a.sets);
            SimBody body = Sim.KERBIN;
            Ship probe = Sim.IonProbe();
            var vessel = new Ship(a.Num("mass", probe.massKg), a.Num("thrust", probe.thrustN), a.Num("isp", probe.ispS));
            int nBurns = (int)a.Num("burns", 8);
            var start = ParseFields(a.Get("start") ?? "");
            var tg = ParseFields(a.Get("target") ?? "");
            Func<Dictionary<string, double>, string, double?> g = (d, k) => d.ContainsKey(k) ? d[k] : (double?)null;
            double sAp = g(start, "ap") ?? g(start, "pe").Value;
            double sPe = g(start, "pe") ?? sAp;
            V3 r, v;
            Sim.StateAtPeriapsis(body, sAp, sPe, g(start, "i") ?? 0.0, g(start, "lan") ?? 0.0, g(start, "argpe") ?? 0.0,
                                 out r, out v);
            var target = new Target(tg.ContainsKey("ap") ? body.radius + tg["ap"] : (double?)null,
                                    tg.ContainsKey("pe") ? body.radius + tg["pe"] : (double?)null,
                                    g(tg, "i"), g(tg, "lan"), argpe: g(tg, "argpe"));
            Elements el0 = Orbit.Elements(r, v, body.mu);
            Target locked = Guidance.Lock(target, el0, cfg);
            Func<double, string> deg = x => F(Py.Degrees(x), 2);
            Func<Dictionary<string, double>, string> desc = d => string.Join(" ", d.Select(kv =>
                kv.Key == "ap" || kv.Key == "pe" ? kv.Key + " " + F(kv.Value / 1e3, 0) + " km" : kv.Key + " " + deg(kv.Value)));
            Console.WriteLine(cfg.Line());
            Console.WriteLine(body.name + ": start " + desc(start) + "  ->  target " + desc(tg));
            Console.WriteLine("vessel " + F(vessel.massKg, 0) + " kg, " + F(vessel.thrustN, 0) + " N, accel "
                              + F(vessel.thrustN / vessel.massKg, 3) + " m/s^2");
            string w = a.Get("within");
            double within = w != null ? ParseDuration(w) : double.NaN;
            Planner.Plan p = Planner.Make(body, vessel, r, v, target, cfg, within);
            Console.WriteLine("budget " + (w != null ? Dur(within) : "none (cheapest)"));
            SimResult res = p.result;
            double ra1 = target.ra ?? res.el.ra, rp1 = target.rp ?? res.el.rp;
            double reference;
            if (!target.ra.HasValue && !target.rp.HasValue)
                reference = locked.normal.HasValue
                    ? Orbit.PlaneChangeDv(el0, V3.Cross(r, v).Unit, locked.normal.Value) : 0.0;
            else
                reference = Orbit.ImpulsiveDv(body.mu, el0.ra, el0.rp, ra1, rp1, Guidance.PlaneError(el0, locked));
            Console.WriteLine((p.feasible ? "PLAN" : "NO PLAN") + ": " + p.reason);
            if (p.table.Count > 0 && a.flags.Contains("table"))
            {
                Console.WriteLine("  cutoffs flown (eta: time, dv):");
                foreach (var row in p.table)
                    Console.WriteLine("    " + F(row[0], 3) + ": " + R(Dur(row[2]), 14) + "  " + R(F(row[3], 1), 7)
                                      + " m/s" + (row[1] > 0 ? "" : "  (did not arrive)"));
            }
            Console.WriteLine("  eta cutoff " + F(p.eta, 3) + "  time " + Dur(res.time) + " (" + F(res.orbits, 1)
                              + " start orbits)  dv " + F(res.dv, 1) + " m/s  xenon " + F(res.propUsed, 2) + " kg");
            Console.WriteLine("  two-burn impulsive " + F(reference, 1) + " m/s  ->  low-thrust costs "
                              + S(reference != 0 ? (res.dv / reference - 1) * 100 : 0, 1) + "%");
            if (res.el != null)
            {
                double inc, lan;
                Orbit.InclinationLan(res.el.rvec, res.el.vvec, out inc, out lan);
                Console.WriteLine("  final ap " + F((res.el.ra - body.radius) / 1e3, 3) + " km  pe "
                                  + F((res.el.rp - body.radius) / 1e3, 3) + " km  i " + deg(inc) + "  lan " + deg(lan)
                                  + "  plane error " + F(Py.Degrees(Guidance.PlaneError(res.el, locked)), 4) + " deg ("
                                  + res.reason + ")");
            }
            Console.WriteLine("  " + res.burns.Count + " burns");
            foreach (Burn b in res.burns.Take(nBurns))
                Console.WriteLine("    t=" + R(Dur(b.t0), 14) + "  " + R(F(b.t1 - b.t0, 0), 7) + " s  " + R(F(b.dv, 2), 6)
                                  + " m/s  nu " + R(F(Py.Degrees(b.nu0), 0), 5) + " -> " + R(F(Py.Degrees(b.nu1), 0), 4) + " deg");
            if (res.burns.Count > nBurns) Console.WriteLine("    ... " + (res.burns.Count - nBurns) + " more");
            return 0;
        }
    }
}
