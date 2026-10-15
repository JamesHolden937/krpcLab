// Delta-v against time.  The guidance burns in arcs round each apsis whose
// width is set by the cutoff eta (Config.ETA_DEFAULT): narrow arcs are many
// near-impulsive burns, cheap and slow; wide arcs finish sooner for more dv.
// How much sooner and how much dearer is not smooth -- waiting for a moon or
// a window to line up adds time in lumps, and a narrower arc is sometimes
// both slower and dearer -- so the trade is measured, not assumed: the whole
// mission is flown in the stand-in at PLAN_GRID cutoffs (in parallel), and
// the options that no other beats on both dv and time are the frontier.
//
// A budget picks from it: the fastest option within a dv limit, the
// cheapest within a time limit, both if both are given; with none, the
// fastest within BALANCE_DV_SHARE of the cheapest.  A budget nothing meets is
// refused, saying what is possible.  Flights that pass a limit stop there
// (Mission.dvLimit/timeLimit), so an impossible budget is known quickly.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;

namespace LowThrust
{
    public class TradeOption
    {
        public double eta, dv, days;
        public double share = 1.0;           // TRANSFER_TIME_SHARE
        public bool done;
        public string reason;
        public MissionResult result;
    }

    public static class Tradeoff
    {
        static readonly CultureInfo IC = CultureInfo.InvariantCulture;
        public const double DAY = 21600.0;

        /// <summary>The cutoffs flown: PLAN_GRID of them from ETA_MAX to
        /// ETA_MIN, even in arc half-width (where the cost lives).</summary>
        public static double[] Grid(Config cfg)
        {
            int n = Math.Max(2, cfg.PLAN_GRID);
            double lo = Math.Acos(cfg.ETA_MAX), hi = Math.Acos(cfg.ETA_MIN);
            var g = new double[n];
            for (int k = 0; k < n; k++) g[k] = Math.Cos(lo + (hi - lo) * k / (n - 1));
            return g;
        }

        /// <summary>The mission flown at every cutoff, in parallel; flights
        /// past dvLimit / timeLimit (s) stop there.  systemFor(i) gives each
        /// thread its own copy of the system only if the caller needs one (the
        /// bodies are read-only here, so one is shared by default).</summary>
        public static List<TradeOption> Scan(Dictionary<string, Body> system, Ship ship, Body body, V3 r, V3 v,
                                             double t0, Body dest, Target target, Config cfg,
                                             double dvLimit = double.PositiveInfinity,
                                             double timeLimit = double.PositiveInfinity)
        {
            double[] etas = Grid(cfg);
            // a route to another planet: every cutoff at every transfer share
            bool planet = dest != body && dest.parent != body && dest != body.parent && body.parent != null;
            double[] shares = planet ? cfg.TRANSFER_SHARES : new[] { 1.0 };
            var grid = new List<Tuple<double, double>>();
            foreach (double sh in shares)
                foreach (double e in etas) grid.Add(Tuple.Create(e, sh));
            var opts = new TradeOption[grid.Count];
            int next = -1;
            ThreadStart work = () =>
            {
                while (true)
                {
                    int i = Interlocked.Increment(ref next);
                    if (i >= grid.Count) return;
                    Config c = cfg.Clone();
                    c.ETA_DEFAULT = grid[i].Item1;
                    c.TRANSFER_TIME_SHARE = grid[i].Item2;
                    var o = new TradeOption { eta = grid[i].Item1, share = grid[i].Item2 };
                    try
                    {
                        MissionResult res = Mission.Fly(system, new Ship(ship.massKg, ship.thrustN, ship.ispS) { cdA = ship.cdA }, body,
                                                        r, v, t0, dest, target.Copy(), c, true, dvLimit, timeLimit);
                        o.done = res.done;
                        o.reason = res.reason;
                        o.dv = res.dv;
                        o.days = (res.t - t0) / DAY;
                        o.result = res;
                    }
                    catch (Exception e) { o.reason = e.Message; }
                    opts[i] = o;
                }
            };
            int nThreads = Math.Max(1, Math.Min(grid.Count, Environment.ProcessorCount - 1));
            var threads = new List<Thread>();
            for (int k = 0; k < nThreads; k++)
            {
                var th = new Thread(work) { IsBackground = true };
                th.Start();
                threads.Add(th);
            }
            foreach (Thread th in threads) th.Join();
            return opts.ToList();
        }

        /// <summary>The options no other beats on both dv and time, fastest
        /// first.</summary>
        public static List<TradeOption> Frontier(IEnumerable<TradeOption> opts)
        {
            var done = opts.Where(o => o != null && o.done).OrderBy(o => o.days).ThenBy(o => o.dv).ToList();
            var front = new List<TradeOption>();
            foreach (TradeOption o in done)
                if (front.Count == 0 || o.dv < front[front.Count - 1].dv - 1e-6) front.Add(o);
            return front;
        }

        /// <summary>At most n of the frontier (fastest first), evenly spread
        /// along it, its two ends and keep always among them.</summary>
        public static List<TradeOption> Thin(List<TradeOption> front, int n, TradeOption keep)
        {
            if (front.Count <= n) return front;
            var idx = new SortedSet<int>();
            for (int k = 0; k < n; k++) idx.Add((int)Math.Round((front.Count - 1) * (double)k / (n - 1)));
            int at = keep == null ? -1 : front.IndexOf(keep);
            if (at >= 0 && !idx.Contains(at))
            {
                // the pick replaces its nearest neighbour that is not an end
                int near = idx.Where(i => i != 0 && i != front.Count - 1).OrderBy(i => Math.Abs(i - at)).First();
                idx.Remove(near);
                idx.Add(at);
            }
            return idx.Select(i => front[i]).ToList();
        }

        /// <summary>The option a budget picks (dvMax m/s, daysMax days; null
        /// is no limit), or null and why not.</summary>
        public static TradeOption Choose(List<TradeOption> scanned, double? dvMax, double? daysMax, Config cfg,
                                         out string why)
        {
            var front = Frontier(scanned);
            why = "";
            if (front.Count == 0)
            {
                var over = scanned.Where(o => o != null && !o.done).ToList();
                why = over.Count > 0 && over.All(o => o.reason != null && o.reason.StartsWith("over the"))
                    ? Impossible(scanned, dvMax, daysMax)
                    : "no cutoff arrives: " + string.Join("; ", scanned.Where(o => o != null).Select(o => o.reason)
                                                                    .Distinct().ToArray());
                return null;
            }
            var ok = front.Where(o => (!dvMax.HasValue || o.dv <= dvMax.Value)
                                      && (!daysMax.HasValue || o.days <= daysMax.Value)).ToList();
            if (ok.Count == 0) { why = Impossible(scanned, dvMax, daysMax); return null; }
            if (dvMax.HasValue && !daysMax.HasValue) return ok.OrderBy(o => o.days).First();
            if (daysMax.HasValue) return ok.OrderBy(o => o.dv).First();
            double cheapest = ok.Min(o => o.dv);
            return ok.Where(o => o.dv <= cheapest * (1 + cfg.BALANCE_DV_SHARE)).OrderBy(o => o.days).First();
        }

        static string Impossible(List<TradeOption> scanned, double? dvMax, double? daysMax)
        {
            var front = Frontier(scanned);
            var parts = new List<string>();
            if (front.Count > 0)
            {
                TradeOption cheap = front.OrderBy(o => o.dv).First(), fast = front.First();
                parts.Add(string.Format(IC, "the cheapest arrives for {0:F0} m/s in {1:F1} days, the fastest in "
                                            + "{2:F1} days for {3:F0} m/s", cheap.dv, cheap.days, fast.days, fast.dv));
            }
            else
            {
                // every flight hit a limit: say how far each got
                var over = scanned.Where(o => o != null).ToList();
                if (dvMax.HasValue && over.Any(o => o.reason == "over the dv budget"))
                    parts.Add(string.Format(IC, "every plan spends more than {0:F0} m/s", dvMax.Value));
                if (daysMax.HasValue && over.Any(o => o.reason == "over the time budget"))
                    parts.Add(string.Format(IC, "every plan takes longer than {0:F1} days", daysMax.Value));
            }
            string ask = (dvMax.HasValue ? string.Format(IC, "{0:F0} m/s", dvMax.Value) : "")
                         + (dvMax.HasValue && daysMax.HasValue ? " and " : "")
                         + (daysMax.HasValue ? string.Format(IC, "{0:F1} days", daysMax.Value) : "");
            return "impossible: no plan within " + ask + (parts.Count > 0 ? "; " + string.Join("; ", parts.ToArray()) : "");
        }

        /// <summary>The impulsive estimate of a transfer from (r, v) about
        /// here to target about dest: Hohmann legs, departure at the
        /// periapsis, capture straight into the target ellipse at its
        /// periapsis, plane changes free.  Low thrust only adds to it (the
        /// trade-off refuses a dv budget below IMPULSIVE_MARGIN of it at once);
        /// 0 where there is no simple route.</summary>
        public static double Impulsive(Body here, V3 r, V3 v, Body dest, Target target, Config cfg)
        {
            Conic c = Orbit.Conic(r, v, here.mu);
            double floor = dest.radius + dest.atmosphere + cfg.SAFE_ALT_M;
            double raT = target.ra ?? target.rp ?? floor, rpT = target.rp ?? target.ra ?? floor;
            if (dest == here)
                return c.e < 1 ? Orbit.ImpulsiveDv(here.mu, c.ra, c.rp, target.ra ?? c.ra, target.rp ?? c.rp) : 0.0;
            double vp = c.e < 1 ? Math.Sqrt(here.mu * (2 / c.rp - 1 / c.a)) : 0.0;
            Func<double, double, double> capture = (mu, vinf) =>
                Math.Max(0.0, Math.Sqrt(vinf * vinf + 2 * mu / rpT) - Math.Sqrt(mu * (2 / rpT - 2 / (rpT + raT))));
            if (dest.parent == here)
            {
                if (c.e >= 1) return 0.0;
                double dep = Math.Max(0.0, Math.Sqrt(here.mu * (2 / c.rp - 2 / (c.rp + dest.sma))) - vp);
                double vinf = Transfer.MoonArrivalVinf(here.mu, c.rp, dest.sma, Math.Sqrt(here.mu / dest.sma));
                return dep + capture(dest.mu, vinf);
            }
            if (here.parent == null || c.e >= 1) return 0.0;
            Body planet = dest.parent == here.parent ? dest
                : (dest.parent != null && dest.parent.parent == here.parent ? dest.parent : null);
            if (planet == null) return 0.0;
            Body star = here.parent;
            double at = (here.sma + planet.sma) / 2;
            double vDep = Math.Abs(Math.Sqrt(star.mu * (2 / here.sma - 1 / at)) - Math.Sqrt(star.mu / here.sma));
            double vArr = Math.Abs(Math.Sqrt(star.mu / planet.sma) - Math.Sqrt(star.mu * (2 / planet.sma - 1 / at)));
            double escape = Math.Max(0.0, Math.Sqrt(vDep * vDep + 2 * here.mu / c.rp) - vp);
            return planet == dest ? escape + capture(dest.mu, vArr) : escape;
        }

        /// <summary>The frontier as lines: days, dv, cutoff; the chosen one
        /// marked.</summary>
        public static List<string> Table(List<TradeOption> scanned, TradeOption chosen)
        {
            return Frontier(scanned).Select(o => string.Format(IC, "{0} {1,7:F1} d  {2,6:F0} m/s  (arcs {3:F3}{4})",
                                                                o == chosen ? ">" : " ", o.days, o.dv, o.eta,
                                                                o.share < 1 ? string.Format(IC, ", transfer {0:F2}", o.share) : ""))
                                     .ToList();
        }
    }
}
