// The stand-in: a point-mass vessel with a constant-thrust engine around a
// spherical planet -- two-body gravity and the rocket equation, the physics
// KSP uses in vacuum.  The planner's predictions fly it; so do the tools.
//
// Each tick the guidance is asked once and its command held for the tick.
// A coasting tick is propagated exactly (Orbit.Coast); a thrusting one by RK4.

using System;
using System.Collections.Generic;

namespace LowThrust
{
    /// <summary>Constant mass, unlimited propellant: the stand-in plans
    /// paths.  Isp only converts dv to propellant for the report.</summary>
    public class Vessel
    {
        public double massKg, thrustN, ispS;
        public Vessel(double massKg, double thrustN, double ispS = 4200.0)
        {
            this.massKg = massKg; this.thrustN = thrustN; this.ispS = ispS;
        }
    }

    public class SimBody
    {
        public string name;
        public double mu, radius, atmosphere;
        public SimBody(string name, double mu, double radius, double atmosphere)
        {
            this.name = name; this.mu = mu; this.radius = radius; this.atmosphere = atmosphere;
        }
        public SimBody(Body b) : this(b.name, b.mu, b.radius, b.atmosphere) { }
    }

    public class Burn { public double t0, t1, dv, nu0, nu1; }

    public class SimResult
    {
        public bool done;
        public string reason;
        public double time, dv, propUsed, orbits, endMass;
        public List<Burn> burns;
        public Elements el;
        public V3 endR, endV;
    }

    public static class Sim
    {
        public const double G0 = 9.80665;

        public static readonly SimBody KERBIN = new SimBody("Kerbin", 3.5316e12, 600e3, 70e3);
        public static Vessel IonProbe() { return new Vessel(5000.0, 2000.0, 4200.0); }

        /// <summary>Propellant dv would take from the vessel's mass.</summary>
        public static double XenonKg(Vessel vessel, double dv)
        {
            return vessel.massKg * (1 - Math.Exp(-dv / (vessel.ispS * G0)));
        }

        /// <summary>(r, v) at periapsis of the orbit with these altitudes (m)
        /// and angles (rad), in the body's frame.</summary>
        public static void StateAtPeriapsis(SimBody body, double apAlt, double peAlt, double inc, double lan,
                                            double argpe, out V3 r, out V3 v)
        {
            double ra = body.radius + apAlt, rp = body.radius + peAlt;
            if (ra < rp) { double t = ra; ra = rp; rp = t; }
            r = Orbit.Rot(new V3(rp, 0.0, 0.0), inc, lan, argpe);
            v = Orbit.Rot(new V3(0.0, Orbit.SpeedAt(body.mu, rp, ra, rp), 0.0), inc, lan, argpe);
        }

        /// <summary>One RK4 step of gravity plus thrust force (inertial
        /// vector, N) on mass m.</summary>
        public static void Rk4(ref V3 r, ref V3 v, double m, double mu, V3 force, double dt)
        {
            V3 k1v = v, k1a = Acc(r, m, mu, force);
            V3 r2 = r + k1v * (dt / 2), v2 = v + k1a * (dt / 2);
            V3 k2v = v2, k2a = Acc(r2, m, mu, force);
            V3 r3 = r + k2v * (dt / 2), v3 = v + k2a * (dt / 2);
            V3 k3v = v3, k3a = Acc(r3, m, mu, force);
            V3 r4 = r + k3v * dt, v4 = v + k3a * dt;
            V3 k4v = v4, k4a = Acc(r4, m, mu, force);
            r = r + ((k1v + k2v * 2) + (k3v * 2 + k4v)) * (dt / 6);
            v = v + ((k1a + k2a * 2) + (k3a * 2 + k4a)) * (dt / 6);
        }

        static V3 Acc(V3 r, double m, double mu, V3 force)
        {
            double rm = r.Norm;
            V3 g = r * (-mu / Math.Pow(rm, 3));
            return g + force * (1 / m);
        }

        /// <summary>How long to hold a command: a fixed step in angle, not
        /// time (r^2/h per radian), capped at period / TICKS_PER_ORBIT.</summary>
        public static double TickFor(Elements el, Config cfg)
        {
            double step = 2 * Math.PI / cfg.TICKS_PER_ORBIT;
            return Math.Min(el.period / cfg.TICKS_PER_ORBIT, el.r * el.r / el.h * step);
        }

        /// <summary>Fly the transfer to target from (r, v) with cutoff
        /// etaCut, within one SOI.</summary>
        public static SimResult Fly(SimBody body, Vessel vessel, V3 r, V3 v, Target target, double etaCut,
                                    Config cfg, bool record = true)
        {
            double mu = body.mu, m = vessel.massKg, t = 0.0, dv = 0.0;
            var burns = new List<Burn>();
            Burn open = null;
            Elements el = Orbit.Elements(r, v, mu);
            double period0 = el != null ? el.period : 0.0;
            double floor = body.radius + body.atmosphere;

            Func<bool, string, SimResult> finish = (done, reason) =>
            {
                if (open != null && record)
                {
                    open.t1 = t;
                    open.nu1 = el != null ? el.nu : 0.0;
                    burns.Add(open);
                }
                return new SimResult
                {
                    done = done, reason = reason, time = t, dv = dv, propUsed = XenonKg(vessel, dv),
                    orbits = period0 != 0 ? t / period0 : 0.0, burns = burns, el = el, endR = r, endV = v,
                    endMass = m
                };
            };

            while (true)
            {
                el = Orbit.Elements(r, v, mu);
                if (el == null) return finish(false, "escaped");
                if (el.rp < floor) return finish(false, "periapsis inside the atmosphere");
                if (t > cfg.MAX_ORBITS * period0) return finish(false, "orbit limit");
                double tick = TickFor(el, cfg);
                double accel = vessel.thrustN / m;
                target = Guidance.Lock(target, el, cfg, open == null);
                Command cmd = Guidance.Command(el, target, accel, tick, etaCut, cfg, mu);
                if (cmd.done) return finish(true, "arrived");
                if (cmd.throttle <= 0)
                {
                    if (open != null)
                    {
                        if (record) { open.t1 = t; open.nu1 = el.nu; burns.Add(open); }
                        open = null;
                    }
                    V3 r1, v1;
                    Orbit.Coast(r, v, mu, tick, out r1, out v1);
                    r = r1; v = v1;
                    t += tick;
                    continue;
                }
                V3 R, T, N;
                Orbit.Rtn(r, v, out R, out T, out N);
                V3 force = (T * cmd.prograde + N * cmd.normal) * (vessel.thrustN * cmd.throttle);
                double h = tick / cfg.THRUST_SUBSTEPS;
                for (int i = 0; i < cfg.THRUST_SUBSTEPS; i++) Rk4(ref r, ref v, m, mu, force, h);
                double burnDv = accel * cmd.throttle * tick;
                dv += burnDv;
                t += tick;
                if (open == null) open = new Burn { t0 = t - tick, dv = 0.0, nu0 = el.nu };
                open.dv += burnDv;
            }
        }
    }

    /// <summary>The cheapest transfer within one SOI that finishes within a
    /// time budget: the stand-in flown over a grid of arc cutoffs, the
    /// highest (cheapest) whose flight meets the deadline.</summary>
    public static class Planner
    {
        public class Plan
        {
            public bool feasible;
            public double eta;
            public SimResult result;
            public string reason;
            public List<double[]> table;    // (eta, done 1/0, time, dv)
        }

        public static Plan Make(SimBody body, Vessel vessel, V3 r, V3 v, Target target, Config cfg,
                                double withinS = double.NaN)
        {
            Func<double, bool, SimResult> fly = (eta, rec) => Sim.Fly(body, vessel, r, v, target, eta, cfg, rec);
            if (double.IsNaN(withinS))
            {
                SimResult res = fly(cfg.ETA_DEFAULT, true);
                return new Plan { feasible = res.done, eta = cfg.ETA_DEFAULT, result = res, reason = res.reason,
                                  table = new List<double[]>() };
            }
            int n = cfg.PLAN_GRID;
            double wLo = Math.Acos(cfg.ETA_MAX), wHi = Math.Acos(cfg.ETA_MIN);
            var table = new List<double[]>();
            for (int k = 0; k < n; k++)
            {
                double eta = Math.Cos(wLo + (wHi - wLo) * k / (n - 1));
                SimResult res = fly(eta, false);
                table.Add(new[] { eta, res.done ? 1.0 : 0.0, res.time, res.dv });
            }
            double[] best = null;
            foreach (var row in table)
                if (row[1] > 0 && row[2] <= withinS && (best == null || row[0] > best[0])) best = row;
            if (best != null)
                return new Plan { feasible = true, eta = best[0], result = fly(best[0], true),
                                  reason = "arrives in time", table = table };
            double[] fastest = null;
            foreach (var row in table)
                if (row[1] > 0 && (fastest == null || row[2] < fastest[2])) fastest = row;
            if (fastest == null)
                return new Plan { feasible = false, eta = table[0][0], result = fly(table[0][0], true),
                                  reason = "no cutoff arrives", table = table };
            return new Plan { feasible = false, eta = fastest[0], result = fly(fastest[0], true),
                              reason = "late: the deadline is shorter than the fastest transfer", table = table };
        }
    }
}
