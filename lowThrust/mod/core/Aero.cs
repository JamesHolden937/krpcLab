// The atmosphere: density by altitude, the drag pass through it, and how
// deep a pass may go.
//
// - Atmosphere: a body's density (kg/m^3) every STEP metres from sea level to
//   the top (where it is 0), interpolated log-linearly.  In the game it is
//   sampled from the body itself (GameSystem); the stand-in's tables (Air)
//   were sampled from the game by lowThrust/tools/atmodump.py.
// - Pass: the flight from the atmosphere's top down and out again, RK4 with
//   gravity and drag against the air (which turns with the body), no thrust.
//   Drag per unit mass is q x dragPerMass (the vessel's drag area over its
//   mass, times the drag scale learned from the passes flown).  Its peak
//   heating index H = sqrt(rho) v^3 / 1e6 is KSP's high-Mach convective
//   coefficient less its constant; the load is H integrated over the pass.
// - Deepest: the lowest periapsis whose pass, flown at AERO_DRAG_MARGIN times
//   the drag and the density, stays inside every limit: the floor (terrain),
//   the heating and its load, the deceleration, and the apoapsis it leaves.
// - AeroState: what the passes flown have taught (drag scale, heating
//   allowed); in the game each pass's measured temperatures move it.

using System;
using System.Collections.Generic;

namespace LowThrust
{
    public class Atmosphere
    {
        public const double STEP = 1000.0;
        public readonly double top;
        public readonly double[] rho;

        public Atmosphere(double top, double[] rho) { this.top = top; this.rho = rho; }

        /// <summary>Density (kg/m^3) at altitude alt (m); 0 at and above the top.</summary>
        public double Density(double alt)
        {
            if (alt >= top) return 0.0;
            if (alt <= 0) return rho[0];
            double x = alt / STEP;
            int i = (int)x;
            if (i >= rho.Length - 1) return 0.0;
            double f = x - i, a = rho[i], b = rho[i + 1];
            if (a <= 0) return 0.0;
            if (b <= 0) return a * (1 - f);
            return a * Math.Exp(f * Math.Log(b / a));
        }
    }

    public class PassResult
    {
        public V3 r, v;                 // where the pass ended (out of the top, or stopped)
        public double t, dt;            // UT at the end; seconds inside
        public bool exited, crashed;
        public double dE;               // specific orbital energy shed, J/kg
        public double dv;               // drag dv, m/s
        public double peakH, load;      // heating index peak, and its integral (s)
        public double peakAcc, peakQ;   // m/s^2, Pa
        public double minAlt;           // m
    }

    public class PassLimits
    {
        public double altMin, hMax, loadMax, accMax, raMin, margin;
        // the periapsis must be liftable to liftTo by an engine of liftAccel
        // in an arc of +-LIFT_ARC around the next apoapsis (0: unchecked)
        public double liftAccel, liftTo;
    }

    /// <summary>What the passes flown have taught: the drag scale (measured
    /// drag over the model's), the heating index and load allowed.  The
    /// stand-in plans with the configured limit and learns nothing.</summary>
    public class AeroState
    {
        public double k = 1.0;
        public double hAllow, loadAllow = double.PositiveInfinity;
        public int passes;
        public bool learn;
        // the last pass's temperatures allowed more than the growth cap
        public bool growing = true;
    }

    public static class Aero
    {
        /// <summary>The stock atmospheres (the stand-in's), sampled from the
        /// game every Atmosphere.STEP m (lowThrust/tools/atmodump.py).</summary>
        public static readonly Dictionary<string, double[]> Air = new Dictionary<string, double[]>
        {
            { "Duna", new double[] {
                0.1333, 0.12, 0.1079, 0.09671, 0.08648, 0.07707, 0.06842, 0.06047, 0.05318, 0.04652,
                0.04049, 0.03508, 0.0303, 0.02603, 0.02213, 0.0186, 0.01546, 0.01273, 0.01043, 0.008607,
                0.007279, 0.006256, 0.005325, 0.004485, 0.003739, 0.003086, 0.002526, 0.002058, 0.001676, 0.001372,
                0.001134, 0.0009525, 0.0008137, 0.0007039, 0.0006096, 0.0005172, 0.0004269, 0.0003477, 0.0002789, 0.0002197,
                0.0001695, 0.0001276, 9.307e-05, 6.535e-05, 4.367e-05, 2.731e-05, 1.548e-05, 7.225e-06, 2.48e-06, 4.942e-07,
                0,
            } },
            { "Eve", new double[] {
                6.172, 5.792, 5.43, 5.083, 4.749, 4.428, 4.118, 3.818, 3.528, 3.247,
                2.975, 2.712, 2.459, 2.217, 1.986, 1.767, 1.563, 1.373, 1.198, 1.037,
                0.8911, 0.7606, 0.6454, 0.546, 0.4625, 0.395, 0.3399, 0.2929, 0.2535, 0.221,
                0.1949, 0.1744, 0.1587, 0.147, 0.1385, 0.1321, 0.1268, 0.1215, 0.1151, 0.1065,
                0.09426, 0.0801, 0.06655, 0.0538, 0.04204, 0.03149, 0.02233, 0.01476, 0.008976, 0.00515,
                0.003456, 0.002916, 0.002447, 0.002046, 0.001708, 0.001427, 0.001196, 0.001006, 0.0008514, 0.0007228,
                0.0006129, 0.0005195, 0.0004424, 0.0003784, 0.000325, 0.0002803, 0.0002426, 0.0002105, 0.0001826, 0.0001578,
                0.0001347, 0.0001135, 9.496e-05, 7.903e-05, 6.554e-05, 5.428e-05, 4.496e-05, 3.732e-05, 3.103e-05, 2.581e-05,
                2.137e-05, 1.746e-05, 1.4e-05, 1.098e-05, 8.376e-06, 6.154e-06, 4.294e-06, 2.771e-06, 1.562e-06, 6.451e-07,
                0,
            } },
            { "Jool", new double[] {
                6.703, 5.9, 5.188, 4.563, 4.019, 3.549, 3.147, 2.807, 2.522, 2.284,
                2.086, 1.915, 1.763, 1.628, 1.51, 1.407, 1.318, 1.242, 1.178, 1.125,
                1.081, 1.045, 1.017, 0.9941, 0.9761, 0.9615, 0.949, 0.9375, 0.9255, 0.9118,
                0.8968, 0.8818, 0.8669, 0.852, 0.8371, 0.8222, 0.8074, 0.7926, 0.7779, 0.7632,
                0.7486, 0.7341, 0.7197, 0.7053, 0.691, 0.6768, 0.6627, 0.6487, 0.6349, 0.6211,
                0.6074, 0.5939, 0.5805, 0.5672, 0.5541, 0.541, 0.5282, 0.5155, 0.5029, 0.4905,
                0.4782, 0.4661, 0.4541, 0.4423, 0.4307, 0.4193, 0.408, 0.3969, 0.386, 0.3752,
                0.3647, 0.3543, 0.344, 0.334, 0.3242, 0.3145, 0.305, 0.2957, 0.2866, 0.2777,
                0.2689, 0.2604, 0.252, 0.2438, 0.2358, 0.2279, 0.2203, 0.2128, 0.2055, 0.1984,
                0.1914, 0.1846, 0.178, 0.1716, 0.1653, 0.1591, 0.1532, 0.1474, 0.1417, 0.1362,
                0.1308, 0.1256, 0.1206, 0.1156, 0.1108, 0.1062, 0.1017, 0.09725, 0.09297, 0.08881,
                0.08477, 0.08084, 0.07702, 0.07331, 0.0697, 0.06618, 0.06276, 0.05944, 0.0562, 0.05304,
                0.04996, 0.04696, 0.04403, 0.04117, 0.03838, 0.03569, 0.03311, 0.03064, 0.02829, 0.02605,
                0.02392, 0.02192, 0.02002, 0.01825, 0.01658, 0.01503, 0.01358, 0.01225, 0.01101, 0.00988,
                0.008845, 0.007904, 0.007053, 0.00629, 0.005609, 0.005007, 0.00448, 0.004025, 0.003637, 0.003312,
                0.003048, 0.002814, 0.002585, 0.002362, 0.002145, 0.001935, 0.001734, 0.001541, 0.001358, 0.001185,
                0.001023, 0.0008722, 0.0007339, 0.0006082, 0.0004958, 0.0003972, 0.0003129, 0.0002434, 0.0001892, 0.0001508,
                0.0001287, 0.0001154, 0.0001031, 9.179e-05, 8.138e-05, 7.184e-05, 6.313e-05, 5.52e-05, 4.801e-05, 4.15e-05,
                3.565e-05, 3.041e-05, 2.573e-05, 2.158e-05, 1.792e-05, 1.471e-05, 1.193e-05, 9.526e-06, 7.482e-06, 5.776e-06,
                4.361e-06, 3.198e-06, 2.259e-06, 1.518e-06, 9.547e-07, 5.464e-07, 2.691e-07, 1.044e-07, 2.679e-08, 2.641e-09,
                0,
            } },
            { "Kerbin", new double[] {
                1.14, 1.009, 0.8915, 0.7865, 0.6923, 0.6076, 0.5312, 0.4621, 0.3998, 0.3412,
                0.2819, 0.233, 0.1925, 0.1589, 0.1311, 0.108, 0.08888, 0.07273, 0.0595, 0.04875,
                0.04003, 0.03294, 0.02708, 0.02225, 0.01832, 0.01513, 0.0125, 0.01028, 0.008436, 0.006932,
                0.005714, 0.004732, 0.00394, 0.003294, 0.002753, 0.0023, 0.001923, 0.001612, 0.001359, 0.001164,
                0.001001, 0.000862, 0.0007485, 0.0006471, 0.000558, 0.0004802, 0.0004127, 0.0003543, 0.0003039, 0.0002601,
                0.0002217, 0.0001884, 0.0001596, 0.000135, 0.0001141, 9.629e-05, 8.114e-05, 6.808e-05, 5.668e-05, 4.667e-05,
                3.818e-05, 3.111e-05, 2.529e-05, 2.032e-05, 1.57e-05, 1.148e-05, 7.738e-06, 4.586e-06, 2.147e-06, 5.638e-07,
                0,
            } },
            { "Laythe", new double[] {
                0.7431, 0.7073, 0.6762, 0.6459, 0.6129, 0.5738, 0.512, 0.4498, 0.394, 0.3444,
                0.3008, 0.2612, 0.2274, 0.1985, 0.1736, 0.1518, 0.1321, 0.1136, 0.09646, 0.08331,
                0.07294, 0.06427, 0.05633, 0.05006, 0.04483, 0.04044, 0.03669, 0.03343, 0.03049, 0.02772,
                0.02498, 0.02213, 0.01912, 0.01662, 0.01455, 0.01281, 0.01129, 0.009918, 0.00859, 0.007244,
                0.006007, 0.004882, 0.003871, 0.002975, 0.002195, 0.001531, 0.0009855, 0.0005579, 0.0002499, 6.302e-05,
                0,
            } },
        };

        public static double Energy(V3 r, V3 v, double mu) { return V3.Dot(v, v) / 2 - mu / r.Norm; }

        static V3 Accel(Body b, V3 r, V3 v, double dpm, double rhoScale, out double rho, out V3 va)
        {
            double rn = r.Norm;
            rho = b.air.Density(rn - b.radius) * rhoScale;
            va = v - V3.Cross(new V3(0.0, 0.0, b.spin), r);
            V3 g = r * (-b.mu / (rn * rn * rn));
            if (rho <= 0) return g;
            return g - va * (0.5 * rho * va.Norm * dpm);
        }

        /// <summary>Fly a pass from (r, v) at UT t (at or above the top,
        /// inbound) until it leaves the top outbound, hits the ground, or
        /// has spent maxS inside.  dragPerMass: drag area over mass (m^2/kg)
        /// times the drag scale; rhoScale multiplies the density (drag and
        /// heating alike).</summary>
        public static PassResult Pass(Body b, V3 r, V3 v, double t, double dragPerMass, double rhoScale = 1.0,
                                      double maxS = 20000.0)
        {
            var res = new PassResult { minAlt = double.PositiveInfinity };
            double top = b.radius + b.air.top;
            double e0 = Energy(r, v, b.mu), t0 = t;
            bool inside = false;
            while (true)
            {
                double rn = r.Norm;
                if (rn < b.radius) { res.crashed = true; break; }
                bool up = V3.Dot(r, v) > 0;
                if (rn > top && (up || inside && up)) { res.exited = true; break; }
                if (rn <= top) inside = true;
                if (t - t0 > maxS) break;
                res.minAlt = Math.Min(res.minAlt, rn - b.radius);
                double rho0;
                V3 va0;
                V3 a1 = Accel(b, r, v, dragPerMass, rhoScale, out rho0, out va0);
                double speed = va0.Norm;
                double h = Py.Clamp(500.0 / Math.Max(v.Norm, 1.0), 0.02, 10.0);
                if (rho0 > 0)
                {
                    double H = Math.Sqrt(rho0) * speed * speed * speed / 1e6;
                    double q = 0.5 * rho0 * speed * speed;
                    res.peakH = Math.Max(res.peakH, H);
                    res.load += H * h;
                    res.peakQ = Math.Max(res.peakQ, q);
                    res.peakAcc = Math.Max(res.peakAcc, q * dragPerMass);
                    res.dv += q * dragPerMass * h;
                }
                double rho;
                V3 va;
                V3 r2 = r + v * (h / 2), v2 = v + a1 * (h / 2);
                V3 a2 = Accel(b, r2, v2, dragPerMass, rhoScale, out rho, out va);
                V3 r3 = r + v2 * (h / 2), v3 = v + a2 * (h / 2);
                V3 a3 = Accel(b, r3, v3, dragPerMass, rhoScale, out rho, out va);
                V3 r4 = r + v3 * h, v4 = v + a3 * h;
                V3 a4 = Accel(b, r4, v4, dragPerMass, rhoScale, out rho, out va);
                r = r + (v + v2 * 2.0 + v3 * 2.0 + v4) * (h / 6);
                v = v + (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (h / 6);
                t += h;
                if (inside) res.dt += h;
            }
            res.r = r; res.v = v; res.t = t;
            res.dE = e0 - Energy(r, v, b.mu);
            return res;
        }

        /// <summary>Mean anomaly (hyperbolic for e &gt; 1) at true anomaly nu
        /// in -pi..pi, signed; and the mean motion.</summary>
        static double Mean(double e, double a, double mu, double nu, out double n)
        {
            n = Math.Sqrt(mu / Math.Pow(Math.Abs(a), 3));
            if (e < 1)
            {
                double E = 2 * Math.Atan(Math.Sqrt((1 - e) / (1 + e)) * Math.Tan(nu / 2));
                return E - e * Math.Sin(E);
            }
            double F = 2 * Py.Atanh(Math.Sqrt((e - 1) / (e + 1)) * Math.Tan(nu / 2));
            return e * Math.Sinh(F) - F;
        }

        /// <summary>Seconds from periapsis back to where the conic (e, a, p)
        /// is inbound at radius R (R above rp).</summary>
        static double FromRadiusToPeriapsis(double e, double a, double p, double mu, double R)
        {
            double cosNu = Py.Clamp((p / R - 1) / Math.Max(e, 1e-12), -1.0, 1.0);
            double n;
            return Mean(e, a, mu, Math.Acos(cosNu), out n) / n;
        }

        /// <summary>Seconds until the conic next crosses radius R inbound: 0 if
        /// already inside it, infinity if it never comes down to R.</summary>
        public static double TimeToRadius(Conic c, double mu, double R)
        {
            if (c.r <= R) return 0.0;
            if (c.rp >= R) return double.PositiveInfinity;
            double tPe = Orbit.TimeToPeriapsis(c, mu);
            if (c.e >= 1 && tPe < 0) return double.PositiveInfinity;
            double back = FromRadiusToPeriapsis(c.e, c.a, c.p, mu, R);
            double t = tPe - back;
            if (t < 0 && c.e < 1) t += c.period;
            return Math.Max(t, 0.0);
        }

        /// <summary>The state where the conic of periapsis radius rp, specific
        /// energy E, periapsis direction peDir and normal n enters radius R
        /// inbound.</summary>
        public static void EntryState(double rp, double E, V3 peDir, V3 n, double mu, double R, out V3 r, out V3 v)
        {
            V3 rP = peDir * rp;
            V3 vP = V3.Cross(n, peDir).Unit * Math.Sqrt(Math.Max(2 * (E + mu / rp), 0.0));
            if (R <= rp) { r = rP; v = vP; return; }
            Conic c = Orbit.Conic(rP, vP, mu);
            double back = FromRadiusToPeriapsis(c.e, c.a, c.p, mu, R);
            Orbit.Propagate(rP, vP, mu, -back, out r, out v);
        }

        /// <summary>The pass of the conic with periapsis radius rp (energy E,
        /// periapsis direction and normal given) reaching periapsis at UT
        /// tPe, from the top.</summary>
        public static PassResult PassAt(Body b, double rp, double E, V3 peDir, V3 n, double tPe, double dragPerMass,
                                        double rhoScale)
        {
            V3 r, v;
            double R = b.radius + b.air.top;
            EntryState(rp, E, peDir, n, b.mu, R, out r, out v);
            Conic c = Orbit.Conic(r, v, b.mu);
            double back = rp < R ? FromRadiusToPeriapsis(c.e, c.a, c.p, b.mu, R) : 0.0;
            return Pass(b, r, v, tPe - back, dragPerMass, rhoScale);
        }

        /// <summary>Whether a pass flown at the margin keeps every limit.</summary>
        public static bool Within(Body b, PassResult hi, PassLimits lim)
        {
            if (hi.crashed || !hi.exited || hi.minAlt < lim.altMin) return false;
            if (hi.peakH > lim.hMax || hi.load > lim.loadMax || hi.peakAcc > lim.accMax) return false;
            Conic c = Orbit.Conic(hi.r, hi.v, b.mu);
            if (c.e >= 1) return true;
            return c.ra >= lim.raMin && (lim.liftAccel <= 0 || LiftSeconds(b.mu, c.ra, c.rp, lim.liftTo, lim.liftAccel)
                                                               <= ArcSeconds(b.mu, c.ra, c.rp, LIFT_ARC));
        }

        /// <summary>The widest arc round apoapsis a lift out of the air may
        /// take (rad).</summary>
        public const double LIFT_ARC = Math.PI / 3;

        /// <summary>Seconds at full thrust to raise the periapsis from rp to
        /// rTo at apoapsis ra.</summary>
        public static double LiftSeconds(double mu, double ra, double rp, double rTo, double accel)
        {
            if (rTo <= rp) return 0.0;
            double v0 = Math.Sqrt(mu * (2 / ra - 2 / (ra + rp))), v1 = Math.Sqrt(mu * (2 / ra - 2 / (ra + rTo)));
            return (v1 - v0) / accel;
        }

        /// <summary>Seconds the ellipse (ra, rp) spends within +-half rad of
        /// its apoapsis.</summary>
        public static double ArcSeconds(double mu, double ra, double rp, double half)
        {
            double a = (ra + rp) / 2, e = (ra - rp) / (ra + rp);
            double E = 2 * Math.Atan(Math.Sqrt((1 - e) / (1 + e)) * Math.Tan((Math.PI - half) / 2));
            double M = E - e * Math.Sin(E);
            return 2 * (Math.PI - M) / Math.Sqrt(mu / (a * a * a));
        }

        /// <summary>The deepest periapsis radius whose pass, flown at the
        /// margin, keeps every limit (energy(rp) the orbit's energy for that
        /// periapsis); null if even a graze does not.</summary>
        public static double? Deepest(Body b, Func<double, double> energy, V3 peDir, V3 n, double tPe,
                                      double dragPerMass, PassLimits lim)
        {
            double top = b.radius + b.air.top;
            Func<double, bool> ok = rp => Within(b, PassAt(b, rp, energy(rp), peDir, n, tPe,
                                                           dragPerMass * lim.margin, lim.margin), lim);
            double lo = b.radius + lim.altMin, hi = top - 10.0;
            if (hi <= lo || !ok(hi)) return null;
            if (ok(lo)) return lo;
            for (int i = 0; i < 30 && hi - lo > 10.0; i++)
            {
                double mid = (lo + hi) / 2;
                if (ok(mid)) hi = mid; else lo = mid;
            }
            return hi;
        }
    }
}
