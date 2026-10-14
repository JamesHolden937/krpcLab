// Targeting for transfers out of the current body's orbit: to one of its
// moons, or (through escape) to a sibling planet.  Pure functions.
//
// Everything is predicted by propagating the two-body coast exactly
// (Orbit.Propagate) against the target body on its rails.
// - ClosestApproach / AimError: where the coast passes the target, against
//   the aim for the capture periapsis (at the SOI entry when it enters).
// - Correction: the smallest impulse that zeroes that miss.
// - PushDv: the prograde push that puts a moon encounter on the aim.
// - Window: the heliocentric departure (Lambert over dates and times).
// - ParkingOrientation: the parking ellipse whose push leaves on the
//   departure asymptote.
// - the capture: its design and its law (CaptureStep, CaptureCommand).

using System;
using System.Collections.Generic;

namespace LowThrust
{
    public class Approach
    {
        public double t;
        public V3 d, vrel;        // vessel - target
        public Approach(double t, V3 d, V3 vrel) { this.t = t; this.d = d; this.vrel = vrel; }
    }

    public class Window
    {
        public double tDep, tof;
        public V3 vinfDep, vinfArr;
    }

    /// <summary>A capture at one destination: its gravity, the ellipse aimed
    /// for (apoapsis raCap), the loosest accepted once past periapsis (raOk),
    /// the periapsis band the burn holds (floor .. rpMax), the surface (or
    /// atmosphere top), the one-pass test (burn time at most fill of the time
    /// within +-arc of periapsis), the share of the path to periapsis a long
    /// capture plans to burn over (k), the periapsis aimed for (rc).</summary>
    public class Capture
    {
        public double mu, raCap, raOk, floor, rpMax, surface, arc, fill, k;
        public double? rc;
    }

    public class CaptureFlight { public double dv, tStart, dStart; }

    public static class Transfer
    {
        public static double ImpactParameter(double rp, double vinf, double mu)
        {
            return rp * Math.Sqrt(1 + 2 * mu / (rp * vinf * vinf));
        }

        /// <summary>The miss distance to aim a coast at, passing body at
        /// relative speed v, for periapsis rp (patched conics), at most 0.8
        /// of the SOI.</summary>
        public static double AimParameter(double rp, double v, Body body)
        {
            v = Math.Max(v, 1.0);
            double x = 1 + 2 * body.mu / (rp * v * v) - 2 * body.mu / (body.Soi * v * v);
            return Math.Min(rp * Math.Sqrt(Math.Max(x, 0.0)), 0.8 * body.Soi);
        }

        /// <summary>Closest approach of the coast from (r, v) at UT t to
        /// target within horizon seconds; with a guess UT, only a window round
        /// it is searched.</summary>
        public static Approach ClosestApproach(V3 r, V3 v, double t, double mu, Body target, double horizon,
                                               int samples = 400, double? guess = null)
        {
            Func<double, double> dist = tt =>
            {
                V3 rv, vv;
                Orbit.Propagate(r, v, mu, tt - t, out rv, out vv);
                return (rv - target.Position(tt)).Norm;
            };
            double loT, hiT;
            int n;
            if (guess.HasValue)
            {
                double span = Math.Min(horizon, Math.Max(Math.Abs(guess.Value - t) * 0.2, 600.0));
                loT = Math.Max(t, guess.Value - span);
                hiT = Math.Min(t + horizon, guess.Value + span);
                n = 40;
            }
            else { loT = t; hiT = t + horizon; n = samples; }
            double step = (hiT - loT) / n;
            int bestI = 0;
            double bestD = Py.Inf;
            for (int i = 0; i <= n; i++)
            {
                double d = dist(loT + i * step);
                if (d < bestD) { bestI = i; bestD = d; }
            }
            double a = loT + Math.Max(0, bestI - 1) * step;
            double b = loT + Math.Min(n, bestI + 1) * step;
            double g = (Math.Sqrt(5) - 1) / 2;
            for (int i = 0; i < 60; i++)
            {
                double c1 = b - g * (b - a), c2 = a + g * (b - a);
                if (dist(c1) < dist(c2)) b = c2; else a = c1;
                if (b - a < 1e-3) break;
            }
            double tc = (a + b) / 2;
            V3 rv1, vv1, rt, vt;
            Orbit.Propagate(r, v, mu, tc - t, out rv1, out vv1);
            target.State(tc, out rt, out vt);
            return new Approach(tc, rv1 - rt, vv1 - vt);
        }

        /// <summary>The closest-approach distance, positive on the prograde
        /// side.</summary>
        public static double SignedMiss(Approach ca, V3 normal)
        {
            V3 u = ca.vrel.Unit;
            double s = ca.d.Norm;
            return V3.Dot(V3.Cross(ca.d, u), normal) > 0 ? s : -s;
        }

        /// <summary>The plane the aim lies in, for an arrival along u:
        /// prograde about normal, or with polar the plane through the poles.</summary>
        public static V3 AimNormal(V3 u, V3 normal, bool polar = false)
        {
            return polar ? V3.Cross(u, normal).Unit : normal;
        }

        /// <summary>Where the coast enters target's SOI on the way to its
        /// closest approach ca, or null if it never enters.</summary>
        public static Approach SoiEntry(V3 r, V3 v, double t, double mu, Body target, Approach ca)
        {
            double R = target.Soi;
            if (ca.d.Norm >= R) return null;
            Func<double, Approach> state = tt =>
            {
                V3 rv, vv, rt, vt;
                Orbit.Propagate(r, v, mu, tt - t, out rv, out vv);
                target.State(tt, out rt, out vt);
                return new Approach(tt, rv - rt, vv - vt);
            };
            double step = R / Math.Max(ca.vrel.Norm, 1.0);
            double lo = ca.t - step;
            for (int i = 0; i < 60; i++)
            {
                if (lo <= t || state(lo).d.Norm >= R) break;
                step *= 1.5;
                lo = ca.t - step;
            }
            lo = Math.Max(lo, t);
            if (state(lo).d.Norm < R) return null;
            double hi = ca.t;
            for (int i = 0; i < 50; i++)
            {
                double mid = (lo + hi) / 2;
                if (state(mid).d.Norm >= R) lo = mid; else hi = mid;
            }
            return state(hi);
        }

        public class Aim { public double s, b; public V3 err, u; }

        /// <summary>How far the coast passes target from the aim for
        /// periapsis rc on the side prograde about normal: measured at the SOI
        /// entry when the coast enters, else at the closest approach.</summary>
        public static Aim AimError(V3 r, V3 v, double t, double mu, Body target, Approach ca, double rc,
                                   V3 normal, bool polar = false)
        {
            Approach e = SoiEntry(r, v, t, mu, target, ca);
            V3 d, vr;
            double b;
            if (e == null)
            {
                d = ca.d; vr = ca.vrel;
                b = AimParameter(rc, vr.Norm, target);
            }
            else
            {
                d = e.d; vr = e.vrel;
                double w = vr.Norm;
                double energy = w * w / 2 - target.mu / target.Soi;
                double h2 = 2 * energy * rc * rc + 2 * target.mu * rc;
                b = Math.Min(Math.Sqrt(Math.Max(h2, 0.0)) / Math.Max(w, 1e-3), 0.95 * target.Soi);
            }
            V3 u = vr.Unit;
            normal = AimNormal(u, normal, polar);
            V3 dPerp = d - u * V3.Dot(d, u);
            V3 err = dPerp - V3.Cross(u, normal).Unit * b;
            double side = V3.Dot(V3.Cross(d, u), normal) > 0 ? 1.0 : -1.0;
            double s = side * (e != null ? dPerp.Norm : d.Norm) - b;
            return new Aim { s = s, err = err, u = u, b = b };
        }

        /// <summary>The impulse that zeroes the miss against the aim for
        /// periapsis rc (AimError), from a numerical Jacobian, minimum-norm.
        /// Returns the impulse; ca the closest approach it was computed from.</summary>
        public static V3 Correction(V3 r, V3 v, double t, double mu, Body target, double horizon, double rc,
                                    V3 normal, out Approach ca, double dvProbe = 0.01, double? guess = null,
                                    bool polar = false)
        {
            ca = ClosestApproach(r, v, t, mu, target, horizon, 400, guess);
            Aim a0 = AimError(r, v, t, mu, target, ca, rc, normal, polar);
            V3 e1 = V3.Cross(a0.u, AimNormal(a0.u, normal, polar)).Unit;
            V3 e2 = V3.Cross(a0.u, e1);
            double E0 = V3.Dot(a0.err, e1), E1 = V3.Dot(a0.err, e2);
            var J0 = new double[3];
            var J1 = new double[3];
            for (int k = 0; k < 3; k++)
            {
                V3 vk = v + V3.Axis(k, dvProbe);
                Approach cak = ClosestApproach(r, vk, t, mu, target, horizon, 400, ca.t);
                Aim ak = AimError(r, vk, t, mu, target, cak, rc, normal, polar);
                J0[k] = (V3.Dot(ak.err, e1) - E0) / dvProbe;
                J1[k] = (V3.Dot(ak.err, e2) - E1) / dvProbe;
            }
            double a11 = 0, a12 = 0, a22 = 0;
            for (int k = 0; k < 3; k++) { a11 += J0[k] * J0[k]; a12 += J0[k] * J1[k]; a22 += J1[k] * J1[k]; }
            double det = a11 * a22 - a12 * a12;
            if (det == 0) return V3.Zero;
            double l1 = -(a22 * E0 - a12 * E1) / det;
            double l2 = -(-a12 * E0 + a11 * E1) / det;
            return new V3(J0[0] * l1 + J1[0] * l2, J0[1] * l1 + J1[1] * l2, J0[2] * l1 + J1[2] * l2);
        }

        /// <summary>Lambert to the B-plane aim point at UT ta.  False if no
        /// arc; vt the target's velocity then.</summary>
        static bool Aimed(V3 r, double t, double mu, Body target, double ta, V3 normal, double? rc, bool polar,
                          out V3 s0, out V3 s1, out V3 vt)
        {
            V3 rt;
            target.State(ta, out rt, out vt);
            if (!Orbit.Lambert(r, rt, ta - t, mu, normal, out s0, out s1)) return false;
            if (!rc.HasValue) return true;
            for (int i = 0; i < 2; i++)
            {
                V3 vrel = s1 - vt;
                V3 u = vrel.Unit;
                double b = AimParameter(rc.Value, vrel.Norm, target);
                V3 side = V3.Cross(u, AimNormal(u, target.Normal, polar)).Unit;
                V3 aim = rt + side * b;
                V3 a0, a1;
                if (!Orbit.Lambert(r, aim, ta - t, mu, normal, out a0, out a1)) break;
                s0 = a0; s1 = a1;
            }
            return true;
        }

        /// <summary>A new transfer from (r, v, t) to target: Lambert over
        /// arrival times in [tLo, tHi], the cheapest in impulse plus arrival
        /// speed.  False if none.</summary>
        public static bool Retarget(V3 r, V3 v, double t, double mu, Body target, double tLo, double tHi,
                                    V3 normal, out V3 dvOut, out double taOut, int n = 24, double? rc = null,
                                    bool polar = false)
        {
            dvOut = V3.Zero; taOut = 0;
            double bestCost = 0;
            bool found = false;
            for (int i = 0; i <= n; i++)
            {
                double ta = tLo + (tHi - tLo) * i / n;
                if (ta <= t + 60) continue;
                V3 rt, vt0;
                target.State(ta, out rt, out vt0);
                double ang = Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(r, rt) / (r.Norm * rt.Norm))));
                if (Math.Abs(ang - Math.PI) < Py.Radians(15)) continue;
                V3 s0, s1, vt;
                if (!Aimed(r, t, mu, target, ta, normal, rc, polar, out s0, out s1, out vt)) continue;
                V3 dv = s0 - v;
                double cost = dv.Norm + (s1 - vt).Norm;
                if (!found || cost < bestCost) { found = true; bestCost = cost; dvOut = dv; taOut = ta; }
            }
            return found;
        }

        /// <summary>The impulse onto the Lambert arc to target's aim point at
        /// UT ta, or null.</summary>
        public static V3? LambertDv(V3 r, V3 v, double t, double mu, Body target, double ta, V3 normal,
                                    double? rc = null, bool polar = false)
        {
            V3 s0, s1, vt;
            if (!Aimed(r, t, mu, target, ta, normal, rc, polar, out s0, out s1, out vt)) return null;
            return s0 - v;
        }

        /// <summary>The fastest arrival an engine of accel can capture in
        /// one pass (0 if none).</summary>
        public static double CapturableVinf(Body body, double accel, double rcMin, double raMax, double arc,
                                            double fill, double? raLoose = null)
        {
            double rc, ra;
            if (!CaptureOrbit(body, 1.0, accel, rcMin, raMax, arc, fill, raLoose, out rc, out ra)) return 0.0;
            double lo = 1.0, hi = 5000.0;
            if (CaptureOrbit(body, hi, accel, rcMin, raMax, arc, fill, raLoose, out rc, out ra)) return hi;
            for (int i = 0; i < 30; i++)
            {
                double mid = (lo + hi) / 2;
                if (CaptureOrbit(body, mid, accel, rcMin, raMax, arc, fill, raLoose, out rc, out ra)) lo = mid;
                else hi = mid;
            }
            return lo;
        }

        /// <summary>(r, v, t) after dv of prograde thrust at accel: from
        /// (r, v, t), or centred on it when centered.  accel 0: an impulse.</summary>
        public static void ProgradeBurn(ref V3 r, ref V3 v, ref double t, double mu, double dv, double accel,
                                        bool centered)
        {
            if (accel == 0) { v = v + v.Unit * dv; return; }
            double dur = dv / accel;
            if (centered)
            {
                V3 r1, v1;
                Orbit.Propagate(r, v, mu, -dur / 2, out r1, out v1);
                r = r1; v = v1;
                t -= dur / 2;
            }
            int n = Math.Max(4, Math.Min(60, (int)(dur / 60.0)));
            double h = dur / n;
            for (int i = 0; i < n; i++) Sim.Rk4(ref r, ref v, 1.0, mu, v.Unit * accel, h);
            t = t + dur;
        }

        /// <summary>The smallest prograde push in (dvMin, dvMax] at (r, v, t)
        /// whose coast passes target on the aim for periapsis rc; null if
        /// none.  horizonFn(r, v): how far ahead to search after it.  With
        /// accel the push is the finite burn the flight flies.</summary>
        public static double? PushDv(V3 r, V3 v, double t, double mu, Body target, double rc, V3 normal,
                                     double dvMax, Func<V3, V3, double> horizonFn, int n = 40,
                                     double dvMin = 0.0, double accel = 0.0, bool centered = true)
        {
            double b = AimParameter(rc, 1e9, target);
            Func<double, double> f = dv =>
            {
                V3 r1 = r, v1 = v;
                double t1 = t;
                ProgradeBurn(ref r1, ref v1, ref t1, mu, dv, accel, centered);
                double h = horizonFn(r1, v1);
                Approach ca = ClosestApproach(r1, v1, t1, mu, target, h, 200);
                if (ca.t > t1 + 0.999 * h) return Math.Abs(SignedMiss(ca, normal)) + b;
                return AimError(r1, v1, t1, mu, target, ca, rc, normal).s;
            };
            double prevDv = dvMin, prevF = f(dvMin);
            for (int i = 1; i <= n; i++)
            {
                double dv = dvMin + (dvMax - dvMin) * i / n;
                double fv = f(dv);
                if ((prevF < 0) != (fv < 0) && Math.Abs(fv) < 20 * b + Math.Abs(prevF))
                {
                    double lo = prevDv, hi = dv, flo = prevF;
                    for (int j = 0; j < 40; j++)
                    {
                        double mid = (lo + hi) / 2;
                        double fm2 = f(mid);
                        if ((fm2 < 0) == (flo < 0)) { lo = mid; flo = fm2; } else hi = mid;
                    }
                    double fm = f((lo + hi) / 2);
                    if (Math.Abs(fm) < Math.Max(0.2 * b, 1e3)) return (lo + hi) / 2;
                }
                prevDv = dv; prevF = fv;
            }
            return null;
        }

        /// <summary>The radii of b's orbit widened by margin of its SOI.</summary>
        public static double[] Band(Body b, double margin)
        {
            return new[] { b.sma * (1 - b.ecc) - margin * b.Soi, b.sma * (1 + b.ecc) + margin * b.Soi };
        }

        /// <summary>The first of others whose SOI the coast from (r, v, t)
        /// enters before UT tEnd: (UT of the closest approach, body), or
        /// null.</summary>
        public static Tuple<double, Body> FirstHazardWhen(V3 r, V3 v, double t, double mu, IList<Body> others,
                                                          double tEnd, double margin = 1.2)
        {
            Tuple<double, Body> hit = null;
            foreach (Body b in others)
            {
                if (tEnd <= t) break;
                Approach ca = ClosestApproach(r, v, t, mu, b, tEnd - t, 300);
                if (ca.d.Norm < margin * b.Soi && (hit == null || ca.t < hit.Item1))
                    hit = Tuple.Create(ca.t, b);
            }
            return hit;
        }

        public static Body FirstHazard(V3 r, V3 v, double t, double mu, IList<Body> others, double tEnd,
                                       double margin = 1.2)
        {
            var hit = FirstHazardWhen(r, v, t, mu, others, tEnd, margin);
            return hit == null ? null : hit.Item2;
        }

        /// <summary>The cheapest Lambert departure from origin to dest
        /// leaving within span seconds of tFrom: least |v_inf out| + weight
        /// |v_inf in|, over flight times 0.5-1.5 of the Hohmann's.</summary>
        public static Window FindWindow(Body origin, Body dest, double tFrom, double muSun, double span,
                                        int nd = 90, int nt = 30, double arrivalWeight = 1.0)
        {
            double aT = (origin.sma + dest.sma) / 2;
            double tH = Math.PI * Math.Sqrt(Math.Pow(aT, 3) / muSun);
            double bestC = Py.Inf, bestTd = 0, bestTof = 0;
            V3? bestVd = null, bestVa = null;
            Func<double, double, Tuple<double, V3, V3>> cost = (td, tof) =>
            {
                V3 r1, v1p, r2, v2p, s0, s1;
                origin.State(td, out r1, out v1p);
                dest.State(td + tof, out r2, out v2p);
                if (!Orbit.Lambert(r1, r2, tof, muSun, origin.Normal, out s0, out s1)) return null;
                V3 vd = s0 - v1p, va = s1 - v2p;
                return Tuple.Create(vd.Norm + arrivalWeight * va.Norm, vd, va);
            };
            Action<double, double> consider = (td, tof) =>
            {
                var c = cost(td, tof);
                double cv = c == null ? Py.Inf : c.Item1;
                if (cv < bestC) { bestC = cv; bestTd = td; bestTof = tof; bestVd = c.Item2; bestVa = c.Item3; }
            };
            for (int i = 0; i <= nd; i++)
            {
                double td = tFrom + span * i / nd;
                for (int j = 0; j <= nt; j++) consider(td, tH * (0.5 + (double)j / nt));
            }
            double td0 = bestTd, tof0 = bestTof, dtd = span / nd, dtof = tH / nt;
            for (int k = 0; k < 3; k++)
            {
                for (int i = -5; i < 6; i++)
                    for (int j = -5; j < 6; j++)
                    {
                        double td = td0 + dtd * i / 5;
                        double tof = tof0 + dtof * j / 5;
                        if (td < tFrom || tof <= 0) continue;
                        consider(td, tof);
                    }
                td0 = bestTd; tof0 = bestTof;
                dtd = dtd / 5; dtof = dtof / 5;
            }
            if (!bestVd.HasValue) return null;
            return new Window { tDep = bestTd, tof = bestTof, vinfDep = bestVd.Value, vinfArr = bestVa.Value };
        }

        /// <summary>(eccentricity, true anomaly of the outgoing asymptote) of
        /// the escape hyperbola with periapsis rp and excess vinf.</summary>
        public static double DepartureNuInf(double mu, double rp, double vinf)
        {
            double e = 1 + rp * vinf * vinf / mu;
            return Math.Acos(-1 / e);
        }

        /// <summary>The parking ellipse (rp, ra) whose periapsis push leaves
        /// on vinfVec, the plane of that family whose change from normalNow
        /// costs least: (normal, periapsis direction, the change's dv).</summary>
        public static Tuple<V3, V3, double> ParkingOrientation(double mu, double rp, double ra, V3 vinfVec,
                                                               V3 normalNow, int steps = 360)
        {
            double vinf = vinfVec.Norm;
            V3 u = vinfVec.Unit;
            double nuInf = DepartureNuInf(mu, rp, vinf);
            V3 reference = Math.Abs(u.z) < 0.9 ? new V3(0.0, 0.0, 1.0) : new V3(1.0, 0.0, 0.0);
            V3 a_ = V3.Cross(u, reference).Unit;
            V3 b_ = V3.Cross(u, a_);
            double e = (ra - rp) / (ra + rp);
            double p = rp * (1 + e);
            double h = Math.Sqrt(mu * p);
            Tuple<V3, V3, double> best = null;
            for (int k = 0; k < steps; k++)
            {
                double psi = 2 * Math.PI * k / steps;
                V3 n = a_ * Math.Cos(psi) + b_ * Math.Sin(psi);
                if (V3.Dot(n, normalNow) <= 0) continue;
                V3 P = u * Math.Cos(nuInf) - V3.Cross(n, u) * Math.Sin(nuInf);
                double delta = Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(n, normalNow))));
                double cost;
                if (delta < 1e-9) cost = 0.0;
                else
                {
                    V3 nodes = V3.Cross(normalNow, n).Unit;
                    V3 evec = P * e;
                    double rNode = Math.Max(Orbit.RadiusToward(p, evec, nodes),
                                            Orbit.RadiusToward(p, evec, nodes * -1.0));
                    cost = 2 * h / rNode * Math.Sin(delta / 2);
                }
                if (best == null || cost < best.Item3) best = Tuple.Create(n, P.Unit, cost);
            }
            return best;
        }

        public static double CaptureDv(double mu, double rp, double ra, double vinf)
        {
            double vHyp = Math.Sqrt(vinf * vinf + 2 * mu / rp);
            double vEll = Math.Sqrt(mu * (2 / rp - 2 / (rp + ra)));
            return vHyp - vEll;
        }

        /// <summary>Seconds a hyperbola of periapsis rp and excess vinf spends
        /// within +-arc rad of periapsis.</summary>
        public static double PassTime(double mu, double rp, double vinf, double arc)
        {
            double e = 1 + rp * vinf * vinf / mu;
            double a = -mu / (vinf * vinf);
            arc = Math.Min(arc, 0.95 * Math.Acos(-1 / e));
            double F = 2 * Py.Atanh(Math.Sqrt((e - 1) / (e + 1)) * Math.Tan(arc / 2));
            double M = e * Math.Sinh(F) - F;
            return 2 * M / Math.Sqrt(mu / Math.Pow(-a, 3));
        }

        public static double MoonArrivalVinf(double mu, double rp, double rMoon, double vMoon)
        {
            double vAp = Math.Sqrt(mu * 2 * rp / (rMoon * (rMoon + rp)));
            return Math.Abs(vMoon - vAp);
        }

        /// <summary>The lowest capture periapsis (>= rcMin) at which an
        /// engine of accel sheds vinf into the ellipse of apoapsis raMax (or
        /// raLoose) in one pass.  False if none.</summary>
        public static bool CaptureOrbit(Body body, double vinf, double accel, double rcMin, double raMax,
                                        double arc, double fill, double? raLoose, out double rcOut,
                                        out double raOut)
        {
            vinf = Math.Max(vinf, 1.0);
            double[] ras = raLoose.HasValue ? new[] { raMax, raLoose.Value } : new[] { raMax };
            foreach (double ra in ras)
            {
                double rc = rcMin;
                while (rc < 0.5 * ra)
                {
                    double need = CaptureDv(body.mu, rc, ra, vinf) / accel;
                    if (need <= fill * PassTime(body.mu, rc, vinf, arc)) { rcOut = rc; raOut = ra; return true; }
                    rc *= 1.1;
                }
            }
            rcOut = raOut = 0;
            return false;
        }

        /// <summary>d(periapsis radius)/d(impulse), numerically.</summary>
        public static V3 RpGradient(V3 r, V3 v, double mu, out Conic c)
        {
            c = Orbit.Conic(r, v, mu);
            var g = new double[3];
            for (int k = 0; k < 3; k++)
            {
                Conic ck = Orbit.Conic(r, v + V3.Axis(k, 0.01), mu);
                g[k] = (ck.rp - c.rp) / 0.01;
            }
            return new V3(g[0], g[1], g[2]);
        }

        /// <summary>Speed to shed at the conic's periapsis to be bound below
        /// ra.</summary>
        public static double CaptureNeed(Conic c, double mu, double ra)
        {
            double rp = c.rp;
            double vPe = Math.Sqrt(Math.Max(0.0, 2 * c.energy + 2 * mu / rp));
            return Math.Max(0.0, vPe - Math.Sqrt(mu * (2 / rp - 2 / (rp + ra))));
        }

        public static double ConicPassTime(Conic c, double mu, double arc)
        {
            if (c.e >= 1) return PassTime(mu, c.rp, Math.Max(c.vinf, 1e-3), arc);
            double E = 2 * Math.Atan(Math.Sqrt((1 - c.e) / (1 + c.e)) * Math.Tan(arc / 2));
            return 2 * (E - c.e * Math.Sin(E)) / Math.Sqrt(mu / Math.Pow(c.a, 3));
        }

        /// <summary>Bound below raCap -- or, once past periapsis, below raOk
        /// -- with the periapsis above the floor.</summary>
        public static bool Captured(V3 r, V3 v, Conic c, Capture cap)
        {
            return c.e < 1 && c.rp >= cap.floor
                   && (c.ra <= cap.raCap * 1.01 || V3.Dot(r, v) > 0 && c.ra <= cap.raOk);
        }

        /// <summary>Outbound, unbound and past raOk: this pass is gone.</summary>
        public static bool CaptureLost(V3 r, V3 v, Conic c, Capture cap)
        {
            return c.e >= 1 && V3.Dot(r, v) > 0 && c.r > cap.raOk;
        }

        /// <summary>The capture law's timing: throttle &gt; 0 to burn now, or
        /// 0 and the seconds it can still coast (wait).  A capture that fits
        /// one pass burns centred on periapsis; a longer one starts once the
        /// energy to shed reaches k of what full thrust removes on the path
        /// left to periapsis, and throttles at that ratio.</summary>
        public static double CaptureStep(V3 r, V3 v, Conic c, Capture cap, double accel, bool started,
                                         out double wait)
        {
            wait = 0.0;
            if (V3.Dot(r, v) >= 0 || started && c.rp < cap.floor) return 1.0;
            if (cap.floor <= c.rp && c.rp <= cap.rpMax)
            {
                double need = CaptureNeed(c, cap.mu, cap.raCap) / accel;
                if (need <= cap.fill * ConicPassTime(c, cap.mu, cap.arc))
                {
                    double w = Orbit.TimeToPeriapsis(c, cap.mu) - need / 2;
                    if (w <= 0) return 1.0;
                    wait = w;
                    return 0.0;
                }
            }
            double excess = c.energy + cap.mu / (c.rp + cap.raOk);
            double path = Math.Max(c.r - c.rp, 1.0);
            double q = excess / (accel * path);
            if (q >= cap.k || started && q > 0) return Math.Min(1.0, Math.Max(q, 0.05));
            if (excess <= 0)
            {
                wait = Math.Max(1.0, Orbit.TimeToPeriapsis(c, cap.mu));
                return 0.0;
            }
            double spare = path - excess / (cap.k * accel);
            double vr = -V3.Dot(r, v) / Math.Max(c.r, 1.0);
            wait = Math.Max(1.0, spare / Math.Max(vr, 1e-3));
            return 0.0;
        }

        /// <summary>The capture law's thrust, relative to the destination:
        /// the impulse it wants now.  Retrograde (at most budget), less its
        /// part along the periapsis gradient at or below the aimed periapsis,
        /// plus whatever keeps the periapsis in the band.</summary>
        public static V3 CaptureCommand(V3 r, V3 v, Conic c, Capture cap, double budget)
        {
            double mu = cap.mu;
            Conic c0;
            V3 g = RpGradient(r, v, mu, out c0);
            double gg = V3.Dot(g, g);
            V3 back = v.Unit * -1.0;
            if (cap.rc.HasValue && c.rp <= cap.rc.Value && gg > 0 && V3.Dot(back, g) < 0)
            {
                V3 side = back - g * (V3.Dot(back, g) / gg);
                if (side.Norm > 0.2) back = side.Unit;
            }
            V3 want = back * Math.Min(Math.Max(CaptureNeed(c, mu, cap.raCap), 0.1), budget);
            double? goal = c.rp < cap.floor ? 1.05 * cap.floor : (c.rp > cap.rpMax ? 0.95 * cap.rpMax : (double?)null);
            if (c.rp < cap.floor) want = V3.Zero;
            if (goal.HasValue && gg > 0) want = want + g * ((goal.Value - c.rp) / gg);
            return want;
        }

        /// <summary>The longest thrusting step of a capture: 5% of the speed.</summary>
        public static double CaptureStepS(V3 v, double accel)
        {
            return Math.Max(1.0, 0.05 * v.Norm / accel);
        }

        /// <summary>Fly the capture law from (r, v) relative to the
        /// destination until captured: its dv, and when and how far out it
        /// first burned -- or null (lost, crashed).</summary>
        public static CaptureFlight FlyCapture(V3 r, V3 v, Capture cap, double accel)
        {
            double mu = cap.mu;
            double dv = 0.0, t = 0.0;
            double[] start = null;
            Conic c = Orbit.Conic(r, v, mu);
            double limit = Orbit.TimeToPeriapsis(c, mu) + 20 * Math.Max(Math.Max(
                c.e >= 1 ? c.vinf : 0.0, CaptureNeed(c, mu, cap.raCap)), 10.0) / accel;
            while (true)
            {
                c = Orbit.Conic(r, v, mu);
                if (Captured(r, v, c, cap)) return new CaptureFlight { dv = dv, tStart = start[0], dStart = start[1] };
                if (c.r < cap.surface || CaptureLost(r, v, c, cap) || t > limit) return null;
                double a = accel;
                double step = 0.02 * c.r / Math.Max(v.Norm, 1.0);
                double wait;
                double rho = CaptureStep(r, v, c, cap, a, start != null, out wait);
                if (rho <= 0)
                {
                    double dt = Math.Min(wait, Math.Max(1.0, step));
                    V3 r1, v1;
                    Orbit.Propagate(r, v, mu, dt, out r1, out v1);
                    r = r1; v = v1;
                    t += dt;
                    continue;
                }
                if (start == null) start = new[] { t, c.r };
                double h = Math.Max(0.5, Math.Min(Math.Min(Math.Min(0.5 * step, 600.0), CaptureStepS(v, a)),
                                                  CaptureNeed(c, mu, cap.raCap) / a));
                V3 want = CaptureCommand(r, v, c, cap, rho * a * h);
                double mag = want.Norm;
                double throttle = Math.Min(1.0, mag / (a * h));
                Sim.Rk4(ref r, ref v, 1.0, mu, want.Unit * (a * throttle), h);
                dv += a * throttle * h;
                t += h;
            }
        }

        /// <summary>An arrival's capture (periapsis rc, excess vinf), flown by
        /// the law from well before it starts: (seconds before periapsis it
        /// starts, dv, distance out), or null.</summary>
        public static double[] CaptureStart(Capture cap, double rc, double vinf, double accel)
        {
            double mu = cap.mu;
            double vp = Math.Sqrt(vinf * vinf + 2 * mu / rc);
            double excess = vinf * vinf / 2 + mu / (rc + cap.raOk);
            double d0 = 1.5 * excess / (cap.k * accel) + 2 * cap.raOk;
            double tau = d0 / Math.Max(vinf, 1.0);
            V3 r, v;
            Orbit.Propagate(new V3(rc, 0.0, 0.0), new V3(0.0, vp, 0.0), mu, -tau, out r, out v);
            CaptureFlight got = FlyCapture(r, v, cap, accel);
            if (got == null) return null;
            return new[] { tau - got.tStart, got.dv, got.dStart };
        }

        /// <summary>Whether the push at periapsis rp from the ellipse (rp, ra)
        /// to one reaching rTarget fits one pass.</summary>
        public static bool PushFits(double mu, double rp, double ra, double rTarget, double accel, double arc,
                                    double fill)
        {
            double v0 = Math.Sqrt(mu * (2 / rp - 2 / (rp + ra)));
            double v1 = Math.Sqrt(mu * (2 / rp - 2 / (rp + rTarget)));
            double e = (ra - rp) / (ra + rp);
            double a = (ra + rp) / 2;
            double E = 2 * Math.Atan(Math.Sqrt((1 - e) / (1 + e)) * Math.Tan(arc / 2));
            double t = 2 * (E - e * Math.Sin(E)) / Math.Sqrt(mu / Math.Pow(a, 3));
            return (v1 - v0) / accel <= fill * t;
        }

        /// <summary>Whether radius r is outside every other moon's band.</summary>
        public static bool ClearOf(double r, IList<Body> others, double margin)
        {
            foreach (Body b in others)
            {
                double lo = b.sma * (1 - b.ecc) - margin * b.Soi;
                double hi = b.sma * (1 + b.ecc) + margin * b.Soi;
                if (lo < r && r < hi) return false;
            }
            return true;
        }
    }
}
