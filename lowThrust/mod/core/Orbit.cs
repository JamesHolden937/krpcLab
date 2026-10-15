// Two-body orbit maths: elements from a state, exact coasting, radii toward
// fixed directions, the impulsive reference.  Pure functions.
//
// Vectors are in an inertial frame centred on the body, x-y its equator, z
// its north pole.  Elements() is for bound orbits (null otherwise); Conic(),
// Propagate(), TimeToPeriapsis() and Lambert() take any conic.

using System;

namespace LowThrust
{
    /// <summary>Osculating elements of a bound orbit.</summary>
    public class Elements
    {
        public double a, e, p, h, ra, rp, nu, r, period;
        public V3 rvec, vvec, evec;
    }

    /// <summary>Any conic: a &lt; 0 and ra = period = inf for a hyperbola; nu
    /// in -pi..pi (negative inbound); vinf the hyperbolic excess speed (0 if
    /// bound).</summary>
    public class Conic
    {
        public double a, e, p, h, rp, ra, nu, r, period, energy, vinf;
        public V3 rvec, vvec, evec;
    }

    public static class Orbit
    {
        public static Elements Elements(V3 r, V3 v, double mu)
        {
            double rmag = r.Norm;
            V3 hvec = V3.Cross(r, v);
            double h = hvec.Norm;
            double energy = V3.Dot(v, v) / 2 - mu / rmag;
            if (energy >= 0 || h == 0) return null;
            double a = -mu / (2 * energy);
            V3 evec = V3.Cross(v, hvec) * (1 / mu) - r * (1 / rmag);
            double e = evec.Norm;
            double p = h * h / mu;
            double nu;
            if (e > 1e-10)
            {
                double cosnu = V3.Dot(evec, r) / (e * rmag);
                nu = Math.Acos(Math.Max(-1.0, Math.Min(1.0, cosnu)));
                if (V3.Dot(r, v) < 0) nu = 2 * Math.PI - nu;
            }
            else nu = 0.0;
            return new Elements
            {
                a = a, e = e, p = p, h = h, ra = a * (1 + e), rp = a * (1 - e), nu = nu,
                r = rmag, period = 2 * Math.PI * Math.Sqrt(Math.Pow(a, 3) / mu),
                rvec = r, vvec = v, evec = evec
            };
        }

        /// <summary>Unit radial, transverse (along-track, in plane) and
        /// normal vectors.</summary>
        public static void Rtn(V3 r, V3 v, out V3 R, out V3 T, out V3 N)
        {
            R = r.Unit;
            N = V3.Cross(r, v).Unit;
            T = V3.Cross(N, R);
        }

        public static V3 Transverse(V3 r, V3 v)
        {
            V3 R, T, N;
            Rtn(r, v, out R, out T, out N);
            return T;
        }

        /// <summary>The orbit's radius in inertial direction u (a unit vector
        /// in the plane): p / (1 + e.u).  Smooth through circular.</summary>
        public static double RadiusToward(double p, V3 evec, V3 direction)
        {
            return p / (1 + V3.Dot(evec, direction));
        }

        /// <summary>Angle (rad, -pi..pi) from in-plane unit ref to position r,
        /// positive in the direction of motion.</summary>
        public static double PlaneAngle(V3 r, V3 v, V3 reference)
        {
            V3 N = V3.Cross(r, v).Unit;
            V3 u = r.Unit;
            return Math.Atan2(V3.Dot(V3.Cross(reference, u), N), V3.Dot(reference, u));
        }

        static double MeanAnomaly(double e, double nu)
        {
            double E = 2 * Math.Atan2(Math.Sqrt(1 - e) * Math.Sin(nu / 2),
                                      Math.Sqrt(1 + e) * Math.Cos(nu / 2));
            return E - e * Math.Sin(E);
        }

        /// <summary>Seconds from the current true anomaly forward to nuTo.</summary>
        public static double TimeTo(Elements el, double nuTo, double mu)
        {
            double dM = Py.Mod(MeanAnomaly(el.e, nuTo) - MeanAnomaly(el.e, el.nu), 2 * Math.PI);
            return dM / (2 * Math.PI) * el.period;
        }

        public static Conic Conic(V3 r, V3 v, double mu)
        {
            double rmag = r.Norm;
            V3 hvec = V3.Cross(r, v);
            double h = hvec.Norm;
            double energy = V3.Dot(v, v) / 2 - mu / rmag;
            V3 evec = V3.Cross(v, hvec) * (1 / mu) - r * (1 / rmag);
            double e = evec.Norm;
            double p = h * h / mu;
            double nu;
            if (e > 1e-10)
            {
                nu = Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(evec, r) / (e * rmag))));
                if (V3.Dot(r, v) < 0) nu = -nu;
            }
            else nu = 0.0;
            bool bound = energy < 0;
            double a = energy != 0 ? -mu / (2 * energy) : Py.Inf;
            return new Conic
            {
                a = a, e = e, p = p, h = h, rp = p / (1 + e), ra = bound ? a * (1 + e) : Py.Inf,
                nu = nu, r = rmag,
                period = bound ? 2 * Math.PI * Math.Sqrt(Math.Pow(a, 3) / mu) : Py.Inf,
                energy = energy, vinf = bound ? 0.0 : Math.Sqrt(2 * energy),
                rvec = r, vvec = v, evec = evec
            };
        }

        /// <summary>Seconds from now to the next periapsis (a hyperbola
        /// outbound: the one just passed, negative).</summary>
        public static double TimeToPeriapsis(Conic c, double mu)
        {
            if (c.e < 1)
            {
                var el = new Elements
                {
                    a = c.a, e = c.e, p = c.p, h = c.h, ra = c.ra, rp = c.rp,
                    nu = Py.Mod(c.nu, 2 * Math.PI), r = c.r, period = c.period,
                    rvec = c.rvec, vvec = c.vvec, evec = c.evec
                };
                return TimeTo(el, 0.0, mu);
            }
            double F = 2 * Py.Atanh(Math.Sqrt((c.e - 1) / (c.e + 1)) * Math.Tan(c.nu / 2));
            double M = c.e * Math.Sinh(F) - F;
            return -M / Math.Sqrt(mu / Math.Pow(-c.a, 3));
        }

        /// <summary>Seconds from now until a hyperbola reaches radius R on
        /// its way out (0 if already past it outbound; inf if bound).</summary>
        public static double TimeToRadiusOut(Conic c, double mu, double R)
        {
            if (c.e <= 1) return Py.Inf;
            if (c.r >= R && c.nu >= 0) return 0.0;
            double n = Math.Sqrt(mu / Math.Pow(-c.a, 3));
            double F0 = 2 * Py.Atanh(Math.Sqrt((c.e - 1) / (c.e + 1)) * Math.Tan(c.nu / 2));
            double ch = (1 + R / -c.a) / c.e;
            double F1 = Math.Log(ch + Math.Sqrt(ch * ch - 1));
            return ((c.e * Math.Sinh(F1) - F1) - (c.e * Math.Sinh(F0) - F0)) / n;
        }

        static void Stumpff(double z, out double C, out double S)
        {
            if (z > 1e-8)
            {
                double sz = Math.Sqrt(z);
                C = (1 - Math.Cos(sz)) / z;
                S = (sz - Math.Sin(sz)) / Math.Pow(sz, 3);
            }
            else if (z < -1e-8)
            {
                double sz = Math.Sqrt(-z);
                C = (Math.Cosh(sz) - 1) / -z;
                S = (Math.Sinh(sz) - sz) / Math.Pow(sz, 3);
            }
            else
            {
                C = 0.5 - z / 24;
                S = 1.0 / 6 - z / 120;
            }
        }

        /// <summary>Propagate any conic dt seconds exactly (universal
        /// variables).</summary>
        public static void Propagate(V3 r, V3 v, double mu, double dt, out V3 r1, out V3 v1)
        {
            if (dt == 0) { r1 = r; v1 = v; return; }
            double r0 = r.Norm;
            double vr0 = V3.Dot(r, v) / r0;
            double alpha = 2 / r0 - V3.Dot(v, v) / mu;
            double sqmu = Math.Sqrt(mu);
            Func<double, double> residual = xx =>
            {
                double zz = alpha * xx * xx;
                double CC, SS;
                Stumpff(zz, out CC, out SS);
                double res = Math.Abs(r0 * vr0 / sqmu * xx * xx * CC
                                      + (1 - alpha * r0) * Math.Pow(xx, 3) * SS + r0 * xx - sqmu * dt);
                return double.IsNaN(res) ? Py.Inf : res;
            };
            double x = sqmu * dt / r0;
            if (alpha > 1e-12)
                x = sqmu * alpha * dt;
            else if (alpha < -1e-12)
            {
                double a = 1 / alpha;
                double sgn = dt > 0 ? 1 : -1;
                double arg = (-2 * mu * alpha * dt)
                             / (V3.Dot(r, v) + sgn * Math.Sqrt(-mu * a) * (1 - r0 * alpha));
                if (arg > 0)
                {
                    double xLong = sgn * Math.Sqrt(-a) * Math.Log(arg);
                    if (residual(xLong) < residual(x)) x = xLong;
                }
            }
            for (int i = 0; i < 200; i++)
            {
                double zz = alpha * x * x;
                double CC, SS;
                Stumpff(zz, out CC, out SS);
                double F = r0 * vr0 / sqmu * x * x * CC + (1 - alpha * r0) * Math.Pow(x, 3) * SS
                           + r0 * x - sqmu * dt;
                double dF = r0 * vr0 / sqmu * x * (1 - zz * SS) + (1 - alpha * r0) * x * x * CC + r0;
                double step = F / dF;
                x -= step;
                if (Math.Abs(step) < 1e-9 * Math.Max(1.0, Math.Abs(x))) break;
            }
            double z = alpha * x * x;
            double C, S;
            Stumpff(z, out C, out S);
            double f = 1 - x * x / r0 * C;
            double g = dt - Math.Pow(x, 3) * S / sqmu;
            r1 = r * f + v * g;
            double rm = r1.Norm;
            double fd = sqmu / (rm * r0) * (alpha * Math.Pow(x, 3) * S - x);
            double gd = 1 - x * x / rm * C;
            v1 = r * fd + v * gd;
        }

        /// <summary>Velocities (v1, v2) of the single-revolution conic from
        /// r1 to r2 in dt seconds, prograde about normal; false if none.
        /// Universal variables, bisection on z (Curtis, algorithm 5.2).</summary>
        public static bool Lambert(V3 r1, V3 r2, double dt, double mu, V3 normal, out V3 v1, out V3 v2)
        {
            v1 = v2 = V3.Zero;
            double m1 = r1.Norm, m2 = r2.Norm;
            double cosT = Math.Max(-1.0, Math.Min(1.0, V3.Dot(r1, r2) / (m1 * m2)));
            double theta = Math.Acos(cosT);
            if (V3.Dot(V3.Cross(r1, r2), normal) < 0) theta = 2 * Math.PI - theta;
            if (Math.Abs(1 - Math.Cos(theta)) < 1e-12) return false;
            double A = Math.Sin(theta) * Math.Sqrt(m1 * m2 / (1 - Math.Cos(theta)));
            if (Math.Abs(A) < 1e-9) return false;
            double sqmu = Math.Sqrt(mu);

            Func<double, double> y = zz =>
            {
                double CC, SS;
                Stumpff(zz, out CC, out SS);
                return m1 + m2 + A * (zz * SS - 1) / Math.Sqrt(CC);
            };
            Func<double, double> F = zz =>
            {
                double CC, SS;
                Stumpff(zz, out CC, out SS);
                double yz2 = y(zz);
                return Math.Pow(yz2 / CC, 1.5) * SS + A * Math.Sqrt(yz2) - sqmu * dt;
            };

            double hi = 4 * Math.Pow(Math.PI, 2) - 1e-6;
            if (F(hi) < 0) return false;
            double lo = -4 * Math.Pow(Math.PI, 2);
            while (true)
            {
                if (lo < -4e5) return false;
                if (y(lo) < 0)
                {
                    double a_ = lo, b_ = hi;
                    for (int i = 0; i < 200; i++)
                    {
                        double mid = (a_ + b_) / 2;
                        if (y(mid) < 0) a_ = mid; else b_ = mid;
                    }
                    lo = b_;
                    break;
                }
                if (F(lo) < 0) break;
                lo *= 2;
            }
            for (int i = 0; i < 200; i++)
            {
                double mid = (lo + hi) / 2;
                if (F(mid) > 0) hi = mid; else lo = mid;
                if (hi - lo < 1e-12) break;
            }
            double yz = y((lo + hi) / 2);
            double f = 1 - yz / m1;
            double g = A * Math.Sqrt(yz / mu);
            double gd = 1 - yz / m2;
            v1 = (r2 - r1 * f) * (1 / g);
            v2 = (r2 * gd - r1) * (1 / g);
            return true;
        }

        /// <summary>(r, v) on the conic with these elements (rad) at true
        /// anomaly nu.</summary>
        public static void StateFromElements(double mu, double a, double e, double inc, double lan,
                                             double argpe, double nu, out V3 r, out V3 v)
        {
            double p = a * (1 - e * e);
            double rr = p / (1 + e * Math.Cos(nu));
            var pos = new V3(rr * Math.Cos(nu), rr * Math.Sin(nu), 0.0);
            double k = Math.Sqrt(mu / p);
            var vel = new V3(-k * Math.Sin(nu), k * (e + Math.Cos(nu)), 0.0);
            r = Rot(pos, inc, lan, argpe);
            v = Rot(vel, inc, lan, argpe);
        }

        /// <summary>Rz(lan) Rx(inc) Rz(argpe).</summary>
        public static V3 Rot(V3 x, double inc, double lan, double argpe)
        {
            double c = Math.Cos(argpe), s = Math.Sin(argpe);
            x = new V3(c * x.x - s * x.y, s * x.x + c * x.y, x.z);
            c = Math.Cos(inc); s = Math.Sin(inc);
            x = new V3(x.x, c * x.y - s * x.z, s * x.y + c * x.z);
            c = Math.Cos(lan); s = Math.Sin(lan);
            return new V3(c * x.x - s * x.y, s * x.x + c * x.y, x.z);
        }

        /// <summary>True anomaly for mean anomaly M on an ellipse.</summary>
        public static double TrueFromMean(double M, double e)
        {
            double E = e < 0.8 ? M : Math.PI;
            for (int i = 0; i < 50; i++)
            {
                double d = (E - e * Math.Sin(E) - M) / (1 - e * Math.Cos(E));
                E -= d;
                if (Math.Abs(d) < 1e-14) break;
            }
            return 2 * Math.Atan2(Math.Sqrt(1 + e) * Math.Sin(E / 2), Math.Sqrt(1 - e) * Math.Cos(E / 2));
        }

        /// <summary>Propagate an elliptic orbit dt seconds exactly (f and g
        /// series).</summary>
        public static void Coast(V3 r, V3 v, double mu, double dt, out V3 r1, out V3 v1)
        {
            double r0 = r.Norm;
            double a = 1 / (2 / r0 - V3.Dot(v, v) / mu);
            double sqmu = Math.Sqrt(mu), sqa = Math.Sqrt(a);
            double sigma0 = V3.Dot(r, v) / sqmu;
            double M = Math.Sqrt(mu / Math.Pow(a, 3)) * dt;
            double k1 = 1 - r0 / a, k2 = sigma0 / sqa;
            double dE = M;
            for (int i = 0; i < 50; i++)
            {
                double s0 = Math.Sin(dE), c0 = Math.Cos(dE);
                double f0 = dE - k1 * s0 + k2 * (1 - c0) - M;
                double step = f0 / (1 - k1 * c0 + k2 * s0);
                dE -= step;
                if (Math.Abs(step) < 1e-13) break;
            }
            double s = Math.Sin(dE), c = Math.Cos(dE);
            double rr = a + (r0 - a) * c + sigma0 * sqa * s;
            double f = 1 - a / r0 * (1 - c);
            double g = a * sigma0 / sqmu * (1 - c) + r0 * Math.Sqrt(a / mu) * s;
            double fd = -Math.Sqrt(mu * a) / (rr * r0) * s;
            double gd = 1 - a / rr * (1 - c);
            r1 = r * f + v * g;
            v1 = r * fd + v * gd;
        }

        /// <summary>Vis-viva: the speed at radius r on the orbit with these
        /// apsides.</summary>
        public static double SpeedAt(double mu, double r, double ra, double rp)
        {
            return Math.Sqrt(mu * (2 / r - 2 / (ra + rp)));
        }

        /// <summary>Unit orbit normal for inclination and longitude of the
        /// ascending node (rad).</summary>
        public static V3 PlaneNormal(double inc, double lan)
        {
            return new V3(Math.Sin(inc) * Math.Sin(lan), -Math.Sin(inc) * Math.Cos(lan), Math.Cos(inc));
        }

        /// <summary>(inclination, lan) of the orbit through (r, v), rad; lan
        /// is 0 for an equatorial orbit.</summary>
        public static void InclinationLan(V3 r, V3 v, out double inc, out double lan)
        {
            V3 n = V3.Cross(r, v).Unit;
            inc = Math.Acos(Math.Max(-1.0, Math.Min(1.0, n.z)));
            lan = Py.Hypot(n.x, n.y) > 1e-12 ? Math.Atan2(n.x, -n.y) : 0.0;
            lan = Py.Mod(lan, 2 * Math.PI);
        }

        public static double Inclination(V3 r, V3 v)
        {
            double inc, lan;
            InclinationLan(r, v, out inc, out lan);
            return inc;
        }

        /// <summary>Impulsive cost of rotating the orbit from one plane to
        /// the other at the higher of its two nodes.</summary>
        public static double PlaneChangeDv(Elements el, V3 from, V3 to)
        {
            double delta = Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(from, to))));
            if (delta == 0) return 0.0;
            V3 nodes = V3.Cross(from, to).Unit;
            double rNode = Math.Max(RadiusToward(el.p, el.evec, nodes),
                                    RadiusToward(el.p, el.evec, nodes * -1.0));
            return 2 * el.h / rNode * Math.Sin(delta / 2);
        }

        /// <summary>Two-burn impulsive reference for an apsides change
        /// (Hohmann-like), with a plane change of di folded into the burn at
        /// the larger radius.  The yardstick the low-thrust dv is compared
        /// against.</summary>
        public static double ImpulsiveDv(double mu, double ra0, double rp0, double ra1, double rp1,
                                         double di = 0.0)
        {
            Func<double, double, bool, double> burn = (va, vb, plane) =>
                !plane ? Math.Abs(vb - va)
                       : Math.Sqrt(Math.Max(0.0, va * va + vb * vb - 2 * va * vb * Math.Cos(di)));
            Func<double, double, double, double> order = (firstAt, tgtOther, tgtFirst) =>
            {
                bool planeFirst = firstAt >= tgtOther;
                double dv1 = burn(SpeedAt(mu, firstAt, ra0, rp0), SpeedAt(mu, firstAt, firstAt, tgtOther),
                                  planeFirst);
                double dv2 = burn(SpeedAt(mu, tgtOther, tgtOther, firstAt),
                                  SpeedAt(mu, tgtOther, tgtOther, tgtFirst), !planeFirst);
                return dv1 + dv2;
            };
            return Math.Min(order(rp0, ra1, rp1), order(ra0, rp1, ra1));
        }
    }
}
