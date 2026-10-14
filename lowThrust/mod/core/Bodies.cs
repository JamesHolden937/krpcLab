// The solar system the planner flies in: bodies on fixed Keplerian rails.
// In the game it is rebuilt from the game's own bodies (Plugin's System);
// Stock() is the stand-in for the command-line tools and the tests.  All
// bodies' equators are parallel in KSP, so every orbit is in one frame: x-y
// the reference plane, longitudes of the node from +x.  Positions are
// relative to the parent, in that parent's non-rotating frame.

using System;
using System.Collections.Generic;

namespace LowThrust
{
    public class Body
    {
        public string name;
        public double mu, radius, atmosphere;
        public Body parent;
        public double sma, ecc, inc, lan, argpe, meanAnomaly0;   // rad, UT 0
        public List<Body> children = new List<Body>();
        /// <summary>The game's own SOI radius (the rails check), or inf.</summary>
        public double gameSoi = Py.Inf;

        public Body(string name, double mu, double radius, double atmosphere = 0.0, Body parent = null,
                    double sma = 0.0, double ecc = 0.0, double inc = 0.0, double lan = 0.0,
                    double argpe = 0.0, double meanAnomaly0 = 0.0)
        {
            this.name = name; this.mu = mu; this.radius = radius; this.atmosphere = atmosphere;
            this.parent = parent; this.sma = sma; this.ecc = ecc; this.inc = inc; this.lan = lan;
            this.argpe = argpe; this.meanAnomaly0 = meanAnomaly0;
        }

        /// <summary>Sphere of influence radius, a (m / M)^0.4 -- as KSP
        /// computes it.</summary>
        public double Soi
        {
            get { return parent == null ? Py.Inf : sma * Math.Pow(mu / parent.mu, 0.4); }
        }

        public double Period { get { return 2 * Math.PI * Math.Sqrt(Math.Pow(sma, 3) / parent.mu); } }

        /// <summary>(r, v) relative to the parent at UT t.</summary>
        public void State(double t, out V3 r, out V3 v)
        {
            double M = meanAnomaly0 + 2 * Math.PI * t / Period;
            double nu = Orbit.TrueFromMean(Py.Mod(M, 2 * Math.PI), ecc);
            Orbit.StateFromElements(parent.mu, sma, ecc, inc, lan, argpe, nu, out r, out v);
        }

        public V3 Position(double t)
        {
            V3 r, v;
            State(t, out r, out v);
            return r;
        }

        /// <summary>Unit normal of this body's orbit about its parent.</summary>
        public V3 Normal { get { return Orbit.PlaneNormal(inc, lan); } }

        public override string ToString() { return name; }
    }

    public static class Bodies
    {
        /// <summary>The stock system (the bodies the transfers use), by
        /// name: the stand-in's.</summary>
        public static Dictionary<string, Body> Stock()
        {
            var sun = new Body("Sun", 1.1723328e18, 261600000.0);
            var b = new Dictionary<string, Body> { { "Sun", sun } };
            Action<string, string, double, double, double, double, double, double, double, double, double> add =
                (name, parent, mu, radius, atm, sma, ecc, inc, lan, argpe, m0) =>
                {
                    var x = new Body(name, mu, radius, atm, b[parent], sma, ecc, Py.Radians(inc),
                                     Py.Radians(lan), Py.Radians(argpe), m0);
                    b[parent].children.Add(x);
                    b[name] = x;
                };
            add("Moho", "Sun", 1.6860938e11, 250e3, 0, 5263138304, 0.2, 7.0, 70.0, 15.0, 3.14);
            add("Eve", "Sun", 8.1717302e12, 700e3, 90e3, 9832684544, 0.01, 2.1, 15.0, 0.0, 3.14);
            add("Kerbin", "Sun", 3.5316e12, 600e3, 70e3, 13599840256, 0.0, 0.0, 0.0, 0.0, 3.14);
            add("Mun", "Kerbin", 6.5138398e10, 200e3, 0, 12000000, 0.0, 0.0, 0.0, 0.0, 1.7);
            add("Minmus", "Kerbin", 1.7658e9, 60e3, 0, 47000000, 0.0, 6.0, 78.0, 38.0, 0.9);
            add("Duna", "Sun", 3.0136321e11, 320e3, 50e3, 20726155264, 0.051, 0.06, 135.5, 0.0, 3.14);
            add("Ike", "Duna", 1.8568369e10, 130e3, 0, 3200000, 0.03, 0.2, 0.0, 0.0, 1.7);
            add("Gilly", "Eve", 8289449.8, 13e3, 0, 31500000, 0.55, 12.0, 80.0, 10.0, 0.9);
            add("Dres", "Sun", 2.1484489e10, 138e3, 0, 40839348203, 0.145, 5.0, 280.0, 90.0, 3.14);
            add("Jool", "Sun", 2.82528e14, 6000e3, 200e3, 68773560320, 0.05, 1.304, 52.0, 0.0, 0.1);
            add("Laythe", "Jool", 1.962e12, 500e3, 50e3, 27184000, 0.0, 0.0, 0.0, 0.0, 3.14);
            add("Vall", "Jool", 2.074815e11, 300e3, 0, 43152000, 0.0, 0.0, 0.0, 0.0, 0.9);
            add("Tylo", "Jool", 2.82528e12, 600e3, 0, 68500000, 0.0, 0.025, 0.0, 0.0, 3.14);
            add("Bop", "Jool", 2.4868349e9, 65e3, 0, 128500000, 0.235, 15.0, 10.0, 25.0, 0.9);
            add("Pol", "Jool", 7.2170208e8, 44e3, 0, 179890000, 0.171, 4.25, 2.0, 15.0, 0.9);
            add("Eeloo", "Sun", 7.4410815e10, 210e3, 0, 90118820000, 0.26, 6.15, 50.0, 260.0, 3.14);
            return b;
        }

        /// <summary>body's position relative to frameBody (an ancestor, or
        /// the body itself) at UT t.</summary>
        public static V3 PositionIn(Body body, Body frameBody, double t)
        {
            V3 pos = V3.Zero;
            for (Body x = body; x != frameBody; x = x.parent) pos = pos + x.Position(t);
            return pos;
        }

        /// <summary>(a, e, inc, lan, argpe, nu) of the orbit through (r, v):
        /// the inverse of Orbit.StateFromElements.  Circular: argpe 0, nu from
        /// the node; equatorial: lan 0, longitudes from +x.</summary>
        public static double[] Angles(V3 r, V3 v, double mu)
        {
            V3 h = V3.Cross(r, v);
            double hn = h.Norm, rn = r.Norm;
            double inc = Math.Acos(Math.Max(-1.0, Math.Min(1.0, h.z / hn)));
            var node = new V3(-h.y, h.x, 0.0);
            double nn = node.Norm;
            V3 ev = V3.Cross(v, h) * (1 / mu) - r * (1 / rn);
            double e = ev.Norm;
            double a = 1 / (2 / rn - V3.Dot(v, v) / mu);
            double lan;
            V3 reference;
            if (nn < 1e-9 * hn) { lan = 0.0; reference = new V3(1.0, 0.0, 0.0); }
            else { lan = Py.Mod(Math.Atan2(node.y, node.x), 2 * Math.PI); reference = node * (1 / nn); }
            V3 w = V3.Cross(h * (1 / hn), reference);
            Func<V3, double> angle = x => Py.Mod(Math.Atan2(V3.Dot(x, w), V3.Dot(x, reference)), 2 * Math.PI);
            if (e < 1e-9) return new[] { a, 0.0, inc, lan, 0.0, angle(r) };
            double argpe = angle(ev);
            double nu = Py.Mod(angle(r) - argpe, 2 * Math.PI);
            return new[] { a, e, inc, lan, argpe, nu };
        }

        public static double MeanFromTrue(double nu, double e)
        {
            double E = 2 * Math.Atan2(Math.Sqrt(1 - e) * Math.Sin(nu / 2), Math.Sqrt(1 + e) * Math.Cos(nu / 2));
            return Py.Mod(E - e * Math.Sin(E), 2 * Math.PI);
        }

        /// <summary>A body on the rails of the state (r, v) about parent at
        /// UT ut: its elements, mean anomaly carried back to UT 0.</summary>
        public static Body FromState(string name, double mu, double radius, double atm, Body parent,
                                     V3 r, V3 v, double ut)
        {
            double[] el = Angles(r, v, parent.mu);
            double n = Math.Sqrt(parent.mu / Math.Pow(el[0], 3));
            double m0 = Py.Mod(MeanFromTrue(el[5], el[1]) - n * ut, 2 * Math.PI);
            var b = new Body(name, mu, radius, atm, parent, el[0], el[1], el[2], el[3], el[4], m0);
            parent.children.Add(b);
            return b;
        }
    }
}
