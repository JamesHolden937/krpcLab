// A 3-vector of doubles: the planner's only vector type.  Every operation
// is the component formula in the same order the solver's original Python
// (common/vec.py) used, so the two give the same numbers.

using System;

namespace LowThrust
{
    public struct V3
    {
        public readonly double x, y, z;

        public V3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }

        public static readonly V3 Zero = new V3(0.0, 0.0, 0.0);

        public static V3 operator +(V3 a, V3 b) { return new V3(a.x + b.x, a.y + b.y, a.z + b.z); }
        public static V3 operator -(V3 a, V3 b) { return new V3(a.x - b.x, a.y - b.y, a.z - b.z); }
        public static V3 operator *(V3 a, double s) { return new V3(a.x * s, a.y * s, a.z * s); }

        public static double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

        public static V3 Cross(V3 a, V3 b)
        {
            return new V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        }

        public double Norm { get { return Math.Sqrt(x * x + y * y + z * z); } }

        public V3 Unit
        {
            get
            {
                double n = Norm;
                if (n < 1e-12) return Zero;
                return new V3(x / n, y / n, z / n);
            }
        }

        public double this[int i] { get { return i == 0 ? x : (i == 1 ? y : z); } }

        public static V3 Axis(int k, double s)
        {
            return k == 0 ? new V3(s, 0.0, 0.0) : (k == 1 ? new V3(0.0, s, 0.0) : new V3(0.0, 0.0, s));
        }

        public override string ToString() { return string.Format("({0}, {1}, {2})", x, y, z); }
    }

    /// <summary>Python's numeric helpers the solver leans on.</summary>
    public static class Py
    {
        public const double Inf = double.PositiveInfinity;

        /// <summary>Python's float modulo: the result has the divisor's sign.</summary>
        public static double Mod(double a, double b)
        {
            double r = a % b;
            if (r != 0 && (r < 0) != (b < 0)) r += b;
            return r;
        }

        public static double Hypot(double a, double b)
        {
            a = Math.Abs(a); b = Math.Abs(b);
            if (a < b) { double t = a; a = b; b = t; }
            if (a == 0) return 0.0;
            if (double.IsInfinity(a)) return a;
            double q = b / a;
            return a * Math.Sqrt(1 + q * q);
        }

        public static double Atanh(double x) { return 0.5 * Math.Log((1 + x) / (1 - x)); }

        public static double Clamp(double x, double lo, double hi) { return Math.Max(lo, Math.Min(hi, x)); }

        public static double Radians(double d) { return d * (Math.PI / 180.0); }
        public static double Degrees(double r) { return r * (180.0 / Math.PI); }

        /// <summary>Python's round() to an integer (half to even).</summary>
        public static long Round(double x) { return (long)Math.Round(x, MidpointRounding.ToEven); }
    }
}
