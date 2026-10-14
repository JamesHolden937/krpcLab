// The within-SOI law: the Hohmann transfer split into many small burns.
//
// Every tick, from the current orbit alone, it answers: thrust or coast,
// prograde or retrograde, how hard.  There is no stored schedule.
//
// Decoupled: at the periapsis arc the apoapsis is driven to its target, at
// the apoapsis arc the periapsis.  The two arcs sit on an apse line held
// fixed in inertial space (Target.apse) through each burn, and the law
// controls the orbit's radius toward those two directions,
// r(phi) = p / (1 + e.u), smooth through circular; between burns the line is
// re-locked to the real periapsis.  The arcs have half-width acos(eta_cut);
// a burn starts half its duration before its apsis.  Outward first and
// inward first, as Hohmann.  Plane change: normal or anti-normal arcs at the
// nodes, on the biggest orbit of the transfer; where a node arc and an apsis
// arc overlap the two burns are one vector.

using System;

namespace LowThrust
{
    /// <summary>ra, rp: radii, m; inc, lan: rad (null = free).  apse: the
    /// in-plane inertial unit vector the periapsis arc is centred on, normal
    /// the target plane's unit normal.  holdApse: the periapsis must lie
    /// along apse (never re-locked; arcs at +-90 deg correct the
    /// eccentricity across the line at the end).  argpe (rad): the
    /// periapsis this far past the ascending node of the target plane.</summary>
    public class Target
    {
        public double? ra, rp, inc, lan, argpe;
        public V3? apse, normal;
        public bool holdApse;

        public Target(double? ra, double? rp, double? inc = null, double? lan = null, V3? apse = null,
                      V3? normal = null, bool holdApse = false, double? argpe = null)
        {
            this.ra = ra; this.rp = rp; this.inc = inc; this.lan = lan; this.apse = apse;
            this.normal = normal; this.holdApse = holdApse; this.argpe = argpe;
        }

        public Target Copy() { return (Target)MemberwiseClone(); }
    }

    public class Command
    {
        public bool done;
        public double throttle, prograde, normal, eta, err, need;
        public Command(bool done, double throttle, double prograde, double normal, double eta, double err,
                       double need = 0.0)
        {
            this.done = done; this.throttle = throttle; this.prograde = prograde; this.normal = normal;
            this.eta = eta; this.err = err; this.need = need;
        }
    }

    public static class Guidance
    {
        /// <summary>A burn: seconds needed at full thrust, sign, eta, err,
        /// seconds until its apsis or node.</summary>
        class Arc { public double need, sign, eta, err, wait; }

        /// <summary>The target with its apse line set: re-locked to the
        /// osculating periapsis whenever the vessel is coasting, held
        /// through a burn.</summary>
        public static Target LockApse(Target target, Elements el, bool coasting = true)
        {
            if (target.apse.HasValue && (target.holdApse || !coasting)) return target;
            if (el.e > 1e-6) { var t = target.Copy(); t.apse = el.evec.Unit; return t; }
            if (!target.apse.HasValue) { var t = target.Copy(); t.apse = el.rvec.Unit; return t; }
            return target;
        }

        /// <summary>LockApse, and the target plane fixed once (with the
        /// node free, the current plane's; on an equatorial orbit, at the
        /// apse line when the plane change may begin).</summary>
        public static Target Lock(Target target, Elements el, Config cfg, bool coasting = true)
        {
            target = LockApse(target, el, coasting);
            if (target.inc.HasValue && !target.normal.HasValue)
            {
                double lan;
                if (target.lan.HasValue) lan = target.lan.Value;
                else
                {
                    double inc0;
                    Orbit.InclinationLan(el.rvec, el.vvec, out inc0, out lan);
                    if (Math.Sin(inc0) < 1e-6)
                    {
                        V3 reference = target.apse.Value;
                        if (target.argpe.HasValue)
                        {
                            double w = target.argpe.Value;
                            lan = Math.Atan2(reference.y, reference.x) - Math.Atan2(
                                Math.Cos(target.inc.Value) * Math.Sin(w), Math.Cos(w));
                        }
                        else if (!(coasting && PlaneMayStart(el, target, cfg))) return target;
                        else lan = Math.Atan2(reference.y, reference.x);
                    }
                }
                var t = target.Copy();
                t.normal = Orbit.PlaneNormal(target.inc.Value, lan);
                target = t;
            }
            if (target.argpe.HasValue && !target.holdApse) target = LockArgpe(target, el);
            return target;
        }

        static Target LockArgpe(Target target, Elements el)
        {
            V3 n;
            if (target.normal.HasValue) n = target.normal.Value;
            else if (!target.inc.HasValue) n = V3.Cross(el.rvec, el.vvec).Unit;
            else return target;
            V3 node = V3.Cross(new V3(0.0, 0.0, 1.0), n);
            if (node.Norm < 1e-6)
            {
                double lan = target.lan ?? 0.0;
                node = new V3(Math.Cos(lan), Math.Sin(lan), 0.0);
            }
            node = node.Unit;
            double w = target.argpe.Value;
            V3 apse = node * Math.Cos(w) + V3.Cross(n, node) * Math.Sin(w);
            var t = target.Copy();
            t.apse = apse.Unit;
            t.holdApse = true;
            return t;
        }

        static bool PlaneMayStart(Elements el, Target target, Config cfg)
        {
            return !target.ra.HasValue || el.ra >= target.ra.Value - cfg.TOLERANCE_M;
        }

        /// <summary>Angle (rad) between the orbit plane and the target
        /// plane; 0 if free.</summary>
        public static double PlaneError(Elements el, Target target)
        {
            if (!target.normal.HasValue)
            {
                if (!target.inc.HasValue) return 0.0;
                return Math.Abs(Orbit.Inclination(el.rvec, el.vvec) - target.inc.Value);
            }
            V3 n = V3.Cross(el.rvec, el.vvec).Unit;
            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(n, target.normal.Value))));
        }

        static V3 InPlane(Elements el, V3 direction)
        {
            V3 n = V3.Cross(el.rvec, el.vvec).Unit;
            return (direction - n * V3.Dot(direction, n)).Unit;
        }

        public static double ApseError(Elements el, Target target)
        {
            if (!target.holdApse || el.e < 0.01) return 0.0;
            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, V3.Dot(el.evec.Unit, InPlane(el, target.apse.Value)))));
        }

        public static bool Arrived(Elements el, Target target, Config cfg)
        {
            return (!target.ra.HasValue || Math.Abs(el.ra - target.ra.Value) < cfg.TOLERANCE_M)
                   && (!target.rp.HasValue || Math.Abs(el.rp - target.rp.Value) < cfg.TOLERANCE_M)
                   && PlaneError(el, target) < Py.Radians(cfg.PLANE_TOLERANCE_DEG)
                   && ApseError(el, target) < Py.Radians(cfg.APSE_TOLERANCE_DEG);
        }

        static double Wrap(double x) { return Math.Atan2(Math.Sin(x), Math.Cos(x)); }

        static Arc Apsis(Elements el, Target target, double accel, double half, Config cfg, double mu,
                         bool planePending)
        {
            V3 apse = target.holdApse ? InPlane(el, target.apse.Value) : target.apse.Value;
            double theta = Orbit.PlaneAngle(el.rvec, el.vvec, apse);
            double dAp = Wrap(theta - Math.PI);
            // at_pe: true this arc moves the apoapsis, false the periapsis,
            // null the +-90 deg arcs (no Hohmann ordering there)
            bool? atPe = Math.Abs(theta) <= Math.Abs(dAp);
            double off;
            double? goal;
            V3 far;
            if (atPe.Value) { off = theta; goal = target.ra; far = apse * -1.0; }
            else { off = dAp; goal = target.rp; far = apse; }
            if (target.holdApse && Math.Abs(off) > half && target.ra.HasValue && target.rp.HasValue)
            {
                V3 side = V3.Cross(V3.Cross(el.rvec, el.vvec).Unit, apse);
                double rPe = Orbit.RadiusToward(el.p, el.evec, apse);
                double rAp = Orbit.RadiusToward(el.p, el.evec, apse * -1.0);
                double gate = cfg.APSE_ARC_GATE;
                if (Math.Abs(rPe - target.rp.Value) < gate * target.rp.Value
                    && Math.Abs(rAp - target.ra.Value) < gate * target.ra.Value)
                {
                    double dQ = Wrap(theta - Math.PI / 2);
                    double dMq = Wrap(theta + Math.PI / 2);
                    if (Math.Abs(dQ) <= half) { off = dQ; far = side * -1.0; }
                    else if (Math.Abs(dMq) <= half) { off = dMq; far = side; }
                    else return null;
                    goal = 2 * target.ra.Value * target.rp.Value / (target.ra.Value + target.rp.Value);
                    atPe = null;
                }
            }
            if (Math.Abs(off) > half || !goal.HasValue) return null;

            double tol = cfg.TOLERANCE_M;
            if (atPe == false && target.ra.HasValue)
                if (el.rp < goal.Value && el.ra < target.ra.Value - tol) return null;
            if (atPe == true && el.ra > goal.Value)
            {
                if (target.rp.HasValue && el.rp > target.rp.Value + tol) return null;
                if (planePending) return null;
            }

            V3 T = Orbit.Transverse(el.rvec, el.vvec);
            double radius = Orbit.RadiusToward(el.p, el.evec, far);
            double err = radius - goal.Value;
            Elements bump = Orbit.Elements(el.rvec, el.vvec + T * cfg.RATE_DV, mu);
            double rate = (Orbit.RadiusToward(bump.p, bump.evec, far) - radius) / cfg.RATE_DV;
            if (rate == 0 || Math.Abs(err) < tol) return null;
            double sign = err * rate > 0 ? -1 : 1;
            double need = Math.Abs(err) / (Math.Abs(rate) * accel);
            double wait = off < 0 ? Orbit.TimeTo(el, el.nu - off, mu) : 0.0;
            return new Arc { need = need, sign = sign, eta = Math.Cos(off), err = err, wait = wait };
        }

        static Arc Plane(Elements el, Target target, double accel, double half, double etaCut, Config cfg,
                         double mu)
        {
            double delta = PlaneError(el, target);
            if (delta < Py.Radians(cfg.PLANE_TOLERANCE_DEG) || !target.normal.HasValue) return null;
            if (!PlaneMayStart(el, target, cfg)) return null;
            V3 n = V3.Cross(el.rvec, el.vvec).Unit;
            V3 nodes = V3.Cross(n, target.normal.Value).Unit;
            double phi = Orbit.PlaneAngle(el.rvec, el.vvec, nodes);
            double off;
            V3 node;
            if (Math.Abs(phi) <= Math.PI / 2) { off = phi; node = nodes; }
            else { off = Math.Atan2(Math.Sin(phi - Math.PI), Math.Cos(phi - Math.PI)); node = nodes * -1.0; }
            double rNode = Orbit.RadiusToward(el.p, el.evec, node);
            double rOther = Orbit.RadiusToward(el.p, el.evec, node * -1.0);
            double weight = rNode / Math.Max(rNode, rOther);
            if (Math.Abs(off) > half || weight < etaCut) return null;
            if (target.rp.HasValue)
            {
                double rpNow = target.holdApse
                    ? Orbit.RadiusToward(el.p, el.evec, InPlane(el, target.apse.Value))
                    : el.rp;
                if (Math.Abs(rpNow - target.rp.Value) > cfg.TOLERANCE_M)
                {
                    double theta = Orbit.PlaneAngle(el.rvec, el.vvec, target.apse.Value);
                    if (Math.Abs(theta) < Math.PI / 2 - half) return null;
                }
            }
            V3 tilt = (target.normal.Value - n * Math.Cos(delta)).Unit;
            V3 T = Orbit.Transverse(el.rvec, el.vvec);
            double sign = V3.Dot(T, tilt) < 0 ? 1 : -1;
            double need = delta * el.h / (rNode * accel);
            double wait = off < 0 ? Orbit.TimeTo(el, el.nu - off, mu) : 0.0;
            return new Arc { need = need, sign = sign, eta = Math.Cos(off) * weight, err = 0.0, wait = wait };
        }

        /// <summary>This tick's thrust command: el the current elements,
        /// target locked (Lock), accel the engine's acceleration at full
        /// throttle now, tickS how long it will be held, etaCut the arc
        /// cutoff.</summary>
        public static Command Command(Elements el, Target target, double accel, double tickS, double etaCut,
                                      Config cfg, double mu)
        {
            if (Arrived(el, target, cfg)) return new Command(true, 0.0, 0.0, 0.0, 1.0, 0.0);
            double half = Math.Acos(Math.Max(-1.0, Math.Min(1.0, etaCut)));
            bool pending = PlaneError(el, target) >= Py.Radians(cfg.PLANE_TOLERANCE_DEG);
            Arc a = Apsis(el, target, accel, half, cfg, mu, pending);
            Arc p = Plane(el, target, accel, half, etaCut, cfg, mu);
            double err = a != null ? a.err : 0.0;
            double total = Py.Hypot(a != null ? a.need : 0.0, p != null ? p.need : 0.0);
            if (a != null && !(a.wait <= total / 2)) a = null;
            if (p != null && !(p.wait <= total / 2)) p = null;
            double along = a != null ? a.need * a.sign : 0.0;
            double across = p != null ? p.need * p.sign : 0.0;
            double need = Py.Hypot(along, across);
            if (need == 0) return new Command(false, 0.0, 0.0, 0.0, 0.0, err);
            double eta = Math.Max(a != null ? a.eta : 0.0, p != null ? p.eta : 0.0);
            return new Command(false, Math.Min(1.0, need / tickS), along / need, across / need, eta, err, need);
        }
    }
}
