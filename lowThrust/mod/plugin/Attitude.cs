// The rotation of the active vessel on rails, integrated from its reaction
// wheels.  On rails the game does not integrate rotation, so this turns the
// vessel's thrust axis toward the commanded direction as its own wheels
// would: the torque the wheels have (pitch/roll/yaw torque, authority
// limiter, only wheels switched on and not broken) and the moment of
// inertia, both measured before the vessel went on rails; the angular rate
// carries from frame to frame; the wheels draw ElectricCharge at the stock
// rate for the input they are given, and with no charge there is no torque.
// The rate is handed back to the rigid bodies when the vessel comes off
// rails.  Off rails it does nothing.
//
// Once the axis is on a slowly turning target (within SETTLED_DEG, the rate
// matched) it is held there exactly, as wheels tracking a target turning at
// orbital rate do, and the frame costs nothing to integrate -- but it is
// charged: the wheels give the torque that turns the vessel's rate with the
// target's (I dw/dt) and holds it against its own gyroscopic torque
// (w x Iw), drawing ElectricCharge for that input.  A hold the wheels cannot
// give (torque or charge short) is let go and the vessel steered again.
//
// Not modelled, so not available: RCS torque and engine gimbal.
//
// The controller runs it first thing in its own physics step (Step), so
// what it decides next sees the vessel as this step leaves it.

using System;
using UnityEngine;

namespace LowThrust
{
    public static class Attitude
    {
        /// <summary>The direction to point the thrust axis, world space;
        /// null: no command (the vessel keeps its rate).</summary>
        internal static Func<Vector3d> target;
        /// <summary>The thrust axis in world space.</summary>
        internal static Func<Vessel, Vector3d> axis;
        // rad/s in ReferenceTransform axes, as the game reports it
        internal static Vector3d omega;
        internal static Vector3d moi;            // t m^2 (x pitch, y roll, z yaw)
        internal static Vector3d alpha;          // rad/s^2 the wheels can give
        internal static double ecRate;           // EC/s at full input on one axis, all wheels
        internal static double ecUsed;
        internal static double ecHold;           // of ecUsed, holding a settled target
        internal static int holdsLost;           // holds the wheels could not give
        internal static double lastError = -1;
        internal static bool settled;
        static Vessel vessel;
        static bool wasPacked;
        static Vector3d lastTarget;
        static bool haveLast;

        // Substep of the rotation integral, s; most substeps a frame.
        const double STEP = 0.05;
        const int MAX_STEPS = 4000;
        // The turn plans to stop with this share of the wheels in reserve.
        const double MARGIN = 0.8;
        // Near the target the rate is the error over SETTLE seconds, and the
        // wheels close a rate error over RESPONSE seconds (no overshoot).
        const double SETTLE = 4.0;
        const double RESPONSE = 0.5;
        // Held exactly once this close, with the rate matched to within this.
        const double SETTLED_DEG = 0.05;
        const double SETTLED_RATE = 1e-3;

        internal static void Reset()
        {
            omega = Vector3d.zero;
            ecUsed = 0;
            ecHold = 0;
            holdsLost = 0;
            target = null;
            settled = false;
            vessel = null;
        }

        /// <summary>One physics step: measure off rails, turn on rails.</summary>
        internal static void Step(Vessel v)
        {
            if (v == null) return;
            if (v != vessel)
            {
                vessel = v;
                omega = Vector3d.zero;
                wasPacked = v.packed;
                haveLast = false;
            }
            if (!v.packed)
            {
                if (wasPacked) HandBack(v);
                Measure(v);
                omega = v.angularVelocity;
                wasPacked = false;
                haveLast = false;
                return;
            }
            wasPacked = true;
            double dt = TimeWarp.fixedDeltaTime;
            if (target == null)
            {
                lastError = -1;
                settled = false;
                haveLast = false;
                if (omega.sqrMagnitude > 1e-12) Rotate(v, omega, dt);
                return;
            }
            Steer(v, dt);
        }

        static void Measure(Vessel v)
        {
            moi = v.MOI;
            Vector3d t = Vector3d.zero;
            double rate = 0;
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                {
                    var w = pm as ModuleReactionWheel;
                    if (w == null || !w.moduleIsEnabled || w.wheelState != ModuleReactionWheel.WheelState.Active)
                        continue;
                    double auth = w.authorityLimiter * 0.01;
                    t += new Vector3d(w.PitchTorque, w.RollTorque, w.YawTorque) * auth;
                    foreach (var r in w.resHandler.inputResources)
                        if (r.name == "ElectricCharge") rate += r.rate * auth;
                }
            ecRate = rate;
            alpha = new Vector3d(moi.x > 0 ? t.x / moi.x : 0, moi.y > 0 ? t.y / moi.y : 0,
                                 moi.z > 0 ? t.z / moi.z : 0);
        }

        static Vector3d AxisBody(Vessel v)
        {
            Vector3d w = axis != null ? axis(v) : (Vector3d)v.ReferenceTransform.up;
            return v.ReferenceTransform.InverseTransformDirection(w).normalized;
        }

        /// <summary>Angle between the thrust axis and the target now, deg
        /// (-1: no target).</summary>
        internal static double ErrorNow(Vessel v)
        {
            if (target == null || v == null) return -1;
            Vector3d a = axis != null ? axis(v) : (Vector3d)v.ReferenceTransform.up;
            return Vector3d.Angle(a, target());
        }

        static void Steer(Vessel v, double dt)
        {
            Transform rt = v.ReferenceTransform;
            Vector3d world = target().normalized;
            Vector3d tb = rt.InverseTransformDirection(world).normalized;
            Vector3d ab = AxisBody(v);
            // the target's own turn rate (world): what holding it takes
            Vector3d targetRate = Vector3d.zero;
            if (haveLast && dt > 0)
            {
                Vector3d c = Vector3d.Cross(lastTarget, world);
                double ang = Math.Atan2(c.magnitude, Vector3d.Dot(lastTarget, world));
                if (c.sqrMagnitude > 0) targetRate = (Vector3d)rt.InverseTransformDirection((Vector3)c.normalized) * (ang / dt);
            }
            lastTarget = world;
            haveLast = true;
            double err = Vector3d.Angle(ab, tb);
            if (settled || (err < SETTLED_DEG && (omega - targetRate).magnitude < SETTLED_RATE))
            {
                if (Charge(v, targetRate, dt))
                {
                    Hold(v, ab, tb, targetRate);
                    return;
                }
                settled = false;
                holdsLost++;
            }

            double need = ecRate * 3 * dt;
            double share = 1;
            if (need > 0) share = Math.Min(1, Available(v, "ElectricCharge") / need);
            int n = (int)Math.Ceiling(dt / STEP);
            if (n > MAX_STEPS) n = MAX_STEPS;
            double h = Math.Min(dt / n, STEP);
            Vector3d input = Vector3d.zero;
            Quaternion turned = Quaternion.identity;
            int i;
            for (i = 0; i < n; i++)
            {
                Vector3d t = Quaternion.Inverse(turned) * (Vector3)tb;
                Vector3d cross = Vector3d.Cross(ab, t);
                double sin = cross.magnitude;
                double angle = Math.Atan2(sin, Vector3d.Dot(ab, t));
                if (angle * Mathf.Rad2Deg < SETTLED_DEG && (omega - targetRate).magnitude < SETTLED_RATE)
                    break;
                Vector3d ax = sin > 1e-12 ? cross / sin : Vector3d.zero;
                double acc = AxisAccel(ax) * MARGIN * share;
                Vector3d want = ax * Math.Min(Math.Sqrt(2 * acc * angle), angle / SETTLE) + targetRate;
                Vector3d dw = want - omega;
                Vector3d u = new Vector3d(Clamp(Div(dw.x / RESPONSE, alpha.x * share)),
                                          Clamp(Div(dw.y / RESPONSE, alpha.y * share)),
                                          Clamp(Div(dw.z / RESPONSE, alpha.z * share)));
                omega += new Vector3d(u.x * alpha.x, u.y * alpha.y, u.z * alpha.z) * share * h;
                input += new Vector3d(Math.Abs(u.x), Math.Abs(u.y), Math.Abs(u.z)) * share * h;
                double mag = omega.magnitude;
                if (mag > 0)
                    turned = turned * Quaternion.AngleAxis((float)(mag * h * Mathf.Rad2Deg), (Vector3)(omega / mag));
            }
            ApplyBody(v, turned);
            double used = ecRate * (input.x + input.y + input.z);
            if (used > 0) ecUsed += Draw(v, "ElectricCharge", used);
            Vector3d tn = Quaternion.Inverse(turned) * (Vector3)tb;
            lastError = Vector3d.Angle(ab, tn);
            // the rest of a long frame, once on target, is held there
            if (i < n || (lastError < SETTLED_DEG && (omega - targetRate).magnitude < SETTLED_RATE))
            {
                settled = true;
                omega = targetRate;
                Hold(v, ab, tb, targetRate);
            }
        }

        /// <summary>What holding the target over dt takes from the wheels:
        /// the torque that brings the rate to the target's, spread over the
        /// step, and the gyroscopic torque at that rate, as input on each
        /// axis; drawn as ElectricCharge.  False (nothing drawn) if an axis
        /// needs more than the wheels' torque or the charge is short.</summary>
        static bool Charge(Vessel v, Vector3d targetRate, double dt)
        {
            if (dt <= 0) return true;
            Vector3d dw = targetRate - omega;
            Vector3d w = targetRate;
            Vector3d iw = new Vector3d(moi.x * w.x, moi.y * w.y, moi.z * w.z);
            Vector3d gyro = Vector3d.Cross(w, iw);
            // angular acceleration asked of each axis, rad/s^2
            Vector3d need = new Vector3d(dw.x / dt + Div(gyro.x, moi.x), dw.y / dt + Div(gyro.y, moi.y),
                                         dw.z / dt + Div(gyro.z, moi.z));
            double ux = Math.Abs(Div(need.x, alpha.x)), uy = Math.Abs(Div(need.y, alpha.y)),
                   uz = Math.Abs(Div(need.z, alpha.z));
            if (ux > 1 || uy > 1 || uz > 1) return false;
            double ec = ecRate * (ux + uy + uz) * dt;
            if (ec <= 0) return true;
            if (Available(v, "ElectricCharge") < ec) return false;
            double got = Draw(v, "ElectricCharge", ec);
            ecUsed += got;
            ecHold += got;
            return true;
        }

        /// <summary>The axis put on the target, turning with it.</summary>
        static void Hold(Vessel v, Vector3d ab, Vector3d tb, Vector3d targetRate)
        {
            Quaternion q = Quaternion.FromToRotation((Vector3)ab, (Vector3)tb);
            ApplyBody(v, q);
            omega = targetRate;
            lastError = 0;
            settled = true;
        }

        /// <summary>A new command: steer again until on it.</summary>
        internal static void Unsettle() { settled = false; }

        static double AxisAccel(Vector3d a)
        {
            double best = double.PositiveInfinity;
            if (Math.Abs(a.x) > 1e-6) best = Math.Min(best, alpha.x / Math.Abs(a.x));
            if (Math.Abs(a.y) > 1e-6) best = Math.Min(best, alpha.y / Math.Abs(a.y));
            if (Math.Abs(a.z) > 1e-6) best = Math.Min(best, alpha.z / Math.Abs(a.z));
            return double.IsInfinity(best) ? 0 : best;
        }

        static double Div(double a, double b) { return b > 0 ? a / b : 0; }
        static double Clamp(double x) { return x < -1 ? -1 : (x > 1 ? 1 : x); }

        static void Rotate(Vessel v, Vector3d w, double dt)
        {
            double mag = w.magnitude;
            ApplyBody(v, Quaternion.AngleAxis((float)(mag * dt * Mathf.Rad2Deg), (Vector3)(w / mag)));
        }

        /// <summary>Rotate the vessel by a rotation in its ReferenceTransform's
        /// axes.</summary>
        static void ApplyBody(Vessel v, Quaternion body)
        {
            Quaternion rt = v.ReferenceTransform.rotation;
            Quaternion world = rt * body * Quaternion.Inverse(rt);
            v.SetRotation(world * v.transform.rotation);
        }

        static void HandBack(Vessel v)
        {
            Vector3 w = v.ReferenceTransform.TransformDirection((Vector3)omega);
            foreach (Part p in v.parts)
                if (p.rb != null) p.rb.angularVelocity = w;
        }

        static double Available(Vessel v, string resource)
        {
            int id = PartResourceLibrary.Instance.GetDefinition(resource).id;
            double amount, max;
            v.GetConnectedResourceTotals(id, out amount, out max);
            return amount;
        }

        static double Draw(Vessel v, string resource, double amount)
        {
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                    if (pm is ModuleReactionWheel)
                        return p.RequestResource(resource, amount);
            return 0;
        }
    }
}
