// The rotation of the active vessel on rails while a mission flies it.  On
// rails the game does not integrate rotation, so this turns the vessel's
// thrust axis toward the commanded direction as its own actuators would:
//
//   - reaction wheels: pitch/roll/yaw torque x authority limiter, only wheels
//     switched on and not broken; ElectricCharge at the stock rate for the
//     input they are given, and with no charge no torque;
//   - engine gimbal: the lit engines' thrust tilted through the gimbal's
//     range (limiter applied, locked gimbals and disabled axes left out),
//     only while the controller is thrusting, scaled by its throttle; free;
//   - RCS, while the vessel's RCS group is on (RCS_MODE): every enabled
//     thruster's force about the centre of mass, per axis the weaker of the
//     two directions; propellant drawn through the RCS part's flow rules for
//     the thrust the torque takes (vacuum Isp).  "assist" turns on wheels and
//     gimbal and fires RCS only on an axis they cannot turn at all (no wheel,
//     no charge) or for a hold beyond them; "always" adds it to every turn.
//
// Torques and the moment of inertia are measured off rails every step (the
// game's own values) and frozen on rails; the angular rate carries from frame
// to frame and is handed back to the rigid bodies when the vessel comes off
// rails, as a rigid rotation about the centre of mass.
//
// None of this can move the orbit.  Every actuator is modelled as pure torque
// (RCS translation is not applied); rotating a packed vessel leaves its
// orbital elements alone, and each rotation on rails is checked: elements
// that changed anyway are put back and counted (`orbitRestores`, logged);
// the hand-back adds w x r to each part's velocity about the centre of mass,
// so the momentum, and the orbit the game computes from it, are unchanged.
// With no mission flying (`Step(v, false)`) it does nothing at all: no
// rotation, no hand-back, no draw.
//
// Once the axis is on a slowly turning target (within SETTLED_DEG, the rate
// matched) it is held there exactly, as actuators tracking a target turning
// at orbital rate do, and the frame costs nothing to integrate -- but it is
// charged: the torque that turns the vessel's rate with the target's (I
// dw/dt) and holds it against its own gyroscopic torque (w x Iw), from the
// wheels first, then the gimbal, then RCS.  A hold they cannot give (torque,
// charge or propellant short) is let go and the vessel steered again.
//
// The controller runs it first thing in its own physics step (Step), so
// what it decides next sees the vessel as this step leaves it.

using System;
using System.Collections.Generic;
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
        /// <summary>RCS_MODE: "assist", "always" or "off".</summary>
        internal static string rcsMode = "assist";
        /// <summary>The controller's throttle as last flown (0 coasting or
        /// turning): the share of the gimbal's torque there is.</summary>
        internal static double thrustShare;
        // rad/s in ReferenceTransform axes, as the game reports it
        internal static Vector3d omega;
        internal static Vector3d moi;            // t m^2 (x pitch, y roll, z yaw)
        internal static Vector3d alpha;          // rad/s^2 a turn plans on (last step)
        internal static Vector3d alphaW;         // rad/s^2 the wheels give
        internal static Vector3d alphaG;         // ... the gimbal at full thrust
        internal static Vector3d alphaR;         // ... RCS
        internal static double ecRate;           // EC/s at full input on one axis, all wheels
        internal static double ecUsed;
        internal static double ecHold;           // of ecUsed, holding a settled target
        internal static double rcsKg;            // RCS propellant drawn
        internal static double rcsHoldKg;        // of rcsKg, holding
        internal static int holdsLost;           // holds the actuators could not give
        internal static int orbitRestores;       // rotations on rails that moved the elements
        internal static int rcsThrusters, gimbals;
        internal static double lastError = -1;
        internal static bool settled;
        static Vessel vessel;
        static bool wasPacked, rotated;
        static Vector3d lastTarget;
        static bool haveLast;
        // RCS: kN of thrust per kN m of torque on each axis, thrust-weighted
        // vacuum Isp, and the propellant each module draws per tonne of the
        // whole pool's flow
        static Vector3d rcsLambda;
        static double rcsIsp;
        static readonly List<RcsProp> rcsProps = new List<RcsProp>();

        class RcsProp
        {
            public Part part;
            public int id;
            public ResourceFlowMode mode;
            public double unitsPerTonne;
        }

        // Substep of the rotation integral, s; most substeps a frame.
        const double STEP = 0.05;
        const int MAX_STEPS = 4000;
        // The turn plans to stop with this share of its authority in reserve.
        const double MARGIN = 0.8;
        // Near the target the rate is the error over SETTLE seconds, and the
        // actuators close a rate error over RESPONSE seconds (no overshoot).
        const double SETTLE = 4.0;
        const double RESPONSE = 0.5;
        // Held exactly once this close, with the rate matched to within this.
        const double SETTLED_DEG = 0.05;
        const double SETTLED_RATE = 1e-3;
        const double G0 = 9.80665;

        internal static void Reset()
        {
            omega = Vector3d.zero;
            ecUsed = 0;
            ecHold = 0;
            rcsKg = 0;
            rcsHoldKg = 0;
            holdsLost = 0;
            orbitRestores = 0;
            thrustShare = 0;
            target = null;
            settled = false;
            vessel = null;
            rotated = false;
        }

        /// <summary>One physics step: measure off rails, turn on rails --
        /// only while a mission flies this vessel (active); otherwise nothing
        /// is touched.</summary>
        internal static void Step(Vessel v, bool active)
        {
            if (v == null) return;
            if (!active)
            {
                // not ours: forget it, so a mission started later begins from
                // the game's own state, and nothing is handed back
                vessel = null;
                rotated = false;
                return;
            }
            if (v != vessel)
            {
                vessel = v;
                omega = Vector3d.zero;
                wasPacked = v.packed;
                rotated = false;
                haveLast = false;
            }
            if (!v.packed)
            {
                if (wasPacked && rotated) HandBack(v);
                Measure(v);
                omega = v.angularVelocity;
                wasPacked = false;
                rotated = false;
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

        // --- measuring (off rails) --------------------------------------------------

        static void Measure(Vessel v)
        {
            moi = v.MOI;
            Transform rt = v.ReferenceTransform;
            Vector3d com = v.CurrentCoM;
            Vector3d tw = Vector3d.zero, tg = Vector3d.zero;
            Vector3d rPos = Vector3d.zero, rNeg = Vector3d.zero, fPos = Vector3d.zero, fNeg = Vector3d.zero;
            double rate = 0, rcsF = 0, rcsFlow = 0;
            int nr = 0, ng = 0;
            rcsProps.Clear();
            var modules = new List<KeyValuePair<ModuleRCS, double>>();
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                {
                    var w = pm as ModuleReactionWheel;
                    if (w != null)
                    {
                        if (!w.moduleIsEnabled || w.wheelState != ModuleReactionWheel.WheelState.Active) continue;
                        double auth = w.authorityLimiter * 0.01;
                        tw += new Vector3d(w.PitchTorque, w.RollTorque, w.YawTorque) * auth;
                        foreach (var r in w.resHandler.inputResources)
                            if (r.name == "ElectricCharge") rate += r.rate * auth;
                        continue;
                    }
                    var rcs = pm as ModuleRCS;
                    if (rcs != null)
                    {
                        if (!rcs.moduleIsEnabled || !rcs.rcsEnabled || rcs.isJustForShow) continue;
                        double isp = rcs.atmosphereCurve.Evaluate(0f);
                        double f = rcs.thrusterPower * rcs.thrustPercentage * 0.01;      // kN a nozzle
                        if (isp <= 0 || f <= 0 || rcs.thrusterTransforms == null) continue;
                        var mask = new Vector3d(rcs.enablePitch ? 1 : 0, rcs.enableRoll ? 1 : 0, rcs.enableYaw ? 1 : 0);
                        double fm = 0;
                        foreach (Transform t in rcs.thrusterTransforms)
                        {
                            if (t == null) continue;
                            Vector3d dir = rcs.useZaxis ? -(Vector3d)t.forward : -(Vector3d)t.up;
                            Vector3d tq = Vector3d.Cross((Vector3d)t.position - com, dir * f);
                            Vector3d b = rt.InverseTransformDirection((Vector3)tq);
                            b = new Vector3d(b.x * mask.x, b.y * mask.y, b.z * mask.z);
                            for (int k = 0; k < 3; k++)
                            {
                                if (b[k] > 1e-9) { rPos[k] += b[k]; fPos[k] += f; }
                                else if (b[k] < -1e-9) { rNeg[k] -= b[k]; fNeg[k] += f; }
                            }
                            fm += f;
                            nr++;
                        }
                        if (fm <= 0) continue;
                        rcsF += fm;
                        rcsFlow += fm / isp;
                        modules.Add(new KeyValuePair<ModuleRCS, double>(rcs, fm));
                        continue;
                    }
                    var g = pm as ModuleGimbal;
                    if (g != null && !g.gimbalLock && g.moduleIsEnabled)
                    {
                        Vector3d t = GimbalTorque(g, rt, com);
                        if (t.sqrMagnitude > 0) { tg += t; ng++; }
                    }
                }
            ecRate = rate;
            alphaW = PerMoi(tw);
            alphaG = PerMoi(tg);
            gimbals = ng;
            rcsThrusters = nr;
            Vector3d tr = Vector3d.zero;
            for (int k = 0; k < 3; k++)
            {
                tr[k] = Math.Min(rPos[k], rNeg[k]);
                double torque = rPos[k] + rNeg[k];
                rcsLambda[k] = torque > 0 ? (fPos[k] + fNeg[k]) / torque : 0;
            }
            alphaR = PerMoi(tr);
            rcsIsp = rcsFlow > 0 ? rcsF / rcsFlow : 0;
            // each module's propellants, per tonne of the pool's flow: its
            // share of the pool's mass flow, split by its mixture
            foreach (var m in modules)
            {
                double share = m.Value / m.Key.atmosphereCurve.Evaluate(0f) / rcsFlow;
                double mix = 0;
                foreach (Propellant pr in m.Key.propellants)
                    if (pr.resourceDef != null) mix += pr.ratio * pr.resourceDef.density;
                if (mix <= 0) continue;
                foreach (Propellant pr in m.Key.propellants)
                    if (pr.resourceDef != null)
                        rcsProps.Add(new RcsProp
                        {
                            part = m.Key.part, id = pr.resourceDef.id, mode = pr.resourceDef.resourceFlowMode,
                            unitsPerTonne = share * pr.ratio / mix
                        });
            }
            alpha = Plan(v, 1, RcsOn(v) ? 1 : 0);
        }

        /// <summary>The gimbal's torque, kN m in ReferenceTransform axes, at
        /// its engine's full thrust: each nozzle's thrust tilted through the
        /// range about the vessel's pitch axis and about the axis across both,
        /// each axis taking the sum of what the two tilts give it.</summary>
        static Vector3d GimbalTorque(ModuleGimbal g, Transform rt, Vector3d com)
        {
            ModuleEngines e = null;
            foreach (PartModule pm in g.part.Modules)
            {
                var me = pm as ModuleEngines;
                if (me != null && me.EngineIgnited && me.isEnabled && !me.flameout) { e = me; break; }
            }
            if (e == null || e.thrustTransforms == null || e.thrustTransforms.Count == 0) return Vector3d.zero;
            double range = g.gimbalRange;
            foreach (float r in new[] { g.gimbalRangeXP, g.gimbalRangeXN, g.gimbalRangeYP, g.gimbalRangeYN })
                if (r >= 0) range = Math.Min(range, r);
            range *= g.gimbalLimiter * 0.01;
            double f = e.maxThrust * e.thrustPercentage * 0.01 / e.thrustTransforms.Count;     // kN a nozzle
            if (range <= 0 || f <= 0) return Vector3d.zero;
            double s = Math.Sin(range * Math.PI / 180.0);
            var mask = new Vector3d(g.enablePitch ? 1 : 0, g.enableRoll ? 1 : 0, g.enableYaw ? 1 : 0);
            Vector3d sum = Vector3d.zero;
            foreach (Transform t in e.thrustTransforms)
            {
                if (t == null) continue;
                Vector3d d = -(Vector3d)t.forward;
                Vector3d r = (Vector3d)t.position - com;
                Vector3d a1 = Vector3d.Cross(d, (Vector3d)rt.right);
                if (a1.sqrMagnitude < 1e-6) a1 = Vector3d.Cross(d, (Vector3d)rt.forward);
                a1 = a1.normalized;
                Vector3d a2 = Vector3d.Cross(d, a1).normalized;
                foreach (Vector3d a in new[] { a1, a2 })
                {
                    Vector3d tq = Vector3d.Cross(r, Vector3d.Cross(a, d) * (f * s));
                    Vector3d b = rt.InverseTransformDirection((Vector3)tq);
                    sum += new Vector3d(Math.Abs(b.x) * mask.x, Math.Abs(b.y) * mask.y, Math.Abs(b.z) * mask.z);
                }
            }
            return sum;
        }

        static Vector3d PerMoi(Vector3d t)
        {
            return new Vector3d(moi.x > 0 ? t.x / moi.x : 0, moi.y > 0 ? t.y / moi.y : 0, moi.z > 0 ? t.z / moi.z : 0);
        }

        static bool RcsOn(Vessel v)
        {
            return rcsMode != "off" && rcsProps.Count > 0 && v.ActionGroups[KSPActionGroup.RCS];
        }

        /// <summary>The authority a turn plans on, rad/s^2 per axis, with
        /// this share of the wheels' charge and of the RCS propellant there.</summary>
        static Vector3d Plan(Vessel v, double ecShare, double rcsShare)
        {
            Vector3d free = alphaW * ecShare + alphaG * thrustShare;
            Vector3d r = alphaR * rcsShare;
            if (rcsMode == "always") return free + r;
            return new Vector3d(free.x > 0 ? free.x : r.x, free.y > 0 ? free.y : r.y, free.z > 0 ? free.z : r.z);
        }

        // --- turning (on rails) -----------------------------------------------------

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

            double ecShare = 1;
            double need = ecRate * 3 * dt;
            if (need > 0) ecShare = Math.Min(1, Available(v, EcId) / need);
            bool rcsOn = RcsOn(v);
            double rcsShare = rcsOn ? RcsShare(v, RcsTonnes(alphaR * dt)) : 0;
            Vector3d a = Plan(v, ecShare, rcsShare);
            alpha = a;
            Vector3d wCap = alphaW * ecShare, gCap = alphaG * thrustShare;
            int n = (int)Math.Ceiling(dt / STEP);
            if (n > MAX_STEPS) n = MAX_STEPS;
            double h = Math.Min(dt / n, STEP);
            Vector3d wheelInput = Vector3d.zero, rcsAccel = Vector3d.zero;
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
                double acc = AxisAccel(ax, a) * MARGIN;
                Vector3d want = ax * Math.Min(Math.Sqrt(2 * acc * angle), angle / SETTLE) + targetRate;
                Vector3d dw = want - omega;
                Vector3d acc3 = new Vector3d(Clamp(Div(dw.x / RESPONSE, a.x)) * a.x,
                                             Clamp(Div(dw.y / RESPONSE, a.y)) * a.y,
                                             Clamp(Div(dw.z / RESPONSE, a.z)) * a.z);
                omega += acc3 * h;
                // the angular acceleration from the wheels first, the gimbal
                // next, RCS for the rest
                for (int k = 0; k < 3; k++)
                {
                    double d = Math.Abs(acc3[k]);
                    double fromW = Math.Min(d, wCap[k]);
                    double rest = d - fromW - Math.Min(d - fromW, gCap[k]);
                    if (alphaW[k] > 0) wheelInput[k] += fromW / alphaW[k] * h;
                    rcsAccel[k] += rest * h;
                }
                double mag = omega.magnitude;
                if (mag > 0)
                    turned = turned * Quaternion.AngleAxis((float)(mag * h * Mathf.Rad2Deg), (Vector3)(omega / mag));
            }
            ApplyBody(v, turned);
            double used = ecRate * (wheelInput.x + wheelInput.y + wheelInput.z);
            if (used > 0) ecUsed += Draw(v, EcId, used);
            if (rcsOn) rcsKg += DrawRcs(RcsTonnes(rcsAccel)) * 1000.0;
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

        /// <summary>What holding the target over dt takes: the torque that
        /// brings the rate to the target's, spread over the step, and the
        /// gyroscopic torque at that rate, from the wheels (drawn as
        /// ElectricCharge), then the gimbal, then RCS (propellant).  False
        /// (nothing drawn) if an axis needs more than they give, or the
        /// charge or propellant is short.</summary>
        static bool Charge(Vessel v, Vector3d targetRate, double dt)
        {
            if (dt <= 0) return true;
            Vector3d dw = targetRate - omega;
            Vector3d w = targetRate;
            Vector3d iw = new Vector3d(moi.x * w.x, moi.y * w.y, moi.z * w.z);
            Vector3d gyro = Vector3d.Cross(w, iw);
            // angular acceleration asked of each axis, rad/s^2
            Vector3d need = new Vector3d(Math.Abs(dw.x / dt + Div(gyro.x, moi.x)), Math.Abs(dw.y / dt + Div(gyro.y, moi.y)),
                                         Math.Abs(dw.z / dt + Div(gyro.z, moi.z)));
            bool rcsOn = RcsOn(v);
            Vector3d input = Vector3d.zero, rcsAccel = Vector3d.zero;
            for (int k = 0; k < 3; k++)
            {
                double fromW = Math.Min(need[k], alphaW[k]);
                double rest = need[k] - fromW - Math.Min(need[k] - fromW, alphaG[k] * thrustShare);
                if (rest > 1e-12 && (!rcsOn || rest > alphaR[k])) return false;
                if (alphaW[k] > 0) input[k] = fromW / alphaW[k];
                rcsAccel[k] = rest * dt;
            }
            double ec = ecRate * (input.x + input.y + input.z) * dt;
            if (ec > 0 && Available(v, EcId) < ec) return false;
            double tonnes = RcsTonnes(rcsAccel);
            if (tonnes > 0 && RcsShare(v, tonnes) < 1) return false;
            if (ec > 0)
            {
                double got = Draw(v, EcId, ec);
                ecUsed += got;
                ecHold += got;
            }
            if (tonnes > 0)
            {
                double kg = DrawRcs(tonnes) * 1000.0;
                rcsKg += kg;
                rcsHoldKg += kg;
            }
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

        static double AxisAccel(Vector3d ax, Vector3d a)
        {
            double best = double.PositiveInfinity;
            if (Math.Abs(ax.x) > 1e-6) best = Math.Min(best, a.x / Math.Abs(ax.x));
            if (Math.Abs(ax.y) > 1e-6) best = Math.Min(best, a.y / Math.Abs(ax.y));
            if (Math.Abs(ax.z) > 1e-6) best = Math.Min(best, a.z / Math.Abs(ax.z));
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
        /// axes, its orbit kept exactly as it was.</summary>
        static void ApplyBody(Vessel v, Quaternion body)
        {
            global::Orbit o = v.orbit;
            double inc = o.inclination, e = o.eccentricity, sma = o.semiMajorAxis, lan = o.LAN,
                   argPe = o.argumentOfPeriapsis, mEp = o.meanAnomalyAtEpoch, epoch = o.epoch;
            CelestialBody body0 = o.referenceBody;
            Quaternion rt = v.ReferenceTransform.rotation;
            Quaternion world = rt * body * Quaternion.Inverse(rt);
            v.SetRotation(world * v.transform.rotation);
            rotated = true;
            if (o.inclination != inc || o.eccentricity != e || o.semiMajorAxis != sma || o.LAN != lan
                || o.argumentOfPeriapsis != argPe || o.meanAnomalyAtEpoch != mEp || o.epoch != epoch
                || o.referenceBody != body0)
            {
                o.SetOrbit(inc, e, sma, lan, argPe, mEp, epoch, body0);
                v.orbitDriver.updateFromParameters();
                orbitRestores++;
            }
        }

        /// <summary>The rate turned on rails, given to the rigid bodies as a
        /// rigid rotation about the centre of mass: each part spins at it and
        /// moves at w x r about the CoM, which sums to nothing -- the CoM's
        /// velocity, the orbit, is the game's.</summary>
        static void HandBack(Vessel v)
        {
            Vector3 w = v.ReferenceTransform.TransformDirection((Vector3)omega);
            Vector3 com = v.CurrentCoM;
            foreach (Part p in v.parts)
                if (p.rb != null)
                {
                    p.rb.angularVelocity = w;
                    p.rb.velocity += Vector3.Cross(w, p.rb.worldCenterOfMass - com);
                }
        }

        // --- resources --------------------------------------------------------------

        /// <summary>Tonnes of RCS propellant for these angular accelerations
        /// x seconds on each axis (rad/s).</summary>
        static double RcsTonnes(Vector3d accelSeconds)
        {
            if (rcsIsp <= 0) return 0;
            double kNs = 0;
            for (int k = 0; k < 3; k++) kNs += Math.Abs(accelSeconds[k]) * moi[k] * rcsLambda[k];
            return kNs / (rcsIsp * G0);
        }

        /// <summary>The share of this much RCS propellant the vessel holds (0..1).</summary>
        static double RcsShare(Vessel v, double tonnes)
        {
            if (tonnes <= 0) return 1;
            double share = 1;
            foreach (RcsProp p in rcsProps)
            {
                double want = p.unitsPerTonne * tonnes;
                if (want > 0) share = Math.Min(share, Available(v, p.id) / want);
            }
            return Math.Max(0, Math.Min(1, share));
        }

        /// <summary>Draw tonnes of RCS propellant through each module's part;
        /// the tonnes delivered.</summary>
        static double DrawRcs(double tonnes)
        {
            if (tonnes <= 0) return 0;
            double got = 1;
            foreach (RcsProp p in rcsProps)
            {
                double want = p.unitsPerTonne * tonnes;
                if (want <= 0) continue;
                double g = p.part.RequestResource(p.id, want, p.mode, false);
                got = Math.Min(got, g / want);
            }
            return tonnes * got;
        }

        static int EcId { get { return PartResourceLibrary.Instance.GetDefinition("ElectricCharge").id; } }

        static double Available(Vessel v, int id)
        {
            double amount, max;
            v.GetConnectedResourceTotals(id, out amount, out max);
            return amount;
        }

        static double Draw(Vessel v, int id, double amount)
        {
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                    if (pm is ModuleReactionWheel)
                        return p.RequestResource(id, amount);
            return 0;
        }
    }
}
