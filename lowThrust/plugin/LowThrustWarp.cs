// LowThrustWarp -- what a low-thrust autopilot needs under rails warp that
// kRPC cannot give it, and nothing more.  A kRPC service, "LowThrust".
//
// 1. The throttle.  kRPC's throttle is latched onto FlightInputHandler, which
//    a vessel on rails does not read, and kRPC's RailsWarpFactor zeroes it
//    before warping.  Persistent Thrust (PT) holds its own throttle, changed
//    during warp only by its throttle keys.  SetThrottle does what those keys
//    do (PT's SetThrottleAfterKey) and nothing else.
//
// 2. Attitude.  On rails the game does not integrate rotation, so a script
//    cannot turn the vessel; PT can only snap it to an SAS mode.  SetAttitude
//    turns the vessel's thrust axis toward a commanded direction *as the
//    vessel's own reaction wheels would*: the rotation is integrated from the
//    torque the wheels have (their pitch/yaw/roll torque, authority limiter,
//    only wheels that are switched on and not broken) and the moment of
//    inertia the game measured before the vessel went on rails; the angular
//    rate carries from frame to frame (the turn speeds up and slows down, it
//    is not a snap); the wheels draw ElectricCharge at the stock rate for the
//    input they are given (rate x (|pitch|+|roll|+|yaw|) x authority, as
//    ModuleReactionWheel does), and with no charge there is no torque.  The
//    angular rate on rails is handed back to the vessel when it comes off
//    rails.  Off rails it does nothing: the vessel is turned by its real
//    wheels (kRPC's autopilot).
//
//    Not modelled yet, so not available: RCS torque (it would burn its
//    propellant), and engine gimbal (it would turn the thrust off the axis).
//    A vessel with neither wheels nor EC cannot turn on rails here.
//
// PT keeps the thrust, the propellant and the power for the engine.  This
// plugin never touches the orbit, the engine, or any resource other than the
// wheels' ElectricCharge.
//
// Build and install: ./build.sh (lowThrust/CLAUDE.md).

using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service.Attributes;
using KRPC.SpaceCenter.ExtensionMethods;
using UnityEngine;
using ReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace LowThrustWarp
{
    [KRPCService(Name = "LowThrust", GameScene = KRPC.Service.GameScene.Flight)]
    public static class LowThrust
    {
        const BindingFlags ALL = BindingFlags.Instance | BindingFlags.Public
                                 | BindingFlags.NonPublic;

        // ---- throttle -----------------------------------------------------

        /// <summary>Persistent Thrust engine modules on the active vessel
        /// (0: PT not installed, or a save made before it was).</summary>
        [KRPCProcedure]
        public static int EngineCount()
        {
            int n = 0;
            foreach (PartModule m in PtModules()) n++;
            return n;
        }

        /// <summary>Set the throttle 0..1 as PT's throttle keys do.</summary>
        [KRPCProcedure]
        public static void SetThrottle(float throttle)
        {
            throttle = Mathf.Clamp01(throttle);
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null) return;
            foreach (PartModule m in PtModules())
            {
                MethodInfo set = m.GetType().GetMethod("SetThrottleAfterKey", ALL);
                if (set != null)
                    set.Invoke(m, new object[] { throttle, false });
            }
            // Not FlightInputHandler: kRPC refuses rails warp while that
            // throttle is open, and PT restores its own throttle when the
            // vessel comes off rails.
            v.ctrlState.mainThrottle = throttle;
        }

        /// <summary>PT's throttle (-1: no PT engine).</summary>
        [KRPCProcedure]
        public static float Throttle()
        {
            foreach (PartModule m in PtModules())
            {
                FieldInfo f = m.GetType().GetField("persistentThrottle", ALL);
                if (f != null) return (float)f.GetValue(m);
            }
            return -1f;
        }

        /// <summary>PT's thrust on the active vessel in its last frame, N.</summary>
        [KRPCProcedure]
        public static double Thrust()
        {
            double sum = 0;
            foreach (PartModule m in PtModules())
            {
                FieldInfo f = m.GetType().GetField("persistentThrust", ALL);
                if (f != null) sum += (double)f.GetValue(m);
            }
            return sum * 1000.0;
        }

        /// <summary>This game's rails warp rates by index (mods such as
        /// BetterTimeWarp change them).</summary>
        [KRPCProcedure]
        public static IList<float> RailsWarpRates()
        {
            return new List<float>(TimeWarp.fetch.warpRates);
        }

        /// <summary>Set the rails warp rate index at once, within the
        /// game's altitude limits (as KSP's own SetRate checks them): the
        /// index actually set.  KSP eases between rates over seconds, and a
        /// pilot changing rate every few ticks spent a third of its time
        /// easing (33x of a 50x warp); the instant change is KSP's own
        /// option.  Unlike kRPC's RailsWarpFactor it does not cut the
        /// throttle.  The index asked for stands: while it is held the rate
        /// climbs back to it whenever the altitude allows (KSP only ever
        /// lowers it), so a coast set at periapsis is not flown at its
        /// limit all the way round.</summary>
        [KRPCProcedure]
        public static int SetRailsWarp(int index)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null) return 0;
            if (index < 0) index = 0;
            if (index >= TimeWarp.fetch.warpRates.Length)
                index = TimeWarp.fetch.warpRates.Length - 1;
            Pace.wantIndex = index;
            index = Pace.AltitudeCap(v, index);
            if (TimeWarp.WarpMode != TimeWarp.Modes.HIGH)
            {
                TimeWarp.fetch.Mode = TimeWarp.Modes.HIGH;
                TimeWarp.SetRate(0, true, true);
            }
            if (TimeWarp.CurrentRateIndex != index)
                TimeWarp.SetRate(index, true, true);
            return index;
        }

        // ---- pace -----------------------------------------------------------

        /// <summary>Run rails warp <paramref name="scale"/> times faster
        /// than the game would (1: as KSP does).  On rails each physics step
        /// still advances UT by 0.02 s x the warp rate; this only runs more of
        /// them per real second (Unity's Time.timeScale, which KSP leaves at
        /// 1 on rails), so the flight is the same flight, sooner -- as far as
        /// the CPU allows.  Off rails it does nothing.</summary>
        [KRPCProcedure]
        public static void SetPace(float scale)
        {
            Pace.scale = Mathf.Max(1f, scale);
        }

        /// <summary>Run on rails until UT <paramref name="ut"/>, then hold the
        /// game there (no physics frames) until the next call: each command
        /// is held for exactly its tick however fast the game runs.  The game
        /// slows to its normal pace for the last frames, so it stops within
        /// one frame (0.02 s x the warp rate).</summary>
        /// <remarks>As the end nears the warp rate steps down (to no lower
        /// than index <paramref name="floorIndex"/>) so the last frames are
        /// short: a coast warped at 10000x would otherwise end up to 200 s
        /// late.</remarks>
        [KRPCProcedure]
        public static void RunUntil(double ut, int floorIndex)
        {
            Pace.holdUT = ut;
            Pace.floorIndex = floorIndex;
            Pace.lastCommand = Time.realtimeSinceStartup;
        }

        /// <summary>Release any hold (the game then runs freely).</summary>
        [KRPCProcedure]
        public static void ReleaseHold()
        {
            Pace.holdUT = double.PositiveInfinity;
        }

        /// <summary>Whether the game is held at RunUntil's time.</summary>
        [KRPCProcedure]
        public static bool Held()
        {
            // read on a stream, every frame while the pilot is connected: a
            // hold is only an orphan once nobody is watching it
            Pace.lastCommand = Time.realtimeSinceStartup;
            return Pace.held;
        }

        /// <summary>Real seconds per game second actually achieved under
        /// rails warp, over the last second (0: not measured).</summary>
        [KRPCProcedure]
        public static double AchievedRate()
        {
            return Pace.achieved;
        }

        // ---- attitude -----------------------------------------------------

        /// <summary>Turn the thrust axis toward <paramref name="direction"/>
        /// (in <paramref name="frame"/>) while on rails, at the rate the
        /// vessel's reaction wheels allow.  Held until ClearAttitude.</summary>
        [KRPCProcedure]
        public static void SetAttitude(Tuple<double, double, double> direction,
                                       ReferenceFrame frame)
        {
            Attitude.target = direction.ToVector().normalized;
            Attitude.frame = frame;
        }

        /// <summary>Stop steering on rails (the vessel then holds its
        /// attitude and angular rate, as an unpowered vessel would).</summary>
        [KRPCProcedure]
        public static void ClearAttitude()
        {
            Attitude.frame = null;
        }

        /// <summary>Angle between the thrust axis and the commanded
        /// direction, degrees (-1: no command).</summary>
        [KRPCProcedure]
        public static double AttitudeError()
        {
            return Attitude.lastError;
        }

        /// <summary>The angular rate on rails, deg/s.</summary>
        [KRPCProcedure]
        public static double AngularRate()
        {
            return Attitude.omega.magnitude * Mathf.Rad2Deg;
        }

        /// <summary>ElectricCharge the wheels have drawn on rails since load.</summary>
        [KRPCProcedure]
        public static double WheelCharge()
        {
            return Attitude.ecUsed;
        }

        /// <summary>The wheels' angular acceleration limits (pitch, roll,
        /// yaw), deg/s^2, as measured before going on rails.</summary>
        [KRPCProcedure]
        public static Tuple<double, double, double> AngularAccelerationLimit()
        {
            Vector3d a = Attitude.alpha * Mathf.Rad2Deg;
            return Tuple.Create(a.x, a.y, a.z);
        }

        /// <summary>PT's state on the first engine, as text (diagnostics).</summary>
        [KRPCProcedure]
        public static string Status()
        {
            var sb = new System.Text.StringBuilder();
            Vessel v = FlightGlobals.ActiveVessel;
            if (v != null)
                sb.AppendFormat("packed={0} rate={1} sas={2} mode={3} moi={4} torque={5} ",
                    v.packed, TimeWarp.CurrentRate, v.ActionGroups[KSPActionGroup.SAS],
                    v.Autopilot.Mode, Attitude.moi, Attitude.torque);
            foreach (PartModule m in PtModules())
            {
                foreach (string name in new[] { "persistentThrottle", "persistentIsp",
                    "persistentThrust", "vesselHeadingVersusManeuverInDegree" })
                {
                    FieldInfo f = m.GetType().GetField(name, ALL);
                    sb.AppendFormat("{0}={1} ", name, f == null ? "?" : f.GetValue(m));
                }
                break;
            }
            return sb.ToString();
        }

        internal static IEnumerable<PartModule> PtModules()
        {
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null) yield break;
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                    if (m.GetType().Name == "PersistentEngine")
                        yield return m;
        }
    }

    /// <summary>The pace of rails warp: more physics frames per real second
    /// (Unity's timeScale), and a hold at the end of each tick.</summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class Pace : MonoBehaviour
    {
        internal static float scale = 1f;
        internal static double holdUT = double.PositiveInfinity;
        internal static bool held;
        internal static double achieved;
        internal static float lastCommand;
        internal static int floorIndex = 1;
        internal static int wantIndex = -1;       // SetRailsWarp's, -1 none

        /// <summary>The highest index up to <paramref name="index"/> the
        /// game allows at the vessel's altitude.</summary>
        internal static int AltitudeCap(Vessel v, int index)
        {
            if (v.LandedOrSplashed) return index;
            double alt = v.mainBody.GetAltitude(v.CoM);
            while (index > 0 && alt < TimeWarp.fetch.GetAltitudeLimit(index, v.mainBody))
                index--;
            return index;
        }
        // The next frame may be this much longer than the longest recent.
        const double FRAME_MARGIN = 1.5;
        float[] frames = new float[8];
        int frameIdx;
        // Steps down so at least this many frames are left at the rate.
        const double FRAMES_LEFT = 20;
        // A hold no command has followed for this long (the pilot died) is
        // released, and the pace returned to normal.
        const float ORPHAN_S = 30f;
        float stockMax;
        bool active;
        double winUT;
        float winReal;

        void Start()
        {
            stockMax = Time.maximumDeltaTime;
            scale = 1f;
            holdUT = double.PositiveInfinity;
            held = false;
            wantIndex = -1;
            winReal = Time.realtimeSinceStartup;
            winUT = Planetarium.GetUniversalTime();
        }

        // kRPC serves its clients from FixedUpdate, which does not run while
        // the game is held (timeScale 0), and from Update only when the game
        // is paused: held, the server is updated from here instead, as kRPC
        // does itself when paused.
        MonoBehaviour krpc;
        MethodInfo krpcUpdate;

        void Update()
        {
            if (!held) return;
            if (krpc == null)
            {
                Type t = Type.GetType("KRPC.Addon, KRPC");
                if (t == null) return;
                krpc = FindObjectOfType(t) as MonoBehaviour;
                krpcUpdate = t.GetMethod("FixedUpdate");
                if (krpc == null) return;
            }
            krpcUpdate.Invoke(krpc, null);
        }

        void OnDestroy()
        {
            if (active) { Time.timeScale = 1f; Time.maximumDeltaTime = stockMax; }
        }

        void LateUpdate()
        {
            double ut = Planetarium.GetUniversalTime();
            float now = Time.realtimeSinceStartup;
            if (now - winReal >= 1f)
            {
                achieved = (ut - winUT) / (now - winReal);
                winUT = ut; winReal = now;
            }
            bool rails = TimeWarp.WarpMode == TimeWarp.Modes.HIGH
                         && TimeWarp.CurrentRateIndex > 0;
            if (FlightDriver.Pause || !rails)
            {
                held = false;
                // a warp the pilot did not set (the player's, or none) is
                // not overridden
                if (!FlightDriver.Pause) wantIndex = -1;
                if (active)
                {
                    Time.timeScale = FlightDriver.Pause ? 0f : 1f;
                    Time.maximumDeltaTime = stockMax;
                    active = false;
                }
                return;
            }
            if (!double.IsInfinity(holdUT) && now - lastCommand > ORPHAN_S)
            {
                holdUT = double.PositiveInfinity;
                scale = 1f;
                wantIndex = -1;
            }
            double left = holdUT - ut;
            // the rate: the one asked for, as far as the altitude allows,
            // stepped down near the end of a hold to no lower than the floor
            int idx = TimeWarp.CurrentRateIndex;
            int want = wantIndex >= 0 ? wantIndex : idx;
            Vessel av = FlightGlobals.ActiveVessel;
            if (av != null) want = AltitudeCap(av, want);
            if (left > 0)
                while (want > Math.Max(1, floorIndex)
                       && 0.02 * TimeWarp.fetch.warpRates[want] * FRAMES_LEFT > left)
                    want--;
            if (want >= 1 && want != idx)
                TimeWarp.SetRate(want, true, true);
            held = left <= 0;
            // The time scale for the next frame.  Unity simulates the real
            // frame time (capped at KSP's 0.04 s) x timeScale, and frame times
            // swing 2-3x, so the next frame is sized from the longest of the
            // recent ones x FRAME_MARGIN: at most half the steps left to the
            // hold, and under one step a frame at the end -- it stops within
            // one step.  (Sized from the last frame alone, a frame twice as
            // long ran 77 steps where 39 were meant, holds 40-56 s late.)
            frames[frameIdx] = Time.unscaledDeltaTime;
            frameIdx = (frameIdx + 1) % frames.Length;
            double fmax = 0.005;
            foreach (float f in frames) fmax = Math.Max(fmax, f);
            fmax = Math.Min(fmax * FRAME_MARGIN, 0.04);
            double ts = held ? 0.0 : scale;
            if (!held && !double.IsInfinity(left))
            {
                double step = 0.02 * TimeWarp.CurrentRate;
                double steps = Math.Max(0.4, 0.5 * left / step);
                ts = Math.Min(ts, steps * 0.02 / fmax);
            }
            Time.timeScale = (float)ts;
            active = true;
        }
    }

    /// <summary>The rotation of the active vessel on rails, integrated from
    /// its reaction wheels.</summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class Attitude : MonoBehaviour
    {
        internal static Vector3d target;
        internal static ReferenceFrame frame;
        // rad/s in ReferenceTransform axes, as the game reports it: relative to
        // the world frame, which rotates with the body below its
        // inverse-rotation altitude -- a vessel fixed in inertial space
        // reads Kerbin's 0.0167 deg/s there, and on rails it is advanced at
        // that rate to stay fixed (measured: 0.0002 deg/s of drift).
        internal static Vector3d omega;
        internal static Vector3d moi;            // kg m^2 (x pitch, y roll, z yaw), t m^2 in KSP units
        internal static Vector3d torque;         // kN m, the wheels' total
        internal static Vector3d alpha;          // rad/s^2
        internal static double ecRate;           // EC/s at full input on one axis, all wheels
        internal static double ecUsed;
        internal static double lastError = -1;
        Vessel vessel;
        bool wasPacked;

        // Substep of the rotation integral, s, and most substeps a frame.
        const double STEP = 0.02;
        const int MAX_STEPS = 50000;
        // The turn plans to stop with this share of the wheels in reserve.
        const double MARGIN = 0.8;
        // Near the target the rate is the error over SETTLE seconds, and the
        // wheels close a rate error over RESPONSE seconds (RESPONSE <=
        // SETTLE / 4: no overshoot).  A pure bang-bang law chattered at the
        // target and drew 1.7 EC/s holding still.
        const double SETTLE = 4.0;
        const double RESPONSE = 0.5;

        void Start()
        {
            omega = Vector3d.zero;
            ecUsed = 0;
            frame = null;
        }

        void FixedUpdate()
        {
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null) return;
            if (v != vessel)
            {
                vessel = v;
                omega = Vector3d.zero;
                wasPacked = v.packed;
            }
            if (!v.packed)
            {
                if (wasPacked) HandBack(v);
                Measure(v);
                omega = v.angularVelocity;
                wasPacked = false;
                return;
            }
            wasPacked = true;
            if (frame == null)
            {
                lastError = -1;
                Coast(v, TimeWarp.fixedDeltaTime);
                return;
            }
            Steer(v, TimeWarp.fixedDeltaTime);
        }

        /// <summary>Inertia and wheel torque, read while the game simulates
        /// the vessel (its MOI is only current off rails).</summary>
        void Measure(Vessel v)
        {
            moi = v.MOI;
            Vector3d t = Vector3d.zero;
            double rate = 0;
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                {
                    var w = pm as ModuleReactionWheel;
                    if (w == null || !w.moduleIsEnabled
                        || w.wheelState != ModuleReactionWheel.WheelState.Active)
                        continue;
                    double auth = w.authorityLimiter * 0.01;
                    t += new Vector3d(w.PitchTorque, w.RollTorque, w.YawTorque) * auth;
                    foreach (var r in w.resHandler.inputResources)
                        if (r.name == "ElectricCharge") rate += r.rate * auth;
                }
            torque = t;
            ecRate = rate;
            alpha = new Vector3d(moi.x > 0 ? t.x / moi.x : 0,
                                 moi.y > 0 ? t.y / moi.y : 0,
                                 moi.z > 0 ? t.z / moi.z : 0);
        }

        /// <summary>The thrust axis in ReferenceTransform coordinates: the
        /// first engine part's up, where PT applies its thrust.</summary>
        static Vector3d ThrustAxis(Vessel v)
        {
            Transform rt = v.ReferenceTransform;
            foreach (PartModule m in LowThrust.PtModules())
                return rt.InverseTransformDirection(m.part.transform.up);
            return Vector3d.up;
        }

        void Coast(Vessel v, double dt)
        {
            if (omega.sqrMagnitude < 1e-12) return;
            Rotate(v, omega, dt);
        }

        void Steer(Vessel v, double dt)
        {
            Transform rt = v.ReferenceTransform;
            Vector3d world = frame.DirectionToWorldSpace(target);
            Vector3d tb = rt.InverseTransformDirection(world).normalized;
            Vector3d ab = ThrustAxis(v).normalized;

            // Charge available for this frame scales the authority, as a
            // wheel short of EC loses torque.
            double need = ecRate * 3 * dt;
            double share = 1;
            if (need > 0)
            {
                double have = Available(v, "ElectricCharge");
                share = Math.Min(1, have / need);
            }
            int n = (int)Math.Ceiling(dt / STEP);
            if (n > MAX_STEPS) n = MAX_STEPS;
            double h = dt / n;
            Vector3d input = Vector3d.zero;      // integral of |u| per axis
            Quaternion turned = Quaternion.identity;   // body rotation so far
            for (int i = 0; i < n; i++)
            {
                // the thrust axis is body-fixed; the target, seen from the
                // body, turns the other way as the body turns
                Vector3d a = ab;
                Vector3d t = Quaternion.Inverse(turned) * (Vector3)tb;
                Vector3d cross = Vector3d.Cross(a, t);
                double sin = cross.magnitude;
                double angle = Math.Atan2(sin, Vector3d.Dot(a, t));
                Vector3d axis = sin > 1e-12 ? cross / sin : Vector3d.zero;
                // fastest along the axis that still stops on the target
                double acc = AxisAccel(axis) * MARGIN * share;
                Vector3d want = axis * Math.Min(Math.Sqrt(2 * acc * angle),
                                                angle / SETTLE);
                // (want has no part along the thrust axis: roll is damped)
                Vector3d dw = want - omega;
                Vector3d u = new Vector3d(
                    Clamp(Div(dw.x / RESPONSE, alpha.x * share)),
                    Clamp(Div(dw.y / RESPONSE, alpha.y * share)),
                    Clamp(Div(dw.z / RESPONSE, alpha.z * share)));
                omega += new Vector3d(u.x * alpha.x, u.y * alpha.y, u.z * alpha.z) * share * h;
                input += new Vector3d(Math.Abs(u.x), Math.Abs(u.y), Math.Abs(u.z)) * share * h;
                double mag = omega.magnitude;
                if (mag > 0)
                    turned = turned * Quaternion.AngleAxis(
                        (float)(mag * h * Mathf.Rad2Deg), (Vector3)(omega / mag));
            }
            ApplyBody(v, turned);
            double used = ecRate * (input.x + input.y + input.z);
            if (used > 0)
                ecUsed += Draw(v, "ElectricCharge", used);
            Vector3d tn = Quaternion.Inverse(turned) * (Vector3)tb;
            lastError = Vector3d.Angle(ab, tn);
        }

        static double AxisAccel(Vector3d axis)
        {
            // acceleration about a unit axis when each body axis may use
            // its whole torque: the axis that runs out first sets it
            double best = double.PositiveInfinity;
            if (Math.Abs(axis.x) > 1e-6) best = Math.Min(best, alpha.x / Math.Abs(axis.x));
            if (Math.Abs(axis.y) > 1e-6) best = Math.Min(best, alpha.y / Math.Abs(axis.y));
            if (Math.Abs(axis.z) > 1e-6) best = Math.Min(best, alpha.z / Math.Abs(axis.z));
            return double.IsInfinity(best) ? 0 : best;
        }

        static double Div(double a, double b) { return b > 0 ? a / b : 0; }
        static double Clamp(double x) { return x < -1 ? -1 : (x > 1 ? 1 : x); }

        static void Rotate(Vessel v, Vector3d w, double dt)
        {
            double mag = w.magnitude;
            ApplyBody(v, Quaternion.AngleAxis((float)(mag * dt * Mathf.Rad2Deg),
                                              (Vector3)(w / mag)));
        }

        /// <summary>Rotate the vessel by a rotation expressed in its
        /// ReferenceTransform's axes.</summary>
        static void ApplyBody(Vessel v, Quaternion body)
        {
            Quaternion rt = v.ReferenceTransform.rotation;
            Quaternion world = rt * body * Quaternion.Inverse(rt);
            v.SetRotation(world * v.transform.rotation);
        }

        /// <summary>Back off rails: give the rigid bodies the rate the
        /// vessel had on rails.</summary>
        void HandBack(Vessel v)
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
