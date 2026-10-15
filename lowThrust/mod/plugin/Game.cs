// The game as the planner sees it, and the honest engine on rails.
//
// Frames.  The planner works in one right-handed inertial frame per body, x-y
// its equator, z its north pole.  KSP's orbital elements (inclination, LAN,
// argument of periapsis) are measured in exactly such a frame, so states are
// read from and written to the game through the elements -- never through
// world-space vectors, whose axes turn with the body below its inverse-
// rotation altitude.  Every body's frame has the same axes, so positions
// relative to a parent add up as the planner's do, and the planner's LAN is
// KSP's.
//
// The engine.  On rails the game does not run the engines; Engines does it as
// they would: the active engines' vacuum thrust and Isp (thrust limiter
// applied), mass flow F / (Isp g0), every propellant drawn in its ratio
// through the part's own flow rules -- ElectricCharge included -- and the
// thrust cut in proportion to the scarcest one.  The thrust is applied along
// the engines' thrust axis as the vessel points it.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace LowThrust
{
    public static class GameSystem
    {
        /// <summary>{name: Body} for every body in the game, on its rails (its
        /// elements, the mean anomaly carried back to UT 0).</summary>
        public static Dictionary<string, Body> Build()
        {
            var outB = new Dictionary<string, Body>();
            foreach (CelestialBody cb in FlightGlobals.Bodies) Make(cb, outB);
            return outB;
        }

        static Body Make(CelestialBody cb, Dictionary<string, Body> outB)
        {
            Body b;
            if (outB.TryGetValue(cb.bodyName, out b)) return b;
            double atm = cb.atmosphere ? cb.atmosphereDepth : 0.0;
            global::Orbit o = cb.orbit;
            if (o == null || cb.referenceBody == null || cb.referenceBody == cb)
                b = new Body(cb.bodyName, cb.gravParameter, cb.Radius, atm);
            else
            {
                Body parent = Make(cb.referenceBody, outB);
                double n = Math.Sqrt(parent.mu / Math.Pow(o.semiMajorAxis, 3));
                double m0 = Py.Mod(o.meanAnomalyAtEpoch - n * o.epoch, 2 * Math.PI);
                b = new Body(cb.bodyName, cb.gravParameter, cb.Radius, atm, parent, o.semiMajorAxis,
                             o.eccentricity, Py.Radians(o.inclination), Py.Radians(o.LAN),
                             Py.Radians(o.argumentOfPeriapsis), m0);
                parent.children.Add(b);
                b.gameSoi = cb.sphereOfInfluence;
            }
            if (cb.atmosphere)
            {
                // density every Atmosphere.STEP m, as the game computes it at the
                // body's base temperature (no latitude or sun terms)
                int n = (int)Math.Ceiling(cb.atmosphereDepth / Atmosphere.STEP) + 1;
                var rho = new double[n];
                for (int i = 0; i < n; i++)
                {
                    double alt = i * Atmosphere.STEP;
                    rho[i] = alt >= cb.atmosphereDepth ? 0.0 : cb.GetDensity(cb.GetPressure(alt), cb.GetTemperature(alt));
                }
                b.air = new Atmosphere(cb.atmosphereDepth, rho);
            }
            b.spin = cb.rotates && cb.rotationPeriod > 0 ? 2 * Math.PI / cb.rotationPeriod : 0.0;
            b.terrain = cb.pqsController != null ? cb.pqsController.mapMaxHeight : 0.0;
            outB[cb.bodyName] = b;
            return b;
        }

        /// <summary>The vessel's drag area (m^2: drag = dynamic pressure x
        /// this) with the air on its nose (nose: the control axis forward; or
        /// on its tail), from the parts' drag cubes as the game would set them
        /// at Mach 6: a prior, until a pass measures it.</summary>
        public static double DragArea(Vessel v, bool nose)
        {
            Vector3 flow = (nose ? 1f : -1f) * v.ReferenceTransform.up;
            double area = 0.0;
            foreach (Part p in v.parts)
            {
                if (p.DragCubes == null || p.DragCubes.None) continue;
                p.DragCubes.SetDrag(-p.partTransform.InverseTransformDirection(flow), 6f);
                area += p.DragCubes.AreaDrag;
            }
            return area * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragMultiplier;
        }

        /// <summary>The largest error between the rails built here and the
        /// game's own orbits horizon seconds ahead: (body, position error m,
        /// SOI error m).</summary>
        public static string Check(Dictionary<string, Body> sys, double ut, double horizon)
        {
            string worst = "";
            double err = -1, soiErr = 0;
            foreach (CelestialBody cb in FlightGlobals.Bodies)
            {
                Body b;
                if (!sys.TryGetValue(cb.bodyName, out b) || b.parent == null) continue;
                V3 game = StateOf(cb.orbit, ut + horizon, null);
                double e = (game - b.Position(ut + horizon)).Norm;
                if (e > err) { err = e; worst = b.name; soiErr = Math.Abs(b.Soi - b.gameSoi); }
            }
            return string.Format("worst body position error {0:F0} m ({1}, {2:G2} s ahead), SOI {3:F0} m",
                                 err, worst, horizon, soiErr);
        }

        /// <summary>Position (and velocity) on a game orbit at UT, from its
        /// elements, in the planner's frame about its reference body.</summary>
        public static V3 StateOf(global::Orbit o, double ut, V3[] vel)
        {
            double nu = o.TrueAnomalyAtUT(ut);
            V3 r, v;
            LowThrust.Orbit.StateFromElements(o.referenceBody.gravParameter, o.semiMajorAxis, o.eccentricity,
                                              Py.Radians(o.inclination), Py.Radians(o.LAN),
                                              Py.Radians(o.argumentOfPeriapsis), nu, out r, out v);
            if (vel != null) vel[0] = v;
            return r;
        }

        /// <summary>Set a game orbit to the conic through (r, v) about body at
        /// UT, through its elements.</summary>
        public static void SetOrbit(global::Orbit o, CelestialBody body, V3 r, V3 v, double ut)
        {
            double[] el = Bodies.Angles(r, v, body.gravParameter);
            double a = el[0], e = el[1], nu = el[5];
            double M;
            if (e < 1) M = Bodies.MeanFromTrue(nu, e);
            else
            {
                double nuS = nu > Math.PI ? nu - 2 * Math.PI : nu;
                double F = 2 * Py.Atanh(Math.Sqrt((e - 1) / (e + 1)) * Math.Tan(nuS / 2));
                M = e * Math.Sinh(F) - F;
            }
            o.SetOrbit(Py.Degrees(el[2]), e, a, Py.Degrees(el[3]), Py.Degrees(el[4]), M, ut, body);
        }

        public static CelestialBody Find(string name)
        {
            foreach (CelestialBody cb in FlightGlobals.Bodies) if (cb.bodyName == name) return cb;
            return null;
        }

        /// <summary>The planner's RTN basis of a vessel in world space (R out,
        /// T along-track, N the orbit normal).  World space is left-handed: the
        /// right-handed normal r x v reads -(r x v) in it.</summary>
        public static void WorldRtn(Vessel v, out Vector3d R, out Vector3d T, out Vector3d N)
        {
            Vector3d r = v.CoMD - v.mainBody.position;
            Vector3d vel = v.obt_velocity;
            R = r.normalized;
            T = (vel - R * Vector3d.Dot(vel, R)).normalized;
            N = -Vector3d.Cross(r, vel).normalized;
        }
    }

    /// <summary>The active engines on rails.</summary>
    public class Engines
    {
        public class Prop
        {
            public int id;
            public string name;
            public double unitsPerSecond;      // at full thrust, all engines
            public ResourceFlowMode mode;
            public Part part;
        }

        public readonly List<ModuleEngines> list = new List<ModuleEngines>();
        public readonly List<Prop> props = new List<Prop>();
        public double thrustN, ispS, massFlowKg;    // full throttle
        Vessel vessel;

        public Engines(Vessel v)
        {
            vessel = v;
            Refresh();
        }

        /// <summary>The engines that are lit, their thrust, Isp and
        /// propellant draw.</summary>
        public void Refresh()
        {
            list.Clear();
            props.Clear();
            thrustN = 0;
            double flowT = 0;      // t/s
            foreach (Part p in vessel.parts)
                foreach (PartModule pm in p.Modules)
                {
                    var e = pm as ModuleEngines;
                    if (e == null || !e.EngineIgnited || e.flameout || !e.isEnabled) continue;
                    double isp = e.atmosphereCurve.Evaluate(0f);
                    double f = e.maxThrust * e.thrustPercentage / 100.0;     // kN
                    if (isp <= 0 || f <= 0) continue;
                    list.Add(e);
                    double mdot = f / (isp * Sim.G0);                         // t/s
                    thrustN += f * 1000.0;
                    flowT += mdot;
                    // the propellants as the part defines them: another mod may
                    // take one off the module at run time (Persistent Thrust
                    // takes ElectricCharge, to draw it itself), and an engine
                    // flown here must still pay for it
                    var defs = Defined(p, e);
                    double mix = 0;
                    foreach (var d in defs) mix += d.Value * d.Key.density;
                    if (mix <= 0) continue;
                    foreach (var d in defs)
                        props.Add(new Prop
                        {
                            id = d.Key.id, name = d.Key.name, unitsPerSecond = d.Value * mdot / mix,
                            mode = d.Key.resourceFlowMode, part = p
                        });
                }
            massFlowKg = flowT * 1000.0;
            ispS = flowT > 0 ? thrustN / 1000.0 / (flowT * Sim.G0) : 0;
        }

        /// <summary>(resource, ratio) of each PROPELLANT in the engine module's
        /// part config (matched by engineID), else the module's own list.</summary>
        static List<KeyValuePair<PartResourceDefinition, double>> Defined(Part p, ModuleEngines e)
        {
            var r = new List<KeyValuePair<PartResourceDefinition, double>>();
            ConfigNode cfg = p.partInfo != null ? p.partInfo.partConfig : null;
            if (cfg != null)
                foreach (ConfigNode m in cfg.GetNodes("MODULE"))
                {
                    string name = m.GetValue("name") ?? "";
                    if (!name.StartsWith("ModuleEngines")) continue;
                    string id = m.GetValue("engineID");
                    if (id != null && id != e.engineID) continue;
                    foreach (ConfigNode pn in m.GetNodes("PROPELLANT"))
                    {
                        PartResourceDefinition def = PartResourceLibrary.Instance.GetDefinition(pn.GetValue("name"));
                        double ratio;
                        if (def != null && double.TryParse(pn.GetValue("ratio"), System.Globalization.NumberStyles.Float,
                                                           System.Globalization.CultureInfo.InvariantCulture, out ratio))
                            r.Add(new KeyValuePair<PartResourceDefinition, double>(def, ratio));
                    }
                    if (r.Count > 0) return r;
                }
            foreach (Propellant pr in e.propellants)
                if (pr.resourceDef != null) r.Add(new KeyValuePair<PartResourceDefinition, double>(pr.resourceDef, pr.ratio));
            return r;
        }

        /// <summary>Draw the propellants for throttle x dt seconds at full
        /// thrust: the share of it the scarcest propellant allowed (0..1).</summary>
        public double Draw(double throttle, double dt)
        {
            if (throttle <= 0 || dt <= 0) return 0;
            double share = 1;
            foreach (Prop p in props)
            {
                double want = p.unitsPerSecond * throttle * dt;
                if (want > 0) share = Math.Min(share, Have(p) / want);
            }
            share = Math.Max(0, Math.Min(1, share));
            if (share <= 0) return 0;
            // drawn through the engine part's own flow rules; what it got is
            // the share delivered
            double got = share;
            foreach (Prop p in props)
            {
                double want = p.unitsPerSecond * throttle * dt * share;
                if (want <= 0) continue;
                double g = p.part.RequestResource(p.id, want, p.mode, false);
                got = Math.Min(got, share * g / want);
            }
            return got;
        }

        /// <summary>What the vessel holds of a propellant, units.</summary>
        public double Have(Prop p)
        {
            double amount, max;
            vessel.GetConnectedResourceTotals(p.id, out amount, out max);
            return amount;
        }

        /// <summary>Each propellant: rate at full thrust, flow mode, what the
        /// vessel holds and what a simulated request offers (the log's check
        /// that the draw sees the tanks).</summary>
        public string Describe()
        {
            var parts = new List<string>();
            foreach (Prop p in props)
                parts.Add(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                                        "{0} {1:G4}/s {2} holds {3:G6} offers {4:G6}", p.name, p.unitsPerSecond, p.mode,
                                        Have(p), p.part.RequestResource(p.id, 1e9, p.mode, true)));
            return "propellants: " + string.Join("; ", parts.ToArray());
        }

        /// <summary>The longest thrusting span (s at full throttle) that draws
        /// at most share of what the vessel holds of each propellant.</summary>
        public double SpanFor(double share)
        {
            double span = double.PositiveInfinity;
            foreach (Prop p in props)
                if (p.unitsPerSecond > 0) span = Math.Min(span, share * Have(p) / p.unitsPerSecond);
            return span;
        }

        /// <summary>The thrust axis in world space (the direction the engines
        /// push the vessel).</summary>
        public Vector3d AxisWorld()
        {
            Vector3d sum = Vector3d.zero;
            foreach (ModuleEngines e in list)
                foreach (Transform t in e.thrustTransforms)
                    sum += -(Vector3d)t.forward;
            if (sum.sqrMagnitude > 0) return sum.normalized;
            return vessel.ReferenceTransform.up;
        }

        /// <summary>Every engine's full vacuum thrust, lit or not, N (a PLAN
        /// before the engines are on).</summary>
        public static double FullThrust(Vessel v)
        {
            double n = 0;
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                {
                    var e = pm as ModuleEngines;
                    if (e != null && e.isEnabled) n += e.maxThrust * e.thrustPercentage / 100.0 * 1000.0;
                }
            return n;
        }

        /// <summary>Vessel mass now, kg (KSP does not update it on rails; its
        /// parts' resources it does).</summary>
        public static double Mass(Vessel v)
        {
            double t = 0;
            foreach (Part p in v.parts) t += p.mass + p.GetResourceMass();
            return t * 1000.0;
        }
    }

    /// <summary>Rails warp the autopilot sets, and the altitude limits it
    /// relaxes while it flies (restored after).</summary>
    public static class Warp
    {
        static readonly Dictionary<CelestialBody, float[]> saved = new Dictionary<CelestialBody, float[]>();

        public static float[] Rates { get { return TimeWarp.fetch.warpRates; } }

        public static bool OnRails(Vessel v)
        {
            return v.packed && TimeWarp.WarpMode == TimeWarp.Modes.HIGH && TimeWarp.CurrentRateIndex > 0;
        }

        /// <summary>Rails warp allowed down to the atmosphere's top (or the
        /// highest terrain plus a margin) on every body.</summary>
        public static void Relax(double terrainMargin)
        {
            foreach (CelestialBody cb in FlightGlobals.Bodies)
            {
                if (cb.timeWarpAltitudeLimits == null || saved.ContainsKey(cb)) continue;
                saved[cb] = (float[])cb.timeWarpAltitudeLimits.Clone();
                double floor = cb.atmosphere ? cb.atmosphereDepth
                    : (cb.pqsController != null ? cb.pqsController.mapMaxHeight : 0.0) + terrainMargin;
                for (int i = 1; i < cb.timeWarpAltitudeLimits.Length; i++)
                    cb.timeWarpAltitudeLimits[i] = (float)Math.Min(cb.timeWarpAltitudeLimits[i], floor);
            }
        }

        public static void Restore()
        {
            foreach (var kv in saved) kv.Key.timeWarpAltitudeLimits = kv.Value;
            saved.Clear();
            lastSet = -1;
            dropped = int.MaxValue;
            droppedEnd = double.NaN;
            droppedBody = null;
        }

        /// <summary>The highest index up to want the game allows at the
        /// vessel's altitude and before its next SOI change.</summary>
        public static int Cap(Vessel v, int want)
        {
            if (v.LandedOrSplashed) return 0;
            double alt = v.mainBody.GetAltitude(v.CoM);
            while (want > 0 && alt < TimeWarp.fetch.GetAltitudeLimit(want, v.mainBody)) want--;
            // The game's own clamp (TimeWarp.ClampRateToOrbitTransitions):
            // above index 3 it drops to 3 once the SOI change is closer than
            // the travel of slowing to rate 3 in one second plus 50 s.  Asked
            // for more each step, it drops again each step (50 times a second,
            // a log line each, 1132 of them in one live Minmus flight).
            global::Orbit o = v.orbit;
            float[] rates = Rates;
            if (o != null && o.patchEndTransition != global::Orbit.PatchTransitionType.FINAL && rates.Length > 3)
            {
                // the game measures from the rate it is at, which lags a set
                double left = o.EndUT - Planetarium.GetUniversalTime();
                double from = Math.Max(Planetarium.TimeScale, 0);
                // ... and checks again a step later, when a step of the rate
                // asked for has gone by
                while (want > 3 && left < (Math.Max(rates[want], from) + rates[3]) / 2.0 + 60.0
                                          + 2 * rates[want] * Time.fixedDeltaTime) want--;
                // and once it has dropped the rate itself, hold under that
                // until the SOI change it dropped for has passed (its time
                // shifts a little whenever thrust rewrites the orbit)
                if (droppedBody == v.mainBody && Planetarium.GetUniversalTime() < droppedEnd + 60 && want > dropped)
                    want = Math.Max(dropped, 1);
            }
            return want;
        }

        static int lastSet = -1, dropped = int.MaxValue;
        static double droppedEnd = double.NaN;
        static CelestialBody droppedBody;

        /// <summary>Whether the game lowered the rate since the last Set
        /// (its clamp ahead of an SOI change): remembered for this patch.</summary>
        public static void NoteDrop(Vessel v)
        {
            int now = TimeWarp.CurrentRateIndex;
            if (lastSet > 0 && now < lastSet && v.orbit != null
                && v.orbit.patchEndTransition != global::Orbit.PatchTransitionType.FINAL)
            {
                dropped = now;
                droppedEnd = v.orbit.EndUT;
                droppedBody = v.mainBody;
            }
        }

        /// <summary>Set the rails rate index at once (KSP eases between rates
        /// over seconds otherwise).</summary>
        public static void Set(int index)
        {
            if (TimeWarp.WarpMode != TimeWarp.Modes.HIGH)
            {
                TimeWarp.fetch.Mode = TimeWarp.Modes.HIGH;
                TimeWarp.SetRate(0, true, false);
            }
            if (TimeWarp.CurrentRateIndex != index) TimeWarp.SetRate(index, true, false);
            lastSet = index;
        }
    }
}
