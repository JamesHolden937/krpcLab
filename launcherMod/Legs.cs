// Each landing leg's live suspension and damage limits, for touchdown limits
// (common/touchdown.py).  Read by reflection where KSP keeps them private:
// ModuleWheelSuspension's spring and damper are set at runtime from the load
// (spring = springRatio x sprung weight; damper ~ sqrt(spring x damperRatio x
// a mass-dependent fudge)), so only the game knows the numbers it is flying.
// ModuleWheelDamage explodes a leg when its down force jumps by
// impactTolerance x explodeMultiplier in one physics tick (scaled by local g / g0).
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using KRPC.Service.Attributes;
using ModuleWheels;
using UnityEngine;

namespace KrpcLabLauncher
{
    [KRPCService(Name = "KrpcLabLegs", GameScene = KRPC.Service.GameScene.Flight)]
    public static class KrpcLabLegs
    {
        static readonly CultureInfo C = CultureInfo.InvariantCulture;
        const BindingFlags Any = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

        static float Private(object o, string name)
        {
            var f = o.GetType().GetField(name, Any);
            return f == null ? float.NaN : Convert.ToSingle(f.GetValue(o));
        }

        /// <summary>One line per leg of the active vessel:
        /// "title|footX|footY|footZ|travel|spring|damper|springRatio|damperRatio|
        /// impactTol|explodeMult|crashTol|frictionPeak|wheelRadius|deployed",
        /// the foot (wheel collider) in the vessel's reference frame (origin
        /// the centre of mass, kRPC's axes).</summary>
        [KRPCProcedure]
        public static IList<string> Legs()
        {
            var rows = new List<string>();
            var v = FlightGlobals.ActiveVessel;
            if (v == null) return rows;
            var inv = Quaternion.Inverse(v.ReferenceTransform.rotation);
            foreach (var part in v.parts)
            {
                var susp = part.FindModuleImplementing<ModuleWheelSuspension>();
                var bas = part.FindModuleImplementing<ModuleWheelBase>();
                if (susp == null || bas == null) continue;
                var dmg = part.FindModuleImplementing<ModuleWheelDamage>();
                var dep = part.FindModuleImplementing<ModuleWheelDeployment>();
                Transform t = part.FindModelTransform(bas.wheelColliderTransformName);
                Vector3 foot = inv * ((t != null ? t.position : part.transform.position)
                                      - (Vector3)v.CoMD);
                var f = new[] {
                    part.partInfo != null ? part.partInfo.title.Replace("|", "/") : part.name,
                    foot.x.ToString("R", C), foot.y.ToString("R", C), foot.z.ToString("R", C),
                    susp.suspensionDistance.ToString("R", C),
                    Private(susp, "spring").ToString("R", C),
                    Private(susp, "damper").ToString("R", C),
                    susp.springRatio.ToString("R", C), susp.damperRatio.ToString("R", C),
                    (dmg != null ? dmg.impactTolerance : float.NaN).ToString("R", C),
                    (dmg != null ? dmg.explodeMultiplier : float.NaN).ToString("R", C),
                    part.crashTolerance.ToString("R", C),
                    bas.frictionPeak.ToString("R", C), bas.radius.ToString("R", C),
                    (dep == null || Mathf.Abs(dep.Position - dep.deployedPosition) < 0.01f) ? "1" : "0" };
                rows.Add(string.Join("|", f));
            }
            return rows;
        }
    }
}
