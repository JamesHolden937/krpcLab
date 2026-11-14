// GimbalTorqueFix: ModuleGimbal.GetPotentialTorque from the gimbal's own actuation.
//
// Stock ModuleGimbal.GetPotentialTorque (what SAS, kRPC's auto-tune and every
// other controller size their gains from) does not model what FixedUpdate does:
//
// - roll: every engine more than minRollOffset off the roll axis is credited
//   (distance from the axis) x thrust x (sin|deflection x| + sin|deflection y|):
//   both gimbal axes' full range at once, at the whole radial lever.  FixedUpdate
//   actually adds the roll input to an engine's pitch and yaw deflection by the
//   side of the axis it sits on, and the torque is what that tilt of the thrust
//   makes about the centre of mass;
// - pitch/yaw: the lever is the whole distance from the CoM rather than the arm
//   perpendicular to the force, and again the sum of both axes' deflection.
//
// On a 13-engine booster kRPC's roll figure was 3-6x what flight measured
// (krpcLab LOG16403/16415), so its roll loop barely pushed and the booster spun
// at 150-250 deg/s through the boostback.
//
// Fix: for each gimbal transform and each axis and sign, take the deflection
// stock's own GimbalRotation gives for that unit input (ranges, limiter,
// minRollOffset, the CoM-side flip, enable flags), rotate each of the engine's
// thrust transforms under it by that deflection, and sum r x dF about the CoM
// (force -thrustTransform.forward x finalThrust x multiplier at its position, as
// ModuleEngines.ThrustUpdate applies it).  Pure geometry: no transform is moved.
// Like stock it is a potential per axis, both signs reported as magnitudes, at the
// thrust the engines are making now; it says nothing of the gimbal's response lag.

using System.Collections.Generic;
using HarmonyLib;
using UnityEngine;

namespace GimbalTorqueFix
{
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public class Loader : MonoBehaviour
    {
        void Awake()
        {
            new Harmony("krpclab.gimbaltorquefix").PatchAll(typeof(Loader).Assembly);
            Debug.Log("[GimbalTorqueFix] patched ModuleGimbal.GetPotentialTorque");
        }
    }

    [HarmonyPatch(typeof(ModuleGimbal), nameof(ModuleGimbal.GetPotentialTorque))]
    static class PotentialTorque
    {
        static readonly Vector3[] Axes = { Vector3.right, Vector3.up, Vector3.forward };

        static bool Prefix(ModuleGimbal __instance, out Vector3 pos, out Vector3 neg)
        {
            pos = Vector3.zero;
            neg = Vector3.zero;
            try
            {
                Compute(__instance, ref pos, ref neg);
            }
            catch (System.Exception e)
            {
                Debug.LogWarning("[GimbalTorqueFix] " + e.Message);
                pos = neg = Vector3.zero;
            }
            return false;
        }

        static void Compute(ModuleGimbal g, ref Vector3 pos, ref Vector3 neg)
        {
            if (g.gimbalLock || !g.moduleIsEnabled || g.vessel == null)
                return;
            Transform reference = g.vessel.ReferenceTransform;
            if (reference == null || g.gimbalTransforms == null || g.initRots == null)
                return;
            if (g.engineMultsList == null)
                g.CreateEngineList();
            Vector3 com = g.vessel.CurrentCoM;
            Vector3 localCoM = reference.InverseTransformPoint(com);
            bool[] enabled = { g.enablePitch, g.enableRoll, g.enableYaw };
            int count = Mathf.Min(g.gimbalTransforms.Count, g.initRots.Count);
            for (int i = 0; i < count; i++)
            {
                Transform t = g.gimbalTransforms[i];
                if (t == null || t.parent == null || i >= g.engineMultsList.Count)
                    continue;
                // The thrusts this transform steers: position, direction, force.
                var points = new List<Vector3>();
                var dirs = new List<Vector3>();
                var forces = new List<float>();
                foreach (KeyValuePair<ModuleEngines, float> pair in g.engineMultsList[i])
                {
                    ModuleEngines engine = pair.Key;
                    if (engine == null || engine.finalThrust <= 0f || engine.thrustTransforms == null)
                        continue;
                    for (int j = 0; j < engine.thrustTransforms.Count; j++)
                    {
                        Transform tt = engine.thrustTransforms[j];
                        if (tt == null || !(tt == t || tt.IsChildOf(t)))
                            continue;
                        float mult = j < engine.thrustTransformMultipliers.Count
                            ? engine.thrustTransformMultipliers[j] : 1f;
                        points.Add(tt.position);
                        dirs.Add(-tt.forward);
                        forces.Add(engine.finalThrust * mult);
                    }
                }
                if (points.Count == 0)
                    continue;
                Quaternion parent = t.parent.rotation;
                Quaternion init = g.initRots[i];
                // Rotation from where the transform is now to its neutral pose.
                Quaternion toNeutral = (parent * init) * Quaternion.Inverse(t.rotation);
                Vector3 pivot = t.position;
                Vector3 baseTorque = Torque(points, dirs, forces, toNeutral, pivot, com);
                for (int axis = 0; axis < 3; axis++)
                {
                    if (!enabled[axis])
                        continue;
                    for (int s = -1; s <= 1; s += 2)
                    {
                        Vector3 d = g.GimbalRotation(t, s * Axes[axis], localCoM);
                        if (d.x == 0f && d.y == 0f)
                            continue;
                        Quaternion local = init
                            * Quaternion.AngleAxis(d.x * g.xMult, Vector3.right)
                            * Quaternion.AngleAxis(d.y * g.yMult, g.flipYZ ? Vector3.forward : Vector3.up);
                        Quaternion toDeflected = (parent * local) * Quaternion.Inverse(t.rotation);
                        Vector3 delta = Torque(points, dirs, forces, toDeflected, pivot, com) - baseTorque;
                        float about = Mathf.Abs(reference.InverseTransformDirection(delta)[axis]);
                        if (s > 0)
                            pos[axis] += about;
                        else
                            neg[axis] += about;
                    }
                }
            }
        }

        // Sum of r x F about the CoM with every thrust rotated by `rot` about `pivot`.
        static Vector3 Torque(List<Vector3> points, List<Vector3> dirs, List<float> forces,
                              Quaternion rot, Vector3 pivot, Vector3 com)
        {
            Vector3 sum = Vector3.zero;
            for (int k = 0; k < points.Count; k++)
            {
                Vector3 p = pivot + rot * (points[k] - pivot);
                Vector3 f = rot * dirs[k] * forces[k];
                sum += Vector3.Cross(p - com, f);
            }
            return sum;
        }
    }
}
