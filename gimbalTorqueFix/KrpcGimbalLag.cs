// KrpcGimbalLag: kRPC's auto-tune paced by the engine gimbals' response lag.
//
// kRPC's AttitudeController.DoAutoTuneAxis sets each axis's rate-loop bandwidth
// (2 zeta omega, from TimeToPeak and Overshoot) as if the torque arrived the
// tick it is commanded.  It models the control surfaces' slew (actuatorSlew,
// actuatorBackoff) but not the gimbals: ModuleGimbal moves a nozzle by
// Mathf.Lerp(current, target, gimbalResponseSpeed x dt) a physics tick, a first-
// order lag of ~1/gimbalResponseSpeed (0.125 s on the Vector).  At full thrust,
// where the gimbals are most of an axis's torque, the tuned loop limit-cycles:
// inputs pinned at +-1 flipping every tick, the nose swinging 15-40 deg/s
// (krpcLab LOG16371, LOG16376/16377).
//
// Fix (until kRPC models it): around DoAutoTuneAxis, cap the axis's bandwidth so
// its closed loop decays no faster than LagFactor x the axis's gimbal lag -- the
// lag weighted by the gimbals' share of the axis's available torque, so it fades
// out as the engines throttle down or stop -- scaling omega so the damping ratio
// (the overshoot target) is kept.  It only ever lowers the bandwidth.  The torque
// shares come from ModuleGimbal.GetPotentialTorque, which GimbalTorqueFix makes
// honest.  Without kRPC installed this patch does nothing.

using System;
using System.Collections.Generic;
using System.Reflection;
using HarmonyLib;
using UnityEngine;

namespace GimbalTorqueFix
{
    [HarmonyPatch]
    static class KrpcGimbalLag
    {
        // The closed loop's decay time is at least this many gimbal lags.
        const double LagFactor = 3.0;

        static Type controller;
        static FieldInfo twiceZetaOmega;
        static FieldInfo omegaSquared;
        static FieldInfo vesselField;
        static PropertyInfo internalVessel;
        static readonly Dictionary<object, KeyValuePair<double, Vector3d>> lagCache =
            new Dictionary<object, KeyValuePair<double, Vector3d>> ();

        static bool Prepare ()
        {
            controller = AccessTools.TypeByName ("KRPC.SpaceCenter.AutoPilot.AttitudeController");
            if (controller == null)
                return false;
            var method = AccessTools.Method (controller, "DoAutoTuneAxis");
            vesselField = AccessTools.Field (controller, "vessel");
            if (method == null || vesselField == null
                || AccessTools.Field (controller, "twiceZetaOmega") == null
                || AccessTools.Field (controller, "omegaSquared") == null) {
                Debug.LogWarning ("[GimbalTorqueFix] kRPC found but its AttitudeController has changed: gimbal lag not patched");
                return false;
            }
            twiceZetaOmega = AccessTools.Field (controller, "twiceZetaOmega");
            omegaSquared = AccessTools.Field (controller, "omegaSquared");
            internalVessel = AccessTools.Property (vesselField.FieldType, "InternalVessel");
            Debug.Log ("[GimbalTorqueFix] patching kRPC AttitudeController.DoAutoTuneAxis for gimbal lag");
            return internalVessel != null;
        }

        static MethodBase TargetMethod ()
        {
            return AccessTools.Method (controller, "DoAutoTuneAxis");
        }

        struct Saved
        {
            public bool changed;
            public double bandwidth, squared;
        }

        static void Prefix (object __instance, int index, Vector3d torque, out Saved __state)
        {
            __state = new Saved ();
            try {
                if (torque [index] <= 0)
                    return;
                double lag = Lag (__instance, torque) [index];
                if (lag <= 0)
                    return;
                var bw = (Vector3d)twiceZetaOmega.GetValue (__instance);
                double cap = 2.0 / (LagFactor * lag);   // 2 zeta omega <= 2 / (F tau)
                if (bw [index] <= cap)
                    return;
                var sq = (Vector3d)omegaSquared.GetValue (__instance);
                __state.changed = true;
                __state.bandwidth = bw [index];
                __state.squared = sq [index];
                double f = cap / bw [index];
                bw [index] = cap;
                sq [index] *= f * f;
                twiceZetaOmega.SetValue (__instance, bw);
                omegaSquared.SetValue (__instance, sq);
            } catch (Exception e) {
                Debug.LogWarning ("[GimbalTorqueFix] kRPC lag: " + e.Message);
            }
        }

        static void Postfix (object __instance, int index, Saved __state)
        {
            if (!__state.changed)
                return;
            var bw = (Vector3d)twiceZetaOmega.GetValue (__instance);
            var sq = (Vector3d)omegaSquared.GetValue (__instance);
            bw [index] = __state.bandwidth;
            sq [index] = __state.squared;
            twiceZetaOmega.SetValue (__instance, bw);
            omegaSquared.SetValue (__instance, sq);
        }

        // Per axis (pitch, roll, yaw): the gimbals' response lag weighted by their share
        // of the axis's available torque, seconds.  Once a physics tick per controller.
        static Vector3d Lag (object instance, Vector3d torque)
        {
            KeyValuePair<double, Vector3d> hit;
            double now = Time.fixedTime;
            if (lagCache.TryGetValue (instance, out hit) && hit.Key == now)
                return hit.Value;
            Vector3d lagged = Vector3d.zero;
            var vessel = internalVessel.GetValue (vesselField.GetValue (instance), null) as Vessel;
            if (vessel != null) {
                foreach (Part part in vessel.parts) {
                    foreach (PartModule module in part.Modules) {
                        var gimbal = module as ModuleGimbal;
                        if (gimbal == null || !gimbal.useGimbalResponseSpeed || gimbal.gimbalResponseSpeed <= 0)
                            continue;
                        Vector3 pos, neg;
                        gimbal.GetPotentialTorque (out pos, out neg);
                        double tau = 1.0 / gimbal.gimbalResponseSpeed;
                        // kN m, where kRPC's torque is N m (ITorqueProviderExtensions x 1000)
                        for (int i = 0; i < 3; i++)
                            lagged [i] += 500.0 * (Math.Abs (pos [i]) + Math.Abs (neg [i])) * tau;
                    }
                }
            }
            Vector3d lag = Vector3d.zero;
            for (int i = 0; i < 3; i++)
                lag [i] = torque [i] > 0 ? Math.Min (lagged [i] / torque [i], 1.0) : 0.0;
            if (lagCache.Count > 64)
                lagCache.Clear ();
            lagCache [instance] = new KeyValuePair<double, Vector3d> (now, lag);
            return lag;
        }
    }
}
