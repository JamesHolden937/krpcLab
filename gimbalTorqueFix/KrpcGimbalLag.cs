// KrpcGimbalLag: kRPC 0.7.0's auto-tune allowed for the engine gimbals' response lag.
//
// kRPC 0.7.0's AttitudeController.DoAutoTuneAxis places each axis's rate-loop poles
// (bandwidth 2 zeta w0 from TimeToPeak and Overshoot, w0^2) as if the torque arrived
// the tick it is commanded.  ModuleGimbal moves a nozzle a fraction
// gimbalResponseSpeed x dt of the way to its target each physics tick, a first-order
// lag (~0.125 s on the Vector), so on a vessel steered mostly by gimbals the loop is
// lightly damped and limit-cycles: inputs pinned at +-1, the nose swinging 15-40
// deg/s (krpcLab LOG16371).
//
// This is a backport of kRPC's own fix (krpc/krpc#1348, unreleased 0.8.0), for
// 0.7.0's DoAutoTuneAxis.  Delete it once the farm and the live game run a kRPC that
// has it (the patch then refuses itself: 0.8.0 has a gimbalLag field).
//   - The lag per axis: each gimbal's time constant -dt / ln(1 - speed dt), weighted
//     by its share of the axis's available torque, at most the largest one.  It fades
//     out as the engines throttle down or stop.
//   - Prefix: cap the bandwidth so the lag's pole 1/lag - 2 zeta w0 stays at or above
//     w0, i.e. 2 zeta w0 + w0 <= 1/lag, scaling w0 so zeta (the overshoot) is kept.
//   - Postfix: replace the PI gains with the ones that place the poles of the lagged
//     plant alpha / (s (1 + lag s)) at s^2 + 2 zeta w0 s + w0^2 and a third pole at
//     1/lag - 2 zeta w0: Kp alpha = bw + lag (w0^2 - bw^2), Ki alpha = w0^2 (1 - lag bw).
//     With no lag these are kRPC's gains unchanged.  The bandwidth floor kRPC applies
//     in between is read back from the gains it set.
// The torque shares come from ModuleGimbal.GetPotentialTorque, which GimbalTorqueFix
// makes honest.  Without kRPC installed this patch does nothing.

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
        static Type controller;
        static FieldInfo twiceZetaOmega;
        static FieldInfo omegaSquared;
        static FieldInfo unflooredKp;
        static FieldInfo vesselField;
        static PropertyInfo internalVessel;
        static PropertyInfo pidKp, pidKi;
        static MethodInfo pidSet;
        static readonly Dictionary<object, KeyValuePair<double, Vector3d>> lagCache =
            new Dictionary<object, KeyValuePair<double, Vector3d>> ();

        static bool Prepare ()
        {
            controller = AccessTools.TypeByName ("KRPC.SpaceCenter.AutoPilot.AttitudeController");
            if (controller == null)
                return false;
            if (AccessTools.Field (controller, "gimbalLag") != null) {
                Debug.Log ("[GimbalTorqueFix] kRPC models the gimbal lag itself: not patched (delete KrpcGimbalLag)");
                return false;
            }
            var method = AccessTools.Method (controller, "DoAutoTuneAxis");
            var pid = AccessTools.TypeByName ("KRPC.SpaceCenter.AutoPilot.PIDController");
            vesselField = AccessTools.Field (controller, "vessel");
            twiceZetaOmega = AccessTools.Field (controller, "twiceZetaOmega");
            omegaSquared = AccessTools.Field (controller, "omegaSquared");
            unflooredKp = AccessTools.Field (controller, "unflooredKp");
            if (pid != null) {
                pidKp = AccessTools.Property (pid, "Kp");
                pidKi = AccessTools.Property (pid, "Ki");
                pidSet = AccessTools.Method (pid, "SetParameters");
            }
            internalVessel = vesselField == null ? null : AccessTools.Property (vesselField.FieldType, "InternalVessel");
            if (method == null || twiceZetaOmega == null || omegaSquared == null || unflooredKp == null
                || internalVessel == null || pidKp == null || pidKi == null || pidSet == null) {
                Debug.LogWarning ("[GimbalTorqueFix] kRPC found but its AttitudeController has changed: gimbal lag not patched");
                return false;
            }
            Debug.Log ("[GimbalTorqueFix] patching kRPC AttitudeController.DoAutoTuneAxis for gimbal lag (krpc#1348 backport)");
            return true;
        }

        static MethodBase TargetMethod ()
        {
            return AccessTools.Method (controller, "DoAutoTuneAxis");
        }

        struct Saved
        {
            public double lag;
            public bool capped;
            public double bandwidth, squared;          // kRPC's own, restored after
            public double cappedBw, cappedSq;          // what the call ran with
        }

        // The PI gains times the angular acceleration for the lagged plant.
        static void LagGains (double bw, double sq, double lag, out double kp, out double ki)
        {
            kp = bw + lag * (sq - bw * bw);
            ki = sq * (1.0 - lag * bw);
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
                var sq = (Vector3d)omegaSquared.GetValue (__instance);
                __state.lag = lag;
                __state.bandwidth = bw [index];
                __state.squared = sq [index];
                double cap = 1.0 / (lag * (bw [index] + Math.Sqrt (Math.Max (sq [index], 0.0))));
                if (cap < 1) {
                    __state.capped = true;
                    bw [index] *= cap;
                    sq [index] *= cap * cap;
                    twiceZetaOmega.SetValue (__instance, bw);
                    omegaSquared.SetValue (__instance, sq);
                }
                __state.cappedBw = bw [index];
                __state.cappedSq = sq [index];
            } catch (Exception e) {
                __state.lag = 0;
                Debug.LogWarning ("[GimbalTorqueFix] kRPC lag: " + e.Message);
            }
        }

        static void Postfix (object __instance, object pid, int index, Vector3d torque, Vector3d moi, Saved __state)
        {
            if (__state.lag <= 0)
                return;
            try {
                if (__state.capped) {
                    var bw = (Vector3d)twiceZetaOmega.GetValue (__instance);
                    var sq = (Vector3d)omegaSquared.GetValue (__instance);
                    bw [index] = __state.bandwidth;
                    sq [index] = __state.squared;
                    twiceZetaOmega.SetValue (__instance, bw);
                    omegaSquared.SetValue (__instance, sq);
                }
                if (torque [index] <= 0 || moi [index] <= 0)
                    return;
                double accelerationInv = moi [index] / torque [index];
                double kp, ki;
                LagGains (__state.cappedBw, __state.cappedSq, __state.lag, out kp, out ki);
                ((double[])unflooredKp.GetValue (__instance)) [index] = kp * accelerationInv;
                // The bandwidth after kRPC's floor, read back from the gains it set
                double floored = (double)pidKp.GetValue (pid, null) / accelerationInv;
                double flooredSq = (double)pidKi.GetValue (pid, null) / accelerationInv;
                LagGains (floored, flooredSq, __state.lag, out kp, out ki);
                pidSet.Invoke (pid, new object[] { kp * accelerationInv, ki * accelerationInv, 0.0, -1.0, 1.0 });
            } catch (Exception e) {
                Debug.LogWarning ("[GimbalTorqueFix] kRPC lag gains: " + e.Message);
            }
        }

        // A gimbal moving a fraction speed x dt of the way to its target each tick:
        // its time constant, s.  The game clamps the fraction to 1, which is no lag.
        static double TimeConstant (double speed, double dt)
        {
            double fraction = speed * dt;
            if (fraction <= 0 || fraction >= 1)
                return 0.0;
            return -dt / Math.Log (1.0 - fraction);
        }

        // Per axis (pitch, roll, yaw): the gimbals' time constant weighted by their share
        // of the axis's available torque, at most the largest one, s.  Once a physics
        // tick per controller.
        static Vector3d Lag (object instance, Vector3d torque)
        {
            KeyValuePair<double, Vector3d> hit;
            double now = Time.fixedTime;
            if (lagCache.TryGetValue (instance, out hit) && hit.Key == now)
                return hit.Value;
            Vector3d lagged = Vector3d.zero;
            double maxLag = 0;
            double dt = Time.fixedDeltaTime;
            var vessel = internalVessel.GetValue (vesselField.GetValue (instance), null) as Vessel;
            if (vessel != null) {
                foreach (Part part in vessel.parts) {
                    foreach (PartModule module in part.Modules) {
                        var gimbal = module as ModuleGimbal;
                        if (gimbal == null || !gimbal.useGimbalResponseSpeed)
                            continue;
                        double tau = TimeConstant (gimbal.gimbalResponseSpeed, dt);
                        if (tau <= 0)
                            continue;
                        maxLag = Math.Max (maxLag, tau);
                        Vector3 pos, neg;
                        gimbal.GetPotentialTorque (out pos, out neg);
                        // kN m, where kRPC's torque is N m (ITorqueProviderExtensions x 1000)
                        for (int i = 0; i < 3; i++)
                            lagged [i] += 500.0 * (Math.Abs (pos [i]) + Math.Abs (neg [i])) * tau;
                    }
                }
            }
            Vector3d lag = Vector3d.zero;
            for (int i = 0; i < 3; i++)
                lag [i] = torque [i] > 0 ? Math.Min (lagged [i] / torque [i], maxLag) : 0.0;
            if (lagCache.Count > 64)
                lagCache.Clear ();
            lagCache [instance] = new KeyValuePair<double, Vector3d> (now, lag);
            return lag;
        }
    }
}
