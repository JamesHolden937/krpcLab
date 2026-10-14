// The farm's handle on the mod: a kRPC service, "LowThrustBridge", in its own
// assembly (LowThrustKRPC.dll) so the mod itself needs no kRPC.  The farm
// harness (lowThrust/tools/ltfly.py) loads a save, starts a flight here and
// polls it; nothing in the flight goes over kRPC.

using System.Collections.Generic;
using KRPC.Service.Attributes;

namespace LowThrust.Bridge
{
    [KRPCService(Name = "LowThrustBridge", GameScene = KRPC.Service.GameScene.Flight)]
    public static class LowThrustBridge
    {
        /// <summary>Fly to target ("body: ap= pe= [i= lan= argpe=]") with
        /// these NAME=VALUE config overrides; MAX_DV=m/s and MAX_DAYS=days set
        /// the window's budget instead.</summary>
        [KRPCProcedure]
        public static void Start(string target, IList<string> settings)
        {
            PanelState.SetTarget(target);
            PanelState.maxDv = PanelState.maxDays = "";
            PanelState.picked = null;
            PanelState.options = new List<TradeOption>();
            var rest = new List<string>();
            foreach (string s in settings)
            {
                if (s.StartsWith("MAX_DV=")) PanelState.maxDv = s.Substring(7);
                else if (s.StartsWith("MAX_DAYS=")) PanelState.maxDays = s.Substring(9);
                else rest.Add(s);
            }
            Controller.overrides = rest;
            Controller.Instance.Begin();
        }

        /// <summary>[mode (Idle, Planning, Flying), the last result, the log
        /// path, flights ended since load].</summary>
        [KRPCProcedure]
        public static IList<string> Status()
        {
            return new List<string> { Controller.mode.ToString(), Controller.result, Controller.logPath,
                                      Controller.flights.ToString() };
        }

        /// <summary>The window's status lines.</summary>
        [KRPCProcedure]
        public static IList<string> Lines() { return new List<string>(Controller.status); }

        [KRPCProcedure]
        public static void Terminate()
        {
            if (Controller.Instance != null) Controller.Instance.Terminate();
        }
    }
}
