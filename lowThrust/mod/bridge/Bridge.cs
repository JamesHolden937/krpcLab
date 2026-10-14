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
        /// these NAME=VALUE config overrides.</summary>
        [KRPCProcedure]
        public static void Start(string target, IList<string> settings)
        {
            PanelState.SetTarget(target);
            Controller.overrides = new List<string>(settings);
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
