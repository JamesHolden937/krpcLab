// Where a vessel was launched from, for pilots that fly back to it (boosterland).
//
// At each launch the launch position is appended to
// GameData/KrpcLabLauncher/PluginData/launches.txt, keyed by the save folder and
// the root part's launchID.  Parts keep their launchID through decoupling, and
// FMRS / quicksaves keep it in the file, while the record lives outside the save
// so a revert or quickload cannot erase it.  KrpcLabLauncher.LaunchSite() looks
// the active vessel's parts up in it, and falls back to the coordinates of the
// vessel's own `launchedFrom` site (stock facilities and launch sites).
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KrpcLabLauncher
{
    public static class LaunchRecords
    {
        static readonly CultureInfo C = CultureInfo.InvariantCulture;

        public static string Path()
        {
            return System.IO.Path.Combine(KSPUtil.ApplicationRootPath,
                "GameData/KrpcLabLauncher/PluginData/launches.txt");
        }

        public static void Record(Vessel v)
        {
            if (v == null || v.rootPart == null) return;
            try
            {
                var p = Path();
                Directory.CreateDirectory(System.IO.Path.GetDirectoryName(p));
                // save|launchID|body|lat|lon|alt|site|ut
                var line = string.Join("|", new[] {
                    HighLogic.SaveFolder ?? "", v.rootPart.launchID.ToString(C),
                    v.mainBody.bodyName, v.latitude.ToString("R", C),
                    v.longitude.ToString("R", C), v.altitude.ToString("R", C),
                    (v.launchedFrom ?? "").Replace("|", "/"),
                    Planetarium.GetUniversalTime().ToString("R", C) });
                File.AppendAllText(p, line + "\n");
                Debug.Log("[KrpcLabLauncher] launch recorded: " + line);
            }
            catch (Exception e)
            {
                Debug.LogWarning("[KrpcLabLauncher] launch not recorded: " + e.Message);
            }
        }

        /// The latest record whose launchID one of the vessel's parts carries.
        public static string[] Find(Vessel v)
        {
            var p = Path();
            if (v == null || !File.Exists(p)) return null;
            var ids = new HashSet<string>();
            foreach (var part in v.parts) ids.Add(part.launchID.ToString(C));
            var save = HighLogic.SaveFolder ?? "";
            string[] best = null;
            foreach (var line in File.ReadAllLines(p))
            {
                var f = line.Split('|');
                if (f.Length < 8 || f[0] != save || !ids.Contains(f[1])
                    || f[2] != v.mainBody.bodyName) continue;
                best = f;                       // later lines win
            }
            return best;
        }

        /// The vessel's `launchedFrom` site as lat/lon/alt, or null.
        public static double[] Site(string name)
        {
            if (string.IsNullOrEmpty(name) || PSystemSetup.Instance == null) return null;
            double lat, lon, alt;
            var fac = PSystemSetup.Instance.GetSpaceCenterFacility(name);
            if (fac != null && fac.spawnPoints != null && fac.spawnPoints.Length > 0)
            {
                fac.spawnPoints[0].GetSpawnPointLatLonAlt(out lat, out lon, out alt);
                return new[] { lat, lon, alt };
            }
            var site = PSystemSetup.Instance.GetLaunchSite(name);
            if (site != null && site.spawnPoints != null && site.spawnPoints.Length > 0)
            {
                site.spawnPoints[0].GetSpawnPointLatLonAlt(out lat, out lon, out alt);
                return new[] { lat, lon, alt };
            }
            return null;
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class LaunchRecorder : MonoBehaviour
    {
        void Start() { GameEvents.onLaunch.Add(OnLaunch); }
        void OnDestroy() { GameEvents.onLaunch.Remove(OnLaunch); }

        void OnLaunch(EventReport report)
        {
            LaunchRecords.Record(FlightGlobals.ActiveVessel);
        }
    }

    [KRPCService(Name = "KrpcLabLaunch", GameScene = KRPC.Service.GameScene.Flight)]
    public static class KrpcLabLaunch
    {
        static readonly CultureInfo C = CultureInfo.InvariantCulture;

        /// <summary>The active vessel's launch site: "source|body|lat|lon|alt|site",
        /// source "record" (written at its launch) or "site" (its launchedFrom
        /// site's spawn point); "" when neither is known.</summary>
        [KRPCProcedure]
        public static string LaunchSite()
        {
            var v = FlightGlobals.ActiveVessel;
            if (v == null) return "";
            var f = LaunchRecords.Find(v);
            if (f != null)
                return string.Join("|", new[] { "record", f[2], f[3], f[4], f[5], f[6] });
            var s = LaunchRecords.Site(v.launchedFrom);
            if (s != null)
                return string.Join("|", new[] { "site", v.mainBody.bodyName,
                    s[0].ToString("R", C), s[1].ToString("R", C), s[2].ToString("R", C),
                    v.launchedFrom });
            return "";
        }
    }
}
