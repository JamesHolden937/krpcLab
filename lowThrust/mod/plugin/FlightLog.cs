// One file per flight: GameData/LowThrust/PluginData/logs/flight-<date>.log,
// read whole.  Events are never suppressed; telemetry is one dense line at
// most once per LOG_INTERVAL_UT game seconds (it re-anchors under warp rather
// than catching up).

using System;
using System.Globalization;
using System.IO;

namespace LowThrust
{
    public class FlightLog : IDisposable
    {
        static readonly CultureInfo IC = CultureInfo.InvariantCulture;
        public readonly string path;
        StreamWriter w;
        readonly double interval;
        double next = double.NegativeInfinity;

        public static string Dir
        {
            get { return Path.Combine(KSPUtil.ApplicationRootPath, "GameData/LowThrust/PluginData/logs"); }
        }

        public FlightLog(double intervalUt)
        {
            interval = intervalUt;
            Directory.CreateDirectory(Dir);
            string stamp = DateTime.Now.ToString("yyyyMMdd-HHmmss", IC);
            path = Path.Combine(Dir, "flight-" + stamp + ".log");
            for (int i = 2; File.Exists(path); i++) path = Path.Combine(Dir, "flight-" + stamp + "-" + i + ".log");
            w = new StreamWriter(path) { AutoFlush = true };
            w.WriteLine("# lowThrust flight log, opened " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", IC));
        }

        public void Event(double ut, string text)
        {
            if (w == null) return;
            w.WriteLine(string.Format(IC, "[{0,11:F2}] {1}", ut, text));
        }

        public void Telemetry(double ut, string text)
        {
            if (w == null || ut < next) return;
            next = ut + interval;
            w.WriteLine(string.Format(IC, "[{0,11:F2}] {1}", ut, text));
        }

        public void Dispose()
        {
            if (w != null) w.Dispose();
            w = null;
        }
    }
}
