using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace BoosterlandTimeScale
{
    /// <summary>
    /// Run the flight faster than real time **without coarsening the physics
    /// step**, which is the thing KSP's own physics warp will not do.
    ///
    /// KSP's physics warp multiplies Time.fixedDeltaTime (0.02 -> 0.08 at 4x)
    /// so the CPU cost per real second stays flat.  That is a direct
    /// integration-fidelity loss, and it is worst exactly where this vehicle
    /// lives: dense air, under thrust, with a controller closing on a target.
    ///
    /// This does the opposite trade.  Time.fixedDeltaTime is left alone, so
    /// every physics step is the same step the vehicle flies at 1x; what
    /// changes is Time.timeScale, which makes Unity run *more* of those steps
    /// per real second.  Same trajectory, more of it per wall-clock second,
    /// linear CPU cost.  KSP's physics is single-threaded, so the ceiling is
    /// one core.
    ///
    /// **The binding constraint is not the CPU, it is control quantization.**
    /// Unity applies control input once per frame, so the autopilot's commands
    /// land on a grid of `timeScale / fps` game-seconds.  Push timeScale up
    /// without pushing fps up and you get faithful physics flown by a degraded
    /// controller -- which is the same failure as running the game at 10 fps,
    /// arriving from the other direction.  So the knob here is that grid:
    /// `quant_s` is the largest control quantum allowed, Time.maximumDeltaTime
    /// is set to it, and the achievable speed follows as `quant_s * fps`.
    ///
    /// Control file, re-read every half second, at the instance root
    /// (KSPUtil.ApplicationRootPath + "timescale.txt"):
    ///
    ///     mode      = off | fixed | adaptive
    ///     scale     = 2.0     # fixed mode: the multiplier to hold
    ///     max_scale = 4.0     # adaptive mode: never go past this
    ///     quant_s   = 0.05    # control quantum ceiling, game-seconds/frame
    ///
    /// and a status file next to it so the harness can read back what was
    /// actually achieved rather than what was asked for.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class TimeScaleDriver : MonoBehaviour
    {
        const string Tag = "[BoosterlandTimeScale] ";

        // Unity's own default, restored on the way out.
        float stockMaxDelta = 0.04f;

        string controlPath;
        string statusPath;

        string mode = "off";
        double requested = 1.0;
        double maxScale = 4.0;
        double quantS = 0.05;

        // What we are actually commanding right now.  Adaptive mode walks this
        // up and down; fixed mode pins it.
        double current = 1.0;

        // Measurement window.  Achieved speed is game time over real time, and
        // it is the only honest way to know whether the main thread kept up --
        // asking for 3x and getting 1.4x looks identical from inside the game.
        double windowRealStart;
        double windowGameStart;
        int windowFrames;
        double achieved = 1.0;
        double fps = 60.0;

        double nextConfigRead;

        void Start()
        {
            stockMaxDelta = Time.maximumDeltaTime;
            string root = KSPUtil.ApplicationRootPath;
            controlPath = Path.Combine(root, "timescale.txt");
            statusPath = Path.Combine(root, "timescale-status.txt");
            ResetWindow();
            ReadConfig();
            Debug.Log(Tag + "armed, control file " + controlPath);
        }

        void ResetWindow()
        {
            windowRealStart = Time.realtimeSinceStartup;
            windowGameStart = Planetarium.GetUniversalTime();
            windowFrames = 0;
        }

        void ReadConfig()
        {
            try
            {
                if (!File.Exists(controlPath))
                {
                    mode = "off";
                    return;
                }
                foreach (string raw in File.ReadAllLines(controlPath))
                {
                    string line = raw.Trim();
                    int hash = line.IndexOf('#');
                    if (hash >= 0) line = line.Substring(0, hash).Trim();
                    if (line.Length == 0) continue;
                    int eq = line.IndexOf('=');
                    if (eq <= 0) continue;
                    string key = line.Substring(0, eq).Trim();
                    string val = line.Substring(eq + 1).Trim();
                    double d;
                    switch (key)
                    {
                        case "mode":
                            mode = val;
                            break;
                        case "scale":
                            if (TryNum(val, out d)) requested = d;
                            break;
                        case "max_scale":
                            if (TryNum(val, out d)) maxScale = d;
                            break;
                        case "quant_s":
                            if (TryNum(val, out d)) quantS = d;
                            break;
                    }
                }
            }
            catch (Exception exc)
            {
                // A half-written file is normal -- the harness rewrites it
                // while we are reading.  Keep the last good configuration
                // rather than falling back to something the operator did not
                // ask for.
                Debug.Log(Tag + "config read failed, keeping last: " + exc.Message);
            }
        }

        static bool TryNum(string s, out double d)
        {
            return double.TryParse(s, NumberStyles.Float,
                                   CultureInfo.InvariantCulture, out d);
        }

        void LateUpdate()
        {
            // **A paused game stays paused.**  KSP pauses by setting
            // Time.timeScale to 0 and this addon wrote it back every frame, so
            // kRPC's `paused = True` read back True while game time ran on --
            // an in-air save could not be held still while the harness
            // started the autopilot (56 game-seconds of free fall from
            // qs_shuttle_cone).  Stand down, and restart the measuring window
            // so the paused frames do not read as a slow game.
            if (FlightDriver.Pause)
            {
                ResetWindow();
                return;
            }

            // KSP's own time warp writes Time.timeScale too.  While it is
            // engaged this addon stands down completely: two things fighting
            // over one property is not a speedup, it is a bug that only shows
            // up under warp.
            if (TimeWarp.CurrentRateIndex > 0 || TimeWarp.CurrentRate > 1.0f)
            {
                if (current != 1.0)
                {
                    current = 1.0;
                    Time.maximumDeltaTime = stockMaxDelta;
                    ResetWindow();
                }
                return;
            }

            double now = Time.realtimeSinceStartup;
            windowFrames++;
            double realElapsed = now - windowRealStart;
            if (realElapsed >= 1.0)
            {
                double gameElapsed = Planetarium.GetUniversalTime() - windowGameStart;
                achieved = gameElapsed / realElapsed;
                fps = windowFrames / realElapsed;
                ResetWindow();
                Retune();
                WriteStatus();
            }

            if (now >= nextConfigRead)
            {
                nextConfigRead = now + 0.5;
                ReadConfig();
                if (mode == "fixed") current = requested;
                if (mode == "off") current = 1.0;
            }

            Apply();
        }

        /// <summary>
        /// Adaptive mode: walk the multiplier toward the fastest the main
        /// thread can actually sustain.  "As fast as possible" has to be
        /// measured, not declared -- the ceiling moves with the part count,
        /// the air density and whatever else is running on the machine, so a
        /// number that held during the coast will not hold in the landing
        /// burn.
        /// </summary>
        void Retune()
        {
            if (mode != "adaptive") return;
            if (achieved >= current * 0.97)
                current *= 1.15;          // keeping up -- ask for more
            else if (achieved < current * 0.90)
                current = Math.Max(1.0, achieved * 0.95);   // fell behind
            if (current > maxScale) current = maxScale;
            if (current < 1.0) current = 1.0;
        }

        void Apply()
        {
            // The control quantum is the real ceiling, and it is set by the
            // frame rate, not by the CPU.  Unity applies control input once a
            // frame, so `timeScale / fps` is the grid the autopilot's commands
            // land on.
            //
            // **That grid is enforced by maximumDeltaTime, not by clamping
            // timeScale**, and clamping it as well was an instability.  The
            // clamp was `min(current, quant_s * fps)` -- but fps is a
            // *function of* timeScale, because a frame that simulates more
            // game time costs more to render.  Below the boundary the clamp
            // never binds and nothing moves; the moment it binds it lowers
            // timeScale, which raises fps, which raises the allowance, which
            // raises timeScale again.  Measured on an instance asked to hold
            // a fixed multiplier:
            //
            //     asked  4  ->  held 4.00   (fps 253, clamp slack)
            //     asked  6  ->  held 6.01   (fps 177, clamp slack)
            //     asked  8  ->  held 4.10   <- clamp binding, oscillating
            //     asked 10  ->  held 1.33   <- collapsed
            //
            // Asking for more made it slower, and the status file reported
            // the collapsed number as though it were the machine's limit.
            // maximumDeltaTime below already caps game-time-per-frame at
            // quant_s, so the grid holds whatever timeScale is; if the main
            // thread cannot keep up, `achieved` falls below `commanded` and
            // says so honestly, which is what the caller needs to know.
            double target = current;
            if (target < 1.0) target = 1.0;

            // maximumDeltaTime is what lets Unity run the extra fixed steps in
            // one frame.  Leave it at the stock 0.04 and the game simply
            // cannot advance faster than 0.04 s of game time per frame, no
            // matter what timeScale says -- the speedup would silently not
            // happen.
            //
            // **It is in UNSCALED seconds, and setting it to quant_s does not
            // bound the control grid.**  Unity clamps the *real* frame time
            // to it and then multiplies by timeScale, so the game-time grid
            // is `min(realFrame, maximumDeltaTime) * timeScale`.  With
            // maximumDeltaTime = 0.05 at 8x and 80 fps the real frame is
            // 0.0125 s, nowhere near the clamp, and the grid came out at
            // 0.0125 * 8 = 0.0997 s -- twice what was asked for, while the
            // status file dutifully reported `max_dt=0.05` as though the
            // guarantee were holding.  Measured: asked 0.05, got 0.126 at 8x
            // and 0.533 at 10x.
            //
            // So divide by the scale.  Then game-time-per-frame is bounded by
            // quant_s whatever the multiplier is, and the cost of a machine
            // that cannot render fast enough shows up where it belongs -- as
            // `achieved` falling below `commanded`, rather than as a control
            // grid quietly going soft.  Holding the grid needs
            // `fps >= target / quant_s`; below that the game runs slow, which
            // is the honest half of the trade.
            float wantMax = (float)Math.Max(quantS / Math.Max(target, 1.0),
                                            Time.fixedDeltaTime);
            if (Math.Abs(Time.maximumDeltaTime - wantMax) > 1e-6f)
                Time.maximumDeltaTime = wantMax;

            // fixedDeltaTime is NOT touched.  That is the whole point: the
            // physics step stays the step the vehicle flies at 1x.
            if (Math.Abs(Time.timeScale - target) > 1e-6)
                Time.timeScale = (float)target;
        }

        void WriteStatus()
        {
            try
            {
                File.WriteAllText(statusPath, string.Format(
                    CultureInfo.InvariantCulture,
                    "mode={0}\ncommanded={1:F3}\nachieved={2:F3}\nfps={3:F1}\n" +
                    "quant_s={4:F4}\nfixed_dt={5:F4}\nmax_dt={6:F4}\n",
                    mode, Time.timeScale, achieved, fps,
                    Time.timeScale / Math.Max(fps, 1.0),
                    Time.fixedDeltaTime, Time.maximumDeltaTime));
            }
            catch (Exception) { /* status is a convenience, never a dependency */ }
        }

        void OnDestroy()
        {
            // Leaving the game at 3x because a flight ended is exactly the
            // kind of state that gets blamed on something else three sessions
            // later.
            Time.timeScale = 1.0f;
            Time.maximumDeltaTime = stockMaxDelta;
        }
    }
}
