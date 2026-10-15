// All tunables for the low-thrust autopilot and its stand-in simulator.
// Control code reads cfg.X; it never hard-codes a number.  Override with
// NAME=VALUE (the tools' --set, the plugin's settings file).

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;

namespace LowThrust
{
    public class Config
    {
        // --- Arrival ----------------------------------------------------------
        // Done when every targeted apsis radius is within this.
        public double TOLERANCE_M = 100.0;
        // ... and the orbit plane within this of the target plane.
        public double PLANE_TOLERANCE_DEG = 0.01;
        // ... and a held periapsis direction within this (eccentric orbits).
        public double APSE_TOLERANCE_DEG = 0.5;
        // The +-90 deg arcs of a held apse line wake once both apsis radii are
        // within this fraction of target (an end-game correction).
        public double APSE_ARC_GATE = 0.01;

        // --- Guidance (Hohmann in small burns) -------------------------------
        // The arcs round each apsis have half-width acos(eta): ETA_DEFAULT is
        // flown when there is no deadline (0.99 = +-8 deg: within ~2% of the
        // impulsive Hohmann), and the planner searches ETA_MIN..ETA_MAX to meet
        // one.  Never 1: the arcs shrink to nothing.
        public double ETA_DEFAULT = 0.99;
        // A mission's orbit-shaping phases widen the arcs (down to this cutoff,
        // +-60 deg) until the apsis changes left fit this many passes.
        public double ETA_WIDE_MIN = 0.5;
        public double ARC_PASSES_MAX = 30.0;
        public double ETA_MAX = 0.999;
        public double ETA_MIN = 0.2;
        // The prograde step (m/s) the radius rates are differenced over.
        public double RATE_DV = 0.01;

        // --- Missions ----------------------------------------------------------
        // Lowest periapsis above a body's atmosphere (or surface) any phase plans.
        public double SAFE_ALT_M = 30000.0;
        // Capture into an ellipse with apoapsis this fraction of the target's
        // SOI (or the final apoapsis, if higher); FINAL then lowers it.
        public double CAPTURE_AP_FRACTION = 0.3;
        // ... or, when an engine is too weak to capture into that in one pass,
        // into one this loose.
        public double CAPTURE_LOOSE_FRACTION = 0.7;
        // An engine too weak to capture inside the planet's SOI matches its
        // motion in the cruise (MATCH), arriving at MATCH_STANDOFF of the SOI
        // radius at ARRIVE_MARGIN of the fastest v_inf it can capture there.
        // MATCH starts once stopping there needs MATCH_SHARE of the engine and
        // ends MATCH_END_S before the aim point.
        public double ARRIVE_MARGIN = 0.8;
        public double MATCH_STANDOFF = 1.05;
        public double MATCH_SHARE = 0.8;
        public double MATCH_END_S = 600.0;
        // A one-pass capture must fit in CAPTURE_FILL of the time within
        // +-CAPTURE_ARC_DEG of periapsis.
        public double CAPTURE_ARC_DEG = 60.0;
        public double CAPTURE_FILL = 0.8;
        // A longer capture burns once the energy to shed reaches this share of
        // what the path left to periapsis can remove at full thrust.
        public double CAPTURE_PATH_SHARE = 0.8;
        // The capture burn holds the periapsis between the safe floor and this
        // multiple of the aimed one.
        public double CAPTURE_RP_BAND = 1.5;
        // Apoapses stay this many of a moon's SOI radii short of its orbit, and
        // a moon transfer's waiting orbit reaches only this fraction of the way.
        public double WAIT_SOI_MARGIN = 1.2;
        public double WAIT_AP_FRACTION = 0.6;
        // Departure parking ellipse apoapsis, as a fraction of the highest one
        // clear of the body's moons and well inside its SOI.
        public double PARK_AP_FRACTION = 0.9;
        // The window is searched from the estimated end of the pump plus this.
        public double WINDOW_LEAD_S = 6 * 3600.0;
        // A planet transfer arrives within this share of the cheapest
        // window's wait plus flight (1: the cheapest).  Set per plan by the
        // dv/time trade (TRANSFER_SHARES).
        public double TRANSFER_TIME_SHARE = 1.0;
        // A moon push is searched up to this multiple of the tangential push
        // that reaches the moon's orbit ...
        public double PUSH_DV_FACTOR = 2.0;
        // ... or, away from periapsis (every 30 deg), up to this multiple of
        // the periapsis Hohmann.
        public double PUSH_OFF_PERIAPSIS = 1.1;
        public double PUSH_HORIZON_S = 30 * 21600.0;
        // ... and only within this fraction of the post-push revolution.
        public double PUSH_REVOLUTION = 0.75;
        // A departure push stops this far past periapsis.
        public double DEPART_ARC_DEG = 60.0;
        // A departure pass is rejected if its escape enters a moon's SOI
        // within this long.
        public double ESCAPE_CHECK_S = 3 * 21600.0;
        // Mid-course corrections: made when the B-plane miss needs more than
        // CORRECT_DV, not within CORRECT_MIN_LEAD_S of the encounter,
        // rechecked after CORRECT_EVERY of the time remaining.
        public double CORRECT_DV = 0.5;
        public double CORRECT_MIN_LEAD_S = 3600.0;
        public double CORRECT_EVERY = 0.1;
        // A moon correction bigger than this, or a closest approach beyond
        // MISS_REPLAN of the moon's SOI, means no encounter is ahead.
        public double CORRECT_MAX_DV = 150.0;
        // A cruise miss beyond RETARGET_MISS of the target's SOI radii, or
        // RETARGET_FRACTION of its distance from the star (whichever is less),
        // is re-planned as a fresh Lambert transfer.
        public double RETARGET_MISS = 20.0;
        public double RETARGET_FRACTION = 0.01;
        public double MISS_REPLAN = 1.5;
        // In the target's SOI the hyperbola's periapsis is nudged until within
        // this fraction of the capture altitude, no later than this before it.
        public double APPROACH_TOL = 0.02;
        public double APPROACH_MIN_LEAD_S = 1800.0;
        public double MISSION_MAX_DAYS = 4000.0;
        // After a slingshot, brake until the apoapsis is inside this fraction
        // of the SOI.
        public double REBIND_SOI_FRACTION = 0.6;

        // --- Aerobraking -----------------------------------------------------
        // At a body with an atmosphere (and a vessel with a drag area), the
        // capture's periapsis is put in the air (its pass sheds energy the
        // capture burn then need not) and the apoapsis is lowered by passes
        // (AEROBRAKE) instead of burns; FINAL lifts the periapsis out after.
        public bool AEROBRAKE = true;
        // Every limit is checked on the pass flown with this much more drag
        // and density than the model and the drag scale learned: the density
        // varies with latitude and the sun, the drag with the attitude.
        public double AERO_DRAG_MARGIN = 1.5;
        // ...except the apoapsis a pass leaves, which is only a target, not
        // a safety limit: it is kept above the final apoapsis with this much
        // more drag than modelled (or the worst a pass of this mission has
        // missed by, if more).  What disagrees with it: each pass's "drag
        // scale a -> b" in the log (b / a is that pass's miss; 0.90-1.21 on
        // LOG10207 after the first).  At the safety margin (2.25x drag with
        // the density) each pass could take ~half of what was left, and the
        // apoapsis crept down to the target over 9 shrinking passes.
        public double AERO_RA_MARGIN = 1.25;
        // A pass's heating index (sqrt(rho) v^3 / 1e6, KSP's high-Mach
        // convective coefficient less its constant) peaks at most at the
        // limit learned: the first pass AERO_HEAT_FIRST; after each, the
        // measured temperature rise of every part (skin and interior, over
        // its maximum) extrapolated linearly to AERO_HEAT_TARGET of the
        // maximum, at most AERO_HEAT_GROWTH times the pass flown; its
        // integral over the pass likewise.  The stand-in plans with
        // AERO_HEAT_PLAN and learns nothing.
        public double AERO_HEAT_FIRST = 30.0;
        public double AERO_HEAT_TARGET = 0.6;
        public double AERO_HEAT_GROWTH = 3.0;
        public double AERO_HEAT_PLAN = 100.0;
        // No pass below the highest terrain plus this, nor below this share
        // of the atmosphere's depth; deceleration at most this (m/s^2).
        public double AERO_TERRAIN_MARGIN_M = 3000.0;
        public double AERO_MIN_ALT_SHARE = 0.2;
        public double AERO_ACCEL_MAX = 20.0;
        // Aerobraking ends once a pass would lower the apoapsis by less than
        // this share of what is left above the final one.
        public double AERO_MIN_GAIN = 0.005;
        // ...or once a burn at periapsis would finish the apoapsis for less
        // than this (m/s): another skim would save less than its revolution
        // costs
        public double AERO_MIN_PASS_DV = 2.0;
        // No pass may leave an orbit whose periapsis the engine cannot lift
        // to this far above the air within +-60 deg of the next apoapsis (at
        // the margin's drag); the exit's arc is widened to fit the lift.
        public double AERO_LIFT_MARGIN_M = 5000.0;
        // In the game: the fastest physics warp rate (x) a pass is flown at
        // -- chosen by rate, not index: BetterTimeWarp's index 3 is 32x --,
        // the SAS mode held through it, and the game seconds the
        // temperatures are still watched after it (the interior peaks late).
        public double AERO_PHYSICS_WARP = 4.0;
        public string AERO_HOLD = "Prograde";
        public double AERO_SOAK_S = 300.0;

        // --- Tick --------------------------------------------------------------
        // Guidance is re-decided once a tick and held; a tick is this fraction
        // of a period.
        public int TICKS_PER_ORBIT = 180;
        // RK4 steps per thrusting tick (coasting ticks are propagated exactly).
        public int THRUST_SUBSTEPS = 4;
        // A thrusting tick changes the velocity by at most this share of it.
        public double THRUST_DV_SHARE = 0.02;

        // --- In the game (the plugin) ---------------------------------------------
        // Thrust only while the thrust axis is within this of the command (the
        // vessel turns at its reaction wheels' rate; the planner assumes it
        // points at once).
        public double ALIGN_DEG = 2.0;
        // Logging only: the steering line reports the thrust dv flown with the
        // axis more than this off its command.
        public double STEER_OFF_DEG = 0.5;
        // A coast is run ahead in prediction (exact two-body) and flown in one
        // go, at most this fraction of a revolution at a time, so the vessel
        // turns to the next burn while it coasts.
        public double COAST_AHEAD = 0.5;
        // Wall milliseconds per rendered frame the autopilot may spend: the
        // warp rate rises while it spends less than half of this and falls
        // when it spends more.
        public double FRAME_BUDGET_MS = 8.0;
        // A thrusting frame draws at most this share of what each propellant
        // (and ElectricCharge) the vessel holds: the warp rate is held down to
        // it, so the vessel's own generators keep up step by step.
        public double STEP_RESOURCE_SHARE = 0.5;
        // Highest rails warp index the autopilot uses (the game's last: -1).
        public int MAX_WARP_INDEX = -1;
        // Rails warp is allowed down to the top of the atmosphere (or this far
        // above the highest terrain), not only above KSP's altitude limits:
        // on rails the orbit is exact at any rate, so the limits only slow the
        // flight.  Restored when the autopilot stops.
        public bool WARP_BELOW_LIMITS = true;
        public double TERRAIN_MARGIN_M = 2000.0;
        // Telemetry at most once per this many game seconds (events always).
        public double LOG_INTERVAL_UT = 600.0;

        // --- Planner ---------------------------------------------------------------
        // The dv/time trade (Tradeoff): the mission is flown in the stand-in at
        // this many arc cutoffs between ETA_MAX and ETA_MIN ...
        public int PLAN_GRID = 13;
        // ... and with no budget given, the fastest option within this share
        // of the cheapest one's dv is flown (the cheapest alone was 3x slower
        // for 0.7% less dv to the Mun).
        public double BALANCE_DV_SHARE = 0.02;
        // ... and, on a route to another planet, at each of these transfer
        // time shares (TRANSFER_TIME_SHARE): faster, dearer Lambert transfers
        // for a budget that has the dv.
        public double[] TRANSFER_SHARES = { 1.0, 0.85, 0.7, 0.55, 0.4 };
        // A budget's stand-in flights stop at this multiple of it (so an
        // impossible budget is known quickly, and still says what is possible).
        public double BUDGET_SCAN_FACTOR = 1.5;
        // A dv budget below this share of the impulsive estimate is refused
        // without flying anything (the estimate is patched conics, not a
        // strict bound, hence the margin).
        public double IMPULSIVE_MARGIN = 0.8;
        // A transfer still running after this many orbits is reported unfinished.
        public int MAX_ORBITS = 20000;

        public Config Clone() { return (Config)MemberwiseClone(); }

        static IEnumerable<FieldInfo> Fields()
        {
            return typeof(Config).GetFields(BindingFlags.Public | BindingFlags.Instance);
        }

        static string Show(object v)
        {
            if (v is double) return ((double)v).ToString("R", CultureInfo.InvariantCulture);
            if (v is bool) return (bool)v ? "True" : "False";
            if (v is double[]) return string.Join(",", ((double[])v).Select(x => Show(x)).ToArray());
            return Convert.ToString(v, CultureInfo.InvariantCulture);
        }

        /// <summary>NAME=VALUE pairs onto this config, coerced to the field's
        /// type.  Throws on an unknown field.</summary>
        public Config Apply(IEnumerable<string> pairs)
        {
            if (pairs == null) return this;
            foreach (string pair in pairs)
            {
                int i = pair.IndexOf('=');
                string name = (i < 0 ? pair : pair.Substring(0, i)).Trim();
                string value = i < 0 ? "" : pair.Substring(i + 1).Trim();
                FieldInfo f = typeof(Config).GetField(name);
                if (f == null) throw new ArgumentException("unknown config field '" + name + "'");
                if (f.FieldType == typeof(bool))
                    f.SetValue(this, Array.IndexOf(new[] { "1", "true", "yes", "on" }, value.ToLowerInvariant()) >= 0);
                else if (f.FieldType == typeof(double[]))
                    f.SetValue(this, value.Split(new[] { ',' }).Select(x => double.Parse(x.Trim(), CultureInfo.InvariantCulture))
                                          .ToArray());
                else if (f.FieldType == typeof(string))
                    f.SetValue(this, value);
                else if (f.FieldType == typeof(int))
                    f.SetValue(this, int.Parse(value, CultureInfo.InvariantCulture));
                else
                    f.SetValue(this, double.Parse(value, CultureInfo.InvariantCulture));
            }
            return this;
        }

        /// <summary>The fields that differ from the defaults, as k=v.</summary>
        public List<string> Differences()
        {
            var b = new Config();
            var r = new List<string>();
            foreach (FieldInfo f in Fields())
                if (Show(f.GetValue(this)) != Show(f.GetValue(b)))
                    r.Add(f.Name + "=" + Show(f.GetValue(this)));
            return r;
        }

        /// <summary>A hash of the whole default set: which baseline
        /// Differences() is from.</summary>
        public static string DefaultsFingerprint()
        {
            var b = new Config();
            var parts = new List<string>();
            foreach (FieldInfo f in Fields()) parts.Add(f.Name + "=" + Show(f.GetValue(b)));
            using (var sha = SHA1.Create())
            {
                byte[] h = sha.ComputeHash(Encoding.UTF8.GetBytes(string.Join(";", parts.ToArray())));
                var sb = new StringBuilder();
                for (int i = 0; i < 4; i++) sb.Append(h[i].ToString("x2"));
                return sb.ToString();
            }
        }

        /// <summary>The config line every log opens with.</summary>
        public string Line()
        {
            var d = Differences();
            return "config: " + (d.Count == 0 ? "(defaults)" : string.Join(" ", d.ToArray()))
                   + " defaults_fingerprint=" + DefaultsFingerprint();
        }
    }
}
