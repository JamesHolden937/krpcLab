// The krpcLab window: a stock KSP window (the game's own skin) with a
// toolbar button, shown in flight only.  It is the only UI krpcLab has:
//
//   * the launcher's menu (common/launcher.py): a button per autopilot and a
//     status line, driven through the kRPC service "KrpcLabLauncher";
//   * an autopilot's panel (common/panel.py): its title, telemetry text and
//     controls (buttons, checkboxes, number fields, choices), described by
//     the autopilot as strings and read back as "id=value" strings;
//   * with neither running, a START SCRIPT button that starts the kRPC
//     server and runs ./run.sh (the launcher) from the krpcLab checkout
//     named in PluginData/launch.cfg (written by build.sh).
//
// Every scene change (a save loaded, a quickload, a revert, the space centre
// or the SPH) and every switch of the active vessel bumps an epoch counter
// the launcher polls: a change while an autopilot flies terminates it and
// brings the menu back.  An autopilot that loads a save or switches vessel
// itself announces it first (ExpectSceneChange).  The statics outlive scene
// changes, so the launcher keeps its menu across them.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using KRPC.Service.Attributes;
using KSP.UI.Screens;
using UnityEngine;

namespace KrpcLab
{
    /// <summary>The launcher's menu (statics outlive a scene change).</summary>
    public static class LauncherState
    {
        public const float ALIVE_S = 5f;        // no ping for this long: launcher gone
        public static List<string> buttons = new List<string>();
        public static string status = "";
        public static bool showButtons = true;
        public static bool open;
        public static int pressed = -1;
        public static float lastPing = -1e9f;
        public static int epoch;
        public static float expectUntil = -1e9f;
        public static string launchNote = "";
        internal static Rect rect = new Rect(30, 0, 300, 10);
        internal static bool placed;

        public static bool Alive { get { return Time.realtimeSinceStartup - lastPing < ALIVE_S; } }
        public static void Ping() { lastPing = Time.realtimeSinceStartup; }

        /// <summary>A scene change or vessel switch, unless one was announced.</summary>
        public static void Bump(string why)
        {
            if (Time.realtimeSinceStartup < expectUntil) return;
            epoch++;
            UnityEngine.Debug.Log("[KrpcLabLauncher] epoch " + epoch + ": " + why);
        }
    }

    /// <summary>One control on an autopilot's panel.</summary>
    public class PanelControl
    {
        public string kind, id, label, text = "";
        public bool on;
        public int clicks, choice;
        public string[] options = new string[0];

        /// <summary>"button|id|label", "toggle|id|label|0/1",
        /// "field|id|label|text", "choice|id|label|index|a;b;c",
        /// "label|id|text".</summary>
        public static PanelControl Parse(string spec)
        {
            string[] f = spec.Split('|');
            var c = new PanelControl { kind = f[0], id = f.Length > 1 ? f[1] : "", label = f.Length > 2 ? f[2] : "" };
            if (c.kind == "toggle" && f.Length > 3) c.on = f[3] == "1";
            if (c.kind == "field" && f.Length > 3) c.text = f[3];
            if (c.kind == "choice")
            {
                if (f.Length > 3) int.TryParse(f[3], out c.choice);
                if (f.Length > 4) c.options = f[4].Split(';');
            }
            return c;
        }

        public string Value()
        {
            switch (kind)
            {
                case "button": return clicks.ToString();
                case "toggle": return on ? "1" : "0";
                case "field": return text;
                case "choice": return choice.ToString();
                default: return "";
            }
        }
    }

    /// <summary>The autopilot's panel.</summary>
    public static class PanelState
    {
        public static string title = "";
        public static Color accent = Color.white;
        public static string body = "";
        public static List<PanelControl> controls = new List<PanelControl>();
        public static bool open;
        public static float lastPing = -1e9f;

        public static bool Alive { get { return open && Time.realtimeSinceStartup - lastPing < LauncherState.ALIVE_S; } }
        public static void Ping() { lastPing = Time.realtimeSinceStartup; }

        /// <summary>Replace the controls; one with the same kind and id
        /// keeps what the player set in it and its click count.</summary>
        public static void SetControls(IList<string> specs)
        {
            var old = new Dictionary<string, PanelControl>();
            foreach (var c in controls) old[c.kind + "|" + c.id] = c;
            var next = new List<PanelControl>();
            foreach (var s in specs)
            {
                var c = PanelControl.Parse(s);
                PanelControl was;
                if (old.TryGetValue(c.kind + "|" + c.id, out was))
                {
                    c.on = was.on;
                    c.text = was.text;
                    c.clicks = was.clicks;
                    if (c.kind == "choice") c.choice = Mathf.Clamp(was.choice, 0, Math.Max(0, c.options.Length - 1));
                }
                next.Add(c);
            }
            controls = next;
        }
    }

    [KRPCService(Name = "KrpcLabLauncher", GameScene = KRPC.Service.GameScene.All)]
    public static class KrpcLabLauncher
    {
        // -- the launcher's menu ------------------------------------------
        /// <summary>Open the window with these buttons and status line.</summary>
        [KRPCProcedure]
        public static void Show(IList<string> buttons, string status)
        {
            LauncherState.buttons = new List<string>(buttons);
            LauncherState.status = status;
            LauncherState.showButtons = true;
            LauncherState.pressed = -1;
            LauncherState.open = true;
            LauncherState.launchNote = "";
            LauncherState.Ping();
        }

        /// <summary>Set the status line, whether the buttons show, and
        /// whether the window is open.</summary>
        [KRPCProcedure]
        public static void Update(string status, bool showButtons, bool open)
        {
            LauncherState.status = status;
            LauncherState.showButtons = showButtons;
            LauncherState.open = open;
            LauncherState.Ping();
        }

        /// <summary>The index of the button pressed since the last call, or
        /// -1.  Also the launcher's heartbeat.</summary>
        [KRPCProcedure]
        public static int Pressed()
        {
            LauncherState.Ping();
            int i = LauncherState.pressed;
            LauncherState.pressed = -1;
            return i;
        }

        /// <summary>Counts scene changes and active-vessel switches; also the
        /// launcher's heartbeat.</summary>
        [KRPCProcedure]
        public static int Epoch()
        {
            LauncherState.Ping();
            return LauncherState.epoch;
        }

        /// <summary>A scene change or vessel switch in the next
        /// <paramref name="seconds"/> is the autopilot's own doing: it does
        /// not count.</summary>
        [KRPCProcedure]
        public static void ExpectSceneChange(float seconds)
        {
            LauncherState.expectUntil = Time.realtimeSinceStartup + seconds;
        }

        /// <summary>The launcher is going: no buttons, window closed.</summary>
        [KRPCProcedure]
        public static void Close()
        {
            LauncherState.buttons = new List<string>();
            LauncherState.status = "";
            LauncherState.open = false;
            LauncherState.lastPing = -1e9f;
        }

        // -- the active vessel's actuators ---------------------------------
        /// <summary>The active vessel's control authority and its control
        /// surfaces' actuators, from the game's own modules (kRPC reports
        /// neither actuator speed nor range).  First line: "total" and the
        /// summed potential torque of every ITorqueProvider (kN m, vessel
        /// axes x pitch, y roll, z yaw: positive then negative).  Then one
        /// line per ModuleControlSurface: actuatorSpeed (deg/s, or the lag
        /// rate when exponential), ctrlSurfaceRange (deg),
        /// useExponentialSpeed (1/0), and its own potential torque; and one
        /// per ModuleGimbal that has a response speed: gimbalResponseSpeed
        /// (1/s), gimbalRange (deg), 2, and its potential torque.  Empty
        /// outside flight.</summary>
        [KRPCProcedure]
        public static IList<string> SurfaceActuators()
        {
            var out_ = new List<string>();
            if (!HighLogic.LoadedSceneIsFlight) return out_;
            var v = FlightGlobals.ActiveVessel;
            if (v == null) return out_;
            var ci = System.Globalization.CultureInfo.InvariantCulture;
            Vector3 tp = Vector3.zero, tn = Vector3.zero;
            var lines = new List<string>();
            foreach (var p in v.parts)
            {
                foreach (PartModule m in p.Modules)
                {
                    var tq = m as ITorqueProvider;
                    if (tq == null) continue;
                    Vector3 pos, neg;
                    try { tq.GetPotentialTorque(out pos, out neg); }
                    catch (Exception) { continue; }
                    tp += Abs(pos);
                    tn += Abs(neg);
                    var gm = m as ModuleGimbal;
                    if (gm != null)
                    {
                        // ModuleGimbal.FixedUpdate: with useGimbalResponseSpeed
                        // the nozzle moves Mathf.Lerp(now, target,
                        // gimbalResponseSpeed x dt) a step -- a first-order lag
                        // of 1/gimbalResponseSpeed s; without it, at once.
                        if (gm.useGimbalResponseSpeed && gm.gimbalResponseSpeed > 0f)
                            lines.Add(string.Format(ci, "{0} {1} 2 {2} {3} {4} {5} {6} {7}",
                                gm.gimbalResponseSpeed, gm.gimbalRange,
                                Mathf.Abs(pos.x), Mathf.Abs(pos.y), Mathf.Abs(pos.z),
                                Mathf.Abs(neg.x), Mathf.Abs(neg.y), Mathf.Abs(neg.z)));
                        continue;
                    }
                    var cs = m as ModuleControlSurface;
                    if (cs == null) continue;
                    lines.Add(string.Format(ci, "{0} {1} {2} {3} {4} {5} {6} {7} {8}",
                        cs.actuatorSpeed, cs.ctrlSurfaceRange, cs.useExponentialSpeed ? 1 : 0,
                        Mathf.Abs(pos.x), Mathf.Abs(pos.y), Mathf.Abs(pos.z),
                        Mathf.Abs(neg.x), Mathf.Abs(neg.y), Mathf.Abs(neg.z)));
                }
            }
            out_.Add(string.Format(ci, "total {0} {1} {2} {3} {4} {5}", tp.x, tp.y, tp.z, tn.x, tn.y, tn.z));
            out_.AddRange(lines);
            return out_;
        }

        static Vector3 Abs(Vector3 a) { return new Vector3(Mathf.Abs(a.x), Mathf.Abs(a.y), Mathf.Abs(a.z)); }

        // -- a test harness's stick ------------------------------------------
        /// <summary>Add this pitch, roll and yaw to the active vessel's input
        /// every frame, where the player's stick goes: in
        /// OnPreAutopilotUpdate, which Vessel.FeedInputFeed calls before
        /// OnFlyByWire (where kRPC adds its clients' input), so an autopilot
        /// reads it back exactly as it reads the player's.  A second kRPC
        /// client cannot do this: kRPC keeps one input set per vessel.  For
        /// farm harnesses (airplane/tools/airfly.py); 0,0,0 adds
        /// nothing.</summary>
        [KRPCProcedure]
        public static void TestStick(float pitch, float roll, float yaw)
        {
            TestStickState.pitch = pitch;
            TestStickState.roll = roll;
            TestStickState.yaw = yaw;
            TestStickState.Hook();
        }

        // -- an autopilot's panel -----------------------------------------
        /// <summary>Open the panel: a title in <paramref name="accent"/>
        /// ("r g b", 0..1) and these controls.</summary>
        [KRPCProcedure]
        public static void PanelOpen(string title, string accent, IList<string> controls)
        {
            PanelState.title = title;
            PanelState.body = "";
            PanelState.controls = new List<PanelControl>();
            PanelState.SetControls(controls);
            var c = accent.Split(' ');
            float r = 1, g = 1, b = 1;
            if (c.Length == 3)
            {
                float.TryParse(c[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out r);
                float.TryParse(c[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out g);
                float.TryParse(c[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out b);
            }
            PanelState.accent = new Color(r, g, b, 1f);
            PanelState.open = true;
            LauncherState.open = true;
            PanelState.Ping();
        }

        /// <summary>Replace the panel's controls (values the player set in
        /// controls that stay are kept).</summary>
        [KRPCProcedure]
        public static void PanelControls(IList<string> controls)
        {
            PanelState.SetControls(controls);
            PanelState.Ping();
        }

        /// <summary>The panel's telemetry text.</summary>
        [KRPCProcedure]
        public static void PanelBody(string body)
        {
            PanelState.body = body;
            PanelState.Ping();
        }

        /// <summary>Every control's value as "id=value": a button's click
        /// count, a checkbox 1/0, a field's text, a choice's index.  Pure, so
        /// it can be streamed; a stream of it is the panel's heartbeat.</summary>
        [KRPCProcedure]
        public static IList<string> PanelRead()
        {
            PanelState.Ping();
            var out_ = new List<string>();
            foreach (var c in PanelState.controls)
                if (c.kind != "label") out_.Add(c.id + "=" + c.Value());
            return out_;
        }

        /// <summary>The autopilot is done: the panel goes.</summary>
        [KRPCProcedure]
        public static void PanelClose()
        {
            PanelState.open = false;
            PanelState.controls = new List<PanelControl>();
            PanelState.lastPing = -1e9f;
        }
    }

    /// <summary>Counts scene changes and vessel switches for the whole game
    /// (not only in flight).</summary>
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public static class TestStickState
    {
        public static float pitch, roll, yaw;
        static Vessel hooked;

        public static void Hook()
        {
            if (!HighLogic.LoadedSceneIsFlight) return;
            var v = FlightGlobals.ActiveVessel;
            if (v == hooked) return;
            if (hooked != null) hooked.OnPreAutopilotUpdate -= Add;
            hooked = v;
            if (v != null) v.OnPreAutopilotUpdate += Add;
        }

        static void Add(FlightCtrlState s)
        {
            s.pitch = Mathf.Clamp(s.pitch + pitch, -1f, 1f);
            s.roll = Mathf.Clamp(s.roll + roll, -1f, 1f);
            s.yaw = Mathf.Clamp(s.yaw + yaw, -1f, 1f);
        }
    }

    public class EpochWatcher : MonoBehaviour
    {
        void Awake()
        {
            DontDestroyOnLoad(this);
            GameEvents.onGameSceneLoadRequested.Add(OnScene);
            GameEvents.onVesselChange.Add(OnVessel);
        }

        void OnDestroy()
        {
            GameEvents.onGameSceneLoadRequested.Remove(OnScene);
            GameEvents.onVesselChange.Remove(OnVessel);
        }

        void OnScene(GameScenes scene) { LauncherState.Bump("scene " + scene); }
        void OnVessel(Vessel v) { LauncherState.Bump("vessel " + (v != null ? v.vesselName : "none")); }
    }

    /// <summary>Starts the kRPC server and the launcher (./run.sh).</summary>
    public static class ScriptStarter
    {
        /// <summary>PluginData/launch.cfg: "root=..." (the krpcLab checkout)
        /// and "home=..." (the user's home), Unix paths.</summary>
        static Dictionary<string, string> Config()
        {
            var d = new Dictionary<string, string>();
            string dir = Path.GetDirectoryName(typeof(ScriptStarter).Assembly.Location);
            string path = Path.Combine(Path.Combine(dir, ".."), Path.Combine("PluginData", "launch.cfg"));
            foreach (var line in File.ReadAllLines(path))
            {
                int i = line.IndexOf('=');
                if (i > 0) d[line.Substring(0, i).Trim()] = line.Substring(i + 1).Trim();
            }
            return d;
        }

        /// <summary>Under Wine the game sees the Unix root as Z:.</summary>
        static bool Wine { get { return Environment.OSVersion.Platform != PlatformID.Unix && Directory.Exists("Z:\\usr\\bin"); } }

        /// <summary>The game's own Unix pid (Wine: /proc/self is the game's
        /// process too), or "" -- the launcher exits when it dies.</summary>
        static string GamePid()
        {
            try
            {
                string stat = File.ReadAllText(Wine ? "Z:\\proc\\self\\stat" : "/proc/self/stat");
                return stat.Split(' ')[0];
            }
            catch (Exception) { return ""; }
        }

        public static string Start()
        {
            try
            {
                if (!KRPC.Core.Instance.AnyRunning) KRPC.Core.Instance.StartAll();
            }
            catch (Exception e) { return "kRPC server did not start: " + e.Message; }
            Dictionary<string, string> cfg;
            try { cfg = Config(); }
            catch (Exception e) { return "no PluginData/launch.cfg (run launcherMod/build.sh): " + e.Message; }
            string root, home;
            if (!cfg.TryGetValue("root", out root)) return "launch.cfg has no root=";
            if (!cfg.TryGetValue("home", out home)) home = "/root";
            string pid = GamePid();
            string script = "cd '" + root + "' && exec ./run.sh >/dev/null 2>&1";
            var psi = new ProcessStartInfo
            {
                FileName = Wine ? "Z:\\usr\\bin\\env" : "/usr/bin/env",
                Arguments = "-i HOME=" + home + " PATH=/usr/local/bin:/usr/bin:/bin"
                            + (pid.Length > 0 ? " KRPCLAB_GAME_PID=" + pid : "")
                            + " /bin/bash -lc \"" + script + "\"",
                UseShellExecute = false,
                CreateNoWindow = true,
            };
            try { Process.Start(psi); }
            catch (Exception e) { return "could not run ./run.sh: " + e.Message; }
            return "starting the launcher ...";
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class LauncherWindow : MonoBehaviour
    {
        const int ID = 0x6b4c4142;              // "kLAB"

        ApplicationLauncherButton button;
        Texture2D icon;
        bool uiHidden;
        GUIStyle statusStyle, bodyStyle, titleStyle;

        void Start()
        {
            GameEvents.onGUIApplicationLauncherReady.Add(AddButton);
            GameEvents.onHideUI.Add(HideUI);
            GameEvents.onShowUI.Add(ShowUI);
            if (ApplicationLauncher.Ready) AddButton();
        }

        void OnDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(AddButton);
            GameEvents.onHideUI.Remove(HideUI);
            GameEvents.onShowUI.Remove(ShowUI);
            if (button != null && ApplicationLauncher.Instance != null)
                ApplicationLauncher.Instance.RemoveModApplication(button);
        }

        void HideUI() { uiHidden = true; }
        void ShowUI() { uiHidden = false; }

        void AddButton()
        {
            if (button != null) return;
            button = ApplicationLauncher.Instance.AddModApplication(
                () => LauncherState.open = true, () => LauncherState.open = false,
                null, null, null, null,
                ApplicationLauncher.AppScenes.FLIGHT | ApplicationLauncher.AppScenes.MAPVIEW,
                Icon());
        }

        void Update()
        {
            // the toolbar button follows the window (the launcher opens and
            // closes it too)
            if (button == null) return;
            bool on = button.toggleButton.CurrentState == KSP.UI.UIRadioButton.State.True;
            if (LauncherState.open && !on) button.SetTrue(false);
            else if (!LauncherState.open && on) button.SetFalse(false);
        }

        /// <summary>A "play" triangle in the launcher's green.</summary>
        Texture2D Icon()
        {
            if (icon != null) return icon;
            const int N = 38;
            icon = new Texture2D(N, N, TextureFormat.ARGB32, false);
            var clear = new Color(0, 0, 0, 0);
            var green = new Color(0.6f, 1f, 0.6f, 1f);
            for (int y = 0; y < N; y++)
                for (int x = 0; x < N; x++)
                {
                    // apex at (30, 19), base x = 10 from y = 7 to 31
                    float half = 12f * (30 - x) / 20f;
                    bool inside = x >= 10 && x <= 30 && Mathf.Abs(y - 19) <= half;
                    icon.SetPixel(x, y, inside ? green : clear);
                }
            icon.Apply();
            return icon;
        }

        void OnGUI()
        {
            if (!LauncherState.open || uiHidden) return;
            GUI.skin = HighLogic.Skin;
            if (!LauncherState.placed)
            {
                LauncherState.rect.y = Screen.height / 2 - 160;
                LauncherState.placed = true;
            }
            LauncherState.rect = GUILayout.Window(ID, LauncherState.rect, Draw, "krpcLab",
                                                  GUILayout.Width(300));
        }

        void Styles()
        {
            if (statusStyle != null) return;
            statusStyle = new GUIStyle(GUI.skin.label) { wordWrap = true, alignment = TextAnchor.MiddleCenter };
            bodyStyle = new GUIStyle(GUI.skin.label) { wordWrap = false, alignment = TextAnchor.UpperLeft, fontSize = 12 };
            titleStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold, fontSize = 14 };
        }

        void Draw(int id)
        {
            Styles();
            if (PanelState.Alive) DrawPanel();
            else if (LauncherState.Alive) DrawMenu();
            else
            {
                GUILayout.Label("The launcher is not running.", statusStyle);
                if (LauncherState.launchNote.Length > 0)
                    GUILayout.Label(LauncherState.launchNote, statusStyle);
                GUILayout.Space(4);
                if (GUILayout.Button("START SCRIPT", GUILayout.Height(30)))
                    LauncherState.launchNote = ScriptStarter.Start();
            }
            GUI.DragWindow();
        }

        void DrawMenu()
        {
            if (LauncherState.status.Length > 0)
                GUILayout.Label(LauncherState.status, statusStyle);
            if (!LauncherState.showButtons) return;
            GUILayout.Space(4);
            for (int i = 0; i < LauncherState.buttons.Count; i++)
                if (GUILayout.Button(LauncherState.buttons[i], GUILayout.Height(30)))
                    LauncherState.pressed = i;
        }

        void DrawPanel()
        {
            titleStyle.normal.textColor = PanelState.accent;
            GUILayout.Label(PanelState.title, titleStyle);
            if (PanelState.body.Length > 0)
                GUILayout.Label(PanelState.body, bodyStyle);
            foreach (var c in PanelState.controls)
            {
                switch (c.kind)
                {
                    case "button":
                        if (GUILayout.Button(c.label, GUILayout.Height(28))) c.clicks++;
                        break;
                    case "toggle":
                        c.on = GUILayout.Toggle(c.on, " " + c.label);
                        break;
                    case "field":
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(c.label, GUILayout.Width(190));
                        c.text = GUILayout.TextField(c.text, GUILayout.Width(80));
                        GUILayout.EndHorizontal();
                        break;
                    case "choice":
                        if (c.label.Length > 0) GUILayout.Label(c.label);
                        if (c.options.Length > 0)
                            c.choice = GUILayout.Toolbar(Mathf.Clamp(c.choice, 0, c.options.Length - 1), c.options);
                        break;
                    case "label":
                        GUILayout.Label(c.label, statusStyle);
                        break;
                }
            }
        }
    }
}
