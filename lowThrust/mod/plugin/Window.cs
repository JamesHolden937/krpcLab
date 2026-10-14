// The low-thrust autopilot's window: a stock KSP window (the game's own
// skin) opened from the toolbar.  The target -- body from a dropdown,
// periapsis, apoapsis, inclination, LAN and argument of periapsis -- the
// highest warp the autopilot may use, PLAN / START / TERMINATE, and the
// controller's status lines.

using System;
using System.Collections.Generic;
using KSP.UI.Screens;
using UnityEngine;

namespace LowThrust
{
    /// <summary>The window's state (statics outlive a scene change, so the
    /// target is still there next flight).</summary>
    public static class PanelState
    {
        public static string body = "";
        // periapsis, apoapsis (km), inclination, LAN, argument of periapsis (deg)
        public static string[] fields = { "", "", "", "", "" };
        public static bool open;
        // highest rails warp index the autopilot may use (0: the game's highest)
        public static int maxWarp;
        // the budget: most dv (m/s) and most time (days) to spend; blank: none
        public static string maxDv = "", maxDays = "";
        // the dv/time options PLAN found (fastest first) and the one picked
        // (by clicking; null: the budget picks)
        public static List<TradeOption> options = new List<TradeOption>();
        public static TradeOption picked;
        internal static Rect rect = new Rect(Screen.width - 380, 120, 330, 10);

        /// <summary>Fill the target from "body: ap= pe= [i= lan= argpe=]".</summary>
        public static void SetTarget(string text)
        {
            int colon = text.IndexOf(':');
            string name = (colon < 0 ? text : text.Substring(0, colon)).Trim();
            body = name.Length == 0 ? name : char.ToUpperInvariant(name[0]) + name.Substring(1).ToLowerInvariant();
            string[] keys = { "pe", "ap", "i", "lan", "argpe" };
            for (int i = 0; i < fields.Length; i++) fields[i] = "";
            if (colon < 0) return;
            foreach (string item in text.Substring(colon + 1).Replace(",", " ")
                         .Split((char[])null, StringSplitOptions.RemoveEmptyEntries))
            {
                int eq = item.IndexOf('=');
                int k = eq < 0 ? -1 : Array.IndexOf(keys, item.Substring(0, eq));
                if (k >= 0) fields[k] = item.Substring(eq + 1);
            }
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class Window : MonoBehaviour
    {
        const int ID = 0x4c54574e;              // "LTWN"
        const string LOCK = "LowThrust_typing";
        static readonly string[] LABELS = {
            "Periapsis (km)", "Apoapsis (km)", "Inclination (°)", "LAN (°)",
            "Arg. of periapsis (°)" };

        ApplicationLauncherButton button;
        Texture2D icon;
        bool uiHidden, listOpen, locked;
        Vector2 listScroll;
        List<KeyValuePair<CelestialBody, int>> bodies;

        void Start()
        {
            GameEvents.onGUIApplicationLauncherReady.Add(AddButton);
            GameEvents.onHideUI.Add(HideUI);
            GameEvents.onShowUI.Add(ShowUI);
            if (ApplicationLauncher.Ready) AddButton();
            if (PanelState.body == "" && FlightGlobals.ActiveVessel != null)
                PanelState.body = FlightGlobals.ActiveVessel.mainBody.bodyName;
        }

        void OnDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(AddButton);
            GameEvents.onHideUI.Remove(HideUI);
            GameEvents.onShowUI.Remove(ShowUI);
            if (button != null && ApplicationLauncher.Instance != null)
                ApplicationLauncher.Instance.RemoveModApplication(button);
            Unlock();
        }

        void HideUI() { uiHidden = true; }
        void ShowUI() { uiHidden = false; }

        void AddButton()
        {
            if (button != null) return;
            button = ApplicationLauncher.Instance.AddModApplication(
                () => PanelState.open = true, () => PanelState.open = false,
                null, null, null, null,
                ApplicationLauncher.AppScenes.FLIGHT | ApplicationLauncher.AppScenes.MAPVIEW,
                Icon());
        }

        void Update()
        {
            // the toolbar button follows the window
            if (button != null)
            {
                bool on = button.toggleButton.CurrentState == KSP.UI.UIRadioButton.State.True;
                if (PanelState.open && !on) button.SetTrue(false);
                else if (!PanelState.open && on) button.SetFalse(false);
            }
        }

        /// <summary>An orbit round a planet, drawn.</summary>
        Texture2D Icon()
        {
            if (icon != null) return icon;
            const int N = 38;
            icon = new Texture2D(N, N, TextureFormat.ARGB32, false);
            var clear = new Color(0, 0, 0, 0);
            var ring = new Color(0.35f, 0.8f, 1f, 1f);
            for (int y = 0; y < N; y++)
                for (int x = 0; x < N; x++)
                {
                    double dx = x - N / 2 + 0.5, dy = (y - N / 2 + 0.5) * 1.8;
                    double rr = Math.Sqrt(dx * dx + dy * dy);
                    double pr = Math.Sqrt((x - N / 2 + 0.5) * (x - N / 2 + 0.5)
                                          + (y - N / 2 + 0.5) * (y - N / 2 + 0.5));
                    Color c = clear;
                    if (pr < 6.5) c = Color.white;
                    else if (Math.Abs(rr - 15) < 1.4) c = ring;
                    if (Math.Abs(x - 31) < 3 && Math.Abs(y - 21) < 3) c = Color.white;
                    icon.SetPixel(x, y, c);
                }
            icon.Apply();
            return icon;
        }

        void OnGUI()
        {
            if (!PanelState.open || uiHidden) { Unlock(); return; }
            GUI.skin = HighLogic.Skin;
            PanelState.rect = GUILayout.Window(ID, PanelState.rect, Draw,
                                               "Low Thrust Transfer",
                                               GUILayout.Width(330));
            // keys typed into a field are not the ship's (or the map's, or
            // staging's): lock them while one of ours has the focus, and a
            // click outside the window gives the focus back
            UnityEngine.Event e = UnityEngine.Event.current;
            if (e.type == EventType.MouseDown && !PanelState.rect.Contains(e.mousePosition))
            {
                GUI.FocusControl(null);
                GUIUtility.keyboardControl = 0;
            }
            string focus = GUI.GetNameOfFocusedControl();
            bool typing = !string.IsNullOrEmpty(focus) && focus.StartsWith("LTW_");
            if (typing && !locked)
            {
                InputLockManager.SetControlLock(ControlTypes.KEYBOARDINPUT, LOCK);
                locked = true;
            }
            else if (!typing) Unlock();
        }

        void Unlock()
        {
            if (!locked) return;
            InputLockManager.RemoveControlLock(LOCK);
            locked = false;
        }

        void Draw(int id)
        {
            Controller c = Controller.Instance;
            bool edit = Controller.mode == Controller.Mode.Idle;

            GUILayout.BeginHorizontal();
            GUILayout.Label("Body", GUILayout.Width(140));
            GUI.enabled = edit;
            if (GUILayout.Button((PanelState.body == "" ? "(choose)" : PanelState.body) + "  \u25bc"))
                listOpen = !listOpen;
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            if (listOpen && edit)
            {
                listScroll = GUILayout.BeginScrollView(listScroll, GUILayout.Height(220));
                foreach (var kv in Bodies())
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Space(14 * kv.Value);
                    if (GUILayout.Button(kv.Key.bodyName))
                    {
                        PanelState.body = kv.Key.bodyName;
                        listOpen = false;
                    }
                    GUILayout.EndHorizontal();
                }
                GUILayout.EndScrollView();
            }

            for (int i = 0; i < LABELS.Length; i++)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(LABELS[i], GUILayout.Width(140));
                GUI.enabled = edit;
                GUI.SetNextControlName("LTW_" + i);
                string v = GUILayout.TextField(PanelState.fields[i], 12);
                if (edit) PanelState.fields[i] = v;
                GUI.enabled = true;
                GUILayout.EndHorizontal();
            }
            GUILayout.Label("Blank is free.  Apoapsis blank: circular.  LAN and "
                            + "argument of periapsis as KSP shows them.", Small());

            // the budget: the trade-off between dv and time
            GUILayout.BeginHorizontal();
            GUILayout.Label("Max \u0394v (m/s)", GUILayout.Width(140));
            GUI.enabled = edit;
            GUI.SetNextControlName("LTW_dv");
            string dvs = GUILayout.TextField(PanelState.maxDv, 8);
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Max time (days)", GUILayout.Width(140));
            GUI.enabled = edit;
            GUI.SetNextControlName("LTW_days");
            string ds = GUILayout.TextField(PanelState.maxDays, 8);
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            if (edit && (dvs != PanelState.maxDv || ds != PanelState.maxDays))
            {
                PanelState.maxDv = dvs;
                PanelState.maxDays = ds;
                PanelState.picked = null;
            }
            if (PanelState.options.Count > 0)
            {
                GUILayout.Label("Options (PLAN): click one to fly it", Small());
                foreach (TradeOption o in PanelState.options)
                {
                    bool on = o == PanelState.picked;
                    string label = (on ? "\u25b6 " : "") + o.days.ToString("F1") + " days, " + o.dv.ToString("F0")
                                   + " m/s";
                    GUI.enabled = edit;
                    if (GUILayout.Button(label)) PanelState.picked = on ? null : o;
                    GUI.enabled = true;
                }
            }

            // the highest warp the autopilot may use (it may go slower: the
            // frame budget, a turn, the propellant one step may draw)
            float[] rates = TimeWarp.fetch != null ? TimeWarp.fetch.warpRates : new float[0];
            if (rates.Length > 1)
            {
                int top = PanelState.maxWarp < 1 || PanelState.maxWarp >= rates.Length ? rates.Length - 1 : PanelState.maxWarp;
                GUILayout.BeginHorizontal();
                GUILayout.Label("Max warp", GUILayout.Width(140));
                if (GUILayout.Button("-", GUILayout.Width(24)) && top > 1) PanelState.maxWarp = top - 1;
                GUILayout.Label(rates[top].ToString("F0") + "x", GUILayout.Width(70));
                if (GUILayout.Button("+", GUILayout.Width(24)) && top < rates.Length - 1) PanelState.maxWarp = top + 1;
                GUILayout.EndHorizontal();
            }

            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            if (edit)
            {
                GUI.enabled = c != null;
                if (GUILayout.Button("PLAN")) c.Plan();
                if (GUILayout.Button("START")) c.Begin();
                GUI.enabled = true;
            }
            else if (GUILayout.Button("TERMINATE") && c != null) c.Terminate();
            GUILayout.EndHorizontal();

            foreach (string line in Controller.status)
            {
                int tab = line.IndexOf('\t');
                if (tab < 0) { GUILayout.Label(line, Small()); continue; }
                GUILayout.BeginHorizontal();
                GUILayout.Label(line.Substring(0, tab), Small(), GUILayout.Width(140));
                GUILayout.Label(line.Substring(tab + 1), Small());
                GUILayout.EndHorizontal();
            }
            GUI.DragWindow();
        }

        GUIStyle small;
        GUIStyle Small()
        {
            if (small == null)
            {
                small = new GUIStyle(GUI.skin.label) { wordWrap = true, fontSize = 12 };
                small.padding = new RectOffset(2, 2, 1, 1);
            }
            return small;
        }

        /// <summary>Every body but the star, planets in order out from it,
        /// each followed by its moons (indent = depth).</summary>
        List<KeyValuePair<CelestialBody, int>> Bodies()
        {
            if (bodies != null) return bodies;
            bodies = new List<KeyValuePair<CelestialBody, int>>();
            CelestialBody sun = Planetarium.fetch != null ? Planetarium.fetch.Sun : null;
            if (sun != null) Add(sun, -1);
            return bodies;
        }

        void Add(CelestialBody b, int depth)
        {
            if (depth >= 0) bodies.Add(new KeyValuePair<CelestialBody, int>(b, depth));
            var kids = new List<CelestialBody>(b.orbitingBodies);
            kids.Sort((x, y) => x.orbit.semiMajorAxis.CompareTo(y.orbit.semiMajorAxis));
            foreach (var k in kids) Add(k, depth + 1);
        }
    }
}
