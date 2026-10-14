// The low-thrust autopilot's window: a stock KSP window (the game's own
// skin, as MechJeb's are) opened from the toolbar.  It holds the target the
// pilot reads -- body from a dropdown, periapsis, apoapsis, inclination,
// LAN and argument of periapsis -- the PLAN / START / TERMINATE buttons, and
// the lines the pilot writes back.  The flying is all the pilot's (Python,
// over kRPC); the window only keeps the fields, counts the clicks and shows
// the status.

using System;
using System.Collections.Generic;
using KRPC.Service.Attributes;
using KSP.UI.Screens;
using UnityEngine;

namespace LowThrustWarp
{
    /// <summary>The window's state, shared between the window and the
    /// kRPC procedures (statics outlive a scene change, so the target is
    /// still there next flight).</summary>
    internal static class PanelState
    {
        internal static string body = "";
        // periapsis, apoapsis (km), inclination, LAN, argument of periapsis (deg)
        internal static string[] fields = { "", "", "", "", "" };
        internal static int plan, start, stop;
        internal static List<string> status = new List<string>();
        internal static bool flying;
        internal static bool open;
        internal static float lastPoll = -100f;
        internal static Rect rect = new Rect(Screen.width - 380, 120, 330, 10);

        internal static bool PilotConnected()
        {
            return Time.realtimeSinceStartup - lastPoll < 3f;
        }
    }

    public static partial class LowThrust
    {
        // ---- the window ------------------------------------------------------

        /// <summary>Open or close the window.</summary>
        [KRPCProcedure]
        public static void PanelOpen(bool open)
        {
            PanelState.open = open;
        }

        /// <summary>Clicks on [PLAN, START, TERMINATE] since load (read on
        /// a stream: the window shows the pilot as connected while it
        /// is).</summary>
        [KRPCProcedure]
        public static IList<int> PanelClicks()
        {
            PanelState.lastPoll = Time.realtimeSinceStartup;
            return new List<int> { PanelState.plan, PanelState.start, PanelState.stop };
        }

        /// <summary>The target as typed: [body, periapsis km, apoapsis km,
        /// inclination, LAN, argument of periapsis (deg)]; "" = left
        /// blank.</summary>
        [KRPCProcedure]
        public static IList<string> PanelTarget()
        {
            var r = new List<string> { PanelState.body };
            foreach (string f in PanelState.fields) r.Add(f.Trim());
            return r;
        }

        /// <summary>Fill the target fields (the pilot's --target).</summary>
        [KRPCProcedure]
        public static void PanelSetTarget(IList<string> target)
        {
            if (target.Count > 0) PanelState.body = target[0];
            for (int i = 0; i < PanelState.fields.Length && i + 1 < target.Count; i++)
                PanelState.fields[i] = target[i + 1];
        }

        /// <summary>The status lines under the buttons; a tab splits a
        /// line into a label and a value.</summary>
        [KRPCProcedure]
        public static void PanelStatus(IList<string> lines)
        {
            PanelState.status = new List<string>(lines);
        }

        /// <summary>Flying: TERMINATE only; not: PLAN and START.</summary>
        [KRPCProcedure]
        public static void PanelFlying(bool flying)
        {
            PanelState.flying = flying;
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class Window : MonoBehaviour
    {
        const int ID = 0x4c54574e;              // "LTWN"
        const string LOCK = "LowThrustWarp_typing";
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
            // the toolbar button follows the window (the pilot opens it)
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
            Event e = Event.current;
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
            bool pilot = PanelState.PilotConnected();
            bool edit = !PanelState.flying;

            GUILayout.BeginHorizontal();
            GUILayout.Label("Body", GUILayout.Width(140));
            GUI.enabled = edit;
            if (GUILayout.Button((PanelState.body == "" ? "(choose)" : PanelState.body) + "  ▼"))
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
                            + "argument of periapsis as KSP shows them.",
                            Small());

            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            if (!PanelState.flying)
            {
                GUI.enabled = pilot;
                if (GUILayout.Button("PLAN")) PanelState.plan++;
                if (GUILayout.Button("START")) PanelState.start++;
                GUI.enabled = true;
            }
            else if (GUILayout.Button("TERMINATE")) PanelState.stop++;
            GUILayout.EndHorizontal();

            if (!pilot)
                GUILayout.Label("Pilot not running: start LOW THRUST TRANSFER from "
                                + "the krpcLab launcher (or ./run.sh --pilot lowthrust).",
                                Small());
            foreach (string line in PanelState.status)
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
