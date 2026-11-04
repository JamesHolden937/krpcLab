// The krpcLab launcher's window: a stock KSP window (the game's own skin)
// with a toolbar button, drawn by this mod and driven by the Python launcher
// (common/launcher.py) through a kRPC service, "KrpcLabLauncher".  The
// launcher hands it the button labels and a status line and reads back which
// button was pressed; the window draws them.  Nothing here knows what an
// autopilot is.
//
// The launcher pings the service at least every PING_S while it runs; when
// it stops (killed, or ./run.sh never started) the window says so instead of
// showing buttons nobody is listening to.

using System.Collections.Generic;
using KRPC.Service.Attributes;
using KSP.UI.Screens;
using UnityEngine;

namespace KrpcLab
{
    /// <summary>The window's contents (statics outlive a scene change, so a
    /// save loaded by an autopilot keeps the menu).</summary>
    public static class LauncherState
    {
        public const float ALIVE_S = 5f;        // no ping for this long: launcher gone
        public static List<string> buttons = new List<string>();
        public static string status = "";
        public static bool showButtons = true;
        public static bool open;
        public static int pressed = -1;
        public static float lastPing = -1e9f;
        internal static Rect rect = new Rect(30, 0, 280, 10);
        internal static bool placed;

        public static bool Alive { get { return Time.realtimeSinceStartup - lastPing < ALIVE_S; } }
        public static void Ping() { lastPing = Time.realtimeSinceStartup; }
    }

    [KRPCService(Name = "KrpcLabLauncher", GameScene = KRPC.Service.GameScene.All)]
    public static class KrpcLabLauncher
    {
        /// <summary>Open the window with these buttons and status line.</summary>
        [KRPCProcedure]
        public static void Show(IList<string> buttons, string status)
        {
            LauncherState.buttons = new List<string>(buttons);
            LauncherState.status = status;
            LauncherState.showButtons = true;
            LauncherState.pressed = -1;
            LauncherState.open = true;
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

        /// <summary>The launcher is going: no buttons, window closed.</summary>
        [KRPCProcedure]
        public static void Close()
        {
            LauncherState.buttons = new List<string>();
            LauncherState.status = "";
            LauncherState.open = false;
            LauncherState.lastPing = -1e9f;
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class LauncherWindow : MonoBehaviour
    {
        const int ID = 0x6b4c4142;              // "kLAB"

        ApplicationLauncherButton button;
        Texture2D icon;
        bool uiHidden;
        GUIStyle statusStyle;

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
                                                  GUILayout.Width(280));
        }

        void Draw(int id)
        {
            if (statusStyle == null)
                statusStyle = new GUIStyle(GUI.skin.label) { wordWrap = true, alignment = TextAnchor.MiddleCenter };
            if (!LauncherState.Alive)
            {
                GUILayout.Label("The launcher is not running.\nStart it with ./run.sh", statusStyle);
            }
            else
            {
                if (LauncherState.status.Length > 0)
                    GUILayout.Label(LauncherState.status, statusStyle);
                if (LauncherState.showButtons)
                {
                    GUILayout.Space(4);
                    for (int i = 0; i < LauncherState.buttons.Count; i++)
                        if (GUILayout.Button(LauncherState.buttons[i], GUILayout.Height(30)))
                            LauncherState.pressed = i;
                }
            }
            GUI.DragWindow();
        }
    }
}
