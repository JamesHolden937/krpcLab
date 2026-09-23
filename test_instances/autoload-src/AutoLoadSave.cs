using UnityEngine;

namespace BoosterlandAutoLoad
{
    /// <summary>
    /// Load a save straight from the main menu, so an unattended instance can
    /// be driven over kRPC.
    ///
    /// kRPC refuses to run outside a game scene: KRPC.Addon checks
    /// CurrentGameScene() and calls Core.StopAll() when it is GameScene.None,
    /// which is what the main menu is.  So an instance parked at the menu
    /// never opens its port, and nothing can connect to tell it to load a
    /// save.  This breaks that circle once; quickfly.py reloads the save
    /// itself for every flight afterwards.
    ///
    /// Runs at MainMenu only, so it cannot affect a flight.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class AutoLoadSave : MonoBehaviour
    {
        const string Tag = "[BoosterlandAutoLoad] ";

        // Wait this many menu frames before loading.  Loading from Start(),
        // or from a coroutine, was tried and does not work: the menu is still
        // constructing, and WaitForSeconds runs on scaled time which is not
        // guaranteed to be advancing yet.  A frame count in Update() depends
        // on nothing but the menu actually rendering.
        const int SettleFrames = 120;

        string folder = "default";
        string save = "quicksave";
        // SPACECENTER, not FLIGHT.  Game.Start() into FLIGHT needs FlightDriver
        // primed with a valid active vessel, and without that it silently does
        // not transition at all -- the menu just stays up while Update() keeps
        // running.  The space centre is a real game scene as far as kRPC is
        // concerned (anything but GameScene.None), so the server starts there,
        // and quickfly.py then loads the quicksave through exactly the path it
        // uses interactively.
        string scene = "SPACECENTER";
        int frames;
        int ticks;
        bool done;

        void Start()
        {
            try
            {
                ConfigNode[] nodes =
                    GameDatabase.Instance.GetConfigNodes("BOOSTERLAND_AUTOLOAD");
                if (nodes != null && nodes.Length > 0)
                {
                    if (nodes[0].HasValue("folder"))
                        folder = nodes[0].GetValue("folder");
                    if (nodes[0].HasValue("save"))
                        save = nodes[0].GetValue("save");
                    if (nodes[0].HasValue("scene"))
                        scene = nodes[0].GetValue("scene");
                }
            }
            catch (System.Exception exc)
            {
                Debug.Log(Tag + "config unreadable, using defaults: " + exc.Message);
            }
            Debug.Log(Tag + "armed for " + folder + "/" + save
                      + ", waiting " + SettleFrames + " menu frames");
        }

        void Awake()
        {
            Debug.Log(Tag + "Awake");
        }

        void OnDestroy()
        {
            // If this fires while we are still waiting, KSP has torn down the
            // addon's GameObject and Update() will never run again -- which
            // looks identical to Unity not running its loop at all.
            Debug.Log(Tag + "OnDestroy (done=" + done + ", ticks=" + ticks + ")");
        }

        void Update()
        {
            // The heartbeat is deliberately BEFORE the `done` guard.  With it
            // after, it stops logging the moment the save is loaded, which
            // looks identical to Unity having stopped running frames -- and
            // that mistake produced a confident wrong diagnosis once already.
            if (++ticks % 60 == 0)
                Debug.Log(Tag + "heartbeat: tick " + ticks
                          + ", scene " + HighLogic.LoadedScene
                          + ", menu frames " + frames
                          + ", loaded " + done);

            if (done)
                return;

            if (HighLogic.LoadedScene != GameScenes.MAINMENU)
                return;
            if (++frames < SettleFrames)
                return;

            done = true;
            Load();
        }

        void Load()
        {
            HighLogic.SaveFolder = folder;

            Game game;
            try
            {
                game = GamePersistence.LoadGame(save, folder, true, false);
            }
            catch (System.Exception exc)
            {
                Debug.LogError(Tag + "LoadGame(" + folder + "/" + save
                               + ") threw: " + exc);
                return;
            }

            if (game == null)
            {
                Debug.LogError(Tag + "no such save: " + folder + "/" + save);
                return;
            }

            try
            {
                game.startScene =
                    (GameScenes) System.Enum.Parse(typeof(GameScenes), scene, true);
            }
            catch (System.Exception exc)
            {
                Debug.LogError(Tag + "bad scene '" + scene + "': " + exc.Message);
            }

            // Game.Start() reads HighLogic.CurrentGame internally.  Without
            // this assignment it throws a NullReferenceException *inside*
            // Game.Start(), Unity swallows it per-callback, and the result is
            // a game that keeps rendering the main menu at 60 fps forever --
            // which is almost impossible to tell from a frozen process.
            HighLogic.CurrentGame = game;

            Debug.Log(Tag + "loading " + folder + "/" + save
                      + " into scene " + game.startScene);
            try
            {
                game.Start();
                Debug.Log(Tag + "game.Start() returned, scene now "
                          + HighLogic.LoadedScene);
            }
            catch (System.Exception exc)
            {
                Debug.LogError(Tag + "game.Start() threw: " + exc);
                // Last resort: ask for the scene directly.
                try
                {
                    HighLogic.LoadScene(GameScenes.SPACECENTER);
                    Debug.Log(Tag + "fell back to HighLogic.LoadScene");
                }
                catch (System.Exception exc2)
                {
                    Debug.LogError(Tag + "LoadScene also threw: " + exc2);
                }
            }
        }
    }
}
