"""In-game launcher: one button per autopilot.

``./run.sh`` with no ``--pilot`` runs this -- or the window's START SCRIPT
button, which starts the kRPC server and runs ``./run.sh``.  The window is
the krpcLab window mod (``launcherMod/``, kRPC service ``KrpcLabLauncher``;
its only UI): a stock KSP window with a toolbar button, shown in flight.
Pressing a button runs that autopilot as a child process with
``--autostart``, so the button *is* the start command.  While the
autopilot's own panel (``common.panel``) is up the menu gives way to it;
when the flight exits the menu comes back, saying how it ended and which
log it wrote.

**It survives the game's scene changes** (a save loaded, the space centre,
the SPH): the menu lives in the mod's statics and the kRPC connection
outlives the scene.  **A scene change or a switch of the active vessel
terminates the flying autopilot** and brings the menu back (the mod's
epoch counter); an autopilot that loads a save or switches vessel itself
announces it first (``panel.expect_scene_change``).  **When the game dies**
(the connection drops, or the game's pid named by ``KRPCLAB_GAME_PID`` is
gone) the autopilot is killed and the launcher exits; an autopilot also
dies with the launcher (``PR_SET_PDEATHSIG``).

Adding an autopilot is one line in ``PILOTS``; the module has to accept
``--address``/``--rpc-port``/``--stream-port`` and ``--autostart``.

Like the autopilots, this prints nothing: the window is the output, and
``run.sh`` sends stdout/stderr to ``logs/stderr.log`` for crashes.  Ctrl-C in
the terminal reaches the flying child too (same process group); the launcher
waits for it to hand the vessel back before it removes its own menu.
"""

import os
import re
import signal
import subprocess
import sys
import time

import krpc

from common import cli, panel, paths

# (button label, run.sh --pilot name, module to run)
PILOTS = [
    ("DEORBIT AND LAND", "spaceplane", "spaceplane.autopilot"),
    ("BOOSTBACK AND LAND", "boosterland", "boosterland.autoland"),
    ("BOOSTBACK FROM FMRS SAVE", "fmrs", "boosterland.fmrs"),
    ("ASCENT TO ORBIT", "ascent", "ascent.autopilot"),
    ("AIRPLANE: CRUISE / HOLD", "airplane", "airplane.autopilot"),
    ("PROP PITCH FOR TOP SPEED", "prop", "propPitch.autopilot"),
]

# run.sh --pilot accepts these as well as the names above.
ALIASES = {"plane": "spaceplane", "booster": "boosterland"}

POLL_S = 0.2
HANDOVER_S = 3.0
CHILD_EXIT_GRACE_S = 30.0
# A scene change under a flying autopilot: SIGINT (it hands the vessel back
# if it still can), then SIGKILL after this.
SCENE_KILL_GRACE_S = 3.0
# The game's death: no vessel to hand back.
DEATH_KILL_GRACE_S = 1.0
GAME_PID_ENV = "KRPCLAB_GAME_PID"


def module_for(name):
    """The module a ``--pilot`` name runs, or None."""
    name = ALIASES.get(name, name)
    for _, pilot, module in PILOTS:
        if pilot == name:
            return module
    return None


class GameGone(Exception):
    """The game (or its kRPC server) is gone: exit."""


class Window:
    """The launcher's menu in the window mod.  The service is reached by
    name strings because the bundle's minifier knows only the kRPC services
    in the game's ``GameData/kRPC/*.json`` and would rename ``show``/
    ``pressed`` to match this file's own methods."""

    PING_S = 1.0            # the mod calls the launcher gone after 5 s

    def __init__(self, conn):
        svc = panel.service(conn)
        if svc is None:
            raise SystemExit("the game has no krpcLab window mod: run "
                             "launcherMod/build.sh and restart the game")
        self._update = getattr(svc, "update")
        self._pressed = getattr(svc, "pressed")
        self._epoch = getattr(svc, "epoch")
        self._close = getattr(svc, "close")
        self._panel_close = getattr(svc, "panel_close")
        self.status = "choose an autopilot"
        self.buttons = True
        self.open = True
        self.pinged = time.monotonic()
        getattr(svc, "show")([label for label, _, _ in PILOTS]
                             + ["QUIT LAUNCHER"], self.status)

    def _send(self):
        self._update(self.status, self.buttons, self.open)
        self.pinged = time.monotonic()

    def pressed(self):
        i = self._pressed()
        self.pinged = time.monotonic()
        return None if i < 0 else i

    def epoch(self):
        """The mod's count of scene changes and vessel switches (also the
        heartbeat)."""
        e = self._epoch()
        self.pinged = time.monotonic()
        return e

    def show(self, visible, status=None):
        if status is not None:
            self.status = status
        self.open = visible
        self._send()

    def show_buttons(self, visible):
        self.buttons = visible
        self._send()

    def close_panel(self):
        """A killed autopilot leaves its panel up until its heartbeat
        lapses: take it down now."""
        self._panel_close()

    def close(self):
        try:
            self._close()
        except Exception:                               # noqa: BLE001
            pass


def log_names():
    try:
        return {n for n in os.listdir(paths.LOGS) if re.fullmatch(r"LOG\d+", n)}
    except OSError:
        return set()


def game_pid():
    try:
        return int(os.environ.get(GAME_PID_ENV, ""))
    except ValueError:
        return None


def alive(pid):
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except OSError:
        return True             # exists, not ours
    return True


def call(fn, *args):
    """A kRPC call that tells the game's death from a scene loading under
    it: a dropped connection raises ``GameGone``; anything else (a call
    made mid-load) returns None."""
    try:
        return fn(*args)
    except (OSError, EOFError, ConnectionError) as exc:
        raise GameGone(repr(exc))
    except Exception:                                   # noqa: BLE001
        return None


def _die_with_parent():
    """In the child: SIGTERM when the launcher dies (Linux only)."""
    try:
        import ctypes
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)
    except Exception:                                   # noqa: BLE001
        pass


def stop(child, grace):
    """SIGINT, then SIGKILL after ``grace`` seconds."""
    if child.poll() is not None:
        return
    try:
        child.send_signal(signal.SIGINT)
        child.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()
    except OSError:
        pass


def fly(module, connection_args, window):
    """Run one autopilot to completion; returns (exit code or None,
    new log names, why it was stopped or None).  A scene change or vessel
    switch stops it; the game's death stops it and raises ``GameGone``."""
    before = log_names()
    command = [sys.executable, "-m", module, "--autostart"] + connection_args
    # Run from ``tools/bundle.py``'s single file, the modules live in it.
    bundle = os.environ.get("KRPCLAB_BUNDLE")
    if bundle:
        command[1:2] = [bundle, "-m"]
    pid = game_pid()
    epoch = call(window.epoch)
    child = subprocess.Popen(command, cwd=paths.ROOT,
                             preexec_fn=_die_with_parent)
    handover = None
    stopped = None
    try:
        while child.poll() is None:
            if handover is None and log_names() - before:
                handover = time.monotonic() + HANDOVER_S
            if handover is not None and time.monotonic() >= handover:
                call(window.show, False)
                handover = float("inf")
            if not alive(pid):
                raise GameGone("game pid %d gone" % pid)
            now = call(window.epoch)
            if epoch is None:
                epoch = now
            elif now is not None and now != epoch:
                stopped = "stopped: save loaded or vessel switched"
                stop(child, SCENE_KILL_GRACE_S)
                break
            time.sleep(POLL_S)
        code = child.returncode
    except GameGone:
        stop(child, DEATH_KILL_GRACE_S)
        raise
    except KeyboardInterrupt:
        # The child got the same SIGINT and is shutting down; let it hand the
        # vessel back before the launcher disappears.
        try:
            child.wait(timeout=CHILD_EXIT_GRACE_S)
        except subprocess.TimeoutExpired:
            child.kill()
        raise
    return code, sorted(log_names() - before, key=lambda n: int(n[3:])), stopped


def parse_args(argv):
    return cli.parser("krpclab", "In-game launcher for the krpcLab "
                      "autopilots.", pilot=False).parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    kwargs = {"name": "krpcLab launcher"}
    forward = []
    kwargs.update(cli.connect_kwargs(args, kwargs.get("name")))
    if args.address:
        forward += ["--address", args.address]
    if args.rpc_port:
        forward += ["--rpc-port", str(args.rpc_port)]
    if args.stream_port:
        forward += ["--stream-port", str(args.stream_port)]

    conn = krpc.connect(**kwargs)
    window = Window(conn)
    pid = game_pid()
    try:
        while True:
            if not alive(pid):
                return 0
            choice = call(window.pressed)
            if choice is None:
                time.sleep(POLL_S)
                continue
            if choice >= len(PILOTS):
                return 0
            label, name, module = PILOTS[choice]
            # The autopilot's panel opens where the menu is, so the window
            # reads as switching to the flight: the menu stays up, saying
            # so, until the autopilot is drawing, and comes back after.
            call(window.show_buttons, False)
            call(window.show, True, "starting %s ..." % label.lower())
            code, logs, stopped = fly(module, forward, window)
            call(window.close_panel)
            ended = stopped or ("finished" if code == 0 else "exited %d" % code)
            status = "%s %s\n%s" % (
                name, ended, ", ".join(logs) if logs else "no log written")
            call(window.show_buttons, True)
            call(window.show, True, status)
    except (KeyboardInterrupt, GameGone):
        return 0
    finally:
        window.close()
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
