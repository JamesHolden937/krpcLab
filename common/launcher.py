"""In-game launcher: one button per autopilot.

``./run.sh`` with no ``--pilot`` runs this.  It puts a window on the
game's screen with a button per entry in ``PILOTS``; pressing one runs that
autopilot as a child process with ``--autostart``, so the button *is* the
start command.  The window is a stock KSP window with a toolbar button,
drawn by ``launcherMod/`` (its kRPC service ``KrpcLabLauncher``); a game
without that mod gets a kRPC panel instead (``LauncherPanel``).  While the
autopilot's own panel (telemetry and TERMINATE, ``common.panel``) is up the
launcher's window hides; when the flight exits it comes back, showing how
it ended and which log it wrote.

Adding an autopilot is one line in ``PILOTS``; the module has to accept
``--address``/``--rpc-port``/``--stream-port`` and ``--autostart``.

Like the autopilots, this prints nothing: the panel is the output, and
``run.sh`` sends stdout/stderr to ``logs/stderr.log`` for crashes.  Ctrl-C in
the terminal reaches the flying child too (same process group); the launcher
waits for it to hand the vessel back before it removes its own panel.
"""

import os
import re
import subprocess
import sys
import time

import krpc

from common import cli

from common import panel, paths

# (button label, run.sh --pilot name, module to run)
PILOTS = [
    ("DEORBIT AND LAND", "spaceplane", "spaceplane.autopilot"),
    ("BOOSTBACK AND LAND", "boosterland", "boosterland.autoland"),
    ("BOOSTBACK FROM FMRS SAVE", "fmrs", "boosterland.fmrs"),
    ("ASCENT TO ORBIT", "ascent", "ascent.autopilot"),
    ("PROP PITCH FOR TOP SPEED", "prop", "propPitch.autopilot"),
]

# run.sh --pilot accepts these as well as the names above.
ALIASES = {"plane": "spaceplane", "booster": "boosterland"}

POLL_S = 0.2
HANDOVER_S = 3.0
CHILD_EXIT_GRACE_S = 30.0


def module_for(name):
    """The module a ``--pilot`` name runs, or None."""
    name = ALIASES.get(name, name)
    for _, pilot, module in PILOTS:
        if pilot == name:
            return module
    return None


class LauncherPanel:
    BUTTON_H = 34

    def __init__(self, conn):
        self.ui = conn.ui
        self.panel = panel.window(conn)
        top = panel.HEIGHT / 2

        title = self.panel.add_text("krpcLab")
        title.rect_transform.position = (0, top - 25)
        title.rect_transform.size = (panel.WIDTH - 20, 24)
        title.size = 16
        title.color = panel.rgba((0.6, 1.0, 0.6))
        title.alignment = self.ui.TextAnchor.middle_center

        self.status = self.panel.add_text("choose an autopilot")
        self.status.rect_transform.position = (0, top - 70)
        self.status.rect_transform.size = (panel.WIDTH - 24, 52)
        self.status.size = 12
        self.status.color = panel.rgba(panel.TEXT)
        self.status.alignment = self.ui.TextAnchor.upper_center

        self.buttons = []
        y = top - 125
        for label, _, _ in PILOTS + [("QUIT LAUNCHER", None, None)]:
            button = self.panel.add_button(label)
            button.rect_transform.position = (0, y)
            button.rect_transform.size = (panel.WIDTH - 40, self.BUTTON_H)
            self.buttons.append(button)
            y -= self.BUTTON_H + 8
        self.clicked = [conn.add_stream(getattr, b, "clicked")
                        for b in self.buttons]

    def pressed(self):
        """Index of the button pressed since the last call, or None."""
        for i, stream in enumerate(self.clicked):
            if stream():
                self.buttons[i].clicked = False
                return i
        return None

    def show(self, visible, status=None):
        if status is not None:
            self.status.content = status
        self.panel.visible = visible

    def tick(self):
        pass

    def show_buttons(self, visible):
        for button in self.buttons:
            button.visible = visible

    def close(self):
        for stream in self.clicked:
            try:
                stream.remove()
            except Exception:                           # noqa: BLE001
                pass
        try:
            self.panel.remove()
        except Exception:                               # noqa: BLE001
            pass


class StockLauncher:
    """The launcher's window as ``launcherMod``'s stock KSP window: the same
    interface as ``LauncherPanel``.  The service is reached by name strings
    because the bundle's minifier knows only the kRPC services in the
    game's ``GameData/kRPC/*.json`` and would rename ``show``/``pressed``
    to match this file's own methods."""

    SERVICE = "krpc_lab_launcher"
    PING_S = 1.0            # the mod calls the launcher gone after 5 s

    @classmethod
    def available(cls, conn):
        return getattr(conn, cls.SERVICE, None) is not None

    def __init__(self, conn):
        svc = getattr(conn, self.SERVICE)
        self._update = getattr(svc, "update")
        self._pressed = getattr(svc, "pressed")
        self._close = getattr(svc, "close")
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

    def tick(self):
        """Keep the window from saying the launcher is gone mid-flight."""
        if time.monotonic() - self.pinged >= self.PING_S:
            self.pressed()

    def show(self, visible, status=None):
        if status is not None:
            self.status = status
        self.open = visible
        self._send()

    def show_buttons(self, visible):
        self.buttons = visible
        self._send()

    def close(self):
        try:
            self._close()
        except Exception:                               # noqa: BLE001
            pass


def make_window(conn):
    """The stock window when the game has ``launcherMod``, else the panel."""
    if StockLauncher.available(conn):
        return StockLauncher(conn)
    return LauncherPanel(conn)


def log_names():
    try:
        return {n for n in os.listdir(paths.LOGS) if re.fullmatch(r"LOG\d+", n)}
    except OSError:
        return set()


def fly(module, connection_args, started, tick=None):
    """Run one autopilot to completion; returns (exit code, new log names).

    ``started`` is called once the autopilot's log exists plus
    ``HANDOVER_S``, by when its own panel has replaced the launcher's;
    ``tick``, if given, every poll while the autopilot runs.
    """
    before = log_names()
    command = [sys.executable, "-m", module, "--autostart"] + connection_args
    # Run from ``tools/bundle.py``'s single file, the modules live in it.
    bundle = os.environ.get("KRPCLAB_BUNDLE")
    if bundle:
        command[1:2] = [bundle, "-m"]
    child = subprocess.Popen(command, cwd=paths.ROOT)
    handover = None
    try:
        while child.poll() is None:
            if started is not None:
                if handover is None and log_names() - before:
                    handover = time.monotonic() + HANDOVER_S
                if handover is not None and time.monotonic() >= handover:
                    started()
                    started = None
            if tick is not None:
                try:
                    tick()
                except Exception:                       # noqa: BLE001
                    tick = None     # the scene was reloaded under it
            time.sleep(POLL_S)
        code = child.returncode
    except KeyboardInterrupt:
        # The child got the same SIGINT and is shutting down; let it hand the
        # vessel back before the launcher disappears.
        try:
            child.wait(timeout=CHILD_EXIT_GRACE_S)
        except subprocess.TimeoutExpired:
            child.kill()
        raise
    return code, sorted(log_names() - before, key=lambda n: int(n[3:]))


def hide(window):
    try:
        window.show(False)
    except Exception:                                   # noqa: BLE001
        pass            # the scene was reloaded under it; rebuilt after


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
    window = make_window(conn)
    try:
        while True:
            choice = window.pressed()
            if choice is None:
                time.sleep(POLL_S)
                continue
            if choice >= len(PILOTS):
                return 0
            label, name, module = PILOTS[choice]
            # The autopilot's panel opens where this one is, so the window
            # reads as switching to the flight: the launcher stays up, saying
            # so, until the autopilot is drawing, and comes back after.
            window.show_buttons(False)
            window.show(True, "starting %s ..." % label.lower())
            code, logs = fly(module, forward, lambda: hide(window),
                             window.tick)
            ended = "finished" if code == 0 else "exited %d" % code
            status = "%s %s\n%s" % (
                name, ended, ", ".join(logs) if logs else "no log written")
            try:
                window.show_buttons(True)
                window.show(True, status)
            except Exception:                           # noqa: BLE001
                # A pilot that loads a save (fmrs) rebuilds the scene, and
                # the panel and this connection go with it: start afresh.
                window.close()
                try:
                    conn.close()
                except Exception:                       # noqa: BLE001
                    pass
                conn = krpc.connect(**kwargs)
                window = make_window(conn)
                window.show(True, status)
    except KeyboardInterrupt:
        return 0
    finally:
        window.close()
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
