"""In-game launcher: one button per autopilot.

``./run.sh`` with no ``--pilot`` runs this.  It puts a small panel on the
game's screen with a button per entry in ``PILOTS``; pressing one hides the
panel and runs that autopilot as a child process with ``--autostart``, so the
button *is* the start command.  The autopilot's own panel (telemetry and
TERMINATE) takes over for the flight, and when it exits the launcher comes
back, showing how the last flight ended and which log it wrote.

Adding an autopilot is one line in ``PILOTS``; the module has to accept
``--address``/``--rpc-port``/``--stream-port`` and ``--autostart``.

Like the autopilots, this prints nothing: the panel is the output, and
``run.sh`` sends stdout/stderr to ``logs/stderr.log`` for crashes.  Ctrl-C in
the terminal reaches the flying child too (same process group); the launcher
waits for it to hand the vessel back before it removes its own panel.
"""

import argparse
import os
import re
import subprocess
import sys
import time

import krpc

from common import paths

# (button label, run.sh --pilot name, module to run)
PILOTS = [
    ("DEORBIT AND LAND", "spaceplane", "spaceplane.autopilot"),
    ("BOOSTBACK AND LAND", "boosterland", "boosterland.autoland"),
]

# run.sh --pilot accepts these as well as the names above.
ALIASES = {"plane": "spaceplane", "booster": "boosterland"}

POLL_S = 0.2
CHILD_EXIT_GRACE_S = 30.0


def module_for(name):
    """The module a ``--pilot`` name runs, or None."""
    name = ALIASES.get(name, name)
    for _, pilot, module in PILOTS:
        if pilot == name:
            return module
    return None


class LauncherPanel:
    WIDTH = 260
    BUTTON_H = 34

    def __init__(self, conn):
        self.ui = conn.ui
        canvas = self.ui.stock_canvas
        screen = canvas.rect_transform.size
        height = 120 + (len(PILOTS) + 1) * (self.BUTTON_H + 8)

        self.panel = canvas.add_panel()
        rect = self.panel.rect_transform
        rect.size = (self.WIDTH, height)
        # Right-hand side: the autopilots' own panels sit on the left.
        rect.position = (screen[0] / 2 - self.WIDTH / 2 - 30, 0)

        title = self.panel.add_text("krpcLab")
        title.rect_transform.position = (0, height / 2 - 22)
        title.rect_transform.size = (self.WIDTH - 20, 24)
        title.size = 16
        title.color = (0.6, 1.0, 0.6)
        title.alignment = self.ui.TextAnchor.middle_center

        self.status = self.panel.add_text("choose an autopilot")
        self.status.rect_transform.position = (0, height / 2 - 62)
        self.status.rect_transform.size = (self.WIDTH - 24, 52)
        self.status.size = 12
        self.status.color = (1.0, 1.0, 1.0)
        self.status.alignment = self.ui.TextAnchor.upper_center

        self.buttons = []
        y = height / 2 - 110
        for label, _, _ in PILOTS + [("QUIT LAUNCHER", None, None)]:
            button = self.panel.add_button(label)
            button.rect_transform.position = (0, y)
            button.rect_transform.size = (self.WIDTH - 40, self.BUTTON_H)
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


def log_names():
    try:
        return {n for n in os.listdir(paths.LOGS) if re.fullmatch(r"LOG\d+", n)}
    except OSError:
        return set()


def fly(module, connection_args):
    """Run one autopilot to completion; returns (exit code, new log names)."""
    before = log_names()
    command = [sys.executable, "-m", module, "--autostart"] + connection_args
    child = subprocess.Popen(command, cwd=paths.ROOT)
    try:
        code = child.wait()
    except KeyboardInterrupt:
        # The child got the same SIGINT and is shutting down; let it hand the
        # vessel back before the launcher disappears.
        try:
            child.wait(timeout=CHILD_EXIT_GRACE_S)
        except subprocess.TimeoutExpired:
            child.kill()
        raise
    return code, sorted(log_names() - before, key=lambda n: int(n[3:]))


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="In-game launcher for the krpcLab autopilots.")
    p.add_argument("--address", default=None)
    p.add_argument("--rpc-port", type=int, default=None)
    p.add_argument("--stream-port", type=int, default=None)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    kwargs = {"name": "krpcLab launcher"}
    forward = []
    if args.address:
        kwargs["address"] = args.address
        forward += ["--address", args.address]
    if args.rpc_port:
        kwargs["rpc_port"] = args.rpc_port
        forward += ["--rpc-port", str(args.rpc_port)]
    if args.stream_port:
        kwargs["stream_port"] = args.stream_port
        forward += ["--stream-port", str(args.stream_port)]

    conn = krpc.connect(**kwargs)
    panel = LauncherPanel(conn)
    try:
        while True:
            choice = panel.pressed()
            if choice is None:
                time.sleep(POLL_S)
                continue
            if choice >= len(PILOTS):
                return 0
            label, name, module = PILOTS[choice]
            panel.show(False)
            conn.ui.message("krpcLab: %s" % label.lower(), duration=3.0)
            code, logs = fly(module, forward)
            ended = "finished" if code == 0 else "exited %d" % code
            panel.show(True, "%s %s\n%s" % (
                name, ended, ", ".join(logs) if logs else "no log written"))
    except KeyboardInterrupt:
        return 0
    finally:
        panel.close()
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
