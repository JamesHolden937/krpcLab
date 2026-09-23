"""In-game telemetry panel with START and TERMINATE buttons.

The panel is the only user-facing output; the script itself is silent.  Button
presses are read through kRPC streams (cheap polling) and the ``clicked`` flag
is reset by us, exactly as in the kRPC user-interface tutorial.
"""

from common import vec


class ControlPanel:
    WIDTH = 300
    HEIGHT = 400

    def __init__(self, conn):
        self.conn = conn
        self.ui = conn.ui
        canvas = self.ui.stock_canvas
        screen = canvas.rect_transform.size

        self.panel = canvas.add_panel()
        rect = self.panel.rect_transform
        rect.size = (self.WIDTH, self.HEIGHT)
        rect.position = (self.WIDTH / 2 + 30 - screen[0] / 2, 0)

        self.title = self.panel.add_text("BOOSTERLAND")
        self.title.rect_transform.position = (0, self.HEIGHT / 2 - 25)
        self.title.rect_transform.size = (self.WIDTH - 20, 24)
        self.title.size = 16
        self.title.color = (1.0, 0.75, 0.2)
        self.title.alignment = self.ui.TextAnchor.middle_center

        self.body = self.panel.add_text("standing by")
        self.body.rect_transform.position = (0, 20)
        self.body.rect_transform.size = (self.WIDTH - 24, self.HEIGHT - 130)
        self.body.size = 13
        self.body.color = (1.0, 1.0, 1.0)
        self.body.alignment = self.ui.TextAnchor.upper_left

        self.start_button = self.panel.add_button("START AUTOLAND")
        self.start_button.rect_transform.position = (0, -self.HEIGHT / 2 + 60)
        self.start_button.rect_transform.size = (self.WIDTH - 40, 32)

        self.stop_button = self.panel.add_button("TERMINATE SCRIPT")
        self.stop_button.rect_transform.position = (0, -self.HEIGHT / 2 + 24)
        self.stop_button.rect_transform.size = (self.WIDTH - 40, 32)

        self._start_clicked = conn.add_stream(
            getattr, self.start_button, "clicked")
        self._stop_clicked = conn.add_stream(
            getattr, self.stop_button, "clicked")

    # -- input -------------------------------------------------------------
    def start_pressed(self):
        if self._start_clicked():
            self.start_button.clicked = False
            return True
        return False

    def terminate_pressed(self):
        if self._stop_clicked():
            self.stop_button.clicked = False
            return True
        return False

    def set_start_label(self, text):
        self.start_button.text.content = text

    # -- output ------------------------------------------------------------
    def update(self, lines):
        self.body.content = "\n".join(lines)

    def close(self):
        for stream in (self._start_clicked, self._stop_clicked):
            try:
                stream.remove()
            except Exception:
                pass
        try:
            self.panel.remove()
        except Exception:
            pass


def telemetry_lines(state, snap, extra):
    """Format the shared telemetry block shown in-game and written to the log."""
    lines = [
        "phase   : %s" % state,
        "alt AGL : %8.0f m" % snap.height_above_pad,
        "vert v  : %8.1f m/s" % snap.vertical_speed,
        "horz v  : %8.1f m/s" % snap.horizontal_speed,
        "speed   : %8.1f m/s" % vec.norm(snap.velocity),
        "mass    : %8.2f t" % (snap.mass / 1000.0),
        "throttle: %8.2f" % snap.throttle,
    ]
    lines.extend(extra)
    return lines
