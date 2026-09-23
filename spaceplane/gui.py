"""In-game panel with START and TERMINATE buttons.

The panel and ``logs/LOG<n>`` are the only output; the script itself is
silent.  Button presses are read through streams and the ``clicked`` flag is
reset by us, as in the kRPC user-interface tutorial.
"""


class ControlPanel:
    WIDTH = 300
    HEIGHT = 420

    def __init__(self, conn):
        self.conn = conn
        self.ui = conn.ui
        canvas = self.ui.stock_canvas
        screen = canvas.rect_transform.size

        self.panel = canvas.add_panel()
        rect = self.panel.rect_transform
        rect.size = (self.WIDTH, self.HEIGHT)
        rect.position = (self.WIDTH / 2 + 30 - screen[0] / 2, 0)

        self.title = self.panel.add_text("SPACEPLANE")
        self.title.rect_transform.position = (0, self.HEIGHT / 2 - 25)
        self.title.rect_transform.size = (self.WIDTH - 20, 24)
        self.title.size = 16
        self.title.color = (0.4, 0.8, 1.0)
        self.title.alignment = self.ui.TextAnchor.middle_center

        self.body = self.panel.add_text("standing by")
        self.body.rect_transform.position = (0, 20)
        self.body.rect_transform.size = (self.WIDTH - 24, self.HEIGHT - 130)
        self.body.size = 13
        self.body.color = (1.0, 1.0, 1.0)
        self.body.alignment = self.ui.TextAnchor.upper_left

        self.start_button = self.panel.add_button("START DEORBIT")
        self.start_button.rect_transform.position = (0, -self.HEIGHT / 2 + 60)
        self.start_button.rect_transform.size = (self.WIDTH - 40, 32)

        self.stop_button = self.panel.add_button("TERMINATE SCRIPT")
        self.stop_button.rect_transform.position = (0, -self.HEIGHT / 2 + 24)
        self.stop_button.rect_transform.size = (self.WIDTH - 40, 32)

        self._start_clicked = conn.add_stream(
            getattr, self.start_button, "clicked")
        self._stop_clicked = conn.add_stream(
            getattr, self.stop_button, "clicked")

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

    def update(self, lines):
        self.body.content = "\n".join(lines)

    def close(self):
        for stream in (self._start_clicked, self._stop_clicked):
            try:
                stream.remove()
            except Exception:                           # noqa: BLE001
                pass
        try:
            self.panel.remove()
        except Exception:                               # noqa: BLE001
            pass
