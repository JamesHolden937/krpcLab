"""The one in-game window every krpcLab panel draws in.

The launcher and each autopilot share its geometry, so pressing a launcher
button reads as the window switching from the menu to the flight's telemetry
rather than a second window opening: the launcher hides its panel and the
autopilot's appears in exactly the same place.

kRPC's UI service gives a panel no background colour, only KSP's translucent
window sprite, whose white frame is uneven (2.5-5 px at 1280x720) and cannot
be restyled.  So the root panel is an empty container of no size, which
draws no sprite, and the window is drawn out of text: a rounded rectangle is
two crossed rectangles and a circle in each corner.  A rectangle is a text
element stuffed with more full-block glyphs than it can hold -- Unity wraps
and truncates them to the element's own rect, so its edges are the rect's at
any UI scale (glyph counts are not: the font is pixel-snapped, 1-3 px a
glyph at small sizes) -- and a circle is one ``FILL_CIRCLE`` glyph.  A red
shape the size of the window goes first and a black one inset by ``BORDER``
over it, which leaves an even red frame with rounded corners.
"""

WIDTH = 300
HEIGHT = 420
MARGIN = 30          # from the left edge of the screen

# OLED palette: true-black ground, dim greys, one accent per autopilot.
BLACK = (0.0, 0.0, 0.0)
TEXT = (0.85, 0.85, 0.85)
DIM = (0.5, 0.5, 0.5)
FRAME = (0.85, 0.0, 0.0)
# Each autopilot's title colour.
ACCENT_BOOSTER = (1.0, 0.75, 0.2)
ACCENT_SPACEPLANE = (0.4, 0.8, 1.0)

BORDER = 5           # frame width, canvas units (2 px at 1280x720)
RADIUS = 16          # outer corner radius, canvas units

FILL_GLYPH = u"█"
FILL_SIZE = 5
# More glyphs than the window holds at any UI scale (~9600 at 1280x720), and
# under Unity's 16k-character limit for one text mesh.
FILL_COUNT = 12000
FILL_CIRCLE = u"●"
# The circle's diameter, and how far below the line's centre it sits, per
# unit of font size -- measured off ``SpaceCenter.screenshot`` at sizes 40
# and 80.  Wrong, the corners bulge or notch where they meet the edges.
CIRCLE_DIAMETER = 0.43
CIRCLE_DROP = 0.045


def rgba(color):
    """A text colour as the server takes it: kRPC 0.7 wants a
    ``StdLib.ColorRGBA`` structure (an RGB tuple is refused); 0.6 and the
    offline fakes take the tuple."""
    try:
        from krpc.services.stdlib import ColorRGBA
    except ImportError:
        return tuple(color)
    c = tuple(float(x) for x in color)
    return ColorRGBA(*(c + (1.0,) * (4 - len(c))))


def _text(conn, panel, content, size, position, rect, color):
    text = panel.add_text(content)
    text.rect_transform.size = rect
    text.rect_transform.position = position
    text.size = size
    text.line_spacing = 1.0
    text.color = rgba(color)
    text.alignment = conn.ui.TextAnchor.middle_center
    return text


def _rounded(conn, panel, width, height, radius, color):
    """A filled ``width`` x ``height`` rectangle with rounded corners,
    centred on the panel."""
    radius = max(0.0, min(radius, width / 2.0, height / 2.0))
    fill = FILL_GLYPH * FILL_COUNT
    _text(conn, panel, fill, FILL_SIZE, (0, 0),
          (width, height - 2 * radius), color)
    _text(conn, panel, fill, FILL_SIZE, (0, 0),
          (width - 2 * radius, height), color)
    if radius <= 0.0:
        return
    size = int(round(2.0 * radius / CIRCLE_DIAMETER))
    drop = CIRCLE_DROP * size
    for sx in (-1, 1):
        for sy in (-1, 1):
            _text(conn, panel, FILL_CIRCLE, size,
                  (sx * (width / 2.0 - radius),
                   sy * (height / 2.0 - radius) + drop),
                  (3 * size, 3 * size), color)


def window(conn):
    """An opaque black panel with a thin red frame at the shared position;
    add content to it."""
    canvas = conn.ui.stock_canvas
    screen = canvas.rect_transform.size
    panel = canvas.add_panel()
    rect = panel.rect_transform
    rect.size = (0, 0)
    rect.position = (WIDTH / 2 + MARGIN - screen[0] / 2, 0)
    # Children draw in the order they are added, so the frame and the fill
    # go first and everything the caller adds afterwards sits on top.
    _rounded(conn, panel, WIDTH, HEIGHT, RADIUS, FRAME)
    _rounded(conn, panel, WIDTH - 2 * BORDER, HEIGHT - 2 * BORDER,
             RADIUS - BORDER, BLACK)
    return panel


class ControlPanel:
    """START and TERMINATE under a title in the autopilot's ``accent``
    colour, and a body of telemetry lines.  Button presses are read through
    kRPC streams (cheap polling) and the ``clicked`` flag is reset here, as
    in the kRPC user-interface tutorial."""

    WIDTH = WIDTH
    HEIGHT = HEIGHT

    def __init__(self, conn, title, start_label, accent):
        self.conn = conn
        self.ui = conn.ui
        self.panel = window(conn)

        self.title = self.panel.add_text(title)
        self.title.rect_transform.position = (0, self.HEIGHT / 2 - 25)
        self.title.rect_transform.size = (self.WIDTH - 20, 24)
        self.title.size = 16
        self.title.color = rgba(accent)
        self.title.alignment = self.ui.TextAnchor.middle_center

        self.body = self.panel.add_text("standing by")
        self.body.rect_transform.position = (0, 20)
        self.body.rect_transform.size = (self.WIDTH - 24, self.HEIGHT - 130)
        self.body.size = 13
        self.body.color = rgba(TEXT)
        self.body.alignment = self.ui.TextAnchor.upper_left

        self.start_button = self.panel.add_button(start_label)
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

    def hide_start(self):
        """``--autostart``: the flight is already running, so no START."""
        self.start_button.visible = False

    def set_start_label(self, text):
        self.start_button.text.content = text

    # -- output ------------------------------------------------------------
    def update(self, lines):
        self.body.content = "\n".join(lines)

    def close(self):
        for stream in (self._start_clicked, self._stop_clicked):
            try:
                stream.remove()
            except Exception:                       # noqa: BLE001
                pass
        try:
            self.panel.remove()
        except Exception:                           # noqa: BLE001
            pass
