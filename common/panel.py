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


def _text(conn, panel, content, size, position, rect, color):
    text = panel.add_text(content)
    text.rect_transform.size = rect
    text.rect_transform.position = position
    text.size = size
    text.line_spacing = 1.0
    text.color = color
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
