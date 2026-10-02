"""The one in-game window every krpcLab panel draws in.

The launcher and each autopilot share its geometry, so pressing a launcher
button reads as the window switching from the menu to the flight's telemetry
rather than a second window opening: the launcher hides its panel and the
autopilot's appears in exactly the same place.

kRPC's UI service gives a panel no background colour, only KSP's translucent
window sprite.  ``window`` makes it opaque and black (an OLED theme) two
ways, either sufficient alone: several panels stacked under the content, so
the sprite's alpha compounds towards opaque, and a black text element of
full-block glyphs over them, which is true black wherever the game's font
has the glyph and draws nothing where it does not.
"""

WIDTH = 300
HEIGHT = 420
MARGIN = 30          # from the left edge of the screen

# OLED palette: true-black ground, dim greys, one accent per autopilot.
BLACK = (0.0, 0.0, 0.0)
TEXT = (0.85, 0.85, 0.85)
DIM = (0.5, 0.5, 0.5)

STACKED_PANELS = 6
FILL_GLYPH = u"█"
FILL_SIZE = 40


def window(conn):
    """An opaque black panel at the shared position; add content to it."""
    canvas = conn.ui.stock_canvas
    screen = canvas.rect_transform.size
    panel = canvas.add_panel()
    rect = panel.rect_transform
    rect.size = (WIDTH, HEIGHT)
    rect.position = (WIDTH / 2 + MARGIN - screen[0] / 2, 0)
    # Children draw in the order they are added, so the fill goes first and
    # everything the caller adds afterwards sits on top of it.
    for _ in range(STACKED_PANELS):
        layer = panel.add_panel()
        layer.rect_transform.size = (WIDTH, HEIGHT)
        layer.rect_transform.position = (0, 0)
    fill = panel.add_text("\n".join(
        [FILL_GLYPH * (WIDTH // 10)] * (HEIGHT // 20)))
    fill.rect_transform.size = (WIDTH - 8, HEIGHT - 8)
    fill.rect_transform.position = (0, 0)
    fill.size = FILL_SIZE
    fill.line_spacing = 0.8
    fill.color = BLACK
    fill.alignment = conn.ui.TextAnchor.middle_center
    return panel
