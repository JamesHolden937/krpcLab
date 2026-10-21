"""The booster's telemetry block for the in-game panel
(``common.panel.ControlPanel``) and the log."""

from common import vec


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
