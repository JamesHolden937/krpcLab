"""Pure: the roll the vehicle sits at on the pad, read as if it were about
to pitch over towards a given heading.

On the pad the nose is vertical, so heading and roll are not defined on
their own (kRPC reads one stack as heading 270, roll -90).  Picture the
vehicle flying level towards ``heading`` with wings level -- nose along the
heading, right wing to its right, belly down -- and pitch it up to vertical
about the right wing: the belly now faces the heading.  The pad roll is the
angle from that attitude to the real one about the nose, positive right
wing down (kRPC's and MechJeb's sense).  Handing it to MechJeb as the
ascent's roll makes a launch towards ``heading`` pitch over without rolling;
any other heading rolls by the difference.

Vectors are (up, north, east): kRPC's vessel surface reference frame.
"""

import math


def pad_roll(right, heading_deg):
    """Roll in degrees, (-180, 180], of a vertical vehicle whose right wing
    points along ``right``, against a pitch-over towards ``heading_deg``."""
    h = math.radians(heading_deg)
    ref_bottom = (0.0, math.cos(h), math.sin(h))
    ref_right = (0.0, math.cos(h + math.pi / 2), math.sin(h + math.pi / 2))
    x = sum(a * b for a, b in zip(right, ref_right))
    y = sum(a * b for a, b in zip(right, ref_bottom))
    return math.degrees(math.atan2(y, x))
