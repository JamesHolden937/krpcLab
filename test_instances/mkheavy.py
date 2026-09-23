#!/usr/bin/env python3
"""Build qs_plane_heavy: the same wing, carrying more weight.

The farm has exactly one airframe, so nothing in it can test whether the
derived numbers actually follow an aircraft.  Mass is the axis that can be
changed without a VAB: a stall speed goes as sqrt(weight), so a heavier
aeroplane needs a faster approach -- and ``STALL_SPEED_M_S`` is a constant
that cannot know.  ``airframe.stall`` reads the mass every flight.

MonoPropellant is the ballast because ``DRAIN`` dumps LiquidFuel and Oxidizer
before entry and leaves this, so it is still aboard at the landing, which is
the mass the landing constants are sized on.
"""
import re, shutil, sys

EXTRA_T = 1.40                  # target added landing mass
DENSITY = 0.004                 # t per unit of MonoPropellant

src, dst = sys.argv[1], sys.argv[2]
# **Binary, because the save is CRLF.**  Reading it in text mode
# converts every line ending to LF and hands KSP a file 14 kB
# smaller than the one it wrote -- a silent corruption of a farm
# save, which is the class of thing that makes a batch a lie.
raw = open(src, "rb").read()
text = raw.decode("utf-8", "surrogateescape")

units = EXTRA_T / DENSITY       # 350 units
block = re.compile(r"(name = MonoPropellant\s*\n(\s*)amount = )([\d.]+)"
                   r"(\s*\n\s*maxAmount = )([\d.]+)")
found = []
def bump(m):
    amount, mx = float(m.group(3)), float(m.group(5))
    share = units / 2.0         # split across the two tanks
    found.append((amount, amount + share))
    return "%s%.1f%s%.1f" % (m.group(1), amount + share,
                             m.group(4), mx + share)

out, n = block.subn(bump, text)
if n != 2:
    sys.exit("expected 2 MonoPropellant tanks, patched %d" % n)
open(dst, "wb").write(out.encode("utf-8", "surrogateescape"))
print("wrote %s" % dst)
for before, after in found:
    print("   MonoPropellant %.1f -> %.1f units" % (before, after))
print("   added %.2f t of landing mass (%.0f units)" % (EXTRA_T, units))
