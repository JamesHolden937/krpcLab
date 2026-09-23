#!/usr/bin/env python3
"""Make alternate quicksaves by applying a delta-v at the separation state.

TEMPORARY TEST HARNESS -- not part of the flight software.

`quickfly.py --delay` can only walk the booster *along* the trajectory it is
already on, so a number calibrated against one quicksave and confirmed with a
delay is still one entry state.  This writes genuinely different ones.

It edits only the vessel's ``ORBIT`` block, and it edits it by applying a
velocity change at the position the vessel already occupies.  That is the
point: `lat`, `lon`, `alt`, `hgt` and `nrm` in the save all describe the
*position*, and leaving the position alone leaves every one of them correct.
A save whose orbit and whose lat/lon disagree is a save KSP places somewhere
neither of them meant.

The delta-v is given in the orbital frame -- prograde, normal, radial-out --
because that is the frame the perturbation means something in: prograde is
"separated hotter or colder", radial is "steeper or shallower", normal is
"off the KSC plane", which is the one that puts a cross-track error in front
of the guidance.

    ./savegen.py --list
    ./savegen.py -o hot   --prograde  60
    ./savegen.py -o cold  --prograde -60
    ./savegen.py -o north --normal    40
    ./savegen.py -o steep --radial   -40

The element/state conversion is done in whatever frame KSP's elements are
expressed in and converted straight back, so it never has to know kRPC's or
KSP's axis handedness -- the frame cancels.
"""

import argparse
import math
import os
import re
import shutil
import sys

SAVE_DIR = os.path.expanduser("~/Kerbal Space Program/saves/default")
VESSEL_NAME = "Untitled Space Craft"
MU_KERBIN = 3.5316000e12

ELEMENTS = ("SMA", "ECC", "INC", "LPE", "LAN", "MNA", "EPH")


# -- element <-> state -----------------------------------------------------
def elements_to_state(el, mu):
    """(r, v) from Keplerian elements.  Angles in degrees except MNA."""
    a, e = el["SMA"], el["ECC"]
    inc, lpe, lan = (math.radians(el[k]) for k in ("INC", "LPE", "LAN"))
    m = el["MNA"]

    if e < 1.0:
        ea = m
        for _ in range(80):                 # Newton on Kepler's equation
            f = ea - e * math.sin(ea) - m
            ea -= f / (1.0 - e * math.cos(ea))
        nu = 2.0 * math.atan2(math.sqrt(1.0 + e) * math.sin(ea / 2.0),
                              math.sqrt(1.0 - e) * math.cos(ea / 2.0))
        r = a * (1.0 - e * math.cos(ea))
    else:
        ha = m
        for _ in range(200):
            f = e * math.sinh(ha) - ha - m
            ha -= f / (e * math.cosh(ha) - 1.0)
        nu = 2.0 * math.atan2(math.sqrt(e + 1.0) * math.tanh(ha / 2.0),
                              math.sqrt(e - 1.0))
        r = a * (1.0 - e * math.cosh(ha))

    p = a * (1.0 - e * e)
    # Perifocal frame, then rotate out by LPE, INC, LAN.
    rp = (r * math.cos(nu), r * math.sin(nu), 0.0)
    k = math.sqrt(mu / p)
    vp = (-k * math.sin(nu), k * (e + math.cos(nu)), 0.0)
    return _perifocal_to_frame(rp, lpe, inc, lan), \
           _perifocal_to_frame(vp, lpe, inc, lan)


def _perifocal_to_frame(x, lpe, inc, lan):
    cw, sw = math.cos(lpe), math.sin(lpe)
    ci, si = math.cos(inc), math.sin(inc)
    co, so = math.cos(lan), math.sin(lan)
    # R = Rz(lan) Rx(inc) Rz(lpe)
    a = (co * cw - so * sw * ci, -co * sw - so * cw * ci,  so * si)
    b = (so * cw + co * sw * ci, -so * sw + co * cw * ci, -co * si)
    c = (sw * si,                 cw * si,                 ci)
    return tuple(sum(row[j] * x[j] for j in range(3)) for row in (a, b, c))


def state_to_elements(r, v, mu, eph):
    rn, vn = _norm(r), _norm(v)
    h = _cross(r, v)
    hn = _norm(h)
    n = (-h[1], h[0], 0.0)                  # node vector, k x h
    nn = _norm(n)

    evec = _sub(_scale(r, vn * vn - mu / rn), _scale(v, _dot(r, v)))
    evec = _scale(evec, 1.0 / mu)
    e = _norm(evec)
    a = 1.0 / (2.0 / rn - vn * vn / mu)

    inc = math.acos(max(-1.0, min(1.0, h[2] / hn)))
    if nn < 1e-9:                           # equatorial: node is undefined
        lan = 0.0
        lpe = math.atan2(evec[1], evec[0])
        if h[2] < 0.0:
            lpe = -lpe
    else:
        lan = math.acos(max(-1.0, min(1.0, n[0] / nn)))
        if n[1] < 0.0:
            lan = 2.0 * math.pi - lan
        lpe = math.acos(max(-1.0, min(1.0, _dot(n, evec) / (nn * e))))
        if evec[2] < 0.0:
            lpe = 2.0 * math.pi - lpe

    nu = math.acos(max(-1.0, min(1.0, _dot(evec, r) / (e * rn))))
    if _dot(r, v) < 0.0:
        nu = 2.0 * math.pi - nu

    if e < 1.0:
        ea = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(nu / 2.0),
                              math.sqrt(1.0 + e) * math.cos(nu / 2.0))
        m = ea - e * math.sin(ea)
    else:
        ha = 2.0 * math.atanh(math.sqrt((e - 1.0) / (e + 1.0))
                              * math.tan(nu / 2.0))
        m = e * math.sinh(ha) - ha

    return {"SMA": a, "ECC": e, "INC": math.degrees(inc),
            "LPE": math.degrees(lpe) % 360.0, "LAN": math.degrees(lan) % 360.0,
            "MNA": m, "EPH": eph}


def _dot(a, b): return sum(x * y for x, y in zip(a, b))
def _norm(a): return math.sqrt(_dot(a, a))
def _sub(a, b): return tuple(x - y for x, y in zip(a, b))
def _add(a, b): return tuple(x + y for x, y in zip(a, b))
def _scale(a, k): return tuple(x * k for x in a)
def _unit(a):
    n = _norm(a)
    return _scale(a, 1.0 / n) if n > 0 else a
def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


# -- the save file ---------------------------------------------------------
def top_level_vessels(lines):
    """``[(index, name, first_line), ...]`` for the FLIGHTSTATE's VESSEL blocks.

    Indented exactly two tabs: a VESSEL nested deeper is a scenario's record of
    one, not the vessel itself, and the file is full of both.
    """
    out, index, start = [], -1, None
    for i, line in enumerate(lines):
        if line.rstrip("\n") == "\t\tVESSEL":
            index += 1
            start = i
        elif start is not None and line.startswith("\t\t\tname = "):
            out.append((index, line.strip()[len("name = "):], start))
            start = None
    return out


def find_orbit_block(lines, vessel_name=None):
    """Line numbers of the ORBIT entries of the vessel that will be flown.

    **Which vessel matters and it is not the one with the obvious name.**  The
    save holds a dozen craft -- every asteroid is a VESSEL, and staging has
    left three pieces of debris and an upper stage behind.  The booster the
    autoland flies is whichever one ``activeVessel`` points at, and here that
    is "Untitled Space Craft Probe" while a *different* craft is called plain
    "Untitled Space Craft".  Editing by name first silently rewrote the upper
    stage's orbit: five "different" entry states flew identical flights
    (LOG1-10 all separated at alt 22240, spd 818.1), which looks exactly like
    a calibration that generalises and is not one.

    So the default is ``activeVessel``, which is the game's own answer to the
    question.  A name still works if one is given.
    """
    vessels = top_level_vessels(lines)
    if vessel_name is None:
        active = None
        for line in lines:
            m = re.match(r"\s*activeVessel = (\d+)\s*$", line)
            if m:
                active = int(m.group(1))
                break
        if active is None:
            raise SystemExit("no activeVessel in the save; pass --vessel")
        match = [v for v in vessels if v[0] == active]
        if not match:
            raise SystemExit("activeVessel = %d but only %d vessels"
                             % (active, len(vessels)))
    else:
        match = [v for v in vessels if v[1] == vessel_name]
        if not match:
            raise SystemExit("no vessel named %r" % vessel_name)
    _, name, start = match[0]
    for j in range(start, len(lines)):
        if lines[j].strip() == "ORBIT":
            end = next(k for k in range(j, len(lines)) if lines[k].strip() == "}")
            return j, end, name
    raise SystemExit("no ORBIT for vessel %r" % name)


def read_elements(lines, lo, hi):
    el = {}
    for i in range(lo, hi):
        m = re.match(r"\s*(\w+) = (\S+)\s*$", lines[i])
        if m and m.group(1) in ELEMENTS:
            el[m.group(1)] = float(m.group(2))
    missing = set(ELEMENTS) - set(el)
    if missing:
        raise SystemExit("ORBIT is missing %s" % ", ".join(sorted(missing)))
    return el


def write_elements(lines, lo, hi, el):
    for i in range(lo, hi):
        m = re.match(r"(\s*)(\w+) = (\S+)\s*$", lines[i])
        if m and m.group(2) in ELEMENTS:
            lines[i] = "%s%s = %.17g\n" % (m.group(1), m.group(2),
                                           el[m.group(2)])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--save-dir", default=SAVE_DIR)
    p.add_argument("--source", default="quicksave")
    p.add_argument("-o", "--out", help="name of the save to write")
    p.add_argument("--vessel", default=None,
                   help="by name; the default is whichever vessel "
                        "the save's activeVessel points at")
    p.add_argument("--mu", type=float, default=MU_KERBIN)
    p.add_argument("--prograde", type=float, default=0.0, metavar="M_S")
    p.add_argument("--normal", type=float, default=0.0, metavar="M_S")
    p.add_argument("--radial", type=float, default=0.0, metavar="M_S")
    p.add_argument("--list", action="store_true",
                   help="print the source state and exit")
    args = p.parse_args(sys.argv[1:] if argv is None else argv)

    src = os.path.join(args.save_dir, args.source + ".sfs")
    with open(src) as fh:
        lines = fh.readlines()
    lo, hi, name = find_orbit_block(lines, args.vessel)
    el = read_elements(lines, lo, hi)
    r, v = elements_to_state(el, args.mu)

    if args.list or not args.out:
        print("%s [%s]: %s" % (args.source, name,
                          " ".join("%s=%.6g" % (k, el[k]) for k in ELEMENTS)))
        print("  r = %.1f  (alt %.1f m)  |v| = %.2f m/s"
              % (_norm(r), _norm(r) - 600000.0, _norm(v)))
        return 0

    prograde = _unit(v)
    normal = _unit(_cross(r, v))
    radial = _unit(_cross(normal, prograde))    # completes the triad
    dv = _add(_add(_scale(prograde, args.prograde),
                   _scale(normal, args.normal)),
              _scale(radial, args.radial))
    v2 = _add(v, dv)
    el2 = state_to_elements(r, v2, args.mu, el["EPH"])

    # The conversion is only trustworthy if it round-trips: this rewrites a
    # real save, and an element set that puts the vessel somewhere else is a
    # flight that measures nothing.  Check the position it implies is the one
    # we started from, since that is what every other field in the file
    # describes.
    r2, v2b = elements_to_state(el2, args.mu)
    if _norm(_sub(r2, r)) > 1.0 or _norm(_sub(v2b, v2)) > 0.01:
        raise SystemExit("element round-trip failed: dr=%.3f m dv=%.4f m/s"
                         % (_norm(_sub(r2, r)), _norm(_sub(v2b, v2))))

    write_elements(lines, lo, hi, el2)
    dst = os.path.join(args.save_dir, args.out + ".sfs")
    with open(dst, "w") as fh:
        fh.writelines(lines)
    meta = os.path.join(args.save_dir, args.source + ".loadmeta")
    if os.path.exists(meta):
        shutil.copyfile(meta, os.path.join(args.save_dir,
                                           args.out + ".loadmeta"))
    print("%s -> %s  dv=(pro %+.1f, nrm %+.1f, rad %+.1f)  "
          "|v| %.2f -> %.2f m/s  alt %.0f m (unchanged)"
          % (args.source, args.out, args.prograde, args.normal, args.radial,
             _norm(v), _norm(v2), _norm(r) - 600000.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
