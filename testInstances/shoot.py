#!/usr/bin/env python3
"""Screenshots of a touchdown, taken by KSP itself.

Each farm instance runs inside its own nested KWin with its own Xwayland
(kwinRun.sh), where KSP is the only client and always has focus.  So a key
sent over XTEST to that Xwayland reaches the game: F2 hides the UI and F1
(``TAKE_SCREENSHOT`` in settings.cfg) writes ``ksp<N>/Screenshots/*.png``.

``--watch`` is a read-only kRPC client beside the autopilot: it waits for the
active vessel to come down through ``--from-m`` above the terrain, shoots
every ``--every`` wall seconds until ``--after-s`` game seconds after it
lands (or is lost), then moves the new PNGs to ``logs/shots/LOG<n>/``,
<n> being the newest log when the burst began.  Run the instance at a
resolution worth looking at (``RES=960x540 ./kwinRun.sh 6``) and the flight
at a low time scale, or the burst samples the touchdown sparsely.

    testInstances/shoot.py 6               # one screenshot now
    testInstances/shoot.py 6 --watch       # a burst through the next touchdown
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def xdisplay(n):
    """The Xwayland display of instance n's nested KWin, found by process
    ancestry: the kwin_wayland with ``--socket ksp<n>-wl`` and its child."""
    ps = subprocess.run(["ps", "-eo", "pid,ppid,args"], capture_output=True,
                        text=True).stdout.splitlines()
    kwin = None
    for line in ps:
        pid, ppid, args = line.split(None, 2) if len(line.split(None, 2)) == 3 else (None, None, "")
        if args.startswith("kwin_wayland") and "--socket ksp%s-wl" % n in args:
            kwin = pid
    for line in ps:
        parts = line.split(None, 2)
        if len(parts) == 3 and parts[1] == kwin and "Xwayland" in parts[2]:
            m = re.search(r"Xwayland (:\d+)", parts[2])
            if m:
                return m.group(1)
    raise SystemExit("no nested Xwayland for ksp%s" % n)


class Keys:
    def __init__(self, disp):
        from Xlib import display, XK, X
        from Xlib.ext import xtest
        self.X, self.xtest = X, xtest
        self.d = display.Display(disp)
        self.code = {k: self.d.keysym_to_keycode(XK.string_to_keysym(k))
                     for k in ("F1", "F2")}

    def press(self, key):
        code = self.code[key]
        self.xtest.fake_input(self.d, self.X.KeyPress, code)
        self.d.sync()
        time.sleep(0.03)
        self.xtest.fake_input(self.d, self.X.KeyRelease, code)
        self.d.sync()


def newest_log():
    logs = glob.glob(os.path.join(ROOT, "logs", "LOG[0-9]*"))
    return max(logs, key=lambda p: int(re.sub(r"\D", "", os.path.basename(p))),
               default=None)


def watch(n, keys, args):
    import krpc
    base = os.path.join(HERE, "ksp%s" % n)
    conn = krpc.connect(name="shoot",
                        rpc_port=int(open(os.path.join(base, ".rpc_port")).read()),
                        stream_port=int(open(os.path.join(base, ".stream_port")).read()))
    sc = conn.space_center
    shots = os.path.join(base, "Screenshots")
    os.makedirs(shots, exist_ok=True)
    while True:                       # wait for a vessel low over the ground
        try:
            v = sc.active_vessel
            if v.flight().surface_altitude < args.from_m and \
                    "flying" in str(v.situation).lower():
                break
        except Exception:
            pass
        time.sleep(0.5)
    before = set(os.listdir(shots))
    log = newest_log()
    if args.hide_ui:
        keys.press("F2")
    landed_ut = None
    t_end = time.time() + args.max_wall_s
    while time.time() < t_end:
        keys.press("F1")
        time.sleep(args.every)
        try:
            ut = sc.ut
            sit = str(sc.active_vessel.situation).lower()
        except Exception:
            break
        if landed_ut is None and ("landed" in sit or "splashed" in sit):
            landed_ut = ut
        if landed_ut is not None and ut - landed_ut > args.after_s:
            break
    if args.hide_ui:
        keys.press("F2")
    time.sleep(1.0)                   # the last PNG is written a frame later
    new = sorted(set(os.listdir(shots)) - before,
                 key=lambda f: os.path.getmtime(os.path.join(shots, f)))
    dest = os.path.join(ROOT, "logs", "shots",
                        os.path.basename(log) if log else "unknown")
    os.makedirs(dest, exist_ok=True)
    for i, f in enumerate(new):
        shutil.move(os.path.join(shots, f),
                    os.path.join(dest, "%03d_%s" % (i, f)))
    print("%d screenshots -> %s" % (len(new), dest))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("instance")
    p.add_argument("--watch", action="store_true")
    p.add_argument("--from-m", type=float, default=40.0)
    p.add_argument("--every", type=float, default=0.2)
    p.add_argument("--after-s", type=float, default=5.0)
    p.add_argument("--max-wall-s", type=float, default=120.0)
    p.add_argument("--hide-ui", action="store_true")
    args = p.parse_args()
    keys = Keys(xdisplay(args.instance))
    if args.watch:
        watch(args.instance, keys, args)
    else:
        keys.press("F1")


if __name__ == "__main__":
    main()
