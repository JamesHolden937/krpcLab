#!/usr/bin/env python3
"""Screenshots of a touchdown, taken by the game through kRPC.

``SpaceCenter.screenshot`` (flight scene only) writes the frame KSP is
rendering, HUD included -- the altimeter and the speed in the frame are a
check on the log.  Run the instance at a resolution worth looking at
(ksp6: ``SCREEN_RESOLUTION_*`` in its settings.cfg, which beats ``RES``,
and ``RES=960x540 ./start.sh 6``) and the flight at a low time scale.

(F1 sent over XTEST into the instance's nested Xwayland does not reach the
game under Wine; that was tried first.)

``--watch`` is a read-only kRPC client beside the autopilot: it waits for the
active vessel to come down through ``--from-m`` above the terrain, swings the
camera low and to the side (``--cam-pitch``, ``--cam-heading`` relative to
the default, ``--cam-distance``), shoots every ``--every`` wall seconds until
``--after-s`` game seconds after the vessel lands or loses parts for good,
and writes ``logs/shots/LOG<n>/NNN_t<ut>.png``, <n> being the newest log when
the burst began.  The game writes the files itself, so the path is given in
its own (Wine) form: the drive letter maps to the directory above the repo.

    testInstances/shoot.py 6               # one screenshot now
    testInstances/shoot.py 6 --watch       # a burst through the next touchdown
"""
import argparse
import glob
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def wine_path(path, drive="X:"):
    rel = os.path.relpath(os.path.abspath(path), os.path.dirname(ROOT))
    return drive + "\\" + rel.replace("/", "\\")


def newest_log():
    logs = glob.glob(os.path.join(ROOT, "logs", "LOG[0-9]*"))
    return max(logs, key=lambda p: int(re.sub(r"\D", "", os.path.basename(p))),
               default=None)


def connect(n):
    import krpc
    base = os.path.join(HERE, "ksp%s" % n)
    return krpc.connect(
        name="shoot",
        rpc_port=int(open(os.path.join(base, ".rpc_port")).read()),
        stream_port=int(open(os.path.join(base, ".stream_port")).read()))


def watch(conn, args):
    sc = conn.space_center
    while True:                       # a vessel low over the ground
        try:
            v = sc.active_vessel
            if (v.flight().surface_altitude < args.from_m
                    and "flying" in str(v.situation).lower()):
                break
        except Exception:
            pass
        time.sleep(0.3)
    log = newest_log()
    dest = os.path.join(ROOT, "logs", "shots",
                        os.path.basename(log) if log else "unknown")
    os.makedirs(dest, exist_ok=True)
    try:
        cam = sc.camera
        cam.pitch = args.cam_pitch
        cam.heading = cam.heading + args.cam_heading
        cam.distance = args.cam_distance
    except Exception:
        pass
    landed_ut, i = None, 0
    t_end = time.time() + args.max_wall_s
    while time.time() < t_end:
        try:
            ut = sc.ut
            sc.screenshot(wine_path(os.path.join(
                dest, "%03d_t%.2f.png" % (i, ut))), 1)
            i += 1
            sit = str(sc.active_vessel.situation).lower()
        except Exception:
            break
        if landed_ut is None and ("landed" in sit or "splashed" in sit):
            landed_ut = ut
        if landed_ut is not None and ut - landed_ut > args.after_s:
            break
        time.sleep(args.every)
    print("%d screenshots -> %s" % (i, dest))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("instance")
    p.add_argument("--watch", action="store_true")
    p.add_argument("--from-m", type=float, default=40.0)
    p.add_argument("--every", type=float, default=0.1)
    p.add_argument("--after-s", type=float, default=5.0)
    p.add_argument("--max-wall-s", type=float, default=180.0)
    p.add_argument("--cam-pitch", type=float, default=2.0)
    p.add_argument("--cam-heading", type=float, default=60.0)
    p.add_argument("--cam-distance", type=float, default=40.0)
    p.add_argument("-o", "--out", default=None, help="one shot to this file")
    args = p.parse_args()
    conn = connect(args.instance)
    if args.watch:
        watch(conn, args)
    else:
        out = args.out or os.path.join(ROOT, "logs", "shots", "now.png")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        conn.space_center.screenshot(wine_path(out), 1)
        print(out)


if __name__ == "__main__":
    main()
