"""Boostback from FMRS's separation save, with no time lost to clicking.

FMRS writes ``FMRS/FMRS_save_<n>.sfs`` at each staging that drops a vessel:
both stages, a moment after separation.  Loading it by hand and then
switching to the booster and pressing START costs seconds of free fall, and
every one of them is boostback delta-v.  This does the whole chain itself:

  load the save -> pause the game as soon as it answers -> pick the booster
  (the heaviest other vessel in flight with engines, near the active one;
  none -> try the next save, since a stack split on the pad is a drop as
  well and FMRS_save_0 may be that) -> make it
  active -> build boosterland's ``Autoland`` while still paused -> unpause
  and fly it from the first tick.

The upper stage is left where the save had it.  FMRS's own bookkeeping
(``save.txt``) is not touched, so FMRS may not know the flight it saved was
resumed this way.

    ./run.sh --pilot fmrs                 # or the launcher's button
    python -m boosterland.fmrs --save FMRS/FMRS_save_1   # only this one
"""

import argparse
import sys
import time

import krpc

from common import vec
from common.logbook import Logbook
from common.pacing import sleeper
from boosterland.autoland import Autoland
from boosterland.config import Config, apply_overrides

SAVES = ["FMRS/FMRS_save_%d" % n for n in range(10)]
CONNECT_TIMEOUT_S = 90.0     # wall time for the scene to come back
POLL_S = 0.1
SWITCH_TIMEOUT_S = 10.0
NEAR_M = 5000.0              # the booster is within this of the active vessel


def connect(args, name):
    return krpc.connect(name=name, address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)


def load_paused(args, save):
    """Load the save and return a fresh connection with the game paused on
    the loaded flight.  `load` invalidates the connection's references, so
    this reconnects rather than reusing anything across it."""
    conn = connect(args, "fmrs-loader")
    try:
        try:
            conn.space_center.load(save)
        except Exception as exc:                    # noqa: BLE001
            raise LookupError("cannot load %r: %r" % (save, exc))
        try:
            conn.krpc.paused = True
        except Exception:                           # noqa: BLE001
            pass                    # scene still building; paused below
    finally:
        try:
            conn.close()
        except Exception:                           # noqa: BLE001
            pass
    deadline = time.time() + CONNECT_TIMEOUT_S
    last = None
    while time.time() < deadline:
        try:
            conn = connect(args, args.name)
            conn.krpc.paused = True
            vessel = conn.space_center.active_vessel
            vessel.name                             # a vessel that answers
            return conn
        except Exception as exc:                    # noqa: BLE001
            last = exc
            try:
                conn.close()
            except Exception:                       # noqa: BLE001
                pass
            time.sleep(POLL_S)
    raise RuntimeError("save %r never came back with a vessel: %r"
                       % (save, last))


def find_booster(sc):
    """The heaviest vessel in flight with engines, other than the active
    one, within NEAR_M of it; None if there is none."""
    main = sc.active_vessel
    frame = main.orbit.body.reference_frame
    here = main.position(frame)
    best = None
    flying = (sc.VesselSituation.flying, sc.VesselSituation.sub_orbital)
    for v in sc.vessels:
        if v == main:
            continue
        try:
            if vec.norm(vec.sub(v.position(frame), here)) > NEAR_M:
                continue
            if v.situation not in flying or not v.parts.engines:
                continue
            mass = v.mass
        except Exception:                           # noqa: BLE001
            continue
        if best is None or mass > best[0]:
            best = (mass, v)
    return None if best is None else best[1]


def switch_to(sc, vessel):
    sc.active_vessel = vessel
    deadline = time.time() + SWITCH_TIMEOUT_S
    while time.time() < deadline:
        try:
            if sc.active_vessel == vessel:
                return
        except Exception:                           # noqa: BLE001
            pass
        time.sleep(0.05)
    raise RuntimeError("switch to the booster timed out")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="fmrs", description="Load FMRS's separation save and fly "
        "the dropped booster home with boosterland.")
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=50000)
    parser.add_argument("--stream-port", type=int, default=50001)
    parser.add_argument("--name", default="boosterland-fmrs")
    parser.add_argument("--save", default=None,
                        help="save to load, relative to the save folder "
                        "(default: the first FMRS_save_<n> with a booster "
                        "in flight)")
    parser.add_argument("--set", action="append", default=[], metavar="K=V",
                        help="override a boosterland Config field")
    parser.add_argument("--autostart", action="store_true",
                        help="accepted for the launcher; this always starts")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    run = None
    conn = None
    status = 0
    reason = "shutdown"
    try:
        t0 = time.time()
        booster = None
        tried = []
        for save in [args.save] if args.save else SAVES:
            try:
                conn = load_paused(args, save)
            except LookupError as exc:
                if args.save:
                    raise
                log.event(0.0, "fmrs: %s" % exc)
                break                   # numbered in order: no more saves
            sc = conn.space_center
            main_name = sc.active_vessel.name
            booster = find_booster(sc)
            if booster is not None:
                break
            log.event(sc.ut, "fmrs: %s has no booster in flight near %r"
                      % (save, main_name))
            tried.append(save)
            conn.close()
            conn = None
        if booster is None:
            raise RuntimeError("no FMRS save with a booster in flight "
                               "(tried %s)" % ", ".join(tried or ["none"]))
        log.event(sc.ut, "fmrs: loaded %s (%.1f s wall), main %r, booster "
                  "%r %.1f t" % (save, time.time() - t0, main_name,
                                 booster.name, booster.mass / 1e3))
        switch_to(sc, booster)
        conn.krpc.paused = True     # a vessel switch can unpause
        run = Autoland(conn, cfg, log)
        run.autostart = True
        run.gui.hide_start()
        conn.krpc.paused = False
        log.event(sc.ut, "fmrs: unpaused, boosterland flying")
        wait = sleeper(cfg, lambda: sc.ut, conn)
        first = True
        while not run.finished:
            snap = run.tick()
            if first and (run.cluster is None
                          or len(run.cluster.groups) <= 1):
                # ascent leaves its throttle-down thrust limits on the
                # booster; boosterland's cluster rewrites them only with
                # more than one engine group (as ascent.fly_booster).
                for e in booster.parts.engines:
                    e.thrust_limit = 1.0
            first = False
            wait(cfg.LOOP_SLEEP_S, snap.ut if snap is not None else sc.ut)
        reason = run.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
    except Exception as exc:                        # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        reason = "error: %r" % exc
        status = 1
        try:
            if conn is not None:
                conn.krpc.paused = False
        except Exception:                           # noqa: BLE001
            pass
    finally:
        if run is not None:
            run.shutdown(reason)
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
