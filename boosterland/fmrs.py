"""Boostback from FMRS's separation save, then back to the main mission.

FMRS writes ``FMRS/FMRS_save_<n>.sfs`` at each staging that drops a vessel:
both stages, a moment after separation.  Loading it by hand, switching to
the booster and pressing START costs seconds of free fall, and every one of
them is boostback delta-v.  This does the whole chain itself:

  save the live game (the main mission) as ``krpclab_main`` -> read the
  FMRS saves on disk, first to last, and take the first holding a booster
  (the vessel in flight with engines closest to the save's active vessel,
  within NEAR_M; a stack split on the pad is a drop too, so FMRS_save_0 may
  have none) -> write a copy with the booster as its active vessel
  (``FMRS/krpclab_boostback``) -> load it, pause as soon as the game
  answers, build boosterland's ``Autoland``, unpause and fly it from the
  first tick.  No vessel switch: the game loads with the booster active.

When the flight ends -- touched down and settled, destroyed, or TERMINATE
-- the panel offers JUMP BACK TO MAIN MISSION.  That saves the booster's
game, takes the booster's VESSEL block out of it and puts it into
``krpclab_main`` in place of whatever that save held under the booster's
persistentId (nothing at all if the booster was destroyed), and loads the
result: the main mission resumes with the booster where it came down.

Both games are edited as text on disk (``common.sfs``), so this needs the
game's save folder on this machine: it is found by the file the first
``save`` writes, or given with ``--save-dir``.  FMRS's own bookkeeping
(``save.txt``) is not touched.

    ./run.sh --pilot fmrs                 # or the launcher's button
    python -m boosterland.fmrs --save FMRS/FMRS_save_1   # only this one
"""

import argparse
import glob
import os
import sys
import time
import uuid

import krpc

from common import paths, vec
from common.logbook import Logbook
from common.pacing import sleeper
from common.sfs import FLYING, Save
from boosterland.autoland import Autoland
from boosterland.config import Config, apply_overrides
from boosterland.gui import ControlPanel

SAVES = ["FMRS/FMRS_save_%d" % n for n in range(10)]
MAIN = "krpclab_main"                 # the main mission, in the save folder
BOOSTBACK = "FMRS/krpclab_boostback"  # the FMRS save with the booster active
LANDED = "krpclab_booster"            # the booster's game at the end
CONNECT_TIMEOUT_S = 90.0     # wall time for the scene to come back
SAVE_TIMEOUT_S = 10.0        # wall time for a save to appear on disk
POLL_S = 0.1
NEAR_M = 5000.0              # the booster is within this of the active vessel
BODY_RADIUS_M = 600000.0     # Kerbin; only scales a distance between neighbours
JUMP_LABEL = "JUMP BACK TO MAIN MISSION"


def connect(args, name):
    return krpc.connect(name=name, address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)


# -- the save folder --------------------------------------------------------
def save_roots():
    """Every game's save folder this machine might be running: the live
    install and the farm's instances."""
    homes = [os.path.expanduser("~/Kerbal Space Program")]
    homes += sorted(glob.glob(os.path.join(paths.INSTANCES, "ksp*")))
    return [d for home in homes
            for d in glob.glob(os.path.join(home, "saves", "*"))
            if os.path.isdir(d)]


def save_as(sc, name, save_dir=None):
    """Save the live game as ``name`` and return the game's save folder.

    kRPC does not say where the game is, so without ``save_dir`` the save
    is first written under a name nobody else will use and looked for in
    every save folder on the machine; then renamed."""
    if save_dir is not None:
        sc.save(name)
        path = os.path.join(save_dir, name + ".sfs")
        deadline = time.time() + SAVE_TIMEOUT_S
        while not os.path.exists(path):
            if time.time() > deadline:
                raise RuntimeError("save %r did not appear in %s"
                                   % (name, save_dir))
            time.sleep(POLL_S)
        return save_dir
    probe = "%s_%s" % (name, uuid.uuid4().hex[:12])
    sc.save(probe)
    deadline = time.time() + SAVE_TIMEOUT_S
    while True:
        found = [d for d in save_roots()
                 if os.path.exists(os.path.join(d, probe + ".sfs"))]
        if found:
            break
        if time.time() > deadline:
            raise RuntimeError("cannot find the game's save folder (no %s.sfs"
                               " under %s); pass --save-dir"
                               % (probe, ", ".join(save_roots()) or "none"))
        time.sleep(POLL_S)
    save_dir = found[0]
    for ext in (".sfs", ".loadmeta"):
        src = os.path.join(save_dir, probe + ext)
        if os.path.exists(src):
            os.replace(src, os.path.join(save_dir, name + ext))
    return save_dir


# -- picking the booster ------------------------------------------------------
def find_booster(save):
    """Index of the vessel in flight with engines closest to the save's
    active vessel and within NEAR_M of it, or None."""
    vessels = save.vessels()
    main = vessels[save.active]
    here = main.position(BODY_RADIUS_M)
    if here is None:
        return None
    best = None
    for index, v in enumerate(vessels):
        if index == save.active or v.sit not in FLYING or not v.has_engines:
            continue
        there = v.position(BODY_RADIUS_M)
        if there is None:
            continue
        distance = vec.norm(vec.sub(there, here))
        if distance <= NEAR_M and (best is None or distance < best[0]):
            best = (distance, index)
    return None if best is None else best[1]


def prepare_boostback(save_dir, names, log, main_id):
    """Write the first FMRS save of this launch holding a booster, with the
    booster made its active vessel, as BOOSTBACK.  Returns (source name,
    main vessel's name, booster Vessel).

    A save belongs to this launch when its active vessel is the live main
    mission's (``main_id``, its persistentId): FMRS leaves the last launch's
    saves on disk when it records no separation (switched off, or not
    armed), and one of those would otherwise be flown as this one's."""
    tried = []
    for name in names:
        path = os.path.join(save_dir, name + ".sfs")
        if not os.path.exists(path):
            break                       # numbered in order: no more saves
        save = Save.read(path)
        main = save.vessels()[save.active]
        main_name = main.name
        if main.persistent_id != main_id:
            log.event(0.0, "fmrs: %s is another launch's (%r, not the live "
                      "main mission): skipped" % (name, main_name))
            tried.append(name + " (another launch)")
            continue
        index = find_booster(save)
        if index is None:
            log.event(0.0, "fmrs: %s has no booster in flight near %r"
                      % (name, main_name))
            tried.append(name)
            continue
        save.active = index
        save.write(os.path.join(save_dir, BOOSTBACK + ".sfs"))
        return name, main_name, save.vessels()[index]
    raise RuntimeError("no FMRS save of this launch with a booster in "
                       "flight (tried %s); is FMRS enabled and armed?"
                       % ", ".join(tried or ["none"]))


def merge_booster(save_dir, booster_id, log):
    """Put the booster's state from LANDED into MAIN, replacing whatever
    MAIN held under its persistentId; a booster gone from LANDED (destroyed)
    is removed from MAIN.  Rewrites MAIN in place."""
    landed = [v for v in Save.read(os.path.join(save_dir, LANDED + ".sfs"))
              .vessels() if v.persistent_id == booster_id]
    path = os.path.join(save_dir, MAIN + ".sfs")
    main = Save.read(path)
    removed = main.remove_vessel(booster_id)
    if landed:
        main.add_vessel(landed[0])
        what = "%s %r at lat %s lon %s" % (
            landed[0].sit, landed[0].name, landed[0].fields.get("lat"),
            landed[0].fields.get("lon"))
    else:
        what = "booster gone (destroyed): left out"
    main.write(path)
    log.event(0.0, "fmrs: main mission: replaced %d copy(ies) of the booster;"
              " %s" % (removed, what))


# -- the game ---------------------------------------------------------------
def load_paused(args, save):
    """Load the save and return a fresh connection with the game paused on
    the loaded flight.  `load` invalidates the connection's references, so
    this reconnects rather than reusing anything across it."""
    conn = connect(args, "fmrs-loader")
    try:
        conn.space_center.load(save)
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


def offer_jump_back(args, save_dir, booster_id, reason, log):
    """Show JUMP BACK TO MAIN MISSION until it or TERMINATE is pressed;
    on JUMP, merge the booster into the main mission and load it."""
    conn = connect(args, args.name)
    gui = ControlPanel(conn, title="BOOSTERLAND", start_label=JUMP_LABEL)
    try:
        gui.update(["booster: %s" % reason, "",
                    "JUMP BACK loads the main mission",
                    "with the booster where it is now."])
        while True:
            if gui.terminate_pressed():
                log.event(0.0, "fmrs: stayed with the booster")
                return
            if gui.start_pressed():
                break
            time.sleep(POLL_S)
        gui.update(["saving the booster ..."])
        save_as(conn.space_center, LANDED, save_dir)
        merge_booster(save_dir, booster_id, log)
    finally:
        gui.close()
    try:
        conn.space_center.load(MAIN)
    finally:
        try:
            conn.close()
        except Exception:                           # noqa: BLE001
            pass
    log.event(0.0, "fmrs: loaded the main mission (%s)" % MAIN)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="fmrs", description="Load FMRS's separation save, fly the "
        "dropped booster home with boosterland, then offer to jump back to "
        "the main mission with the booster where it came down.")
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=50000)
    parser.add_argument("--stream-port", type=int, default=50001)
    parser.add_argument("--name", default="boosterland-fmrs")
    parser.add_argument("--save", default=None,
                        help="save to fly, relative to the save folder "
                        "(default: the first FMRS_save_<n> with a booster "
                        "in flight)")
    parser.add_argument("--save-dir", default=None,
                        help="the game's save folder (default: found by "
                        "saving the main mission and looking for it)")
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
    save_dir = None
    booster_id = None
    try:
        t0 = time.time()
        conn = connect(args, "fmrs-saver")
        save_dir = save_as(conn.space_center, MAIN, args.save_dir)
        conn.close()
        conn = None
        log.event(0.0, "fmrs: main mission saved as %s in %s"
                  % (MAIN, save_dir))
        main_save = Save.read(os.path.join(save_dir, MAIN + ".sfs"))
        main_id = main_save.vessels()[main_save.active].persistent_id
        source, main_name, booster = prepare_boostback(
            save_dir, [args.save] if args.save else SAVES, log, main_id)
        booster_id = booster.persistent_id
        if main_id == booster_id:
            raise RuntimeError("the live game is flying the booster, so it "
                               "is not the main mission: switch back first")
        conn = load_paused(args, BOOSTBACK)
        sc = conn.space_center
        vessel = sc.active_vessel
        log.event(sc.ut, "fmrs: loaded %s with %r active (%.1f s wall), main "
                  "%r, %.1f t" % (source, vessel.name, time.time() - t0,
                                  main_name, vessel.mass / 1e3))
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
                for e in vessel.parts.engines:
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
    try:
        if run is not None:
            run.shutdown(reason)
            run = None
            if not reason.startswith("interrupted"):
                offer_jump_back(args, save_dir, booster_id, reason, log)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                        # noqa: BLE001
        log.event(0.0, "FATAL jump back: %r" % exc)
        status = 1
    finally:
        if run is not None:
            run.shutdown(reason)
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
