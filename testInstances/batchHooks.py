#!/usr/bin/env python3
"""batchHooks.py pre|post -- what krpcLab checks and asks for at a farm batch's
start and end.  Run by ~/.local/bin/farmlaunch through `.farmlaunch/pre` and
`.farmlaunch/post` (every batch script runs under farmlaunch, testInstances/
farmGuard.sh), with the lock held; the output lands in the batch's own output,
which is what the session reads when the batch ends.

The procedures that used to be written in CLAUDE.md as "before a batch ..." /
"after each batch ..." live here instead, so they arrive at the moment they
apply rather than depending on being remembered:

pre   saves on every instance match saves/ (syncSaves.sh check); every
      instance's GameData is the same game; no flight or sim left running;
      the farm was restarted since the last batch ended (swap grows round by
      round and biases results); the code the batch flies is committed.
      Warnings only: the batch still starts (exit 0).
post  swap after the batch; flights or sims left running; then what to do
      now (read, handoff, commit, clear) and, every AUDIT_EVERY batches, the
      whole-vehicle audit.
Stdlib only.  State: logs/.batchHooks.json (git-ignored).
"""
import glob
import hashlib
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)
STATE = os.path.join(ROOT, "logs", ".batchHooks.json")
AUDIT_EVERY = 5
PILOTS = ("boosterland", "spaceplane", "ascent", "airplane", "propPitch", "recon", "kspSim")
CODE_EXT = (".py", ".sh", ".cs", ".cfg", ".craft", ".sfs", ".json")


def say(msg):
    print("batchHooks: " + msg, flush=True)


def load():
    try:
        with open(STATE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save(st):
    os.makedirs(os.path.dirname(STATE), exist_ok=True)
    with open(STATE + ".tmp", "w") as f:
        json.dump(st, f)
    os.replace(STATE + ".tmp", STATE)


def pgrepf(pat):
    r = subprocess.run([os.path.expanduser("~/.local/bin/pgrepf"), "-a", pat],
                       capture_output=True, text=True)
    return [l for l in r.stdout.splitlines() if l.strip()]


def game_pids():
    return [int(l.split()[0]) for l in pgrepf(r"KSP_x64\.exe")
            if "krpclab" in l.lower() or "testinstances" in l.lower()]


def boot_time():
    with open("/proc/stat") as f:
        for l in f:
            if l.startswith("btime"):
                return int(l.split()[1])
    return 0


def started_at(pid):
    try:
        with open("/proc/%d/stat" % pid) as f:
            ticks = int(f.read().rsplit(")", 1)[1].split()[19])
    except (OSError, IndexError, ValueError):
        return None
    return boot_time() + ticks / os.sysconf("SC_CLK_TCK")


def swap_gb():
    used = 0
    try:
        with open("/proc/swaps") as f:
            for l in list(f)[1:]:
                used += int(l.split()[3])
    except OSError:
        return None
    return used / 1048576.0


def hhmm(t):
    return time.strftime("%H:%M", time.localtime(t))


def instances():
    return sorted(int(os.path.basename(d)[3:]) for d in glob.glob(os.path.join(HERE, "ksp[0-9]*"))
                  if os.path.basename(d)[3:].isdigit() and os.path.isdir(os.path.join(d, "GameData")))


def gamedata_sig(n):
    gd = os.path.join(HERE, "ksp%d" % n, "GameData")
    h = hashlib.md5()
    for d, subs, files in os.walk(gd):
        subs[:] = sorted(s for s in subs if s != "PluginData")
        for f in sorted(files):
            if f.endswith((".log", ".cache", ".ConfigSHA", ".TechTree", ".Physics")):
                continue
            p = os.path.join(d, f)
            try:
                h.update(("%s %d\n" % (os.path.relpath(p, gd), os.path.getsize(p))).encode())
            except OSError:
                pass
    return h.hexdigest()


def batch_tree():
    """The tree the batch flies: the git top level of the batch script's dir."""
    cmd = os.environ.get("FARMLAUNCH_CMD", "").split()
    d = os.path.dirname(os.path.realpath(cmd[0])) if cmd and os.sep in cmd[0] else os.getcwd()
    r = subprocess.run(["git", "-C", d, "rev-parse", "--show-toplevel"], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else ROOT


def pilot_of_cmd():
    cmd = os.environ.get("FARMLAUNCH_CMD", "")
    for p in PILOTS:
        if p + "/" in cmd or "pilot " + p in cmd:
            return p
    return "<pilot>"


def stray():
    return pgrepf(r"krpcLab\.py --pilot|quickglide\.py|quickfly\.py|stackfly\.py|kspSim[.]run")


def pre():
    st = load()
    warn = []
    now = time.time()
    # saves
    r = subprocess.run([os.path.join(HERE, "syncSaves.sh"), "check"], capture_output=True, text=True)
    if r.returncode:
        lines = (r.stdout + r.stderr).strip().splitlines()
        warn.append("SAVES DIFFER from saves/ on the instances (two experiments reported as one):\n    "
                    + "\n    ".join(lines[:12]) + ("\n    ..." if len(lines) > 12 else "")
                    + "\n  fix: testInstances/syncSaves.sh push (or pull the newer one into saves/ first)")
    # same game
    sigs = {}
    for n in [n for n in instances() if n < 6]:   # 0-5: the batch farm (ksp6 is pruned differently)
        sigs.setdefault(gamedata_sig(n), []).append(n)
    if len(sigs) > 1:
        groups = sorted(sigs.values(), key=len, reverse=True)
        warn.append("GAMEDATA DIFFERS between instances (not the same game): groups %s; "
                    "re-clone the odd ones (mkclone.sh), don't patch" % groups)
    # leftovers
    s = stray()
    if s:
        warn.append("FLIGHTS/SIMS ALREADY RUNNING before this batch started (a killed batch's "
                    "children?):\n    " + "\n    ".join(s[:8]))
    # restart since the last batch
    games = game_pids()
    last_end = st.get("last_end")
    if games and last_end:
        old = [p for p in games if (started_at(p) or now) < last_end]
        if old:
            warn.append("FARM NOT RESTARTED since the last batch ended (%s): %d game(s) older than "
                        "that; swap biases results round by round -- for a comparison, "
                        "cd testInstances && ./stop.sh && ./start.sh first" % (hhmm(last_end), len(old)))
    # uncommitted code in the tree the batch flies
    tree = batch_tree()
    r = subprocess.run(["git", "-C", tree, "status", "--porcelain", "--untracked-files=no"],
                       capture_output=True, text=True)
    dirty = [l[3:] for l in r.stdout.splitlines() if l[3:].endswith(CODE_EXT)]
    if dirty:
        warn.append("UNCOMMITTED CODE in %s (commit before a batch on new code, so it can be undone): %s"
                    % (tree, " ".join(dirty[:10])))
    sw = swap_gb()
    st.update(pre_swap=sw, pre_time=now)
    save(st)
    for w in warn:
        say("WARNING " + w)
    say("pre: %s; %d instance(s), %d game(s) up, swap %.1f GB"
        % ("%d warning(s) above" % len(warn) if warn else "ok (saves match, one game, nothing stray)",
           len(instances()), len(games), sw or 0))
    return 0


AUDIT = """AUDIT DUE (every %d batches; this was batch %d): stop and audit the whole vehicle and the whole
  flight, not the phase you are in, before the next batch on it:
  (1) missing controls (testInstances/actuators.py <instance> <save> lists them): list every actuator and setting the craft exposes through kRPC (control-surface
      axes, deploy angles, RCS, airbrakes, gimbals, drain valves, gear, brakes, SAS/throttle modes) and
      every one the autopilot never commands or leaves at whatever the craft file happened to set (the
      shuttle found five in one afternoon after ~2800 flights; `Drain Mode` was right only by inheritance).
  (2) breadth: glance at the other phases' summaries (oscsum.py, landsum.py, `loop rate`, touchdown
      survival) on more than one orbit/save and, when there is one, more than one airframe of the family.
      The largest error is rarely in the part you are looking at."""

NEXT = """NOW (batch over; token cost is calls x context, so a fresh context per batch):
  1. Read the result (above, where the pilot has a summary tool; else its summary tools), not raw logs.
  2. Rewrite docs/%s/handoff.md's section so a cold session can carry on: the result with its
     log/batch names, what it means, the exact next batch to fly and why, and anything the user said
     that no doc records (a standing instruction goes into CLAUDE.md or memory, not only the handoff).
     Update that pilot's row in HANDOFF.md.
  3. Commit.
  4. Next batch: launch it detached (farmlaunch from a session prints the steps), then clear self.
     No next batch: clear self (mcp__ccd_session_mgmt__clear_session "self") with a one-line summary.
     Skip the clear only while the user is mid-conversation (a question or decision open) or the
     context is still small (under ~100k tokens)."""


# Per pilot: the summary tool that reads a batch's logs (argv + log paths).
SUMMARY = {"boosterland": [os.path.join(ROOT, "boosterland", "tools", "logsum.py")]}
MAX_LINES = 80


def summary(t0):
    """The batch's result, so the session that reads this needs no further call to see it:
    the harness's rot file(s) written during the batch and, where the pilot has one, its
    one-line-per-flight summary of the logs the batch wrote."""
    rots = sorted(f for f in glob.glob(os.path.join(ROOT, "logs", "rot-*.txt"))
                  if os.path.getmtime(f) >= t0)
    for f in rots[-2:]:
        with open(f, errors="replace") as fh:
            lines = fh.read().splitlines()
        say("result %s (%d lines%s):" % (os.path.relpath(f, ROOT), len(lines),
            ", last %d shown" % MAX_LINES if len(lines) > MAX_LINES else ""))
        print("\n".join(lines[-MAX_LINES:]), flush=True)
    logs = sorted((f for f in glob.glob(os.path.join(ROOT, "logs", "LOG[0-9]*"))
                   if os.path.getmtime(f) >= t0),
                  key=lambda f: int(os.path.basename(f)[3:]) if os.path.basename(f)[3:].isdigit() else 0)
    tool = SUMMARY.get(pilot_of_cmd())
    if tool and logs:
        r = subprocess.run([sys.executable] + tool + logs, capture_output=True, text=True, timeout=120)
        out = r.stdout.strip().splitlines()
        say("%s over the batch's %d log(s) %s..%s:" % (os.path.basename(tool[0]), len(logs),
            os.path.basename(logs[0]), os.path.basename(logs[-1])))
        print("\n".join(out[-MAX_LINES:]) or (r.stderr.strip()[-500:] or "(no output)"), flush=True)
    elif logs:
        say("the batch wrote %d log(s): %s..%s" % (len(logs), os.path.basename(logs[0]),
                                                    os.path.basename(logs[-1])))


def post():
    st = load()
    now = time.time()
    rc = os.environ.get("FARMLAUNCH_RC", "?")
    sw = swap_gb()
    pre_sw = st.get("pre_swap")
    say("post: batch exit=%s; swap %.1f GB (%s at its start)"
        % (rc, sw or 0, "%.1f" % pre_sw if pre_sw is not None else "?"))
    if sw is not None and pre_sw is not None and (sw - pre_sw > 2 or sw > 12):
        say("WARNING swap grew %.1f -> %.1f GB during the batch: its late rounds may be biased; "
            "compare round by round, and restart the farm before the next comparison" % (pre_sw, sw))
    s = stray()
    if s:
        say("WARNING still running after the batch (stop them; a loaded sim keeps simulating):\n    "
            + "\n    ".join(s[:8]))
    n = st.get("count", 0) + 1
    st.update(count=n, last_end=now, last_rc=rc)
    save(st)
    summary(st.get("pre_time") or float(os.environ.get("FARMLAUNCH_START") or now))
    print(NEXT % pilot_of_cmd(), flush=True)
    if n % AUDIT_EVERY == 0:
        print(AUDIT % (AUDIT_EVERY, n), flush=True)
    return 0


if __name__ == "__main__":
    op = sys.argv[1] if len(sys.argv) > 1 else ""
    if op not in ("pre", "post"):
        sys.exit("usage: batchHooks.py pre|post")
    try:
        sys.exit(pre() if op == "pre" else post())
    except Exception as e:   # a broken check must never stop a batch
        say("%s check crashed: %r" % (op, e))
        sys.exit(0)
