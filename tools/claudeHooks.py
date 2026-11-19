#!/usr/bin/env python3
"""claudeHooks.py EVENT -- krpcLab's Claude Code hooks (.claude/settings.json).

Each tells the session what to do at the moment a rule applies, instead of the
rule sitting in CLAUDE.md waiting to be remembered.  Reads the hook's JSON on
stdin, prints a hook JSON reply (or nothing), always exits 0.

  session   SessionStart: HANDOFF.md (the index) as context, and the farm's state
            (up since when, swap; restart before the first comparison).
  bash      PreToolUse(Bash): refuses the offline suite while a farm game runs
            (six instances + the suite = killed by memory pressure).
  edit      PreToolUse(Edit|Write|NotebookEdit): refuses code edits in the main
            tree while a farm batch flies it (the batch rebuilds the bundle from
            the tree: an edit mid-batch splits it into two experiments).
  notice    UserPromptSubmit / PostToolUse(Bash): a new .git-bundle-error.log
            (the post-commit bundle or background suite failed) is reported once;
            and the CONTEXT LIMIT: once the conversation's context (the last
            assistant turn's input + cache tokens, read from the transcript) is
            over CONTEXT_LIMIT, a hard order to write the handoff and clear,
            repeated every CONTEXT_REPEAT_S until it is done (the user,
            2026-11-18: a judgement call to skip the clear "while the context is
            small" was skipped at 205k).
"""
import json
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "testInstances"))
import batchHooks as bh  # noqa: E402

LOCK = os.path.join(os.environ.get("XDG_RUNTIME_DIR") or "/tmp", "farmlaunch-kspfarm.lock")
CODE_EXT = (".py", ".sh", ".cs", ".cfg", ".craft", ".sfs")
CONTEXT_LIMIT = int(os.environ.get("KRPCLAB_CONTEXT_LIMIT", "100000"))   # tokens
CONTEXT_REPEAT_S = 600          # the order is repeated this often until cleared


def reply(event, **kw):
    out = {"hookSpecificOutput": dict(hookEventName=event, **kw)}
    print(json.dumps(out))


def deny(reason):
    reply("PreToolUse", permissionDecision="deny", permissionDecisionReason=reason)


def batch_holder():
    """(pid, cwd, cmd) of the live farmlaunch holding the farm lock, else None."""
    try:
        with open(LOCK) as f:
            line = f.read().strip()
    except OSError:
        return None
    m = re.match(r"pid (\d+) since (\S+) \(cwd (.*?)\): (.*)", line)
    if not m:
        return None
    pid = int(m.group(1))
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            if b"farmlaunch" not in f.read():
                return None
    except OSError:
        return None
    return pid, m.group(2), m.group(3), m.group(4)


def farm_state():
    games = bh.game_pids()
    if not games:
        return "Farm: down."
    t = min(bh.started_at(p) or time.time() for p in games)
    sw = bh.swap_gb() or 0
    h = batch_holder()
    s = ("Farm: %d game(s) up since %s (%.1f h), swap %.1f GB." % (len(games), bh.hhmm(t),
         (time.time() - t) / 3600, sw))
    if h:
        s += " A batch holds the farm: %s since %s." % (h[3][:120], h[1])
    else:
        s += " No batch running: restart it (testInstances/stop.sh && ./start.sh) before the first comparison."
    return s


def session(inp):
    try:
        with open(os.path.join(ROOT, "HANDOFF.md")) as f:
            handoff = f.read()
    except OSError:
        handoff = "(no HANDOFF.md)"
    ctx = ("krpcLab session start (tools/claudeHooks.py). %s\n\nHANDOFF.md, already read for you "
           "(do not Read it again); next read docs/<pilot>/handoff.md for the pilot you work on, then "
           "its CLAUDE.md:\n\n%s" % (farm_state(), handoff))
    reply("SessionStart", additionalContext=ctx)


SUITE = re.compile(r"(-m\s+unittest|bundle\.py\b[^;|&]*--suite)")


def bash(inp):
    cmd = (inp.get("tool_input") or {}).get("command", "")
    if SUITE.search(cmd) and bh.game_pids():
        deny("The offline suite beside a running farm is killed by memory pressure (six instances already "
             "put 8-13 GB into zram). Stop the farm (cd testInstances && ./stop.sh), run the suite, then "
             "restart the farm (./start.sh) and fly. If a batch holds the farm, run the suite after it ends.")


def edit(inp):
    ti = inp.get("tool_input") or {}
    path = os.path.realpath(ti.get("file_path") or ti.get("notebook_path") or "")
    if not path.startswith(ROOT + os.sep) or not path.endswith(CODE_EXT):
        return
    rel = os.path.relpath(path, ROOT)
    if rel.startswith((".wt-", ".claude" + os.sep, "logs" + os.sep, "dist" + os.sep)):
        return
    h = batch_holder()
    if not h or os.path.realpath(h[2]) != ROOT:
        return
    deny("A farm batch is flying the main tree (%s, since %s): it rebuilds dist/krpcLab.py from these "
         "sources, so an edit now splits the batch into two experiments. Build the next change in a "
         "worktree instead (git worktree add .wt-<name>; edit there), and merge it after the batch ends."
         % (h[3][:120], h[1]))


def context_tokens(path):
    """The context the last assistant turn was sent with, from the transcript
    (input + cache read + cache creation), or None."""
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 4_000_000))
            lines = f.read().decode("utf-8", "replace").splitlines()
    except OSError:
        return None
    for line in reversed(lines):
        if '"usage"' not in line:
            continue
        try:
            u = json.loads(line).get("message", {}).get("usage")
        except ValueError:
            continue
        if u:
            return (u.get("input_tokens", 0) + u.get("cache_read_input_tokens", 0)
                    + u.get("cache_creation_input_tokens", 0))
    return None


def context_order(inp):
    tokens = context_tokens(inp.get("transcript_path") or "")
    if tokens is None or tokens < CONTEXT_LIMIT:
        return None
    stamp = os.path.join(ROOT, "logs", ".ctxOrder-%s" % (inp.get("session_id") or "x"))
    try:
        if time.time() - os.path.getmtime(stamp) < CONTEXT_REPEAT_S:
            return None
    except OSError:
        pass
    os.makedirs(os.path.dirname(stamp), exist_ok=True)
    with open(stamp, "w"):
        pass
    return ("HARD RULE -- CONTEXT LIMIT: this conversation's context is %dk tokens, over the %dk limit "
            "(tools/claudeHooks.py, the user 2026-11-18). Not a judgement call. Finish only the step in hand "
            "(if you are answering the user, answer first), then: (1) rewrite the handoff of every pilot "
            "touched (docs/<pilot>/handoff.md + HANDOFF.md's row) with the results so far, the running job "
            "and what to do with its outcome; (2) commit; (3) if a batch is flying or about to be: launch "
            "it detached and arm the relay task (~/.claude/docs/autoContinue.md steps 2-4); (4) clear self "
            "(mcp__ccd_session_mgmt__clear_session \"self\"). Do not start new analysis or another "
            "batch in this context." % (tokens // 1000, CONTEXT_LIMIT // 1000))


def bundle_error():
    log = os.path.join(ROOT, ".git-bundle-error.log")
    stamp = os.path.join(ROOT, "logs", ".bundleErrorSeen")
    try:
        t = os.path.getmtime(log)
        if os.path.getsize(log) == 0:
            return None
    except OSError:
        return None
    try:
        if os.path.getmtime(stamp) >= t:
            return None
    except OSError:
        pass
    os.makedirs(os.path.dirname(stamp), exist_ok=True)
    with open(stamp, "w"):
        pass
    with open(log, errors="replace") as f:
        tail = "".join(f.readlines()[-15:])
    return ("The post-commit hook's bundle build or background offline suite FAILED "
            "(%s, %s). Fix it before the next batch: every flight flies dist/krpcLab.py. Tail:\n%s"
            % (log, bh.hhmm(t), tail))


def notice(inp):
    parts = [m for m in (bundle_error(), context_order(inp)) if m]
    if parts:
        reply(inp.get("hook_event_name") or "PostToolUse",
              additionalContext="\n\n".join(parts))


if __name__ == "__main__":
    ev = sys.argv[1] if len(sys.argv) > 1 else ""
    try:
        inp = json.load(sys.stdin)
    except ValueError:
        inp = {}
    try:
        {"session": session, "bash": bash, "edit": edit, "notice": notice}[ev](inp)
    except Exception:
        pass
    sys.exit(0)
