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
            and the CONTEXT LIMIT: when a Bash call has just launched a farm
            batch (BATCH_LAUNCH: `detachjob start`, `farmlaunch`, a batch
            script) and the conversation's context (the last assistant turn's
            input + cache tokens, read from the transcript) is over
            CONTEXT_LIMIT, a hard order to write the handoff and clear.  Only
            then: the batch's wait is the free moment, and an order mid-analysis
            cut work the user wanted finished (the user, 2026-11-18: "only
            triggers after you start a farm batch"; earlier the same day a
            judgement call to skip the clear was skipped at 205k).
  stop      Stop: refuses to end the turn while work is left (the user, 2026-11-18: "make
            sure you don't stop when there's work left to do"): tracked changes uncommitted;
            a batch ended with no handoff committed since; or nothing flying and the turn
            does not end on a question to the user or a line `DONE: <why nothing is left>`.
            Never blocks a scheduled relay run or while a detachjob runs (waiting is the
            procedure); lets the stop through after STOP_MAX_BLOCKS refusals in a row.
"""
import json
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "testInstances"))
import batchHooks as bh  # noqa: E402

LOCK = os.path.join(os.environ.get("XDG_RUNTIME_DIR") or "/tmp", "farmlaunch-kspfarm.lock")
CODE_EXT = (".py", ".sh", ".cs", ".cfg", ".craft", ".sfs")
CONTEXT_LIMIT = int(os.environ.get("KRPCLAB_CONTEXT_LIMIT", "100000"))   # tokens
# A Bash command that starts a farm batch (farmlaunch refuses from a session
# unless under detachjob, so a session's launches go through `detachjob start`).
BATCH_LAUNCH = re.compile(r"\bdetachjob\s+start\b|\bfarmlaunch\b|\b[bs]rot\.sh\b|\bfarmfly\b")


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


def batch_launched(inp):
    """Did this hook's Bash call start a farm batch?"""
    if inp.get("hook_event_name") != "PostToolUse" or inp.get("tool_name") != "Bash":
        return False
    command = (inp.get("tool_input") or {}).get("command") or ""
    # Only the command itself: a heredoc body or a quoted string that merely names a
    # batch script is text (a handoff written by a heredoc fired the order, 2026-11-18).
    command = command.split("<<", 1)[0]
    command = re.sub(r"'[^']*'|\"(?:[^\"\\]|\\.)*\"", "''", command)
    return bool(BATCH_LAUNCH.search(command))


def context_order(inp):
    if not batch_launched(inp):
        return None
    tokens = context_tokens(inp.get("transcript_path") or "")
    if tokens is None or tokens < CONTEXT_LIMIT:
        return None
    return ("HARD RULE -- CONTEXT LIMIT: a farm batch just started and this conversation's context is "
            "%dk tokens, over the %dk limit (tools/claudeHooks.py, the user 2026-11-18). Not a judgement "
            "call. Finish only the step in hand (if you are answering the user, answer first), then: (1) rewrite the handoff of every pilot "
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


STOP_MAX_BLOCKS = 4
STOP_STATE = os.path.join(ROOT, "logs", ".stopHook.json")


def transcript_texts(path):
    """(first user message text, last assistant text) from the transcript."""
    first = last = ""
    try:
        with open(path, errors="replace") as f:
            lines = f.readlines()
    except (OSError, TypeError):
        return first, last
    def text(e):
        c = (e.get("message") or {}).get("content")
        if isinstance(c, str):
            return c
        return "\n".join(x.get("text", "") for x in c or () if isinstance(x, dict) and x.get("type") == "text")
    for line in lines:
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get("type") == "user" and not first:
            first = text(e)
        elif e.get("type") == "assistant":
            t = text(e)
            if t.strip():
                last = t
    return first, last


def detachjob_running():
    r = subprocess.run(["systemctl", "--user", "list-units", "detachjob-*", "--state=active",
                        "--no-legend", "--plain"], capture_output=True, text=True, timeout=5)
    return [l.split()[0] for l in r.stdout.splitlines() if l.strip()]


def git(*a):
    return subprocess.run(["git", "-C", ROOT] + list(a), capture_output=True, text=True, timeout=10)


def stop_reasons(inp):
    first, last = transcript_texts(inp.get("transcript_path"))
    last = inp.get("last_assistant_message") or last
    if "narrow mechanical relay" in first:
        return []
    paras = [x for x in re.split(r"\n\s*\n", last.strip()) if x.strip()]
    if paras and re.search(r"\?\s*(\*\*)?\s*$", paras[-1], re.M):
        return []            # the turn ends on a question to the user: theirs to answer
    if detachjob_running() or batch_holder():
        return []            # waiting on a batch is the procedure (relay armed, clear)
    why = []
    dirty = [l[3:] for l in git("status", "--porcelain", "--untracked-files=no").stdout.splitlines()]
    if dirty:
        why.append("Tracked files are uncommitted (%s): finish and commit them, or revert."
                   % ", ".join(dirty[:6]))
    st = bh.load()
    end = st.get("last_end")
    if end:
        r = git("log", "-1", "--format=%ct", "--", "HANDOFF.md", "docs")
        t = int(r.stdout.strip() or 0)
        if t < end:
            why.append("Batch ended at %s and no handoff has been committed since: read its result "
                       "(detachjob status NAME --tail 150), write docs/<pilot>/handoff.md, commit, "
                       "launch the next batch." % bh.hhmm(end))
    if not why and not re.search(r"(?m)^\s*\**DONE:", last):
        why.append("Nothing is flying. If any step is left (your plan's pending items, the pilot "
                   "handoff's next step, the next batch), do it now. Stop only if blocked on the user "
                   "(end on the question) or truly done: end with a line `DONE: <why nothing is left>`.")
    return why


def stop(inp):
    sid = inp.get("session_id") or "?"
    try:
        with open(STOP_STATE) as f:
            st = json.load(f)
    except (OSError, ValueError):
        st = {}
    why = stop_reasons(inp)
    n = st.get(sid, 0) + 1 if why else 0
    st = {sid: n}
    try:
        with open(STOP_STATE, "w") as f:
            json.dump(st, f)
    except OSError:
        pass
    if why and n <= STOP_MAX_BLOCKS:
        print(json.dumps({"decision": "block", "reason":
                          "Work left (krpcLab stop hook, %d/%d):\n- " % (n, STOP_MAX_BLOCKS)
                          + "\n- ".join(why)}))


if __name__ == "__main__":
    ev = sys.argv[1] if len(sys.argv) > 1 else ""
    try:
        inp = json.load(sys.stdin)
    except ValueError:
        inp = {}
    try:
        {"session": session, "bash": bash, "edit": edit, "notice": notice, "stop": stop}[ev](inp)
    except Exception:
        pass
    sys.exit(0)
