#!/usr/bin/env bash
#
# watchdog.sh start|stop|status [n]... -- keep the named instances alive.
#
# The farm has four documented ways to lose every instance at once (see
# docs/test-instances.md) and they have one thing in common: nothing here
# notices.  The compositors survive, `ss -ltn` still shows nothing, and the
# next batch is launched into four dead ports -- or worse, a flight already
# in the air stops producing telemetry and the harness sits on it until its
# timeout.  In a project whose bottleneck is in-game measurement, an hour of
# dead farm is the most expensive failure there is.
#
# So the supervision is separate from the cause.  This does not care *why* an
# instance went away: it polls "is the process there and is the port open",
# and brings back anything that is not.  That covers the suspend, the GPU
# context loss, the OOM kill and the session restart with one mechanism.
#
# Two things it has to get right, both of them already paid for elsewhere:
#
#  * **A booting instance is not a dead one.**  A clone takes minutes to
#    reach the menu and open its port, so an instance is only judged after
#    BOOT_GRACE seconds, and the clock restarts on every launch.
#  * **Clear the corpse before relaunching.**  The nested compositor does not
#    exit with the game and keeps holding the `kspN-wl` socket; start.sh's
#    own notes record three of them accumulating and every instance launched
#    afterwards dying ~75 s after its kRPC port came up.  So a restart goes
#    through stop.sh, which knows how to kill the compositor too.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDFILE="$HERE/.watchdog.pid"
LOG="$HERE/watchdog.log"
INTERVAL="${WATCHDOG_INTERVAL:-20}"
BOOT_GRACE="${WATCHDOG_BOOT_GRACE:-480}"

instances() {
  ls -d "$HERE"/ksp[0-9]* 2>/dev/null | sed 's#.*/ksp##' | grep -E '^[0-9]+$'
}

port_of() { cat "$HERE/ksp$1/.rpc_port" 2>/dev/null; }

alive() {
  local n="$1" port
  pgrep -f "ksp$n/KSP_x64.exe" >/dev/null 2>&1 || return 1
  port="$(port_of "$n")"
  [ -n "$port" ] || return 0          # process up, port not named: not dead
  ss -ltn 2>/dev/null | grep -q ":$port " || return 1
  return 0
}

say() { printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "$LOG"; }

# What took it down, asked positively rather than guessed.  docs/test-instances
# has a whole section on not hunting a reaper that does not exist; this writes
# the answer into the log at the moment it is still findable.
verdict() {
  local since="-3min"
  # ``-q``, not a grep for any output: journalctl prints "-- No entries --"
  # on stdout when there are none, which a ``grep -q .`` reads as a hit --
  # and the first restart this watchdog ever performed was logged as "a
  # suspend" on a machine that had not suspended.
  journalctl --user -b -u systemd-suspend.service --since "$since" -q \
      --no-pager 2>/dev/null | grep -q "systemd-suspend" \
      && { echo "a suspend"; return; }
  journalctl -b --since "$since" --no-pager 2>/dev/null \
      | grep -qiE "Xid|NVRM|EGL_CONTEXT_LOST|GPU has fallen" \
      && { echo "a GPU context loss"; return; }
  coredumpctl list --since "$since" --no-pager 2>/dev/null | grep -q SIG \
      && { echo "something on this machine dumped core"; return; }
  journalctl -b --since "$since" --no-pager 2>/dev/null \
      | grep -qiE "oomd|Out of memory|Killed process" \
      && { echo "an OOM kill"; return; }
  echo "no known cause found"
}

# **Say when the card is nearly full, because that is what kills the farm.**
# Four full-res instances were measured at 14.9 GB of a 16.3 GB card, and an
# exhausted card does not fail politely: it loses GL contexts, and a lost
# context takes down every GL client on the machine at once -- the farm, the
# browser, and the desktop's smoothness with them.  That is the real shape of
# the "everything died together" entry in docs/test-instances.md.  One line
# in the log when it gets tight turns the next occurrence from a mystery into
# a number.
vram_warn() {
  local used total pct
  read -r used total < <(nvidia-smi --query-gpu=memory.used,memory.total \
      --format=csv,noheader,nounits 2>/dev/null | tr -d ',')
  [ -n "${used:-}" ] && [ -n "${total:-}" ] || return 0
  pct=$(( 100 * used / total ))
  [ "$pct" -ge "${WATCHDOG_VRAM_WARN_PCT:-88}" ] \
    && say "VRAM ${used}/${total} MiB (${pct}%) -- a full card loses GL contexts"
  return 0
}

# **An instance can pass every liveness test and still be dead to work.**
# One was found with its process up, its port open, kRPC answering every
# query, the vessel reading `orbiting` and 240 fps on the clock -- and game
# time frozen to the millisecond, with zero parts on a twenty-three part
# vessel.  `alive()` cannot see that: the process is there and the port is
# open, which is all it asks.  The harness could not see it either, so the
# flight launched into a paused game, logged nothing, and burned its whole
# timeout before reporting a position as though it were a landing.
#
# So ask the one question that separates a running game from a stopped one.
# It **warns and does not restart**, deliberately: a frozen clock is also
# what a save load and a scene transition look like for a few seconds, and
# restarting on that would destroy a measurement in progress to fix a
# condition that was about to clear.  Frozen across several consecutive
# checks is the same "held for a while, not taken on one tick" rule the
# grounded backstop uses, and for the same reason.
#
# **Verified only in the negative.**  Running instances are correctly
# reported as advancing; the frozen branch has not been exercised against a
# genuinely wedged instance, because the one that produced this was
# restarted before the check existed and kRPC offers no way to pause a game
# on purpose.  Treat a warning from it as a hypothesis worth confirming with
# `./timescale.py N` -- a wedged instance reports `achieved 0.00x` at a
# healthy frame rate -- until it has caught one in the wild.
clock_frozen() {                      # 0 = frozen, 1 = advancing or unknown
  local n="$1" rpc stream
  rpc="$(port_of "$n")"; [ -n "$rpc" ] || return 1
  stream="$(cat "$HERE/ksp$n/.stream_port" 2>/dev/null)"; [ -n "$stream" ] || return 1
  "$HERE/../.venv/bin/python" - "$rpc" "$stream" <<'PY' >/dev/null 2>&1
import sys, time
try:
    import krpc
    c = krpc.connect(name="watchdog-clock", rpc_port=int(sys.argv[1]),
                     stream_port=int(sys.argv[2]))
except Exception:
    sys.exit(1)                       # cannot ask: not an answer, not a fault
try:
    a = c.space_center.ut
    time.sleep(2.0)
    sys.exit(0 if c.space_center.ut <= a else 1)
except Exception:
    sys.exit(1)
finally:
    try:
        c.close()
    except Exception:
        pass
PY
}

supervise() {
  local -A launched=()
  local -A frozen=()
  local n now
  for n in "$@"; do launched[$n]=0; frozen[$n]=0; done
  say "watchdog up on instances: $*"
  local ticks=0
  while :; do
    now="$(date +%s)"
    ticks=$(( ticks + 1 ))
    [ $(( ticks % 15 )) -eq 1 ] && vram_warn
    # Cheap tests every tick; this one costs a kRPC round trip, so it runs
    # rarely and only against instances that already look healthy.
    if [ $(( ticks % 30 )) -eq 7 ]; then
      for n in "$@"; do
        alive "$n" || continue
        if clock_frozen "$n"; then
          frozen[$n]=$(( ${frozen[$n]} + 1 ))
          [ "${frozen[$n]}" -ge "${WATCHDOG_FROZEN_STRIKES:-3}" ] \
            && say "ksp$n: port open but game time has not advanced in ${frozen[$n]} checks -- wedged, not flying"
        else
          frozen[$n]=0
        fi
      done
    fi
    for n in "$@"; do
      if alive "$n"; then
        launched[$n]=0
        continue
      fi
      # Still inside its boot window?  Leave it alone.
      if [ "${launched[$n]}" != 0 ] \
         && [ $(( now - ${launched[$n]} )) -lt "$BOOT_GRACE" ]; then
        continue
      fi
      # **A boot that is still making progress is not a hung boot.**  A
      # deadline alone cannot tell "loading slowly" from "wedged", and it
      # gets the answer wrong exactly when a boot is unusually slow -- after
      # a TEXTURE_QUALITY change, say, when every instance rebuilds its PNG
      # cache and takes several times as long.  Killing it then restarts the
      # rebuild from the beginning, so the deadline makes the instance it is
      # policing permanently unable to finish.  KSP.log growing is the
      # progress signal, and it costs a stat.
      if [ -n "$(find "$HERE/ksp$n/KSP.log" -newermt "-${INTERVAL} seconds" \
                 2>/dev/null)" ]; then
        continue
      fi
      if [ "${launched[$n]}" != 0 ]; then
        say "ksp$n never opened its port within ${BOOT_GRACE}s -- relaunching"
      else
        say "ksp$n is gone ($(verdict)) -- relaunching"
      fi
      "$HERE/stop.sh" "$n" >/dev/null 2>&1
      setsid "$HERE/kwin-run.sh" "$n" >/dev/null 2>&1 </dev/null &
      launched[$n]="$now"
    done
    sleep "$INTERVAL"
  done
}

case "${1:-status}" in
  start)
    shift
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "watchdog already running (pid $(cat "$PIDFILE"))"; exit 0
    fi
    [ $# -gt 0 ] || set -- $(instances)
    setsid "$0" __supervise "$@" >/dev/null 2>&1 </dev/null &
    echo $! > "$PIDFILE"
    echo "watchdog started (pid $(cat "$PIDFILE")) on ksp: $*  log: $LOG"
    ;;
  __supervise) shift; supervise "$@" ;;
  stop)
    if [ -f "$PIDFILE" ]; then
      kill "$(cat "$PIDFILE")" 2>/dev/null && echo "watchdog stopped"
      rm -f "$PIDFILE"
    else
      echo "watchdog not running"
    fi
    ;;
  status)
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "watchdog running (pid $(cat "$PIDFILE"))"
    else
      echo "watchdog not running"
    fi
    for n in $(instances); do
      alive "$n" && echo "  ksp$n up on $(port_of "$n")" \
                 || echo "  ksp$n DOWN"
    done
    [ -f "$LOG" ] && { echo "  -- last restarts --"; tail -5 "$LOG"; }
    ;;
  *) echo "usage: watchdog.sh start|stop|status [n]..." >&2; exit 2 ;;
esac
