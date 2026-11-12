# procs.sh -- sourced by start.sh, stop.sh, watchdog.sh: which processes are
# farm instance N's.  Not executable on its own.
#
# **Never by `pgrep -f "kspN/KSP_x64.exe"`.**  Under Proton the game process
# itself is `X:\krpcLab\testInstances\ksp0\KSP_x64.exe` -- backslashes, a
# drive letter -- so that pattern matches only umu-run and Proton's python
# wrapper, never the game, its wineserver or its winedevice.exe.  stop.sh
# killed those wrappers, printed "remaining: 0", and four games ran on at
# ~300% CPU each for 50 minutes (2026-11-11).
#
# An instance's processes are found three ways, any one of which suffices:
#   - its systemd scope's cgroup (`kspN-boosterland.scope`, start.sh);
#   - WINEPREFIX=<prefixN> in its environment (wineserver, winedevice, the
#     game: everything Proton starts, however it was launched -- the
#     watchdog's relaunch uses setsid, not a scope);
#   - its command line naming kspN's directory in either slash form, or
#     the compositor's `kspN-wl` socket.
# Shell processes whose command line merely mentions the path (a caller's
# `bash -c '... ksp0 ...'`) are skipped, as are this shell and its ancestors.
FARM_HERE="${FARM_HERE:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

# inst_pids N -- every live pid belonging to instance N, plus their
# descendants (the compositor's dbus-daemon and Xwayland carry no mark of
# their own), one per line.  One python3 pass over /proc: a fork per process
# in bash takes seconds.
inst_pids() {
  python3 - "$1" "$FARM_HERE" "$$" <<'PY'
import os, sys
n, here, me = sys.argv[1], sys.argv[2], int(sys.argv[3])
real = lambda p: os.path.realpath(p)
pfx = {f"WINEPREFIX={here}/prefix{n}", f"WINEPREFIX={real(f'{here}/prefix{n}')}"}
dirs = (f"{here}/ksp{n}/", real(f"{here}/ksp{n}") + "/",
        f"testInstances\\ksp{n}\\", f"krpclab\\ksp{n}\\")
scope = f"/ksp{n}-boosterland.scope"
shells = {"bash", "sh", "dash", "zsh", "timeout", "flock"}
def rd(pid, f):
    try:
        with open(f"/proc/{pid}/{f}", "rb") as h: return h.read().decode(errors="replace")
    except OSError: return None
par, hit = {}, set()
for e in os.listdir("/proc"):
    if not e.isdigit(): continue
    pid = int(e); st = rd(pid, "stat")
    if st is None: continue
    par[pid] = int(st.rsplit(")", 1)[1].split()[1])
    if any(l.endswith(scope) for l in (rd(pid, "cgroup") or "").splitlines()): hit.add(pid); continue
    env = rd(pid, "environ")
    if env and pfx & set(env.split("\0")): hit.add(pid); continue
    if (rd(pid, "comm") or "").strip() in shells: continue
    cmd = (rd(pid, "cmdline") or "").replace("\0", " ") + " "
    if any(d in cmd for d in dirs) or f" ksp{n}-wl " in cmd: hit.add(pid)
skip, p = set(), me          # the caller and its ancestors are never targets
while p > 1 and p in par: skip.add(p); p = par[p]
grew = True
while grew:
    grew = False
    for c, pp in par.items():
        if c not in hit and pp in hit: hit.add(c); grew = True
for p in sorted(hit - skip): print(p)
PY
}

# game_pid N -- the KSP_x64.exe of instance N (exit 1 if none).
game_pid() {
  local p
  for p in $(inst_pids "$1"); do
    [ "$(cat /proc/$p/comm 2>/dev/null)" = KSP_x64.exe ] && { echo "$p"; return 0; }
  done
  return 1
}

# farm_instances -- the instance numbers that exist on disk.
farm_instances() {
  ls -d "$FARM_HERE"/ksp[0-9]* 2>/dev/null | sed 's#.*/ksp##' | grep -E '^[0-9]+$'
}
