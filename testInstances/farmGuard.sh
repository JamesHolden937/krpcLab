# farmGuard.sh -- sourced first by every farm (and kspSim) batch script:
#   . "<root>/testInstances/farmGuard.sh"
# Re-executes the calling script under ~/.local/bin/farmlaunch, unless it
# already runs inside one (FARMLAUNCH set: an outer batch script, or a
# farmlaunch typed by hand).  So every batch, however it was started:
#   - prints "farmlaunch: ... launch ..." as its first line;
#   - refuses at once (exit 75) when another batch holds the farm, instead
#     of queueing behind it -- the farm is one instrument, and two batches on
#     it would distort each other's timing anyway;
#   - is killed and reported (exit 124) if no flight log (logs/LOG<n>) has
#     appeared within FARM_START_DEADLINE seconds (default 900: a farm restart
#     plus the first save load fit well inside it).
# Why: 2026-11-11 lost ~1 h to batches that were waiting, silently, on
# something that would never happen (HANDOFF / docs/testInstances.md).
# FARMGUARD=0 skips it (e.g. a test of the guard itself).
if [ -z "${FARMLAUNCH:-}" ] && [ "${FARMGUARD:-1}" != 0 ]; then
  if command -v farmlaunch >/dev/null 2>&1; then
    _fg_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    exec farmlaunch --lock kspfarm --start-deadline "${FARM_START_DEADLINE:-900}" \
         --started-glob "$_fg_root/logs/LOG*" -- "$0" "$@"
  fi
  echo "farmGuard: no farmlaunch on PATH (~/.local/bin/farmlaunch); running unguarded" >&2
fi
