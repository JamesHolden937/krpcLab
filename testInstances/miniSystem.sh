#!/usr/bin/env bash
#
# miniSystem.sh [n]... -- put the MiniSystem planet patch (~/minisystem) and a working Kopernicus into
# base/ and into the named clones (default 0-5).  Since 2026-11-06 the farm runs this MODIFIED Kerbol
# system, not stock: planets' orbits x0.2, Kerbol GM and radius x0.2, moons' orbits x0.5.  Kerbin itself
# (mass, radius, atmosphere, solar day 21600 s) is stock, but its SIDEREAL rotation is 21349.47 s
# (stock 21549.43), because the year is 5x shorter.  See ~/minisystem/CLAUDE.md.
#
# Also fixes Kopernicus, which never loaded on the farm before: strip.txt removed KSPTextureLoader, which
# Kopernicus 1.0.25x needs, so the game silently ran without Kopernicus.  The three Kopernicus folders
# are copied from the live install.  MiniSystem is copied, not symlinked, so the farm only changes when
# this is re-run.
#
# Clones get the same treatment as mkclone.sh: hardlinks to base/, with the .cfg files unshared.
# Run it with the farm stopped.  `miniSystem.sh --stock [n]...` removes MiniSystem again (stock system,
# Kopernicus still fixed).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KSPFARM="${KSPFARM:-$HOME/kspfarm}"; BASE="$KSPFARM/base"
LIVE="${KSP_SRC:-$HOME/KSP-Retooled}/GameData"
MINI="${MINISYSTEM:-$HOME/minisystem}/GameData/MiniSystem"
[ -f "$MINI/MiniSystem.cfg" ] || { echo "no $MINI/MiniSystem.cfg (run ~/minisystem/tools/genPatch.py)" >&2; exit 1; }
. "$HERE/procs.sh"
for N in $(farm_instances); do
  if game_pid "$N" >/dev/null; then echo "farm is running (ksp$N) -- ./stop.sh first" >&2; exit 1; fi
done
STOCK=0; [ "${1:-}" = --stock ] && { STOCK=1; shift; }
[ $# -eq 0 ] && set -- 0 1 2 3 4 5
if [ $STOCK = 1 ]; then
  rm -rf "$BASE/GameData/MiniSystem"
  for N in "$@"; do rm -rf "$HERE/ksp$N/GameData/MiniSystem" "$HERE/ksp$N/GameData/ModuleManager.ConfigCache"; done
  echo "MiniSystem removed: base/ and ksp$* run the STOCK system -- update the notes (grep -rl MiniSystem ~/krpcLab ~/kspfarm/README.md ~/.claude/docs/ksp.md)"
  exit 0
fi
DIRS=(Kopernicus KSPTextureLoader BurstPQS MiniSystem)

for d in Kopernicus KSPTextureLoader BurstPQS; do
  rm -rf "$BASE/GameData/$d"; cp -a "$LIVE/$d" "$BASE/GameData/$d"
done
rm -rf "$BASE/GameData/Kopernicus/Cache"/*        # written by the game: must never be shared
rm -rf "$BASE/GameData/MiniSystem"; cp -a "$MINI" "$BASE/GameData/MiniSystem"
echo "base: Kopernicus $(tr -d " \n" < "$BASE/GameData/Kopernicus/Plugins/Kopernicus.version" | grep -o "\"VERSION\":\"[^\"]*" | cut -d\" -f4), MiniSystem $(sha256sum "$BASE/GameData/MiniSystem/MiniSystem.cfg" | cut -c1-12)"

unshare() { [ -f "$1" ] || return 0; cp -a --remove-destination "$1" "$1.tmp$$" && mv -f "$1.tmp$$" "$1"; }
export -f unshare
for N in "$@"; do
  G="$HERE/ksp$N/GameData"; [ -d "$G" ] || { echo "no ksp$N" >&2; continue; }
  for d in "${DIRS[@]}"; do
    rm -rf "$G/$d"; cp -al "$BASE/GameData/$d" "$G/$d"
    find "$G/$d" -type f \( -name '*.cfg' -o -name '*.json' -o -name '*.txt' -o -name '*.version' \) \
         -exec bash -c 'unshare "$0"' {} \;
  done
  rm -f "$G/ModuleManager.ConfigCache" "$G/ModuleManager.ConfigSHA"   # force MM to re-patch
  echo "ksp$N: done"
done
