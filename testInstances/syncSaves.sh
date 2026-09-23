#!/usr/bin/env bash
#
# syncSaves.sh -- keep every farm instance flying the same saves.
#
# The reference copies live in ../saves/ and are tracked in git.  Each clone
# keeps its own saves/default/, and those drift: a save made on one instance
# (entrysave.py, savegen.py) exists only there, and a clone that was not
# refreshed keeps an old version under the same name.  Before this script,
# qs_plane.sfs existed in two versions across the six clones, and a batch that
# spread across both was two experiments reported as one.
#
#   ./syncSaves.sh check            # which instances differ from ../saves/
#   ./syncSaves.sh push [N ...]     # ../saves/ -> instances (default: all)
#   ./syncSaves.sh pull N NAME ...  # instance N's NAME.sfs -> ../saves/
#
# After `pull`, commit the new save and `push` it to the rest of the farm.
# `push` replaces files rather than writing into them: a clone's files start
# out hardlinked to base/, and writing through a hardlink edits every clone.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAVES="$(cd "$HERE/../saves" && pwd)"

instances() {
  if [ $# -gt 0 ]; then printf '%s\n' "$@"; return; fi
  for d in "$HERE"/ksp[0-9]*; do [ -d "$d" ] && basename "$d" | sed 's/^ksp//'; done
}

cmd="${1:-check}"; shift || true
case "$cmd" in
  check)
    status=0
    for n in $(instances); do
      dst="$HERE/ksp$n/saves/default"
      for src in "$SAVES"/*.sfs; do
        name="$(basename "$src")"
        if [ ! -f "$dst/$name" ]; then
          echo "ksp$n: missing $name"; status=1
        elif ! cmp -s "$src" "$dst/$name"; then
          echo "ksp$n: differs  $name"; status=1
        fi
      done
    done
    [ $status -eq 0 ] && echo "all instances match $SAVES"
    exit $status ;;
  push)
    for n in $(instances "$@"); do
      dst="$HERE/ksp$n/saves/default"
      [ -d "$dst" ] || { echo "no $dst" >&2; continue; }
      cp --remove-destination -p "$SAVES"/*.sfs "$SAVES"/*.loadmeta "$dst/"
      echo "ksp$n: $(ls "$SAVES"/*.sfs | wc -l) saves"
    done ;;
  pull)
    n="${1:?usage: syncSaves.sh pull N NAME ...}"; shift
    [ $# -gt 0 ] || { echo "usage: syncSaves.sh pull N NAME ..." >&2; exit 2; }
    src="$HERE/ksp$n/saves/default"
    for name in "$@"; do
      name="${name%.sfs}"
      cp -p "$src/$name.sfs" "$SAVES/" || exit 1
      [ -f "$src/$name.loadmeta" ] && cp -p "$src/$name.loadmeta" "$SAVES/"
      echo "saves/$name.sfs <- ksp$n"
    done ;;
  *)
    echo "usage: syncSaves.sh check | push [N ...] | pull N NAME ..." >&2; exit 2 ;;
esac
