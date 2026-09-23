#!/usr/bin/env bash
#
# mkclone.sh <n> -- make instance ksp<n>, its wine prefix, and its launcher.
#
# The clone is hardlinked against base/, then every file KSP might rewrite in
# place has its link broken back into a private copy.  A clone therefore costs
# about 10 MB of disk instead of 7.5 GB.  base/ is never run, so nothing
# writes to the shared inodes.
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="$HERE/base"
N="${1:?usage: mkclone.sh <n>}"
DIR="$HERE/ksp$N"
PREFIX="$HERE/prefix$N"
SRCPREFIX="${KSP_PREFIX:-$HOME/.local/share/ksp-prefix}"

# Ports start at 50100 so a clone never collides with the normal install's
# 50000/50001 -- you can play while these fly.
RPC=$((50100 + 2 * N))
STREAM=$((50101 + 2 * N))

[ -d "$BASE" ] || { echo "no base/ -- run ./mkbase.sh first" >&2; exit 1; }
[ -e "$DIR" ] && { echo "ksp$N already exists" >&2; exit 1; }

echo "clone  : $DIR"
echo "ports  : rpc $RPC  stream $STREAM"

cp -al "$BASE" "$DIR"

# --- break the links on everything writable -------------------------------
# cp -al hardlinks files but recreates directories, so a NEW file in a clone
# is already private.  Only pre-existing files are shared, and these are the
# ones KSP or a mod rewrites.
unshare() {                       # replace a hardlink with a private copy
  [ -f "$1" ] || return 0
  cp -a --remove-destination "$1" "$1.tmp$$" && mv -f "$1.tmp$$" "$1"
}
export -f unshare
find "$DIR/GameData" -type f \
     \( -name '*.cfg' -o -name '*.json' -o -name '*.txt' -o -name '*.version' \) \
     -exec bash -c 'unshare "$0"' {} \;
for f in "$DIR/settings.cfg" "$DIR/Physics.cfg" "$DIR/buildID64.txt"; do unshare "$f"; done
find "$DIR/saves" "$DIR/Ships" -type f -exec bash -c 'unshare "$0"' {} \;
# PluginData/KSPBurst* is a generated, version-keyed, read-only cache: shared.

# --- this clone's kRPC ports ----------------------------------------------
K="$DIR/GameData/kRPC/PluginData/settings.cfg"
"$HERE/setports.py" "$K" "$RPC" "$STREAM"
grep -qE "autoStartServers = True" "$K" || { echo "autostart not set" >&2; exit 1; }

# --- its own wine prefix --------------------------------------------------
# One prefix per instance: wine rewrites system.reg/user.reg in place, and a
# shared prefix would have N processes racing on them.
if [ -d "$PREFIX" ]; then
  echo "  reusing existing $PREFIX"
elif [ -d "$SRCPREFIX" ]; then
  cp -a "$SRCPREFIX" "$PREFIX"
else
  echo "  no prefix at $SRCPREFIX -- umu will build one on first launch"
  mkdir -p "$PREFIX"
fi

# --- launcher -------------------------------------------------------------
cat > "$DIR/run-ksp.sh" <<LAUNCH
#!/usr/bin/env bash
# Launch this instance.  Env knobs:
#   BACKEND=headless|sdl|none  (default headless -- and it must stay that way
#                               for unattended runs; see below)
#   FPS=<n>                    (default 60)
#   RES=<w>x<h>                (default 640x360)
#
# headless gives this instance its own gamescope compositor and its own virtual
# output, where it is the only window and so is always focused.  That is not a
# cosmetic choice: Unity stops calling Update() on an unfocused window, and
# KSP's asset loading, scene transitions and dialogs are all frame-driven, so
# an unfocused instance stops making progress altogether.  Several instances as
# ordinary windows in one desktop session cannot all be focused, so all but one
# would hang.  See keepNotes.md.
#
# sdl and none put a real window in your session; use them to watch one
# instance, not to run several.
#   FPS=<n>                (default 60)
#   RES=<w>x<h>            (default 640x360)
#
# Note there is no -single-instance: that flag is exactly what stops the
# normal launcher from running two copies at once.
set -euo pipefail

DIR="$DIR"
PREFIX="$PREFIX"
COMPAT="\$HOME/.steam/root/compatibilitytools.d"
PROTONPATH="\$(ls -d "\$COMPAT"/GE-Proton* 2>/dev/null | sort -V | tail -n1)"
[ -n "\$PROTONPATH" ] || { echo "no GE-Proton in \$COMPAT" >&2; exit 1; }

RES="\${RES:-640x360}"
W="\${RES%x*}"; H="\${RES#*x}"

export __GL_SHADER_DISK_CACHE=1
export __GL_SHADER_DISK_CACHE_SKIP_CLEANUP=1
export __GL_SHADER_DISK_CACHE_PATH="\$PREFIX/nvcache"
export DXVK_STATE_CACHE=1
export DXVK_STATE_CACHE_PATH="\$PREFIX"
export WINEDEBUG=-all
export ENABLE_GAMESCOPE_WSI=0     # explicit-sync semaphores crash on NVIDIA
export GAMEID=0
export PROTONPATH
export WINEPREFIX="\$PREFIX"
export STORE=none
mkdir -p "\$__GL_SHADER_DISK_CACHE_PATH"

cd "\$DIR"

KSP_ARGS=(
  -force-d3d11
  -screen-fullscreen 0 -popupwindow
  -screen-width "\$W" -screen-height "\$H"
)

if [ "\${BACKEND:-headless}" = none ]; then
  exec umu-run "\$DIR/KSP_x64.exe" "\${KSP_ARGS[@]}"
fi

exec gamescope \\
  --backend "\${BACKEND:-headless}" \\
  -W "\$W" -H "\$H" -w "\$W" -h "\$H" \\
  -r "\${FPS:-60}" \\
  -- umu-run "\$DIR/KSP_x64.exe" "\${KSP_ARGS[@]}"
LAUNCH
chmod +x "$DIR/run-ksp.sh"

printf '%s\n' "$RPC" > "$DIR/.rpc_port"
printf '%s\n' "$STREAM" > "$DIR/.stream_port"

echo "  disk   : $(du -sh --exclude=PluginData "$DIR" | cut -f1) private (+ hardlinks into base/)"
echo "  launch : $DIR/run-ksp.sh"
