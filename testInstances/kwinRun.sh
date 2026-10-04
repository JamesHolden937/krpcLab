#!/usr/bin/env bash
#
# kwinRun.sh <n> -- run instance ksp<n> inside its own nested KWin on a
# virtual framebuffer.
#
# The point is focus.  KSP stops running frames when its window is not focused,
# and everything after the loading screen (the menu, scene transitions, the
# autoload plugin) is frame-driven.  A nested compositor with exactly one
# client gives that client focus unconditionally, and --virtual means it costs
# no visible output -- so N instances can each be "the focused window" at once.
#
# kwin_wayland is already part of the desktop session here; nothing to install.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N="${1:?usage: kwinRun.sh <n>}"
DIR="$HERE/ksp$N"
PREFIX="$HERE/prefix$N"
# **The render resolution, which nobody looks at.**  This is passed to KSP as
# `-screen-width/-screen-height`, and a command line beats settings.cfg -- so
# the 640x360 set in mkbase.sh was never what these rendered at; 900x520 was,
# which is 3.6x the pixels for an off-screen window whose only consumer is
# kRPC.  Nothing in this project reads a pixel.
#
# It cannot go to zero: KSP is frame-driven, and everything after the loading
# screen (the menu, scene transitions, the autoload addon) needs frames to
# happen at all.  So it wants to be the smallest size that still renders, not
# the smallest size that is readable.  Override with RES=WxH to compare.
RES="${RES:-160x100}"; W="${RES%x*}"; H="${RES#*x}"

# **NTSync off by default.**  GE-Proton 10 turns on wineserver's NTSync
# (/dev/ntsync, "wineserver: NTSync up and running!" in the boot log) unless
# PROTON_NO_NTSYNC is set.  Under it, a random one of six instances died
# mid-load, silently, at the same ReStock line in KSP.log -- 3 deaths in 5
# farm starts on 2026-10-03/04, with no OOM and no error, while a lone retry
# always came up.  NTSYNC=1 puts it back for comparison; the measurement is
# in docs/testInstances.md.
if [ "${NTSYNC:-0}" = 1 ]; then NTSYNC_ENV=""; else NTSYNC_ENV="export PROTON_NO_NTSYNC=1"; fi

cat > "$DIR/.inner.sh" <<INNER
#!/usr/bin/env bash
set -euo pipefail
export PROTONPATH="\$(ls -d "\$HOME/.steam/root/compatibilitytools.d"/GE-Proton* | sort -V | tail -1)"
export GAMEID=0 STORE=none
export WINEPREFIX="$PREFIX"
export __GL_SHADER_DISK_CACHE=1 __GL_SHADER_DISK_CACHE_PATH="$PREFIX/nvcache"
export DXVK_STATE_CACHE=1 DXVK_STATE_CACHE_PATH="$PREFIX"
export WINEDEBUG=-all
${NTSYNC_ENV}
mkdir -p "\$__GL_SHADER_DISK_CACHE_PATH"
cd "$DIR"
exec umu-run "$DIR/KSP_x64.exe" -force-d3d11 ${KSP_EXTRA:-} \\
# -popupwindow is KEPT.  Dropping it was tried and is worse: the load then
# stalls at 1208 log lines instead of reaching the menu, so the popup window
# hint is not what costs the window its focus.
     -screen-fullscreen 0 -popupwindow -screen-width $W -screen-height $H
INNER
chmod +x "$DIR/.inner.sh"

# Each nested KWin needs its own wayland socket name, or they collide.
#
# **And its own D-Bus session, which is what stops Alt+F4 killing the farm.**
# A nested kwin_wayland registers its shortcuts with KGlobalAccel as *global*
# ones, so on the shared session bus all four of these sit alongside the
# desktop's own KWin on `Window Close = Alt+F4`.  One keypress at the desktop
# is then delivered to every nested compositor as well, and each dutifully
# closes the one client it has: KSP.  All four instances exit within seconds
# of each other, the compositors survive because losing a client does not end
# them, and what is left on disk is four orderly Unity `OnDestroy` cascades
# ending in "Server 'Default Server' stopped".
#
# That signature is in docs/testInstances.md twice -- once as "the graphical
# session restarting" and once as "unexplained" -- and the tell was written
# down both times without being read: an orderly OnDestroy cascade is a
# **clean quit request**, not a crash.  Nothing was reaping these processes;
# they were being politely asked to close, and they were saying yes.
#
# dbus-run-session gives the compositor a private bus, so its global
# shortcuts are global only to itself.  The kRPC server is TCP on localhost
# and does not care.
exec dbus-run-session -- \
     kwin_wayland --virtual --width "$W" --height "$H" \
     --xwayland --no-lockscreen \
     --socket "ksp$N-wl" \
     -- "$DIR/.inner.sh"
