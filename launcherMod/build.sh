#!/usr/bin/env bash
# Build the krpcLab window mod (KrpcLabLauncher.dll) -- the launcher's menu,
# every autopilot's panel, START SCRIPT -- and install it into the farm
# (base/ and every clone) and the live game, with PluginData/launch.cfg
# naming this checkout (START SCRIPT runs its ./run.sh).
#
#   launcherMod/build.sh
#
# Output: launcherMod/bin/KrpcLabLauncher.dll (git-ignored)
#   -> GameData/KrpcLabLauncher/Plugins/.  Needs kRPC (its service is the
# launcher's only way in).  The game's assemblies come from the farm's base
# copy (KSP=<dir> to override).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HERE/.."
KSP="${KSP:-$ROOT/testInstances/base}"
M="$KSP/KSP_x64_Data/Managed"
BIN="$HERE/bin"
mkdir -p "$BIN"
mcs -optimize -nologo -target:library -out:"$BIN/KrpcLabLauncher.dll" \
    -r:"$M/Assembly-CSharp.dll" -r:"$M/Assembly-CSharp-firstpass.dll" -r:"$M/UnityEngine.CoreModule.dll" \
    -r:"$M/UnityEngine.dll" -r:"$M/UnityEngine.UI.dll" -r:"$M/UnityEngine.IMGUIModule.dll" \
    -r:"$M/UnityEngine.TextRenderingModule.dll" -r:"$M/UnityEngine.InputLegacyModule.dll" -r:"$M/UnityEngine.AnimationModule.dll" \
    -r:"$KSP/GameData/kRPC/KRPC.Core.dll" \
    "$HERE"/*.cs
install_into() {   # $1: a game directory
    local g="$1/GameData"
    [ -d "$g/kRPC" ] || return 0
    mkdir -p "$g/KrpcLabLauncher/Plugins"
    cp -f --remove-destination "$BIN/KrpcLabLauncher.dll" "$g/KrpcLabLauncher/Plugins/"
    # where START SCRIPT finds ./run.sh
    mkdir -p "$g/KrpcLabLauncher/PluginData"
    printf 'root=%s\nhome=%s\n' "$(cd "$ROOT" && pwd)" "$HOME" \
        > "$g/KrpcLabLauncher/PluginData/launch.cfg"
}
for d in "$ROOT/testInstances/base" "$ROOT"/testInstances/ksp[0-9]*; do install_into "$d"; done
install_into "${KSP_LIVE:-$HOME/Kerbal Space Program}"
