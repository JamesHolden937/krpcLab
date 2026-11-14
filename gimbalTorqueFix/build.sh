#!/usr/bin/env bash
# Build GimbalTorqueFix.dll (a Harmony patch of ModuleGimbal.GetPotentialTorque;
# see GimbalTorqueFix.cs) against the stock 1.12.5 reference assemblies and
# install it into the farm (base/ and every clone).  --live installs it into the
# live game too (~/KSP-Retooled, or KSP_LIVE).
#
#   gimbalTorqueFix/build.sh [--live]
#
# Output: gimbalTorqueFix/bin/GimbalTorqueFix.dll (git-ignored)
#   -> GameData/GimbalTorqueFix/Plugins/.  Needs 000_Harmony.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HERE/.."
KSP="${KSP:-${KSPFARM:-$HOME/kspfarm}/stockref}"
M="$KSP/KSP_x64_Data/Managed"
BIN="$HERE/bin"
mkdir -p "$BIN"
mcs -optimize -nologo -target:library -nostdlib -noconfig \
    -r:"$M/mscorlib.dll" -r:"$M/System.dll" -r:"$M/System.Core.dll" -out:"$BIN/GimbalTorqueFix.dll" \
    -r:"$M/Assembly-CSharp.dll" -r:"$M/UnityEngine.CoreModule.dll" -r:"$M/UnityEngine.dll" \
    -r:"$KSP/GameData/000_Harmony/0Harmony.dll" "$HERE"/*.cs
install_into() {   # $1: a game directory
    local g="$1/GameData"
    [ -d "$g/000_Harmony" ] || return 0
    mkdir -p "$g/GimbalTorqueFix/Plugins"
    cp -f --remove-destination "$BIN/GimbalTorqueFix.dll" "$g/GimbalTorqueFix/Plugins/"
}
for d in "$ROOT/testInstances/base" "$ROOT"/testInstances/ksp[0-9]*; do install_into "$d"; done
if [ "${1:-}" = "--live" ]; then install_into "${KSP_LIVE:-$HOME/KSP-Retooled}"; fi
echo "built $BIN/GimbalTorqueFix.dll"
