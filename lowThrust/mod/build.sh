#!/usr/bin/env bash
# Build the lowThrust mod and its tools, and install the mod.
#
#   lowThrust/mod/build.sh          # build; install into the farm (base/ and
#                                   # every clone) and the live game
#   lowThrust/mod/build.sh test     # build, then run the tests
#
# Outputs (lowThrust/mod/bin/, git-ignored):
#   LowThrust.dll      the mod: the solver and the in-game autopilot
#                      -> GameData/LowThrust/Plugins/  (no other mod needed)
#   LowThrustKRPC.dll  the farm's kRPC bridge (needs kRPC) -> farm only
#   lowthrust.exe      the command-line tools (mono): mission, fly
#   tests.exe          the solver's tests (mono)
# The game's assemblies come from the farm's base copy (LT_KSP to override).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HERE/../.."
KSP="${LT_KSP:-$ROOT/testInstances/base}"
M="$KSP/KSP_x64_Data/Managed"
BIN="$HERE/bin"
mkdir -p "$BIN"
mcs -optimize -nologo -out:"$BIN/lowthrust.exe" -r:System.Core.dll "$HERE"/core/*.cs "$HERE"/tools/Tools.cs
mcs -optimize -nologo -out:"$BIN/tests.exe" -r:System.Core.dll "$HERE"/core/*.cs "$HERE"/tests/Tests.cs
if [ -d "$M" ]; then
  mcs -optimize -nologo -target:library -out:"$BIN/LowThrust.dll" -r:System.Core.dll \
      -r:"$M/Assembly-CSharp.dll" -r:"$M/Assembly-CSharp-firstpass.dll" -r:"$M/UnityEngine.CoreModule.dll" \
      -r:"$M/UnityEngine.dll" -r:"$M/UnityEngine.UI.dll" -r:"$M/UnityEngine.PhysicsModule.dll" \
      -r:"$M/UnityEngine.IMGUIModule.dll" -r:"$M/UnityEngine.InputLegacyModule.dll" \
      -r:"$M/UnityEngine.TextRenderingModule.dll" -r:"$M/UnityEngine.AnimationModule.dll" \
      "$HERE"/core/*.cs "$HERE"/plugin/*.cs
  mcs -optimize -nologo -target:library -out:"$BIN/LowThrustKRPC.dll" -r:"$BIN/LowThrust.dll" \
      -r:"$M/Assembly-CSharp.dll" -r:"$M/UnityEngine.CoreModule.dll" \
      -r:"$KSP/GameData/kRPC/KRPC.Core.dll" "$HERE"/bridge/Bridge.cs
  install_into() {   # $1: a game directory; $2: "farm" to add the bridge
    local g="$1/GameData"
    [ -d "$g" ] || return 0
    rm -rf "$g/LowThrustWarp"
    mkdir -p "$g/LowThrust/Plugins"
    cp -f --remove-destination "$BIN/LowThrust.dll" "$g/LowThrust/Plugins/"
    if [ "${2:-}" = farm ]; then cp -f --remove-destination "$BIN/LowThrustKRPC.dll" "$g/LowThrust/Plugins/"; fi
  }
  for d in "$ROOT/testInstances/base" "$ROOT"/testInstances/ksp[0-9]*; do install_into "$d" farm; done
  install_into "${LT_LIVE:-$HOME/Kerbal Space Program}"
else
  echo "no KSP assemblies at $M: built the tools only" >&2
fi
if [ "${1:-}" = test ]; then mono "$BIN/tests.exe"; fi
