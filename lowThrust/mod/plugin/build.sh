#!/usr/bin/env bash
# Build LowThrustWarp.dll against the farm's base game and install it into
# base/ and every farm clone.  The live install: copy the dll to
# "<KSP>/GameData/LowThrustWarp/" by hand.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
B="$HERE/../../testInstances/base"
M="$B/KSP_x64_Data/Managed"
mcs -target:library -out:"$HERE/LowThrustWarp.dll" \
    -r:"$M/Assembly-CSharp.dll" -r:"$M/UnityEngine.CoreModule.dll" \
    -r:"$M/UnityEngine.dll" -r:"$M/UnityEngine.UI.dll" -r:"$M/UnityEngine.PhysicsModule.dll" \
    -r:"$M/UnityEngine.IMGUIModule.dll" -r:"$M/UnityEngine.InputLegacyModule.dll" -r:"$M/UnityEngine.TextRenderingModule.dll" -r:"$M/UnityEngine.AnimationModule.dll" \
    -r:"$B/GameData/kRPC/KRPC.Core.dll" -r:"$B/GameData/kRPC/KRPC.SpaceCenter.dll" \
    "$HERE"/*.cs
for d in "$B" "$HERE"/../../testInstances/ksp[0-9]*; do
  [ -d "$d/GameData" ] || continue
  mkdir -p "$d/GameData/LowThrustWarp"
  cp -f --remove-destination "$HERE/LowThrustWarp.dll" "$d/GameData/LowThrustWarp/"
done
