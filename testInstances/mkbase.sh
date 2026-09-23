#!/usr/bin/env bash
#
# mkbase.sh -- build testInstances/base, a stripped copy of the live KSP
# install for unattended measurement flights.
#
# The live install is only ever READ.  Everything is written under
# testInstances/.  Graphics mods listed in strip.txt are never copied, so
# this is a ~7 GB copy rather than a 14 GB one.
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${KSP_SRC:-$HOME/Kerbal Space Program}"
BASE="$HERE/base"

[ -f "$SRC/KSP_x64.exe" ] || { echo "no KSP at: $SRC" >&2; exit 1; }
command -v tar >/dev/null || { echo "tar required" >&2; exit 1; }

if [ -e "$BASE" ]; then
  echo "base/ already exists -- remove it first if you mean to rebuild:"
  echo "  rm -rf '$BASE'"
  exit 1
fi

echo "source : $SRC"
echo "base   : $BASE"

# --- what never gets copied -----------------------------------------------
# tar matches these against archive member names, which start "./".
EX=()
while read -r m; do
  case "$m" in ''|\#*) continue ;; esac
  EX+=( --exclude="./GameData/$m" )
done < "$HERE/strip.txt"
EX+=( --exclude="./GameData/Blackrack_TUFX.cfg" )   # the TUFX profile, orphaned

# Regenerable caches: leaving them would make KSP trust a part database built
# against a different GameData.  Deleting them costs one slow first boot.
EX+=( --exclude="./GameData/ModuleManager.ConfigCache"
      --exclude="./GameData/ModuleManager.ConfigSHA"
      --exclude="./GameData/ModuleManager.Physics"
      --exclude="./GameData/ModuleManager.TechTree"
      --exclude="./PartDatabase.cfg" )

# Bulk we do not need, and the previous session's own output.
EX+=( --exclude="./Screenshots" --exclude="./thumbs" --exclude="./Replays"
      --exclude="./Logs" --exclude="./KSP.log" --exclude="./CKAN"
      --exclude="./DisabledMods" --exclude="./saves/*/Backup" )

# --- second round: texture data inside mods that STAY (1.2 GB) ------------
# Nothing here is a part model or a part .cfg.  KSP derives a part's drag cube
# from its MESH, so dropping the surface textures of a kept mod leaves mass and
# drag area untouched -- verified: identical 27.25 t separation mass and `cda`
# within 1% of an unstripped instance from the same quicksave.  It costs ~1400
# `Texture '...' not found` lines per load, which is what those are.
# See keepNotes.md, "Removed, second round".
#
# ReStock: the .mu meshes stay; only the surface textures and the engine-plume
# models go.  Its DRAG_CUBE / @mass patches are what keep the vehicle the same
# vehicle, and they are in Patches/, not here.
EX+=( --exclude="./GameData/ReStock/Assets/*/*.dds"
      --exclude="./GameData/ReStock/FX" )
# Squad/SquadExpansion: agency logos, IVA interiors (flown uncrewed), stock
# plume models, tutorials, mission content, stock craft.  No part touched.
EX+=( --exclude="./GameData/Squad/Agencies"
      --exclude="./GameData/Squad/Spaces"
      --exclude="./GameData/Squad/Tutorials"
      --exclude="./GameData/Squad/FX/*.mu"
      --exclude="./GameData/SquadExpansion/*/Banners"
      --exclude="./GameData/SquadExpansion/*/Missions"
      --exclude="./GameData/SquadExpansion/*/Ships" )
# The regenerable half of the KSPCF cache.  Its SIBLINGS -- the answered
# first-run prompts in the same PluginData directory -- must NOT be excluded;
# deleting those is the silent loading-screen hang documented below.
EX+=( --exclude="./GameData/KSPCommunityFixes/PluginData/TextureCache" )

mkdir -p "$BASE"
tar -C "$SRC" -cf - "${EX[@]}" . | tar -C "$BASE" -xf -

# --- mod-local caches -----------------------------------------------------
# GameData/*/PluginData is deliberately COPIED, not wiped.  It holds the
# answers to mods' first-run prompts, and KSPCommunityFixes asks about its
# loading optimisations (Making History parts, IVA, the PNG texture cache)
# with a modal dialog *during the loading screen*.  With those answers gone
# the loader sits on a dialog that an unattended, 640x360, off-screen
# instance can never click -- which looks exactly like a hang at
# "Preloading Asset Bundle Definitions", and cost most of a session to find.
# The regenerable part (KSPCommunityFixes/PluginData/TextureCache) is NOT
# copied any more -- the second-round strip deleted most of the textures it
# cached, so it would be a cache of files that are gone.  It rebuilds itself;
# the stripped base still reaches kRPC in 39 s.

# --- the auto-load plugin -------------------------------------------------
# kRPC stops its servers outside a game scene, and the main menu is one, so an
# unattended instance has to load a save by itself before anything can connect.
# See README.md, "Why there is a plugin in here".
if [ -f "$HERE/autoloadSrc/AutoLoadSave.cs" ]; then
  M="$BASE/KSP_x64_Data/Managed"
  if command -v mcs >/dev/null; then
    mcs -target:library -out:"$HERE/autoloadSrc/BoosterlandAutoLoad.dll" \
        -r:"$M/Assembly-CSharp.dll" -r:"$M/UnityEngine.dll" \
        -r:"$M/UnityEngine.CoreModule.dll" \
        "$HERE/autoloadSrc/AutoLoadSave.cs" || {
          echo "plugin build failed" >&2; exit 1; }
  fi
  if [ -f "$HERE/autoloadSrc/BoosterlandAutoLoad.dll" ]; then
    D="$BASE/GameData/BoosterlandAutoLoad"
    mkdir -p "$D"
    cp -f "$HERE/autoloadSrc/BoosterlandAutoLoad.dll" "$D/"
    cp -f "$HERE/autoloadSrc/autoload.cfg" "$D/"
    echo "auto-load plugin installed"
  else
    echo "WARNING: no BoosterlandAutoLoad.dll and no mcs -- instances will" >&2
    echo "         sit at the main menu and never open their kRPC port." >&2
  fi
fi

# --- the time-scale plugin ------------------------------------------------
# Runs a flight faster than real time WITHOUT touching Time.fixedDeltaTime, so
# every physics step is the step the vehicle flies at 1x.  KSP's own physics
# warp does the opposite -- it multiplies fixedDeltaTime to hold CPU cost flat,
# which is an integration-fidelity loss exactly where this vehicle lives.
# Off by default; ../tools/timescale.py turns it on per instance.  See
# keepNotes.md, "Flying faster than real time".
if [ -f "$HERE/timescaleSrc/TimeScale.cs" ]; then
  M="$BASE/KSP_x64_Data/Managed"
  if command -v mcs >/dev/null; then
    mcs -target:library -out:"$HERE/timescaleSrc/BoosterlandTimeScale.dll" \
        -r:"$M/Assembly-CSharp.dll" -r:"$M/UnityEngine.dll" \
        -r:"$M/UnityEngine.CoreModule.dll" \
        "$HERE/timescaleSrc/TimeScale.cs" || {
          echo "timescale plugin build failed" >&2; exit 1; }
  fi
  if [ -f "$HERE/timescaleSrc/BoosterlandTimeScale.dll" ]; then
    D="$BASE/GameData/BoosterlandTimeScale"
    mkdir -p "$D"
    cp -f "$HERE/timescaleSrc/BoosterlandTimeScale.dll" "$D/"
    # The control file lives at the instance root, not in GameData: it is
    # per-instance state, and mkclone.sh gives each clone its own copy of
    # everything writable there.
    printf 'mode = off\nscale = 1.0\nmax_scale = 8.0\nquant_s = 0.05\n' \
        > "$BASE/timescale.txt"
    echo "time-scale plugin installed (off by default)"
  fi
fi

# --- frame rate: vsync off, and only as many frames as the speedup needs ---
# SYNC_VBL pins the instance to the virtual output's 60 Hz, and that pins the
# achievable time scale: Unity applies control input once a frame, so the
# autopilot's commands land on a grid of `timeScale / fps`.  At 60 fps and a
# 0.05 s quantum the ceiling is 3x; with vsync off and 90 fps it is 4.2x.
#
# **More frames is not better.**  Measured on the booster: 196 fps reached
# 3.38x and 89 fps reached 4.22x in the same wall-clock time, because
# rendering and physics share the one main thread and KSP's physics is
# single-threaded.  The optimum is the LOWEST frame rate that still meets the
# quantum, `fps ~= target_speed / quant_s`, not the highest the machine can
# draw.  Nothing reads these pixels.
sed -i 's/^SYNC_VBL = .*/SYNC_VBL = 0/; s/^FRAMERATE_LIMIT = .*/FRAMERATE_LIMIT = 90/' \
    "$BASE/settings.cfg"

# --- kRPC: start the server without being asked ---------------------------
K="$BASE/GameData/kRPC/PluginData/settings.cfg"
sed -i 's/^\(\s*autoStartServers\s*=\s*\).*/\1True/' "$K"
grep -q "autoStartServers = True" "$K" || { echo "kRPC autostart not set" >&2; exit 1; }

# --- graphics settings: small window, cheap frame, physics untouched -------
S="$BASE/settings.cfg"
set_key() { sed -i "s/^\(\s*$1\s*=\s*\).*/\1$2/" "$S"; }
# Kept in step with kwinRun.sh's RES, though the command line it passes wins
# over these: an instance started some other way should still come up small.
set_key SCREEN_RESOLUTION_WIDTH   160
set_key SCREEN_RESOLUTION_HEIGHT  100
set_key FULLSCREEN                False
set_key QUALITY_PRESET            0
set_key ANTI_ALIASING             0
# **VRAM is the constraint, and it was measured rather than assumed.**  This
# line used to read "dropping it only saves VRAM, which is not the constraint
# on a 16 GB card".  Four instances at full-res textures were measured at
# 4673 + 4685 + 2879 + 2659 MiB -- 14.9 GB of a 16.3 GB card, leaving ~700 MB
# for the whole desktop.  That is not a comfort problem: an exhausted card
# loses GL contexts, and a lost context takes down *every* GL client at once,
# which is the "all four instances died together" entry in
# docs/testInstances.md and the browser core dump that was mistaken for its
# cause.  The desktop going glitchy whenever the farm runs is the same
# shortage seen from the other side.
#
# Quarter-res costs nothing that is measured here.  Nothing in this project
# reads a pixel: the flight software works from kRPC telemetry and
# simulate_aerodynamic_force_at, and texture resolution is a load-time mipmap
# choice that touches no mesh, no DRAG_CUBE and no mass -- which is the thing
# ReStock is kept for.  It does invalidate the KSPCommunityFixes PNG cache,
# so the first boot after changing it rebuilds ~370 MB per instance, once.
set_key TEXTURE_QUALITY           "${TEXTURE_QUALITY:-3}"
set_key SHADOWS_QUALITY           0
set_key LIGHT_QUALITY             8
set_key CELESTIAL_BODIES_CAST_SHADOWS False
set_key PLANET_SCATTER            False
set_key PLANET_SCATTER_FACTOR     0
# 60 fps is a cap, not a target.  KSP runs at most
# PHYSICS_FRAME_DT_LIMIT / fixedDeltaTime = 0.04 / 0.02 = 2 physics steps per
# rendered frame, so real-time physics needs >= 25 fps.  60 leaves 2.4x
# headroom and stops the GPU rendering hundreds of frames nobody looks at.
# PHYSICS_FRAME_DT_LIMIT itself is NOT touched: it sets the timestep.
set_key FRAMERATE_LIMIT           60

echo
echo "base built: $(du -sh "$BASE" | cut -f1)"
echo "  GameData: $(du -sh "$BASE/GameData" | cut -f1)  ($(ls "$BASE/GameData" | wc -l) entries)"
echo "next: ./mkclone.sh <n>"
