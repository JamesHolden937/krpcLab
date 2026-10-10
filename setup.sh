#!/usr/bin/env bash
#
# setup.sh -- set up krpcLab against your own KSP install.
#
#   ./setup.sh                          # asks for everything
#   ./setup.sh --ksp "/path/to/Kerbal Space Program"
#   ./setup.sh --ksp PATH --farm 6 -y   # unattended, with a six-instance farm
#   ./setup.sh --ksp PATH --no-farm -y  # just the Python side
#
# What it does, in order:
#   1. finds and checks your KSP install (it is only ever READ, never written)
#      and the mods the reference saves were flown with;
#   2. builds the Python environment (.venv with krpc 0.6.0; .venv-pypy too if
#      a PyPy is unpacked under .pypy/);
#   3. regenerates the derived saves (saves/derived.txt);
#   4. optionally builds the measurement farm: testInstances/base (a stripped
#      ~7.5 GB copy of your install) and N clones (~1.3 GB each), and copies
#      the reference saves into them.
#
# The install path is remembered in .kspPath (git-ignored), so re-running
# setup.sh offers it as the default.  See docs/farmSetup.md for the farm's
# requirements and README.md for what to do next.
set -uo pipefail
cd "$(dirname "$0")"
ROOT="$PWD"

KSP=""; FARM=""; YES=0
while [ $# -gt 0 ]; do
  case "$1" in
    --ksp)     KSP="${2:?--ksp needs a path}"; shift 2 ;;
    --ksp=*)   KSP="${1#--ksp=}"; shift ;;
    --farm)    FARM="${2:?--farm needs a count}"; shift 2 ;;
    --no-farm) FARM=0; shift ;;
    -y|--yes)  YES=1; shift ;;
    -h|--help) sed -n '3,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1 (see --help)" >&2; exit 2 ;;
  esac
done

say()  { printf '%s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'error: %s\n' "$*" >&2; exit 1; }
ask() {  # ask "question" default -> REPLY
  local q="$1" d="${2:-}"
  if [ "$YES" -eq 1 ]; then REPLY="$d"; return; fi
  read -r -p "$q${d:+ [$d]} " REPLY || REPLY=""
  [ -n "$REPLY" ] || REPLY="$d"
}
yes_no() {  # yes_no "question" y|n
  ask "$1 (y/n)" "$2"
  case "$REPLY" in [Yy]*) return 0 ;; *) return 1 ;; esac
}

# --- 1. the KSP install ------------------------------------------------------
if [ -z "$KSP" ]; then
  guess=""
  [ -f .kspPath ] && guess="$(cat .kspPath)"
  if [ -z "$guess" ]; then
    for c in "${KSP_SRC:-}" "$HOME/Kerbal Space Program" \
             "$HOME/.steam/steam/steamapps/common/Kerbal Space Program" \
             "$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program"; do
      [ -n "$c" ] && [ -d "$c/GameData" ] && { guess="$c"; break; }
    done
  fi
  ask "Path to your KSP install (the folder holding GameData):" "$guess"
  KSP="$REPLY"
fi
KSP="${KSP/#\~/$HOME}"
KSP="${KSP%/}"
[ -d "$KSP/GameData" ] || die "no GameData in '$KSP' -- that is not a KSP install"
printf '%s\n' "$KSP" > .kspPath
say "KSP install: $KSP"

windows=0
if [ -f "$KSP/KSP_x64.exe" ]; then
  windows=1
elif [ -f "$KSP/KSP.x86_64" ]; then
  warn "this is the native Linux build. ./run.sh works against it, but the farm"
  warn "is built from the Windows build (KSP_x64.exe, run under Proton)."
else
  warn "no KSP_x64.exe or KSP.x86_64 in '$KSP'"
fi
[ -f "$KSP/readme.txt" ] && say "  $(grep -m1 '^Version' "$KSP/readme.txt")" \
  "(the farm was built from build 03190)"

# kRPC is the one mod nothing works without; the rest change the vehicle.
G="$KSP/GameData"
if [ -d "$G/kRPC" ]; then
  v="$(sed -n '1s/^## \[v\(.*\)\].*/\1/p' "$G/kRPC/CHANGELOG.md" 2>/dev/null)"
  say "  kRPC ${v:-(version unknown)}"
  [ -n "$v" ] && [ "$v" != "0.6.0" ] && \
    warn "kRPC $v: requirements.txt pins the client to 0.6.0 to match the server"
else
  warn "kRPC is not installed. Install kRPC 0.6.0 (CKAN, or"
  warn "https://github.com/krpc/krpc/releases) -- the autopilots talk to it."
fi
missing=()
for m in ReStock AtmosphereAutopilot FullAutoStrut 001_AxisGroupController \
         B9PartSwitch Kopernicus ModularFlightIntegrator KSPCommunityFixes \
         000_Harmony 000_KSPBurst BurstPQS Shabby 000_ClickThroughBlocker \
         001_ToolbarControl PersistentThrust; do
  [ -e "$G/$m" ] || missing+=("$m")
done
ls "$G"/ModuleManager*.dll >/dev/null 2>&1 || missing+=("ModuleManager")
if [ ${#missing[@]} -gt 0 ]; then
  warn "mods the reference saves were flown with are missing: ${missing[*]}"
  warn "they change mass, drag and control surfaces -- see docs/farmSetup.md"
fi

# --- 2. Python ---------------------------------------------------------------
command -v python3 >/dev/null || die "python3 is required"
if [ ! -x .venv/bin/python ]; then
  say "creating .venv"
  python3 -m venv .venv || die "could not create .venv"
fi
if ! .venv/bin/python -c "import krpc" 2>/dev/null; then
  say "installing requirements into .venv"
  .venv/bin/pip install --quiet -r requirements.txt || die "pip install failed"
fi
if ls -d .pypy/pypy3*/bin/pypy3 >/dev/null 2>&1; then
  say "PyPy found under .pypy/: building .venv-pypy"
  kspSim/tools/pypysetup.sh >/dev/null || warn "pypysetup.sh failed (CPython still works)"
else
  say "PyPy: not set up (optional, ~7x faster tests: see kspSim/tools/pypysetup.sh)"
fi

# --- 3. derived saves ----------------------------------------------------------
testInstances/syncSaves.sh check >/dev/null 2>&1 || true
say "saves: $(ls saves/*.sfs | wc -l) reference quicksaves in saves/"

# --- 4. the farm (optional) ----------------------------------------------------
if [ -z "$FARM" ]; then
  ram=$(awk '/MemTotal/ {print int($2 / 1048576)}' /proc/meminfo)
  suggest=$(( ram / 5 )); [ "$suggest" -gt 6 ] && suggest=6; [ "$suggest" -lt 1 ] && suggest=1
  say ""
  say "The measurement farm runs headless copies of KSP in parallel (~4 GB RAM"
  say "each, ~7.5 GB of disk for the base plus ~1.3 GB per instance)."
  say "You do not need it to fly an autopilot in your own game (./run.sh)."
  if yes_no "Build the farm now?" n; then
    ask "How many instances? (this machine: ${ram} GB RAM)" "$suggest"
    FARM="$REPLY"
  else
    FARM=0
  fi
fi
case "$FARM" in ''|*[!0-9]*) die "--farm needs a number, got '$FARM'" ;; esac

if [ "$FARM" -gt 0 ]; then
  [ "$windows" -eq 1 ] || die "the farm needs the Windows build (KSP_x64.exe)"
  need=(tar mcs umu-run kwin_wayland ss systemd-inhibit)
  lack=(); for t in "${need[@]}"; do command -v "$t" >/dev/null || lack+=("$t"); done
  [ ${#lack[@]} -eq 0 ] || die "the farm needs: ${lack[*]} (see docs/farmSetup.md)"
  free_gb=$(df -BG --output=avail testInstances | tail -1 | tr -dc 0-9)
  want_gb=$(( 8 + 2 * FARM ))
  [ "$free_gb" -ge "$want_gb" ] || \
    die "about ${want_gb} GB free needed under testInstances/, ${free_gb} GB available"
  if [ -e testInstances/base ]; then
    say "testInstances/base exists: keeping it (rm -rf it to rebuild from $KSP)"
  else
    say "building testInstances/base from $KSP (reads only; takes a few minutes)"
    KSP_SRC="$KSP" testInstances/mkbase.sh || die "mkbase.sh failed"
  fi
  for (( n=0; n<FARM; n++ )); do
    if [ -d "testInstances/ksp$n" ]; then
      say "testInstances/ksp$n exists: keeping it"
    else
      testInstances/mkclone.sh "$n" || die "mkclone.sh $n failed"
    fi
  done
  testInstances/syncSaves.sh push >/dev/null && testInstances/syncSaves.sh check
fi

say ""
say "Done.  Next:"
say "  ./run.sh                    # in-game launcher (start KSP with kRPC first)"
say "  .venv/bin/python -m unittest   # offline tests, no KSP needed"
[ "$FARM" -gt 0 ] && say "  cd testInstances && ./start.sh   # bring the farm up (docs/farmSetup.md)"
exit 0
