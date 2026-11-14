#!/usr/bin/env bash
# krpcmain.sh [--client] [--no-live] -- kRPC built from source at a pinned commit
# of krpc/krpc main (unreleased 0.8.0) and installed everywhere krpcLab flies:
#
#   server  ~/kspfarm/{base,stockref,krpclab/ksp*} and the live game (~/KSP-Retooled,
#           or KSP_LIVE; --no-live skips it): GameData/kRPC replaced, each install's
#           PluginData (its port) kept, 0.7 settings keys migrated to 0.8's
#           tcpip_address/tcpip_port.  The first install's old kRPC goes to .krpc070/.
#   client  the wheel into .venv and .venv-pypy (whichever exist).  --client: only this.
#
# Why main and not a release: kRPC's gimbal-lag tuning (krpc/krpc#1348) is only there.
# Switch to the 0.8.0 release once it ships (pip install krpc==0.8.0 and the mod zip).
# Needs bazel (bazelisk in ~/.local/bin); the build is hermetic, ~4 min the first time.
# Games must be stopped.  Source: KRPC_SRC (default ~/krpc, cloned if missing).
set -euo pipefail
COMMIT=34c70de        # krpc/krpc main, 2026-11-14 (#1352); #1348 is 2eca3b4
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${KRPC_SRC:-$HOME/krpc}"
FARM="${KSPFARM:-$HOME/kspfarm}"
LIVE="${KSP_LIVE:-$HOME/KSP-Retooled}"
client_only=0; live=1
for a in "$@"; do case "$a" in --client) client_only=1 ;; --no-live) live=0 ;;
    *) echo "unknown argument $a" >&2; exit 2 ;; esac; done

echo "kRPC main $COMMIT from $SRC"
[ -d "$SRC/.git" ] || git clone -q https://github.com/krpc/krpc.git "$SRC"
git -C "$SRC" fetch -q origin main
git -C "$SRC" checkout -q "$COMMIT"
(cd "$SRC" && bazel build //:krpc //client/python 2>&1 | tail -1)
WHEEL="$(ls "$SRC"/bazel-bin/client/python/krpc-*.whl)"
ZIP="$(ls "$SRC"/bazel-bin/krpc-*.zip)"

for v in .venv .venv-pypy; do
    [ -x "$ROOT/$v/bin/pip" ] || continue
    "$ROOT/$v/bin/pip" install -q --force-reinstall "$WHEEL"
    echo "client: $v $("$ROOT/$v/bin/python" -c 'import krpc; print(krpc.__version__)')"
done
[ "$client_only" = 1 ] && exit 0

if pgrep -x 'Unity Main Thre' >/dev/null; then echo "a game is running: stop it first" >&2; exit 1; fi
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
unzip -q "$ZIP" -d "$TMP"
games=("$FARM/base" "$FARM/stockref" "$FARM"/krpclab/ksp[0-9]*)
[ "$live" = 1 ] && games+=("$LIVE")
for d in "${games[@]}"; do
    g="$d/GameData"; [ -d "$g/kRPC" ] || continue
    n="$(basename "$d")"
    if grep -q '"MINOR": 7,' "$g/kRPC/kRPC.version" && [ ! -d "$ROOT/.krpc070/$n" ]; then
        mkdir -p "$ROOT/.krpc070"; cp -a "$g/kRPC" "$ROOT/.krpc070/$n"
    fi
    cp -a "$g/kRPC/PluginData" "$TMP/pd"
    rm -rf "$g/kRPC"; cp -a "$TMP/GameData/kRPC" "$g/kRPC"
    rm -rf "$g/kRPC/PluginData"; mv "$TMP/pd" "$g/kRPC/PluginData"
    cfg="$g/kRPC/PluginData/settings.cfg"
    # 0.7's per-server keys -> 0.8's (one port; stream_port dropped)
    python3 -I - "$cfg" <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("key = address\n", "key = tcpip_address\n").replace("key = rpc_port\n", "key = tcpip_port\n")
s = re.sub(r"\t*Item\n\t*\{\n\t*key = stream_port\n\t*value = \d+\n\t*\}\n", "", s)
s = re.sub(r"\t*oneRPCPerUpdate = \w+\n", "", s)
open(p, "w").write(s)
PY
    echo "server: $n port $(grep -A1 'key = tcpip_port' "$cfg" | grep -o '[0-9]\+' | head -1)"
done
"$ROOT/launcherMod/build.sh"
