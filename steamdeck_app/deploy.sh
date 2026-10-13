#!/usr/bin/env bash
#
# Builds (only when needed) and packages the Steam Deck app into one file,
# dist/spotmicro-deck-setup.sh, and optionally installs it on a Deck.
#
#     ./deploy.sh                    package (rebuild only if src/ changed)
#     ./deploy.sh --push [deck@ip]   ...then copy it to the Deck and install it there
#     ./deploy.sh --rebuild          force a rebuild first
#
# The setup file is self-contained (binary + installer). Run it on any Deck
# with no build tools at all, e.g. after a SteamOS reinstall:
#     bash spotmicro-deck-setup.sh [robot-ip]
#
# The cross-build links against the Deck's own libSDL2. It's fetched from the
# Deck once (over ssh) and cached in ~/.cache/spotmicro-deck/, so later builds
# don't need the Deck at all.
#
# Env: DECK (default deck@192.168.1.192), ROBOT_HOST (default 192.168.1.185).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

DECK="${DECK:-deck@192.168.1.192}"
ROBOT_HOST="${ROBOT_HOST:-192.168.1.185}"
PUSH=0
REBUILD=0
while [ $# -gt 0 ]; do
    case "$1" in
        --push) PUSH=1; if [ $# -gt 1 ] && [[ "$2" != --* ]]; then DECK="$2"; shift; fi ;;
        --rebuild) REBUILD=1 ;;
        -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
        *) echo "unknown option $1 (see --help)" >&2; exit 1 ;;
    esac
    shift
done

BIN=target/x86_64-unknown-linux-gnu/release/spotmicro-deck
CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/spotmicro-deck"
SDL_LIB="$CACHE/libSDL2-2.0.so.0"
SSH_OPTS=(-o ConnectTimeout=8)

# ── build, if anything that goes into the binary is newer than it ────────────
stale() {
    [ "$REBUILD" = 1 ] || [ ! -f "$BIN" ] && return 0
    [ -n "$(find src assets Cargo.toml Cargo.lock -newer "$BIN" -print -quit)" ]
}
if stale; then
    if [ ! -f "$SDL_LIB" ]; then
        echo "==> Fetching the Deck's libSDL2 from $DECK (once; cached in $CACHE)"
        mkdir -p "$CACHE"
        ssh "${SSH_OPTS[@]}" "$DECK" 'cat /usr/lib/libSDL2-2.0.so.0' > "$SDL_LIB.part"
        mv "$SDL_LIB.part" "$SDL_LIB"
    fi
    echo "==> Cross-building for the Deck"
    ./cross_build_x86_64.sh "$SDL_LIB"
else
    echo "==> $BIN is up to date (no changes in src/); not rebuilding"
fi

# ── package: a self-extracting installer ────────────────────────────────────
VERSION="$(git describe --always --dirty 2>/dev/null || echo unknown) $(date -r "$BIN" '+%Y-%m-%d %H:%M')"
mkdir -p dist
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
install -m 755 "$BIN" "$STAGE/spotmicro-deck"
install -m 755 install_on_deck.sh "$STAGE/install_on_deck.sh"
echo "$VERSION" > "$STAGE/VERSION"

OUT=dist/spotmicro-deck-setup.sh
{
    cat <<EOF
#!/usr/bin/env bash
# SpotMicro Steam Deck app installer — build $VERSION
# Run on the Deck (Desktop Mode terminal, or over ssh):  bash $(basename "$OUT") [robot-ip]
# Without a robot address it keeps the one already set (or uses $ROBOT_HOST on a first install).
set -euo pipefail
TMP="\$(mktemp -d)"; trap 'rm -rf "\$TMP"' EXIT
sed '1,/^__PAYLOAD__\$/d' "\$0" | base64 -d | tar -xz -C "\$TMP"
HOST="\${1:-\$(cat "\$HOME/.config/spotmicro-deck/host" 2>/dev/null || echo $ROBOT_HOST)}"
"\$TMP/install_on_deck.sh" "\$HOST"
echo "Build: \$(cat "\$TMP/VERSION")"
exit 0
__PAYLOAD__
EOF
    tar -cz -C "$STAGE" . | base64 -w 76
} > "$OUT.part"
chmod 755 "$OUT.part"
mv "$OUT.part" "$OUT"
sha256sum "$OUT" | sed 's|dist/||' > "$OUT.sha256"
echo "==> $OUT ($(du -h "$OUT" | cut -f1), build $VERSION)"

# ── install on the Deck ──────────────────────────────────────────────────────
if [ "$PUSH" = 1 ]; then
    echo "==> Installing on $DECK"
    scp "${SSH_OPTS[@]}" -q "$OUT" "$DECK:/tmp/spotmicro-deck-setup.sh"
    ssh "${SSH_OPTS[@]}" "$DECK" 'bash /tmp/spotmicro-deck-setup.sh && rm -f /tmp/spotmicro-deck-setup.sh'
fi
