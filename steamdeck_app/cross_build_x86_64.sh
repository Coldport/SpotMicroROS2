#!/usr/bin/env bash
#
# Cross-compiles the Steam Deck app for x86_64 Linux from any Linux machine
# (e.g. the robot's Raspberry Pi), without installing a compiler system-wide
# and without touching the Deck. SteamOS has no C compiler or SDL2 headers, and
# a Pi builds aarch64 binaries by default, so this is the easiest way to get a
# binary the Deck can run.
#
# It downloads Zig (a self-contained C toolchain — checksum-verified against
# ziglang.org and kept in ~/.cache/spotmicro-zig) and uses it only as the
# linker, linking against a copy of the Deck's own libSDL2.
#
# One-time: get the Deck's SDL2 library (the Deck must be reachable by ssh):
#     ssh deck@<deck-ip> 'cat /usr/lib/libSDL2-2.0.so.0' > libSDL2-2.0.so.0
# Then:
#     ./cross_build_x86_64.sh ./libSDL2-2.0.so.0
# Output: target/x86_64-unknown-linux-gnu/release/spotmicro-deck
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

SDL_LIB="${1:-${SDL2_LIB:-}}"
[ -f "$SDL_LIB" ] || { sed -n '12,16p' "$0" >&2; echo "usage: $0 <path-to-x86_64-libSDL2-2.0.so.0>" >&2; exit 1; }

[ -f "$HOME/.cargo/env" ] && source "$HOME/.cargo/env"
command -v cargo >/dev/null || { echo "Rust isn't installed (https://rustup.rs)." >&2; exit 1; }
rustup target add x86_64-unknown-linux-gnu >/dev/null 2>&1 || true
command -v readelf >/dev/null || { echo "readelf (binutils) is required." >&2; exit 1; }
[ "$(readelf -h "$SDL_LIB" | awk '/Machine:/ {print $2}')" = "Advanced" ] || {
    echo "$SDL_LIB is not an x86_64 library." >&2; exit 1; }

CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/spotmicro-zig"
mkdir -p "$CACHE"
case "$(uname -m)" in
    x86_64) ZIG_ARCH=x86_64 ;;
    aarch64|arm64) ZIG_ARCH=aarch64 ;;
    *) echo "Unsupported build host $(uname -m)." >&2; exit 1 ;;
esac

if ! ls "$CACHE"/zig-*/zig >/dev/null 2>&1; then
    echo "==> Downloading Zig ($ZIG_ARCH-linux)"
    curl -fsSL https://ziglang.org/download/index.json -o "$CACHE/index.json"
    read -r ZIG_URL ZIG_SHA < <(ZIG_INDEX="$CACHE/index.json" python3 - "$ZIG_ARCH" <<'PY'
import json, os, sys
idx = json.load(open(os.environ['ZIG_INDEX']))
versions = [k for k in idx if k != 'master']
newest = max(versions, key=lambda v: tuple(int(x) for x in v.split('.')))
entry = idx[newest][f'{sys.argv[1]}-linux']
print(entry['tarball'], entry['shasum'])
PY
    )
    curl -fsSL -o "$CACHE/zig.tar.xz" "$ZIG_URL"
    echo "$ZIG_SHA  $CACHE/zig.tar.xz" | sha256sum -c - >/dev/null || { echo "Zig checksum mismatch!" >&2; rm -f "$CACHE/zig.tar.xz"; exit 1; }
    tar -xf "$CACHE/zig.tar.xz" -C "$CACHE"
    rm -f "$CACHE/zig.tar.xz"
fi
ZIG="$(ls -d "$CACHE"/zig-*/ | head -1)zig"

# Target the oldest glibc the Deck's SDL2 itself needs, so the binary runs anywhere that library does.
GLIBC="$(readelf -V "$SDL_LIB" | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sed 's/GLIBC_//' | sort -uV | tail -1)"
GLIBC="${GLIBC:-2.17}"
echo "==> Linking against $(basename "$SDL_LIB"), targeting glibc $GLIBC"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/sdl"
cp "$SDL_LIB" "$WORK/sdl/libSDL2-2.0.so.0"
ln -s libSDL2-2.0.so.0 "$WORK/sdl/libSDL2.so"
cat > "$WORK/zigcc" <<EOF
#!/bin/sh
exec "$ZIG" cc -target x86_64-linux-gnu.$GLIBC "\$@"
EOF
chmod +x "$WORK/zigcc"

export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER="$WORK/zigcc"
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS="-L $WORK/sdl"
export ZIG_LOCAL_CACHE_DIR="$CACHE/cache" ZIG_GLOBAL_CACHE_DIR="$CACHE/cache"

cargo build --release --target x86_64-unknown-linux-gnu

OUT="target/x86_64-unknown-linux-gnu/release/spotmicro-deck"
echo
echo "Built: $(pwd)/$OUT"
readelf -h "$OUT" | grep -E 'Machine'
echo "Copy it (and install_on_deck.sh) to the Deck, then run ./install_on_deck.sh <robot-ip>"
