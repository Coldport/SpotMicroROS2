#!/usr/bin/env bash
#
# Builds the Steam Deck app natively (for whatever machine you run this on).
# The Deck needs an x86_64 build: run this on an x86_64 Linux PC (or in Desktop
# Mode on the Deck itself), or let the GitHub workflow build it — see README.md.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

[ -f "$HOME/.cargo/env" ] && source "$HOME/.cargo/env"
if ! command -v cargo >/dev/null 2>&1; then
    echo "Rust isn't installed. Install it with:  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh" >&2
    exit 1
fi
if ! pkg-config --exists sdl2 2>/dev/null && [ ! -e /usr/lib/libSDL2.so ]; then
    echo "SDL2 development files not found. On Debian/Ubuntu:  sudo apt install libsdl2-dev" >&2
    exit 1
fi

cargo test --release
cargo build --release
echo
echo "Built: $(pwd)/target/release/spotmicro-deck  ($(uname -m))"
