#!/usr/bin/env bash
#
# Run this ON THE STEAM DECK (Desktop Mode, in a terminal) from the folder that
# contains the spotmicro-deck binary:
#
#     ./install_on_deck.sh 192.168.1.185
#
# It copies the binary to ~/spotmicro-deck, remembers the robot's address, and
# adds a launcher so SpotMicro shows up in Steam's "Add a Non-Steam Game" list.
set -euo pipefail

HOST="${1:-}"
BIN="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/spotmicro-deck"
[ -f "$BIN" ] || { echo "spotmicro-deck binary not found next to this script." >&2; exit 1; }
[ -n "$HOST" ] || { echo "usage: $0 <robot-ip-or-hostname>   (e.g. 192.168.1.185)" >&2; exit 1; }

DEST="$HOME/spotmicro-deck"
mkdir -p "$DEST" "$HOME/.config/spotmicro-deck" "$HOME/.local/share/applications"
install -m 755 "$BIN" "$DEST/spotmicro-deck"
echo "$HOST" > "$HOME/.config/spotmicro-deck/host"

cat > "$HOME/.local/share/applications/spotmicro-deck.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=SpotMicro
Comment=Camera, map and controller link to the SpotMicro robot
Exec=$DEST/spotmicro-deck
Icon=applications-games
Terminal=false
Categories=Game;
DESKTOP

echo "Installed to $DEST, robot address set to $HOST."
echo
echo "Next: in Steam (Desktop Mode) choose  Games > Add a Non-Steam Game to My Library,"
echo "tick SpotMicro, then Add Selected Programs. Switch back to Gaming Mode and it's"
echo "in your library. To change the robot address later, edit ~/.config/spotmicro-deck/host"
echo "or put  --host <address>  in the game's Launch Options."
