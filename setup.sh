#!/usr/bin/env bash
#
# setup.sh — one-shot dev environment setup for SpotMicroROS2 on a Raspberry Pi
# (Ubuntu LTS, Raspberry Pi 5 assumed for the GPIO/I2C bits).
#
# What this does:
#   1. Installs OS packages needed to build a proper venv, talk to I2C/GPIO
#      hardware, and (optionally) forward Tkinter GUIs over SSH/X11.
#   2. Recreates .venv from scratch with --system-site-packages (so ROS2's
#      rclpy/launch_ros/ament_* are importable once you source a ROS2 setup.bash).
#   3. Installs the project's Python dependencies into .venv.
#   4. Adds your user to the "dialout", "i2c", and "video" groups so
#      PCA9685/servo scripts (Servo_calib.py, test_IK.py) can open
#      /dev/gpiochip4 and /dev/i2c-1, and the usb_cam driver can open the
#      camera's /dev/videoN, all without sudo.
#   5. Installs the sllidar_ros2 udev rule so the RPLIDAR A2M12 always shows
#      up at the stable path /dev/rplidar instead of a shifting /dev/ttyUSBx.
#   6. Installs and enables a systemd service that runs foxglove_bridge
#      (ROS2 WebSocket server for viewing /scan etc. from a browser) on boot.
#   7. Installs ros-jazzy-usb-cam and the spot_micro_camera udev rule so the
#      robot's USB webcam always shows up at the stable path /dev/spotmicro_cam.
#
# Safe to re-run any time.
#
# NOTE: group membership changes only apply to *new* login sessions. After
# running this the first time (or any time it reports a group was added),
# close and reopen your terminal (or run `exec su - "$USER"`) before running
# hardware scripts.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$REPO_DIR/.venv"

log()  { printf '\n\033[1;34m==>\033[0m %s\n' "$1"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$1"; }

NEEDS_NEW_SESSION=0

# ---------------------------------------------------------------------------
# 1. OS packages
# ---------------------------------------------------------------------------
log "Installing required apt packages (sudo)"
sudo apt-get update -qq
sudo apt-get install -y \
    python3.12-venv \
    i2c-tools \
    xauth \
    x11-apps

# ---------------------------------------------------------------------------
# 2. Recreate the virtualenv
# ---------------------------------------------------------------------------
log "Recreating virtualenv at $VENV_DIR (--system-site-packages, so ROS2 packages are visible once sourced)"
rm -rf "$VENV_DIR"
python3 -m venv --system-site-packages "$VENV_DIR"

log "Upgrading pip"
"$VENV_DIR/bin/pip" install --upgrade pip -q

# ---------------------------------------------------------------------------
# 3. Project Python dependencies
# ---------------------------------------------------------------------------
log "Installing project Python dependencies into the venv"
"$VENV_DIR/bin/pip" install -q \
    flask \
    flask-socketio \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-pca9685 \
    adafruit-blinka \
    rpi-lgpio

# ---------------------------------------------------------------------------
# 4. Hardware group permissions (GPIO via gpiochip4, I2C via /dev/i2c-1)
# ---------------------------------------------------------------------------
log "Checking dialout/i2c group membership for $USER"

add_group_if_missing() {
    local group="$1"
    if ! getent group "$group" >/dev/null; then
        warn "Group '$group' does not exist on this system — skipping (are you on a Raspberry Pi?)"
        return
    fi
    if id -nG "$USER" | tr ' ' '\n' | grep -qx "$group"; then
        echo "  already in '$group'"
    else
        sudo usermod -aG "$group" "$USER"
        echo "  added $USER to '$group'"
        NEEDS_NEW_SESSION=1
    fi
}

add_group_if_missing dialout
add_group_if_missing i2c
add_group_if_missing video

# ---------------------------------------------------------------------------
# 5. RPLIDAR A2M12 udev rule (stable /dev/rplidar symlink)
# ---------------------------------------------------------------------------
LIDAR_RULE_SRC="$REPO_DIR/src/sllidar_ros2/scripts/rplidar.rules"
LIDAR_RULE_DST="/etc/udev/rules.d/rplidar.rules"
if [ -f "$LIDAR_RULE_SRC" ]; then
    log "Installing RPLIDAR udev rule ($LIDAR_RULE_DST)"
    if sudo cmp -s "$LIDAR_RULE_SRC" "$LIDAR_RULE_DST" 2>/dev/null; then
        echo "  already installed"
    else
        sudo cp "$LIDAR_RULE_SRC" "$LIDAR_RULE_DST"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "  installed and reloaded udev"
    fi
else
    warn "src/sllidar_ros2 not found — skipping RPLIDAR udev rule"
fi

# ---------------------------------------------------------------------------
# 6. foxglove_bridge systemd service (autostart on boot)
# ---------------------------------------------------------------------------
FOXGLOVE_UNIT_SRC="$REPO_DIR/systemd/foxglove-bridge.service"
FOXGLOVE_UNIT_DST="/etc/systemd/system/foxglove-bridge.service"
if command -v ros2 >/dev/null 2>&1 || [ -d /opt/ros ]; then
    if [ -f "$FOXGLOVE_UNIT_SRC" ]; then
        log "Installing foxglove_bridge systemd service"
        sudo apt-get install -y ros-jazzy-foxglove-bridge

        RENDERED_UNIT="$(mktemp)"
        sed -e "s|__USER__|$USER|g" -e "s|__REPO_DIR__|$REPO_DIR|g" \
            "$FOXGLOVE_UNIT_SRC" > "$RENDERED_UNIT"

        if sudo cmp -s "$RENDERED_UNIT" "$FOXGLOVE_UNIT_DST" 2>/dev/null; then
            echo "  already installed"
        else
            sudo cp "$RENDERED_UNIT" "$FOXGLOVE_UNIT_DST"
            sudo systemctl daemon-reload
            sudo systemctl enable --now foxglove-bridge.service
            echo "  installed, enabled, and started"
        fi
        rm -f "$RENDERED_UNIT"
    else
        warn "systemd/foxglove-bridge.service not found — skipping"
    fi
else
    warn "ROS2 not found at /opt/ros — skipping foxglove_bridge service"
fi

# ---------------------------------------------------------------------------
# 7. USB webcam (Anker PowerConf C200) — usb_cam driver + udev rule
# ---------------------------------------------------------------------------
CAMERA_RULE_SRC="$REPO_DIR/src/spot_micro_camera/udev/spotmicro_cam.rules"
CAMERA_RULE_DST="/etc/udev/rules.d/spotmicro_cam.rules"
if command -v ros2 >/dev/null 2>&1 || [ -d /opt/ros ]; then
    log "Installing usb_cam driver"
    sudo apt-get install -y ros-jazzy-usb-cam
else
    warn "ROS2 not found at /opt/ros — skipping usb_cam driver install"
fi

if [ -f "$CAMERA_RULE_SRC" ]; then
    log "Installing camera udev rule ($CAMERA_RULE_DST)"
    if sudo cmp -s "$CAMERA_RULE_SRC" "$CAMERA_RULE_DST" 2>/dev/null; then
        echo "  already installed"
    else
        sudo cp "$CAMERA_RULE_SRC" "$CAMERA_RULE_DST"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "  installed and reloaded udev"
    fi
else
    warn "src/spot_micro_camera not found — skipping camera udev rule"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
log "Done."
echo "Activate the venv with:"
echo "    source \"$VENV_DIR/bin/activate\""
echo
echo "For ROS2 python packages (rclpy, launch_ros, ...), also source your ROS2 setup, e.g.:"
echo "    source /opt/ros/jazzy/setup.bash"

if [ "$NEEDS_NEW_SESSION" -eq 1 ]; then
    warn "Group membership changed. This only takes effect in a NEW login session."
    echo "  Close and reopen this terminal (or run: exec su - \"$USER\") before running"
    echo "  hardware scripts like Servo_calib.py / test_IK.py, otherwise you'll still"
    echo "  hit 'can not open gpiochip' or I2C permission errors."
fi

cat <<'EOF'

Quick hardware checks once your session is refreshed:
    i2cdetect -y 1          # should show your PCA9685 (default 0x40, plus 0x70 all-call)
    ls -l /dev/rplidar      # should exist once the RPLIDAR A2M12 is plugged in
    ls -l /dev/spotmicro_cam  # should exist once the USB camera is plugged in

Build and launch the RPLIDAR driver:
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select sllidar_ros2
    source install/setup.bash
    ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/rplidar

Build and launch the camera driver:
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select spot_micro_camera
    source install/setup.bash
    ros2 launch spot_micro_camera camera.launch.py

foxglove_bridge runs as a systemd service (autostarts on boot) and serves a
WebSocket at ws://<pi-ip>:8765 — view /scan, the camera feed, etc. by
connecting to it from https://app.foxglove.dev. Restart the service after any
colcon build so it picks up the freshly-built workspace overlay:
    sudo systemctl restart foxglove-bridge

Running GUI scripts (e.g. Servo_calib.py's Tkinter window) over SSH/VSCode
Remote-SSH also needs X11 forwarding set up on your LOCAL machine (this
script only installs the Pi-side xauth/x11-apps pieces) — see the
"Running GUI scripts over SSH" section in README.md for those steps.
EOF
