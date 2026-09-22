# SpotMicroROS2

## Setup

Run the setup script to prepare the dev environment (venv, Python deps, GPIO/I2C group permissions):

```
./setup.sh
```

Re-run it any time — it's safe to run repeatedly. See the script's output for any manual follow-up (e.g. it will tell you to open a new terminal session if it had to change your group membership).

## Running GUI scripts (Tkinter) over SSH / VSCode Remote-SSH

Scripts like `Test _and_calib/Servo_calib.py` open a Tkinter window. If you run them over SSH (including VSCode's Remote-SSH integrated terminal) without X11 forwarding, you'll hit:

```
_tkinter.TclError: no display name and no $DISPLAY environment variable
```

`setup.sh` installs the Pi-side pieces (`xauth`, `x11-apps`), but X11 forwarding also requires configuration on your **local** machine — the one you're SSHing/connecting from:

1. **Run an X server locally.**
   - Linux: already built in, nothing to install.
   - macOS: install and launch [XQuartz](https://www.xquartz.org/).
   - Windows: install and run [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (or use WSLg if applicable), and make sure it's set to allow connections.

2. **Enable X11 forwarding in your local SSH config.** In VSCode: `Ctrl/Cmd+Shift+P` → "Remote-SSH: Open SSH Configuration File" → pick the config file used to connect to the Pi (usually `~/.ssh/config` on macOS/Linux, `C:\Users\<you>\.ssh\config` on Windows). Add these two lines inside the existing `Host` block for the Pi (don't create a duplicate `Host` block):

   ```
   Host SpotMicro
       HostName 192.168.x.x
       User r
       ForwardX11 yes
       ForwardX11Trusted yes
   ```

3. **Force a clean reconnect.** In VSCode: `Ctrl/Cmd+Shift+P` → "Remote-SSH: Kill VS Code Server on Host..." → select the Pi's host, then reconnect.

4. **Verify** in a fresh remote terminal:

   ```
   echo $DISPLAY
   ```

   It should print something like `localhost:10.0`. If it's empty, X11 forwarding isn't active yet — double check steps 1–3.

Once `$DISPLAY` is set, GUI scripts will render normally.

## LIDAR (RPLIDAR A2M12)

`src/sllidar_ros2` is Slamtec's official ROS2 driver, vendored into this workspace as source (it isn't shipped as a prebuilt apt package). It supports RPLIDAR A1/A2/A3/S1/S2/S3/C1/T1; the A2M12 is the one wired up here, at its default baud rate of **256000**.

`./setup.sh` installs the driver's udev rule so the lidar always appears at the stable path `/dev/rplidar`, regardless of which `/dev/ttyUSBx` it happens to enumerate as.

Build and launch:

```
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sllidar_ros2
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/rplidar
```

Sanity checks:

```
ls -l /dev/rplidar          # confirm the udev rule matched the device
ros2 topic echo /scan       # confirm scan data is publishing (in another terminal)
```

### Motor spins then stops / `Can not start scan: 80008002!`

`80008002` is the SDK's `SL_RESULT_OPERATION_TIMEOUT` code. If it shows up right after the motor starts spinning (and the motor then stalls), the Pi's USB ports aren't supplying enough current to the lidar. A Raspberry Pi 5 limits total USB current to **600mA by default** unless it detects a genuine 5V/5A USB-PD supply at boot — which the RPLIDAR A2M12's motor can exceed on its own.

If you're using the official 27W (5V/5A) Pi 5 power supply, force the higher 1.6A limit in `/boot/firmware/config.txt`:

```
usb_max_current_enable=1
```

then reboot. **Only do this with a supply actually rated for 5V/5A** — forcing the limit with a weaker supply can brown out the Pi itself instead of just the lidar. If you're not sure of your supply's rating, use a powered USB hub for the lidar instead.

### Viewing the scan

RViz2 needs real OpenGL/GLX, which plain SSH X11 forwarding generally can't provide (most X servers block indirect GLX) — you'll see errors like `Failed to create an OpenGL context`. Two ways around that:

**Option A — Foxglove in a browser (recommended, no display/VNC setup needed).** [Foxglove](https://foxglove.dev) is a web app that renders scans, TF, etc. entirely client-side over a WebSocket — nothing X11/GLX-related involved.

`./setup.sh` installs `ros-jazzy-foxglove-bridge` and sets it up as a systemd service from the template at [`systemd/foxglove-bridge.service`](systemd/foxglove-bridge.service), so it's always up — it starts on boot and restarts itself if it crashes. It sources both `/opt/ros/jazzy/setup.bash` and this workspace's `install/setup.bash` (if already built), and listens on port `8765`.

```
sudo systemctl status foxglove-bridge     # check it's running
sudo systemctl restart foxglove-bridge    # pick up a fresh colcon build
sudo journalctl -u foxglove-bridge -f     # tail its logs
sudo systemctl disable --now foxglove-bridge   # stop it / stop autostart
```

To view the lidar:

1. On your local machine, open [https://app.foxglove.dev](https://app.foxglove.dev) in Chrome, click **Open connection**, choose **Foxglove WebSocket**, and enter `ws://<pi-ip>:8765` (find the Pi's IP with `hostname -I`).
2. Add a **3D** panel, add the `/scan` topic to it, and set its Fixed Frame to `laser`. Points should appear live as the lidar spins.

Note: the service only bridges whatever ROS2 nodes/topics already exist — it doesn't start the lidar driver itself. Launch `sllidar_ros2` separately (see above) before expecting to see `/scan` in Foxglove.

**Option B — RViz2 via a VNC virtual desktop on the Pi.** Slower (software-rendered) but shows the exact upstream RViz config. One-time setup:
```
sudo apt-get install tigervnc-standalone-server tigervnc-common openbox
vncpasswd                                          # set a VNC password
vncserver :1 -localhost yes -geometry 1280x800     # starts an openbox desktop on :1
```
From your local machine, tunnel and connect:
```
ssh -L 5901:localhost:5901 <user>@<pi-ip>
```
then point any VNC viewer at `localhost:5901`. Once connected, on the Pi:
```
DISPLAY=:1 ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py serial_port:=/dev/rplidar
```
RViz should appear inside the VNC session.

## Camera (Intel RealSense D405 depth camera)

`src/spot_micro_camera` launches the apt-installed `ros-jazzy-realsense2-camera` driver with params
tuned for the robot's [RealSense D405](https://www.intelrealsense.com/depth-camera-d405/),
publishing color and depth as standard ROS2 image topics.

`./setup.sh` installs `ros-jazzy-realsense2-camera` (which pulls in `librealsense2` and the `rs-*`
command-line tools) and a udev rule that makes the camera's `/dev/videoN` nodes and USB device
readable without sudo. Plug the camera into a **USB 3** port (blue) — it's on USB 3.2 in the
tested setup, and Intel recommends USB 3 for the full-rate streams.

Build and launch:

```
source /opt/ros/jazzy/setup.bash
colcon build --packages-select spot_micro_camera
source install/setup.bash
ros2 launch spot_micro_camera camera.launch.py
```

Launch arguments:

- `serial_no:=<serial>` — open a specific camera (see `rs-enumerate-devices -s`); the default
  picks the first RealSense found, which is fine with one camera.
- `pointcloud:=true` — also publish a colored point cloud (about 9% CPU on the Pi 5 at 640x480).
- `colorize_depth:=true` — for *viewing*: turns `depth/image_rect_raw` into a color-mapped `rgb8`
  image (see [Viewing depth](#viewing-depth) below). This replaces the raw millimeter values, so
  leave it off for anything that consumes depth.

Published topics, all under `/spotmicro_cam/d405/`:

| Topic | Type |
|---|---|
| `color/image_raw` (+ `/compressed`) | `sensor_msgs/Image` RGB8 |
| `depth/image_rect_raw` (+ `/compressedDepth`) | `sensor_msgs/Image` 16UC1, millimeters (`rgb8` with `colorize_depth:=true`) |
| `color/camera_info`, `depth/camera_info` | `sensor_msgs/CameraInfo` |
| `depth/color/points` (only with `pointcloud:=true`) | `sensor_msgs/PointCloud2` |

TF frames are prefixed `d405_` (`d405_link`, `d405_color_optical_frame`, ...).

Sanity checks:

```
rs-enumerate-devices -s                                     # should list "RealSense D405" and its serial
ros2 topic hz /spotmicro_cam/d405/color/image_raw/compressed   # ~30 Hz (in another terminal)
```

### Notes on `config/camera_params.yaml`

- **Color comes from the same sensor as depth** on the D405 (there's no separate RGB camera), so
  the color profile is `depth_module.color_profile`, not `rgb_camera.color_profile`. All streams
  share one resolution at a time.
- **Streams are 640x480 @ 30 fps.** The camera also supports 848x480 up to 90 fps and 1280x720 up
  to 30 fps (`rs-enumerate-devices -c`); edit the two `*_profile` values to change it.
- **Infrared streams are explicitly disabled.** Left to its defaults the driver also opens Infra 1/2
  at 848x480, which conflicts with the 640x480 depth/color profile on the shared sensor and
  results in *no frames at all* (`Frames didn't arrived within 5 seconds`). If you enable them, set
  a matching `depth_module.infra_profile`.
- **The point cloud filter is named `pointcloud__neon_` on the Pi** (ARM builds), so the
  `pointcloud.enable` parameter in the driver's own docs/`rs_launch.py` is silently ignored here.
  `camera.launch.py` sets both names.

### Permission errors when running the `rs-*` tools

`rs-enumerate-devices` and friends print a wall of `Mipi device capability could not be grabbed /
Permission denied` errors on the Pi: librealsense probes every `/dev/video*` node, including the
Pi's unrelated video-codec/ISP ones, which are only readable by the `video` group. The camera is
still found and they're harmless, but joining the `video` group (`./setup.sh` does this; takes
effect in a new login session) should silence them. Filter them out meanwhile with
`... 2>&1 | grep -v "Mipi device\|get_dev_capabilities"`.

### Viewing depth

The raw depth image is 16-bit millimeters (`16UC1`). The data itself is fine — checked by reading
a frame directly — but it is not something every viewer renders sensibly (in Foxglove it came out
looking like noise). Two ways around that:

- Launch with `colorize_depth:=true` and point an Image panel at
  `/spotmicro_cam/d405/depth/image_rect_raw` (or its `/compressed` variant). The color range and
  preset are the driver's `colorizer.min_distance` / `max_distance` / `color_scheme` parameters.
- Launch with `pointcloud:=true` and add a **3D** panel on `/spotmicro_cam/d405/depth/color/points`.

Don't subscribe to `depth/image_rect_raw/compressed` while `colorize_depth` is off: that topic uses
the JPEG plugin, which can't encode 16-bit depth, and the driver logs
`[CompressedPublisher]: [16UC1] is not a color format` for it. Use `.../compressedDepth` (or the
raw topic) for raw depth, and `/compressed` only for the colorized image.

### Viewing the feed

Same [Foxglove](https://foxglove.dev) setup used for the lidar — `foxglove-bridge` bridges
whatever ROS2 topics already exist, so just launch `spot_micro_camera` (see above) with the
bridge running:

1. On your local machine, open [https://app.foxglove.dev](https://app.foxglove.dev), click **Open
   connection**, choose **Foxglove WebSocket**, and enter `ws://<pi-ip>:8765`.
2. Add an **Image** panel and set its topic to `/spotmicro_cam/d405/color/image_raw/compressed`
   (for depth see [Viewing depth](#viewing-depth) above).

Note: like the lidar, the foxglove-bridge service only bridges nodes/topics that already exist —
it doesn't launch the camera driver itself.

## Robot bringup and SLAM

`src/spot_micro_bringup` starts the whole sensing stack in one command. It **does not** start the
gait controller or move the robot.

```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch spot_micro_bringup robot.launch.py
```

This runs `robot_state_publisher` (the URDF), the lidar, IMU, D405 camera, a static
`odom → base_link`, and `slam_toolbox`. Each piece can be switched off:
`lidar:=false imu:=false camera:=false slam:=false`. `serial_port:=` overrides the lidar port
(default `/dev/rplidar`).

### TF tree

```
map ─(slam_toolbox)→ odom ─(static identity)→ base_link ─┬→ laser
                                                          ├→ imu_link
                                                          ├→ d405_link → d405_*_optical_frame
                                                          └→ leg links (only with joint states)
```

`base_link` is the body centre. The sensor mounting offsets live in one labelled block of
properties near the top of the "Sensors" section of
`spot_micro_description/urdf/spot_micro.urdf.xacro`; the ones marked `MEASURE` are estimates —
check them against the robot, since they decide where `/scan` and the camera sit relative to the
body. In particular, `lidar_yaw` assumes the scan's 0° points at the robot's front: put an object
directly ahead and confirm it shows up straight ahead in Foxglove.

### No odometry yet

Nothing publishes real odometry until the gait controller runs, so `odom → base_link` is a static
identity and slam_toolbox builds the map from **scan matching alone**. That works for slow, smooth
movement (e.g. carrying the robot around) but will drift or lose track if you move quickly.
`config/slam_toolbox.yaml` sets `minimum_travel_distance`/`minimum_travel_heading` to 0 for this —
with a static odom slam_toolbox would otherwise never process a scan past the first. When the gait
controller is running it publishes the real `odom → base_link`, so launch with `static_odom:=false`
(two publishers of the same transform will fight) and restore larger travel thresholds.

The IMU isn't fused into anything yet. When you want that, `imu_filter_madgwick` (orientation) and
`robot_localization` (EKF) are the usual next steps; see the covariance note in the IMU section.

### Viewing the map

Foxglove: add a **3D** panel, set the display frame to `map`, and enable the `/map` and `/scan`
topics (plus `/tf` to see the frames). Check the map from the terminal with
`ros2 topic echo --once /map --qos-durability transient_local --field info` — `/map` is latched and
only republishes when the map changes (every few seconds), so `ros2 topic hz /map` shows nothing.

To save a map (writes `my_map.pgm` + `my_map.yaml`):

```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/r/my_map'}}"
```

`result=0` in the response means it worked. This needs `ros-jazzy-nav2-map-server` (`setup.sh`
installs it) — slam_toolbox doesn't declare that dependency, and without it the call fails with
`result=255`.

## Steam Deck app

`steamdeck_app/` is a native app (Rust + SDL2 — no Python, no runtime to install) that runs on the
Steam Deck, including in Gaming Mode. It shows the robot's camera as the main display with status
readouts, the SLAM map as a minimap in the lower-left (like a game HUD), and sends the Deck's
controller input back to the robot. **L2 cycles the main view: camera → depth → lidar → map.**

```
Steam Deck app ──MQTT──► Mosquitto broker (on the robot) ◄──► deck_bridge node ◄──► ROS2
```

The Deck has no ROS, so everything goes through an MQTT broker on the robot and the
`spot_micro_deck` package's `deck_bridge` node translates to/from ROS topics. Camera and lidar
subscriptions (and depth colourising) only run while a Deck is connected and looking at that view,
so an idle robot pays nothing for it.

### Robot side

`./setup.sh` installs Mosquitto (opened to the LAN, port 1883) and the Python MQTT library. On
the robot the bridge normally comes up by itself, see "Starting on boot" below. If no bridge is
running, `robot.launch.py` starts one with everything else (`deck:=false` to skip it). Check it's
alive from the robot:

```
mosquitto_sub -h localhost -t 'spotmicro/#' -v | cut -c1-120   # all traffic
ros2 topic echo /spotmicro_deck/joy                            # the Deck's controller input
```

### Starting on boot, and launching from the Deck

`setup.sh` installs and enables `spotmicro-deck.service` (from `systemd/`). On every boot the Pi
starts Mosquitto (already a service), then `deck.launch.py`: the `deck_bridge` plus a small
`stack_launcher` node. It starts **no sensors and nothing that moves**. The workspace is also
sourced from `~/.bashrc`, so new terminals have it.

Once the Deck app is connected, **hold L1 + R1 together for 3 seconds** and `stack_launcher` runs
`ros2 launch spot_micro_bringup robot.launch.py deck:=false`: LiDAR, IMU, camera and SLAM, sensing
only. **Hold them again for 3 seconds to stop it.** The chord is a toggle, and it only stops the
stack it started itself. Notes:

- Releasing either button restarts the 3 s; holding on afterwards does not launch twice.
- It refuses if a `robot.launch.py` is already running (say, one you started by hand), because two
  stacks fight over the LiDAR, IMU and camera. The reason is in the journal.
- If the stack exits or crashes, hold the buttons again to relaunch it.
- `robot.launch.py` defaults `deck` to false while a bridge is already running, so launching it by
  hand doesn't start a second one (the bridge's MQTT client id is fixed and two would keep kicking
  each other off the broker).
- Stopping the stack sends it Ctrl-C (so the LiDAR motor stops cleanly), then SIGTERM after 20 s
  and SIGKILL 5 s later if it is stuck. The bridge keeps running, so the Deck stays connected and
  the same chord starts it again.
- It never stops a `robot.launch.py` you started by hand; stop that with Ctrl-C in its terminal.
- Restarting the service (`sudo systemctl restart spotmicro-deck`) also shuts a Deck-launched stack
  down cleanly.
- Follow it with `journalctl -u spotmicro-deck -f`.

**Security:** `setup.sh` sets the broker to `allow_anonymous true` — anyone on your network can view
the streams and publish fake controller input. Fine on a private network while the robot can't
move; before the Deck drives the gait controller, enable a password (steps are in
`src/spot_micro_deck/config/mosquitto-spotmicro.conf`; `deck_params.yaml` and the app both accept a
username/password).

### Getting the app onto the Deck

The Deck is x86_64, so it needs an x86_64 build — a build made on the Pi is aarch64 and only runs
on the Pi. **Building on the Deck itself doesn't work out of the box:** SteamOS ships no C
compiler, `pkg-config` or SDL2 headers (`cargo build` there fails with ``linker `cc` not found``),
and its root filesystem is read-only. Rather than modifying SteamOS, cross-compile from the Pi (or
any Linux machine) with `steamdeck_app/cross_build_x86_64.sh`. It downloads Zig (checksum-verified,
cached in `~/.cache/spotmicro-zig`) purely as a linker and links against a copy of the Deck's own
SDL2:

```
ssh deck@<deck-ip> 'cat /usr/lib/libSDL2-2.0.so.0' > libSDL2-2.0.so.0
cd steamdeck_app && ./cross_build_x86_64.sh ../libSDL2-2.0.so.0
# -> steamdeck_app/target/x86_64-unknown-linux-gnu/release/spotmicro-deck
```

This is how the current build was made and it runs on a Steam Deck (SteamOS 3.8, Desktop Mode).
Alternatives: the GitHub workflow `.github/workflows/steamdeck-app.yml` builds it on push (untested),
or `steamdeck_app/build.sh` on an x86_64 PC with Rust and `libsdl2-dev`.

Copy `spotmicro-deck` and `install_on_deck.sh` to the Deck and, in a Desktop Mode terminal, run
`./install_on_deck.sh <robot-ip>`. Then in Steam: **Games → Add a Non-Steam Game to My Library →
SpotMicro**. It appears in your Gaming Mode library. If L2 doesn't respond, make sure the game's
controller layout is a Gamepad template (not Keyboard/Mouse).

The robot's address comes from, in order: `--host` in the game's Launch Options,
`$SPOTDECK_HOST`, `~/.config/spotmicro-deck/host`, else `spotmicro.local` (that name may not
resolve from the Deck — using the robot's IP is safer). Also `--port`, `--user`, `--password`, `--prefix`, `--windowed`.
Until it connects, the screen says why (broker unreachable + the error, or robot offline).

### Controls and what the robot receives

Only L2 does something in the app itself (Tab/Space also cycle views on a keyboard). Everything else
is published for later use on **`/spotmicro_deck/joy`** (`sensor_msgs/Joy`, ~30 Hz, header frame
`deck`) — the gait/teleop code can subscribe to it. It publishes all zeros if the Deck disconnects
or goes quiet for 0.5 s, and `/spotmicro_deck/connected` (`std_msgs/Bool`) says whether a Deck is
connected.

| `Joy.axes[i]` | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| meaning | left stick X | left stick Y | right stick X | right stick Y | L2 | R2 |

Sticks are −1..1 with **left and up positive** (ROS convention); triggers are 0..1 (0 released) —
not the inverted −1..1 of ROS's xbox joy driver.

`Joy.buttons[i]`, 1 = pressed: `0 a`, `1 b`, `2 x`, `3 y`, `4 l1`, `5 r1`, `6 back`, `7 start`,
`8 guide`, `9 l3`, `10 r3`, `11 dpad_up`, `12 dpad_down`, `13 dpad_left`, `14 dpad_right`,
`15-18 paddle1-4` (the Deck's back buttons: SDL's P1/P2 are the right/left upper R4/L4, P3/P4 the
right/left lower R5/L5), `19 l2`, `20 r2` (digital copies of the triggers). The same names are in
`src/spot_micro_deck/spot_micro_deck/protocol.py`, which defines all the MQTT topics and payloads.

### What has and hasn't been tested

Tested on the Pi with the real lidar, IMU, D405 and SLAM running: the robot side end to end over
MQTT (streams, view switching, joystick → `/joy`, last-will/neutral-on-disconnect), the app's
rendering of all four views and its not-connected screens (via SDL's dummy video driver and
screenshots), and the L2/stick/button logic and wire parsing in unit tests (`cargo test`,
`pytest src/spot_micro_deck/test`).

Also tested **on a real Steam Deck** (SteamOS 3.8, Desktop Mode), over Wi-Fi, against the running
robot stack: the x86_64 binary starts, SDL2 finds the Deck's controller, L2 cycles all four views,
and the app receives about 20 fps of camera or depth video plus the map, scan and status. That run
used SDL's dummy video driver (no window) so it was screenshotted, not watched on screen.

**Not tested:** Gaming Mode itself, Steam Input's layouts, physical L2/paddle presses (input was
scripted, apart from resting stick noise that arrived from the real controller), the GitHub
workflow, and the app's actual on-screen rendering on the Deck's GPU. Expect to adjust things on
first contact.

**Wi-Fi:** the video needs roughly 6-7 Mbps. It worked over the Deck's 2.4 GHz link in testing, but
that's close to what that link can carry; if it stutters, use 5 GHz, or lower `video_max_fps` /
`jpeg_quality` in `deck_params.yaml`. Wi-Fi power-save on the Deck can also add lag.

**Don't run two robot stacks at once.** Each `sllidar_node` stops the lidar motor when it shuts
down, so a second `robot.launch.py` (even one that fails to start) can silence the lidar for the
first, and two `deck_bridge` nodes will kick each other off the broker.

The `lidar` view draws the robot's front pointing up, assuming the laser's 0° is the robot's front
(`lidar_yaw` in the URDF — see the bringup section).

## IMU (Adafruit BNO055, 9-DOF absolute orientation)

`src/spot_micro_imu` is a custom driver node for the
[Adafruit BNO055 breakout](https://www.adafruit.com/product/2472), wired over I2C. The BNO055
has an onboard Cortex-M0 that fuses its accelerometer, gyroscope, and magnetometer itself, so the
node just polls the chip's already-fused output over I2C and republishes it as standard ROS2
messages. `spot_micro_imu/bno055.py` talks to the chip directly at the register level via
`smbus2` (`python3-smbus2`, installed as a system package by `setup.sh`) rather than through
Adafruit's CircuitPython/Blinka stack — that stack normally lives in this repo's `.venv`, but
`ros2 run`/`ros2 launch` always execute the node under the system python3 (see the note in
`setup.sh`), so a venv-only dependency wouldn't be importable by the node at runtime.

Wiring (I2C, shared with the PCA9685 servo board on the same bus):

| BNO055 pin | Pi pin              |
|------------|----------------------|
| Vin        | 3V3                  |
| GND        | GND                  |
| SDA        | SDA (GPIO2)          |
| SCL        | SCL (GPIO3)          |

Default I2C address is `0x28`; if that conflicts with another device on the bus, pull the
breakout's `ADR` pin high to switch it to `0x29` and set `i2c_address: 41` in
`src/spot_micro_imu/config/imu_params.yaml`.

Build and launch:

```
source /opt/ros/jazzy/setup.bash
colcon build --packages-select spot_micro_imu
source install/setup.bash
ros2 launch spot_micro_imu imu.launch.py
```

This publishes:

- `/spotmicro_imu/imu/data` — `sensor_msgs/Imu`: fused orientation (quaternion), angular
  velocity, and gravity-compensated linear acceleration
- `/spotmicro_imu/imu/mag` — `sensor_msgs/MagneticField`: magnetometer, in tesla
- `/spotmicro_imu/imu/temperature` — `sensor_msgs/Temperature`: onboard die temperature

Sanity checks:

```
i2cdetect -y 1                        # confirm the BNO055 shows up at 0x28
ros2 topic hz /spotmicro_imu/imu/data # confirm it's publishing (in another terminal)
ros2 topic echo /spotmicro_imu/imu/data
```

### Calibration

The BNO055 needs a brief calibration pass after power-up before its orientation output is fully
accurate — the node logs the calibration status (`sys`/`gyro`/`accel`/`mag`, each 0-3) every 5
seconds until all four reach 3. Gyro and accel calibrate quickly by sitting still and then moving
through a few different orientations; the magnetometer calibrates by slowly moving the sensor
through a figure-8 pattern in the air, away from large metal objects/motors.

### Viewing the data

Same [Foxglove](https://foxglove.dev) setup used for the lidar and camera — connect to
`ws://<pi-ip>:8765` with `foxglove-bridge` running, then:

- A **Raw Messages** panel on `/spotmicro_imu/imu/data` shows orientation/gyro/accel as they update.
- A **Plot** panel can chart individual fields (e.g. `angular_velocity.z`) over time.

### Why fixed covariance values

The BNO055 doesn't report per-sample uncertainty over I2C, so `bno055_node.py` publishes fixed,
rough placeholder covariances (order-of-magnitude from the datasheet's noise specs) rather than a
real per-reading estimate. If you feed this into an EKF (e.g. `robot_localization`), tune those
constants — or replace them with your own — first.
