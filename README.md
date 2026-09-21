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
