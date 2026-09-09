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
