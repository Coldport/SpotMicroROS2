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
