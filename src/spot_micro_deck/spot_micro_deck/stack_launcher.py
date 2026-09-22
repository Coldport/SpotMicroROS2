#!/usr/bin/env python3
"""
stack_launcher — starts and stops the robot sensing stack from the Steam Deck.

Runs on the robot at boot next to deck_bridge. When a Deck is connected and its
L1 + R1 buttons are held together for `hold_s` seconds it toggles the stack:
if it isn't running it runs `command` (by default:
`ros2 launch spot_micro_bringup robot.launch.py deck:=false`, the sensing stack
without a second bridge); if the stack this node started is running it shuts it
down (SIGINT, like Ctrl-C). The bridge itself keeps running, so the Deck stays
connected and can start the stack again.

It never launches while another robot.launch.py is already running, because two
stacks fight over the LiDAR, IMU and camera, and it never stops a stack it did
not start itself.

Subscribes (relative to the node's namespace, /spotmicro_deck when launched from
deck.launch.py):
  joy        sensor_msgs/Joy   from deck_bridge; buttons[4]=L1, buttons[5]=R1
  connected  std_msgs/Bool     True while a Deck is connected (latched)

This does not start the gait controller or anything that moves the robot; that
is up to `command`, which is sensing-only by default.
"""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from spot_micro_deck.chord import ChordDetector

# Index of each button in the Joy message (see protocol.BUTTON_NAMES).
_L1, _R1 = 4, 5


class StackLauncher(Node):

    def __init__(self):
        super().__init__('stack_launcher')
        p = self.get_parameter
        self.declare_parameter('hold_s', 3.0)
        self.declare_parameter('buttons', [_L1, _R1])
        self.declare_parameter(
            'command', ['ros2', 'launch', 'spot_micro_bringup', 'robot.launch.py', 'deck:=false'])
        # pgrep -f pattern for "the stack is already up"; launching is refused if it matches.
        self.declare_parameter('busy_pattern', 'robot.launch.py')
        self.declare_parameter('shutdown_timeout_s', 20.0)

        self._command = list(p('command').value)
        self._busy_pattern = p('busy_pattern').value
        self._shutdown_timeout = float(p('shutdown_timeout_s').value)
        self._chord = ChordDetector(p('buttons').value, p('hold_s').value)
        self._hold_s = float(p('hold_s').value)

        self._connected = False
        self._proc = None
        self._stopping_since = None   # set once we've asked the stack to stop
        self._stop_escalation = 0     # 0 = SIGINT sent, 1 = SIGTERM sent, 2 = SIGKILL sent
        self._last_progress = 0

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_subscription(
            Bool, 'connected', self._on_connected,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5, self._reap)
        self.get_logger().info(
            f'Hold L1+R1 on the Deck for {self._hold_s:.0f}s to start ({" ".join(self._command)}) '
            'or, when it is running, to stop the robot stack.')

    # ── input ────────────────────────────────────────────────────────────────

    def _on_connected(self, msg):
        self._connected = bool(msg.data)
        if not self._connected:
            self._chord.reset()

    def _on_joy(self, msg):
        if not self._connected:
            return
        now = time.monotonic()
        fired = self._chord.update(msg.buttons, now)
        held = int(self._chord.held_for(now))
        if held and held != self._last_progress and held < self._hold_s:
            what = 'stop' if self._stack_running() else 'start'
            self.get_logger().info(f'L1+R1 held {held}/{self._hold_s:.0f}s (to {what} the stack) ...')
        self._last_progress = held
        if fired:
            self._toggle()

    # ── launching ────────────────────────────────────────────────────────────

    def _stack_running(self):
        return self._proc is not None and self._proc.poll() is None

    def _other_stack_running(self):
        try:
            found = subprocess.run(['pgrep', '-f', self._busy_pattern],
                                   capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.SubprocessError) as e:
            self.get_logger().warn(f'Could not check for a running stack ({e}); not launching.')
            return True
        return found.returncode == 0

    def _toggle(self):
        if not self._stack_running():
            self._launch()
        elif self._stopping_since is None:
            self._request_stop()
        else:
            self.get_logger().info('The stack is already shutting down; give it a moment.')

    def _launch(self):
        if self._other_stack_running():
            self.get_logger().warn(
                f'"{self._busy_pattern}" is already running (started by hand?); not launching a '
                'second stack over the same hardware.')
            return
        self.get_logger().info('Launching the robot stack: ' + ' '.join(self._command))
        # Own session so we can signal the whole launch tree, and so a Ctrl-C to
        # this node doesn't hit it directly. Output goes to our stdout (the journal).
        self._proc = subprocess.Popen(self._command, start_new_session=True)

    def _signal_stack(self, sig):
        try:
            os.killpg(self._proc.pid, sig)
        except ProcessLookupError:
            pass

    def _request_stop(self):
        """Ask the stack to stop (SIGINT, like Ctrl-C) without blocking; _reap follows up."""
        self.get_logger().info('Stopping the robot stack ...')
        self._stopping_since = time.monotonic()
        self._stop_escalation = 0
        self._signal_stack(signal.SIGINT)

    def _reap(self):
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is None:
            # Still up. If we asked it to stop and it is dragging its feet, escalate.
            if self._stopping_since is not None:
                waited = time.monotonic() - self._stopping_since
                if self._stop_escalation == 0 and waited > self._shutdown_timeout:
                    self.get_logger().warn('The stack did not stop in time; terminating it.')
                    self._stop_escalation = 1
                    self._signal_stack(signal.SIGTERM)
                elif self._stop_escalation == 1 and waited > self._shutdown_timeout + 5.0:
                    self.get_logger().warn('The stack is still running; killing it.')
                    self._stop_escalation = 2
                    self._signal_stack(signal.SIGKILL)
            return
        if self._stopping_since is not None:
            self.get_logger().info('The robot stack has stopped. Hold L1+R1 to start it again.')
        else:
            self.get_logger().warn(
                f'The robot stack exited with code {code}; hold L1+R1 again to relaunch it.')
        self._proc = None
        self._stopping_since = None

    def stop_stack(self):
        """Blocking shutdown of the stack, for when this node itself is exiting.

        Plain prints: by now the ROS context is usually gone, so the node logger
        can't publish to /rosout.
        """
        if not self._stack_running():
            return
        print('[stack_launcher] Node stopping: shutting the robot stack down ...', flush=True)
        self._signal_stack(signal.SIGINT)
        try:
            self._proc.wait(timeout=self._shutdown_timeout)
        except subprocess.TimeoutExpired:
            print('[stack_launcher] The stack did not stop in time; terminating it.', flush=True)
            self._signal_stack(signal.SIGTERM)


def main(args=None):
    rclpy.init(args=args)
    node = StackLauncher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_stack()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
