"""
track_node — follows an object picked on the Steam Deck in the color camera
stream, for gait_node's sentry mode to point the camera at.

  Deck (touch-drag a box / D-pad + A, in sentry) -> MQTT <p>/deck/track
  -> deck_bridge -> `cmd_topic` (std_msgs/String JSON) -> this node
  -> `status_topic` (std_msgs/String JSON) -> gait_node (turns the body)
                                           -> deck_bridge -> MQTT <p>/track (box overlay)

Commands (JSON):
  {"cmd": "start", "box": [x, y, w, h]}   box in the image, normalised 0..1
                                          (x, y = top-left; +y down)
  {"cmd": "stop"}

Status (JSON), on every processed frame while active, else ~2 Hz:
  {"state": "idle" | "starting" | "tracking" | "lost",
   "box": [x, y, w, h] | null,            normalised, as above
   "err_deg": [yaw, pitch] | null,        target's angle off the image centre:
                                          yaw + = left of centre, pitch + = below
   "seq": n}

The angle uses a pinhole model with the camera's field of view
(`fov_deg_realsense` / `fov_deg_usb`, [horizontal, vertical]); it only needs to
be roughly right, since gait_node closes the loop on it.

Camera topics are only subscribed while tracking (decoding JPEGs costs the Pi
CPU), and tracking stops by itself when gait_node leaves sentry mode, or after
`lost_timeout_s` without finding the target again.

OpenCV trackers: `tracker` = "csrt" (default: most robust), "kcf" (faster,
loses things more easily) or "mil" (slow). Measured on the Pi 5 at 320 px,
JPEG decode included: csrt ~50 fps, kcf ~240, mil ~14 — so the tracking runs
at the camera's rate, and the work is all in OpenCV's C++ (a C++ node would
gain nothing worth having).
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# Lambdas, not the functions themselves: some OpenCV builds lack a tracker, so
# look it up only when that tracker is picked.
# pylint: disable=unnecessary-lambda
TRACKERS = {
    'csrt': lambda: cv2.TrackerCSRT_create(),
    'kcf': lambda: cv2.TrackerKCF_create(),
    'mil': lambda: cv2.TrackerMIL_create(),
}
# pylint: enable=unnecessary-lambda
MIN_BOX_PX = 8


def parse_command(text):
    """-> ('start', (x, y, w, h)) or ('stop',); raises ValueError."""
    msg = json.loads(text)
    cmd = msg.get('cmd') if isinstance(msg, dict) else None
    if cmd == 'stop':
        return ('stop',)
    if cmd == 'start':
        box = clamp_box([float(v) for v in msg['box']])
        if box is None:
            raise ValueError('empty box')
        return ('start', box)
    raise ValueError(f'unknown track command {cmd!r}')


def clamp_box(box):
    """Normalised [x, y, w, h] clipped to the image, or None if nothing is left."""
    if len(box) != 4 or not all(math.isfinite(v) for v in box):
        return None
    x, y, w, h = box
    if w < 0:
        x, w = x + w, -w
    if h < 0:
        y, h = y + h, -h
    x0, y0 = min(max(x, 0.0), 1.0), min(max(y, 0.0), 1.0)
    x1, y1 = min(max(x + w, 0.0), 1.0), min(max(y + h, 0.0), 1.0)
    if x1 - x0 <= 0.0 or y1 - y0 <= 0.0:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def angle_error(box, fov_deg):
    """Normalised box -> (yaw, pitch) deg of its centre off the optical axis.
    yaw + = left of centre (turn left to face it), pitch + = below (nose down)."""
    x, y, w, h = box
    u = 2.0 * (x + w / 2.0) - 1.0      # -1 left edge .. +1 right edge
    v = 2.0 * (y + h / 2.0) - 1.0      # -1 top .. +1 bottom
    hfov, vfov = (math.radians(a) for a in fov_deg)
    yaw = -math.degrees(math.atan(u * math.tan(hfov / 2.0)))
    pitch = math.degrees(math.atan(v * math.tan(vfov / 2.0)))
    return yaw, pitch


class TrackNode(Node):

    def __init__(self):
        super().__init__('track_node')
        self.declare_parameter('color_topic', '/spotmicro_cam/d405/color/image_raw/compressed')
        self.declare_parameter('usb_color_topic', '/spotmicro_cam/usb/image_raw/compressed')
        self.declare_parameter('cmd_topic', '/spotmicro_track/cmd')
        self.declare_parameter('status_topic', '/spotmicro_track/status')
        self.declare_parameter('mode_topic', '/spotmicro_move/mode')
        self.declare_parameter('tracker', 'csrt')
        self.declare_parameter('process_width', 320)       # px the tracker works at
        self.declare_parameter('max_hz', 30.0)
        self.declare_parameter('lost_timeout_s', 5.0)
        self.declare_parameter('fov_deg_realsense', [87.0, 58.0])   # D405 color, [h, v]
        self.declare_parameter('fov_deg_usb', [70.0, 43.0])         # typical 720p webcam

        p = self.get_parameter
        name = str(p('tracker').value)
        if name not in TRACKERS:
            raise ValueError(f"tracker '{name}' not in {sorted(TRACKERS)}")
        self._make_tracker = TRACKERS[name]
        self._width = max(int(p('process_width').value), 64)
        self._period = 1.0 / max(float(p('max_hz').value), 1.0)
        self._lost_timeout = float(p('lost_timeout_s').value)
        self._fov = {'realsense': [float(v) for v in p('fov_deg_realsense').value],
                     'usb': [float(v) for v in p('fov_deg_usb').value]}
        self._topics = {'realsense': p('color_topic').value, 'usb': p('usb_color_topic').value}

        self._state = 'idle'
        self._box = None              # normalised, latest
        self._tracker = None
        self._camera = None           # 'realsense' | 'usb', whichever frames come from
        self._lost_since = None
        self._next_frame = 0.0
        self._seq = 0
        self._subs = {}

        self._status_pub = self.create_publisher(String, p('status_topic').value, 10)
        self.create_subscription(String, p('cmd_topic').value, self._on_cmd, 10)
        self.create_subscription(
            String, p('mode_topic').value, self._on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5, self._heartbeat)
        self.get_logger().info(f'track_node up ({name}); waiting for a target from the Deck.')

    # ── commands ─────────────────────────────────────────────────────────────

    def _on_cmd(self, msg):
        try:
            cmd = parse_command(msg.data)
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'Ignoring bad track command: {e}')
            return
        if cmd[0] == 'stop':
            self._stop('stopped from the Deck')
            return
        self._box = cmd[1]
        self._tracker = None
        self._lost_since = None
        self._state = 'starting'
        self._subscribe(True)
        self.get_logger().info('Tracking target at ' + ', '.join(f'{v:.2f}' for v in self._box))
        self._publish()

    def _on_mode(self, msg):
        try:
            mode = json.loads(msg.data).get('mode')
        except (ValueError, AttributeError):
            return
        if mode != 'sentry' and self._state != 'idle':
            self._stop(f'gait_node left sentry ({mode})')

    def _stop(self, why):
        if self._state != 'idle':
            self.get_logger().info(f'Tracking stopped: {why}.')
        self._state = 'idle'
        self._box = None
        self._tracker = None
        self._subscribe(False)
        self._publish()

    def _subscribe(self, on):
        if on and not self._subs:
            self._subs = {kind: self.create_subscription(
                CompressedImage, topic, lambda m, k=kind: self._on_frame(m, k), qos_profile_sensor_data)
                for kind, topic in self._topics.items()}
        elif not on:
            for sub in self._subs.values():
                self.destroy_subscription(sub)
            self._subs = {}

    # ── frames ───────────────────────────────────────────────────────────────

    def _on_frame(self, msg, kind):
        if self._state == 'idle':
            return
        now = time.monotonic()
        if now < self._next_frame:
            return
        self._next_frame = now + self._period
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if img is None:
            return
        if img.shape[1] > self._width * 1.25:
            scale = self._width / img.shape[1]
            img = cv2.resize(img, (self._width, max(1, round(img.shape[0] * scale))),
                             interpolation=cv2.INTER_AREA)
        self._camera = kind
        h, w = img.shape[:2]

        if self._tracker is None:
            x, y, bw, bh = self._box
            px = (int(x * w), int(y * h), max(int(bw * w), MIN_BOX_PX), max(int(bh * h), MIN_BOX_PX))
            px = (min(px[0], w - px[2]), min(px[1], h - px[3]), min(px[2], w), min(px[3], h))
            self._tracker = self._make_tracker()
            self._tracker.init(img, px)
            self._state = 'tracking'
        else:
            ok, (bx, by, bw, bh) = self._tracker.update(img)
            box = clamp_box([bx / w, by / h, bw / w, bh / h]) if ok else None
            if box is not None:
                self._box = box
                self._state = 'tracking'
                self._lost_since = None
            else:
                if self._lost_since is None:
                    self._lost_since = now
                    self.get_logger().info('Lost the target; still looking.')
                self._state = 'lost'
                if now - self._lost_since > self._lost_timeout:
                    self._stop(f'target not seen for {self._lost_timeout:.0f} s')
                    return
        self._publish()

    # ── status ───────────────────────────────────────────────────────────────

    def _heartbeat(self):
        if self._state == 'idle' or not self._subs:
            self._publish()
        elif time.monotonic() - self._next_frame > 2.0:
            # Active but no frames: say so (gait_node treats a stale status as not tracking).
            self.get_logger().warn('Tracking, but no camera frames — is the camera running?',
                                   throttle_duration_sec=10.0)
            self._publish()

    def _publish(self):
        self._seq += 1
        err = None
        if self._state == 'tracking' and self._box is not None:
            fov = self._fov.get(self._camera or 'realsense', self._fov['realsense'])
            err = [round(a, 2) for a in angle_error(self._box, fov)]
        msg = {'state': self._state, 'seq': self._seq,
               'box': [round(v, 4) for v in self._box] if self._box is not None else None,
               'err_deg': err}
        self._status_pub.publish(String(data=json.dumps(msg, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = TrackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
