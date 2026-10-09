"""
Bridge between the SpotMicro Steam Deck app and ROS2, over MQTT.

The Deck has no ROS install, so it talks to the robot through a Mosquitto broker
(see protocol.py for the topics and payloads). This node:

  * Deck -> robot: turns the Deck's controller input into sensor_msgs/Joy.
  * Robot -> Deck: streams the camera (color, or a colorized depth image), the
    lidar scan, the SLAM map and a JSON robot-state summary.

Camera and lidar subscriptions only exist while a Deck is online and looking at
that view, and depth is only colorized/encoded while the depth view is on
screen, so an idle robot spends no CPU on any of it.

Publishes (namespaced under 'spotmicro_deck'):
  joy        sensor_msgs/Joy   Deck buttons/sticks/triggers. Axis and button
                               order is documented in protocol.py. All zeros
                               when the Deck disconnects or goes quiet.
  connected  std_msgs/Bool     True while a Deck is connected (latched)
"""

import json
import math
import os
import threading
import time
import zlib

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, Imu, Joy, LaserScan
from std_msgs.msg import Bool

from spot_micro_deck import protocol as P

# Camera frames are big (a color JPEG is ~150 KB), and best-effort transport
# loses messages that have to be fragmented into many UDP packets, so the image
# subscriptions must be reliable. depth=1 keeps only the newest frame if we fall behind.
_IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

_NO_RETURN_MM = 0xFFFF
_MAX_SCAN_POINTS = 2048
_VIEW_HEARTBEAT_TIMEOUT_S = 3.0


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rpy_deg_from_quat(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch), math.degrees(_yaw_from_quat(q))


class DeckBridge(Node):

    def __init__(self):
        super().__init__('deck_bridge')

        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('topic_prefix', P.DEFAULT_PREFIX)

        self.declare_parameter('color_topic', '/spotmicro_cam/d405/color/image_raw/compressed')
        self.declare_parameter('depth_topic', '/spotmicro_cam/d405/depth/image_rect_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('imu_topic', '/spotmicro_imu/imu/data')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('video_max_fps', 20.0)
        self.declare_parameter('scan_rate_hz', 10.0)
        self.declare_parameter('state_rate_hz', 5.0)
        # The D405 is a close-range camera, but rooms are bigger than 0.5 m, so
        # colorize out to 2 m by default. Nearer = warmer.
        self.declare_parameter('depth_min_m', 0.1)
        self.declare_parameter('depth_max_m', 2.0)
        self.declare_parameter('input_timeout_s', 0.5)

        p = self.get_parameter
        self._t = P.topics(p('topic_prefix').value)
        self._jpeg_quality = int(p('jpeg_quality').value)
        self._video_period = 1.0 / float(p('video_max_fps').value)
        self._scan_period = 1.0 / float(p('scan_rate_hz').value)
        self._depth_min_mm = float(p('depth_min_m').value) * 1000.0
        self._depth_max_mm = float(p('depth_max_m').value) * 1000.0
        self._input_timeout = float(p('input_timeout_s').value)
        self._map_frame = p('map_frame').value
        self._base_frame = p('base_frame').value

        self._joy_pub = self.create_publisher(Joy, 'joy', 10)
        self._connected_pub = self.create_publisher(
            Bool, 'connected',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        # State shared with the MQTT thread.
        self._lock = threading.Lock()
        self._deck_online = False
        self._view = 'color'
        self._view_time = 0.0
        self._last_input = 0.0
        self._input_stale = True

        # ROS-thread only.
        self._next_video = {'color': 0.0, 'depth': 0.0}
        self._last_scan = 0.0
        self._counts = {'color': 0, 'depth': 0, 'scan': 0, 'imu': 0, 'input': 0, 'dropped': 0}
        self._last_state_time = time.monotonic()
        self._rates = {k: 0.0 for k in self._counts}
        self._imu_rpy = None
        self._imu_time = 0.0
        self._nearest = None
        self._map_payload = None
        self._map_info = None
        self._map_time = 0.0
        self._start = time.monotonic()
        self._warned_depth_encoding = False
        self._subs = {}
        self._topic_names = {
            'color': p('color_topic').value,
            'depth': p('depth_topic').value,
            'scan': p('scan_topic').value,
        }

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Always on: both are cheap, and the map is cached so a Deck that
        # connects late gets it at once from the broker's retained message.
        self.create_subscription(
            OccupancyGrid, p('map_topic').value, self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Imu, p('imu_topic').value, self._on_imu, qos_profile_sensor_data)

        self._connected_pub.publish(Bool(data=False))
        self._setup_mqtt()

        self.create_timer(0.1, self._reconcile_subscriptions)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0 / float(p('state_rate_hz').value), self._publish_state)

        self.get_logger().info(
            f'Deck bridge up: MQTT {p("mqtt_host").value}:{p("mqtt_port").value}, '
            f'topics under "{p("topic_prefix").value}/". Waiting for a Steam Deck.')

    # ── MQTT ─────────────────────────────────────────────────────────────────

    def _setup_mqtt(self):
        p = self.get_parameter
        client = mqtt.Client(client_id='spotmicro_deck_bridge', clean_session=True)
        if p('mqtt_username').value:
            client.username_pw_set(p('mqtt_username').value, p('mqtt_password').value)
        client.will_set(self._t['robot_online'], '0', qos=1, retain=True)
        client.max_queued_messages_set(100)   # drop video frames rather than lag if the link stalls
        client.reconnect_delay_set(min_delay=1, max_delay=10)
        client.on_connect = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        client.on_message = self._on_mqtt_message
        self._mqtt = client
        # connect_async + loop_start keeps retrying if the broker isn't up yet.
        client.connect_async(p('mqtt_host').value, int(p('mqtt_port').value), keepalive=15)
        client.loop_start()

    def _on_mqtt_connect(self, client, _userdata, _flags, rc):
        if rc != 0:
            self.get_logger().error(f'MQTT connect refused (rc={rc}); check username/password.')
            return
        self.get_logger().info('Connected to MQTT broker.')
        client.subscribe([(self._t['deck_online'], 1), (self._t['deck_view'], 0),
                          (self._t['deck_input'], 0)])
        client.publish(self._t['robot_online'], '1', qos=1, retain=True)
        if self._map_payload is not None:
            client.publish(self._t['map'], self._map_payload, qos=1, retain=True)

    def _on_mqtt_disconnect(self, _client, _userdata, rc):
        if rc != 0:
            self.get_logger().warn('Lost MQTT broker connection; reconnecting.')

    def _on_mqtt_message(self, _client, _userdata, msg):
        try:
            if msg.topic == self._t['deck_input']:
                self._handle_input(msg.payload)
            elif msg.topic == self._t['deck_view']:
                self._handle_view(msg.payload.decode('utf-8', 'replace').strip())
            elif msg.topic == self._t['deck_online']:
                self._handle_deck_online(msg.payload.decode('utf-8', 'replace').strip() == '1')
        except Exception as exc:   # never let a bad message kill the MQTT thread
            self.get_logger().warn(f'Ignoring bad message on {msg.topic}: {exc}',
                                   throttle_duration_sec=5.0)

    def _handle_input(self, payload):
        _seq, axes, buttons = P.decode_input(payload)
        with self._lock:
            self._last_input = time.monotonic()
            self._input_stale = False
        self._counts['input'] += 1
        self._publish_joy(axes, buttons)

    def _handle_view(self, name):
        if name not in P.VIEW_NAMES:
            raise P.ProtocolError(f'unknown view "{name}"')
        with self._lock:
            self._view = name
            self._view_time = time.monotonic()

    def _handle_deck_online(self, online):
        with self._lock:
            changed = online != self._deck_online
            self._deck_online = online
            if online:
                self._view_time = time.monotonic()
        if changed:
            self.get_logger().info('Steam Deck connected.' if online else 'Steam Deck disconnected.')
            self._connected_pub.publish(Bool(data=online))
            if not online:
                self._publish_neutral_joy()

    def _deck_active(self):
        """A Deck is online and still sending its view heartbeat."""
        with self._lock:
            return (self._deck_online
                    and time.monotonic() - self._view_time < _VIEW_HEARTBEAT_TIMEOUT_S)

    # ── Joy ──────────────────────────────────────────────────────────────────

    def _publish_joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'deck'
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self._joy_pub.publish(msg)

    def _publish_neutral_joy(self):
        self._publish_joy([0.0] * len(P.AXIS_NAMES), [0] * len(P.BUTTON_NAMES))

    def _watchdog(self):
        # If the Deck stays connected but stops sending (Wi-Fi hiccup, app
        # frozen), don't leave the last stick position latched on /joy.
        with self._lock:
            stale = (self._deck_online and not self._input_stale
                     and time.monotonic() - self._last_input > self._input_timeout)
            if stale:
                self._input_stale = True
        if stale:
            self.get_logger().warn('No input from the Deck; publishing neutral Joy.')
            self._publish_neutral_joy()

    # ── on-demand ROS subscriptions ──────────────────────────────────────────

    def _reconcile_subscriptions(self):
        active = self._deck_active()
        with self._lock:
            view = self._view
        wanted = {
            'color': active and view == 'color',
            'depth': active and view == 'depth',
            'scan': active,
        }
        factories = {
            'color': lambda: self.create_subscription(
                CompressedImage, self._topic_names['color'], self._on_color, _IMAGE_QOS),
            'depth': lambda: self.create_subscription(
                Image, self._topic_names['depth'], self._on_depth, _IMAGE_QOS),
            'scan': lambda: self.create_subscription(
                LaserScan, self._topic_names['scan'], self._on_scan, qos_profile_sensor_data),
        }
        for name, want in wanted.items():
            if want and name not in self._subs:
                self._subs[name] = factories[name]()
            elif not want and name in self._subs:
                self.destroy_subscription(self._subs.pop(name))

    # ── ROS -> MQTT ──────────────────────────────────────────────────────────

    def _video_due(self, view):
        # Fixed-schedule limiter: a plain "at least N ms since the last frame"
        # check quantises badly against a 30 Hz camera (a 20 fps cap would keep
        # only every second frame, i.e. 15 fps).
        now = time.monotonic()
        if now < self._next_video[view]:
            return False
        self._next_video[view] = max(self._next_video[view] + self._video_period, now)
        return True

    def _send_video(self, view, jpeg):
        info = self._mqtt.publish(self._t[f'video_{view}'], jpeg, qos=0)
        if info.rc == mqtt.MQTT_ERR_SUCCESS:
            self._counts[view] += 1
        else:
            self._counts['dropped'] += 1

    def _publish_jpeg(self, view, image_bgr):
        ok, buf = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if ok:
            self._send_video(view, buf.tobytes())

    def _on_color(self, msg):
        if not self._video_due('color'):
            return
        data = bytes(msg.data)
        if data[:2] == b'\xff\xd8':               # already a JPEG: forward it untouched
            self._send_video('color', data)
            return
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)   # e.g. PNG
        if image is not None:
            self._publish_jpeg('color', image)

    def _colorize_depth(self, depth_mm):
        span = max(self._depth_max_mm - self._depth_min_mm, 1.0)
        scaled = np.clip((depth_mm.astype(np.float32) - self._depth_min_mm) * (255.0 / span), 0, 255)
        image = cv2.applyColorMap(255 - scaled.astype(np.uint8), cv2.COLORMAP_TURBO)  # near = warm
        image[depth_mm == 0] = 0                                                       # no reading = black
        return image

    def _on_depth(self, msg):
        if not self._video_due('depth'):
            return
        if msg.encoding in ('16UC1', 'mono16'):
            depth = np.frombuffer(msg.data, dtype='>u2' if msg.is_bigendian else '<u2')
            image = self._colorize_depth(depth.reshape(msg.height, msg.width))
        elif msg.encoding in ('rgb8', 'bgr8'):    # driver's colorizer is on: already a picture
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                image = image[:, :, ::-1]
        else:
            if not self._warned_depth_encoding:
                self.get_logger().warn(f'Unsupported depth encoding "{msg.encoding}".')
                self._warned_depth_encoding = True
            return
        self._publish_jpeg('depth', image)

    def _on_scan(self, msg):
        self._counts['scan'] += 1
        now = time.monotonic()
        if now - self._last_scan < self._scan_period:
            return
        self._last_scan = now
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        step = max(1, math.ceil(len(ranges) / _MAX_SCAN_POINTS))
        ranges = ranges[::step]
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        mm = np.where(valid, np.minimum(ranges * 1000.0, _NO_RETURN_MM - 1), _NO_RETURN_MM)
        mm = mm.astype('<u2')
        # Rotate the scan from the laser frame into the robot (base) frame so the
        # app can draw it with front = up, however the lidar is mounted.
        angle_min = msg.angle_min + self._laser_yaw(msg.header.frame_id)
        if valid.any():
            i = int(np.argmin(np.where(valid, ranges, np.inf)))
            deg = math.degrees(angle_min + i * step * msg.angle_increment)
            self._nearest = {'range_m': round(float(ranges[i]), 3),
                             'angle_deg': round((deg + 180.0) % 360.0 - 180.0, 1)}
        else:
            self._nearest = None
        self._mqtt.publish(self._t['scan'], P.pack_scan(
            angle_min, msg.angle_increment * step, msg.range_max, mm.tobytes()), qos=0)

    def _laser_yaw(self, frame_id):
        """Yaw (rad) of the scan frame relative to the base frame; 0 until TF is up."""
        try:
            t = self._tf_buffer.lookup_transform(self._base_frame, frame_id or 'laser', Time())
        except tf2_ros.TransformException:
            return 0.0
        return _yaw_from_quat(t.transform.rotation)

    def _on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int8)
        info = msg.info
        self._map_payload = P.pack_map(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, zlib.compress(grid.tobytes(), 6))
        self._map_info = {'w': info.width, 'h': info.height, 'res': round(info.resolution, 4)}
        self._map_time = time.monotonic()
        self._mqtt.publish(self._t['map'], self._map_payload, qos=1, retain=True)

    def _on_imu(self, msg):
        self._counts['imu'] += 1
        self._imu_rpy = _rpy_deg_from_quat(msg.orientation)
        self._imu_time = time.monotonic()

    # ── state summary ────────────────────────────────────────────────────────

    def _lookup_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except tf2_ros.TransformException:
            return None
        tr = t.transform
        return {'x': round(tr.translation.x, 3), 'y': round(tr.translation.y, 3),
                'yaw_deg': round(math.degrees(_yaw_from_quat(tr.rotation)), 1)}

    @staticmethod
    def _read_cpu_temp_c():
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            return None

    def _build_state(self):
        now = time.monotonic()
        dt = max(now - self._last_state_time, 1e-3)
        self._last_state_time = now
        self._rates = {k: round(v / dt, 1) for k, v in self._counts.items()}
        for k in self._counts:
            self._counts[k] = 0
        with self._lock:
            view = self._view
        imu = None
        if self._imu_rpy is not None and now - self._imu_time < 2.0:
            roll, pitch, yaw = self._imu_rpy
            imu = {'roll': round(roll, 1), 'pitch': round(pitch, 1), 'yaw': round(yaw, 1)}
        map_state = None
        if self._map_info is not None:
            map_state = dict(self._map_info, age_s=round(now - self._map_time, 1))
        return {
            'v': 1,
            'time': round(time.time(), 2),
            'uptime_s': int(now - self._start),
            'view': view,
            'imu': imu,
            'pose': self._lookup_pose(),
            'nearest': self._nearest,
            'map': map_state,
            'rates': self._rates,
            'depth_range_m': [round(self._depth_min_mm / 1000.0, 2), round(self._depth_max_mm / 1000.0, 2)],
            'sys': {'cpu_temp_c': self._read_cpu_temp_c(), 'load1': round(os.getloadavg()[0], 2)},
        }

    def _publish_state(self):
        if not self._deck_active():
            for k in self._counts:
                self._counts[k] = 0
            self._last_state_time = time.monotonic()
            return
        self._mqtt.publish(self._t['state'], json.dumps(self._build_state(), separators=(',', ':')),
                           qos=0)

    def shutdown(self):
        self._publish_neutral_joy()
        if self._mqtt.is_connected():
            info = self._mqtt.publish(self._t['robot_online'], '0', qos=1, retain=True)
            deadline = time.monotonic() + 2.0
            while not info.is_published() and time.monotonic() < deadline:
                time.sleep(0.02)
        self._mqtt.loop_stop()
        self._mqtt.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = DeckBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:   # best effort: the broker may already be gone
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
