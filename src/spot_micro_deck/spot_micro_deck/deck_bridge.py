"""
Bridge between the SpotMicro Steam Deck app and ROS2, over MQTT.

The Deck has no ROS install, so it talks to the robot through a Mosquitto broker
(see protocol.py for the topics and payloads). This node:

  * Deck -> robot: turns the Deck's controller input into sensor_msgs/Joy.
  * Robot -> Deck: streams the camera (color, or a colorized depth image), the
    lidar scan, the SLAM map and a JSON robot-state summary.
  * Works out which camera is in use — the RealSense D405 (color + depth) or a
    plain USB webcam (color only) — from what is plugged in (sysfs, via
    spot_micro_camera.detect) and which driver is publishing, follows the right
    color topic, and tells the Deck (state "camera").
  * Settings: publishes the live-tunable settings (settings.py) with their
    current values, and applies changes the Deck sends — gait_node's through
    its parameter services, the bridge's own video settings directly. Accepted
    changes are also written into the params YAML files (gait_params_files /
    bridge_params_files, set by deck.launch.py), so the next launch uses them.
  * Map actions from the settings menu: clear the SLAM map, or save it into
    `maps_dir` as map_<date>_<time> — a map_server .pgm/.yaml (the static map)
    plus slam_toolbox's serialized pose graph (.posegraph/.data, which its
    localization mode can load). static_map.* symlinks point at the newest save.
    "Load saved map" hands that pose graph back to slam_toolbox, so SLAM carries
    on from the saved map (the robot must start where that map was begun).
  * The robot's mic (the Anker webcam's; audio.py): streamed as mu-law on
    <prefix>/audio while a Deck is online and the "Robot mic" setting is on.
  * The saved static map goes to the Deck too (retained), for the auto-nav view.
  * Auto nav: goals / cancels / speed steps from the Deck go to nav_node
    (spot_micro_move), and its status comes back in the state message ("nav").
    "The robot is here" from the Deck reloads the saved pose graph into
    slam_toolbox with the robot at that pose (START_AT_GIVEN_POSE), so a robot
    that didn't start where the map was begun can still navigate on it.

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
import subprocess
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
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage, Image, Imu, Joy, LaserScan
from geometry_msgs.msg import PoseStamped
from slam_toolbox.srv import DeserializePoseGraph, Reset, SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, Empty, Int32, String

from spot_micro_deck import audio
from spot_micro_deck import param_file
from spot_micro_deck import protocol as P
from spot_micro_deck import settings as S
from spot_micro_deck import static_map

try:
    from spot_micro_camera.detect import detect_camera
except ImportError:          # camera package not built: report only what is streaming
    detect_camera = None

# Camera frames are big (a color JPEG is ~150 KB), and best-effort transport
# loses messages that have to be fragmented into many UDP packets, so the image
# subscriptions must be reliable. depth=1 keeps only the newest frame if we fall behind.
_IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

_NO_RETURN_MM = 0xFFFF
_MAX_SCAN_POINTS = 2048
_VIEW_HEARTBEAT_TIMEOUT_S = 3.0
_CAMERA_CHECK_S = 2.0
_SETTINGS_PERIOD_S = 1.0
_SAVE_DELAY_S = 1.5          # write params files once the D-pad has been still this long
# A parameter-service reply that hasn't come back by then is treated as lost. The
# first call right after gait_node starts is often dropped ("failed to send
# response (timeout)" in its log) before the service has matched our client;
# without this the bridge waited on it forever and the Deck showed every
# setting as unavailable.
_PARAM_CALL_TIMEOUT_S = 2.0
_BRIDGE_SETTINGS = [s.key for s in S.SETTINGS if s.target == 'bridge' and s.kind != 'action']
# Run by the Deck's "Shut down robot" button. Needs passwordless sudo for the
# bridge's user (the default on a Raspberry Pi / this robot's Ubuntu user).
_SHUTDOWN_CMD = ['sudo', '-n', 'systemctl', 'poweroff']
_GAIT_SETTINGS = [s.key for s in S.SETTINGS if s.target == 'gait']


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
        self.declare_parameter('usb_color_topic', '/spotmicro_cam/usb/image_raw/compressed')
        self.declare_parameter('gait_node', '/gait_node')
        self.declare_parameter('armed_topic', '/spotmicro_move/armed')
        self.declare_parameter('mode_topic', '/spotmicro_move/mode')
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
        # Robot mic -> Deck (audio.py). audio_device: 'auto' (first USB mic, i.e.
        # the webcam), part of a card's name, or an ALSA device like 'plughw:0,0'.
        self.declare_parameter('audio_enabled', False)
        self.declare_parameter('audio_gain_db', 0.0)
        self.declare_parameter('audio_device', 'auto')
        self.declare_parameter('audio_rate', 16000)
        # ':'-separated params files to write accepted Deck setting changes into
        # (deck.launch.py fills these in); empty = changes last until restart.
        self.declare_parameter('gait_params_files', '')
        self.declare_parameter('bridge_params_files', '')
        # Where the Deck's "Save map" writes the SLAM map ('~' is expanded).
        self.declare_parameter('maps_dir', '~/spotmicro_maps')
        # Auto nav (spot_micro_move nav_node).
        self.declare_parameter('nav_goal_topic', '/spotmicro_move/nav_goal')
        self.declare_parameter('nav_cancel_topic', '/spotmicro_move/nav_cancel')
        self.declare_parameter('nav_status_topic', '/spotmicro_move/nav_status')
        self.declare_parameter('nav_speed_topic', '/spotmicro_move/nav_speed')
        # Sentry object tracking (spot_micro_move track_node).
        self.declare_parameter('track_cmd_topic', '/spotmicro_track/cmd')
        self.declare_parameter('track_status_topic', '/spotmicro_track/status')

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
        self._color_topics = {'realsense': p('color_topic').value, 'usb': p('usb_color_topic').value}
        self._camera = {'type': None, 'model': '', 'depth': False, 'streaming': False}

        # Settings: gait_node's are read/written through its parameter services.
        self._gait_params = AsyncParameterClient(self, p('gait_node').value)
        self._gait_values = {}
        self._gait_running = False
        self._gait_get_sent = None     # monotonic time of the outstanding get, or None
        self._gait_get_seq = 0
        self._set_pending = {}         # key -> (value, sent at) awaiting gait_node's reply
        self._armed = False
        self._mode = None              # gait_node's mode message (dict), see _on_mode
        self._set_queue = []           # (key, value) from the MQTT thread
        self._last_settings = 0.0
        self._save_files = {t: self._params_files(p(f'{t}_params_files').value)
                            for t in ('gait', 'bridge')}
        self._unsaved = {'gait': {}, 'bridge': {}}
        self._last_change = 0.0
        self.add_on_set_parameters_callback(self._on_own_params)

        self._slam_reset = self.create_client(Reset, '/slam_toolbox/reset')
        self._slam_save = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._slam_serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._slam_load = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self._maps_dir = os.path.expanduser(p('maps_dir').value)
        self._saving_map = False
        self._static_map_payload = self._load_static_map()

        self._nav_goal_pub = self.create_publisher(PoseStamped, p('nav_goal_topic').value, 10)
        self._nav_cancel_pub = self.create_publisher(Empty, p('nav_cancel_topic').value, 10)
        self._nav_speed_pub = self.create_publisher(Int32, p('nav_speed_topic').value, 10)
        self._nav = None               # nav_node's status message (dict)
        self.create_subscription(
            String, p('nav_status_topic').value, self._on_nav_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._track_cmd_pub = self.create_publisher(String, p('track_cmd_topic').value, 10)
        self.create_subscription(String, p('track_status_topic').value, self._on_track_status, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Always on: both are cheap, and the map is cached so a Deck that
        # connects late gets it at once from the broker's retained message.
        self.create_subscription(
            OccupancyGrid, p('map_topic').value, self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Imu, p('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Bool, p('armed_topic').value, self._on_armed,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            String, p('mode_topic').value, self._on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self._mic = None               # audio.MicStream while streaming
        self._mic_exit = None          # why ffmpeg stopped by itself (set from its thread)
        self._mic_retry_at = 0.0
        self._mic_reported = ''        # last mic problem told to the Deck (once per try)
        self._audio_seq = 0

        self._connected_pub.publish(Bool(data=False))
        self._setup_mqtt()

        self.create_timer(0.1, self._reconcile_subscriptions)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0 / float(p('state_rate_hz').value), self._publish_state)
        self.create_timer(_CAMERA_CHECK_S, self._update_camera)
        self.create_timer(0.05, self._settings_tick)
        self._update_camera()

        for target, files in self._save_files.items():
            self.get_logger().info(f'Deck {target} setting changes are saved to: '
                                   f'{", ".join(files) or "(nowhere: they last until restart)"}')
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
                          (self._t['deck_input'], 0), (self._t['deck_set'], 1),
                          (self._t['deck_nav'], 1), (self._t['deck_track'], 1)])
        client.publish(self._t['robot_online'], '1', qos=1, retain=True)
        if self._map_payload is not None:
            client.publish(self._t['map'], self._map_payload, qos=1, retain=True)
        client.publish(self._t['static_map'], self._static_map_payload, qos=1, retain=True)

    def _on_mqtt_disconnect(self, _client, _userdata, rc):
        if rc != 0:
            self.get_logger().warn('Lost MQTT broker connection; reconnecting.')

    def _on_mqtt_message(self, _client, _userdata, msg):
        try:
            if msg.topic == self._t['deck_input']:
                self._handle_input(msg.payload)
            elif msg.topic == self._t['deck_view']:
                self._handle_view(msg.payload.decode('utf-8', 'replace').strip())
            elif msg.topic == self._t['deck_set']:
                key, value = P.decode_set(msg.payload)
                with self._lock:
                    self._set_queue.append((key, value))
            elif msg.topic == self._t['deck_nav']:
                self._handle_nav(P.decode_nav(msg.payload))
            elif msg.topic == self._t['deck_track']:
                self._handle_track(P.decode_track(msg.payload))
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

    def _handle_nav(self, cmd):
        if cmd[0] == 'cancel':
            self._nav_cancel_pub.publish(Empty())
            return
        if cmd[0] == 'speed':
            self._nav_speed_pub.publish(Int32(data=cmd[1]))
            return
        if cmd[0] == 'pose':
            self._set_robot_pose(*cmd[1:])
            return
        goal = PoseStamped()
        goal.header.frame_id = self._map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = cmd[1], cmd[2]
        goal.pose.orientation.w = 1.0
        self._nav_goal_pub.publish(goal)

    def _handle_track(self, cmd):
        out = {'cmd': 'stop'} if cmd[0] == 'stop' else {'cmd': 'start', 'box': cmd[1]}
        self._track_cmd_pub.publish(String(data=json.dumps(out, separators=(',', ':'))))

    def _on_track_status(self, msg):
        # Small and only frequent while tracking: straight through, for the Deck's box overlay.
        if self._deck_active():
            self._mqtt.publish(self._t['track'], msg.data, qos=0)

    def _on_nav_status(self, msg):
        try:
            nav = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(nav, dict):
            self._nav = nav

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
        self._reconcile_mic(active)
        with self._lock:
            view = self._view
        wanted = {
            'color': active and view == 'color',
            'depth': active and view == 'depth' and self._camera['type'] != 'usb',
            'scan': active,
        }
        factories = {
            'color': lambda: self.create_subscription(
                CompressedImage, self._color_topic(), self._on_color, _IMAGE_QOS),
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

    def _reconcile_mic(self, active):
        """Run the mic while a Deck is online and wants it; restart it when the
        gain changes; after a failure, retry every few seconds (telling the
        Deck why once, not every time)."""
        want = active and bool(self.get_parameter('audio_enabled').value)
        gain = float(self.get_parameter('audio_gain_db').value)
        now = time.monotonic()
        with self._lock:
            exited, self._mic_exit = self._mic_exit, None
        if exited and self._mic is not None:
            self._mic = None
            self._mic_failed(exited, now)
        if self._mic is not None and (not want or self._mic.gain_db != gain):
            self._mic.stop()
            self._mic = None
            if not want:
                self.get_logger().info('Robot mic off.')
        if not want:
            self._mic_reported = ''
            self._mic_retry_at = 0.0
            return
        if self._mic is not None or now < self._mic_retry_at:
            return
        try:
            device = audio.find_device(self.get_parameter('audio_device').value)
            audio.check_access(device)
            mic = audio.MicStream(device, self._on_mic_chunk, self._on_mic_exit,
                                  rate=int(self.get_parameter('audio_rate').value), gain_db=gain)
            mic.start()
        except audio.AudioError as exc:
            self._mic_failed(str(exc), now)
            return
        self._mic = mic
        self.get_logger().info(f'Robot mic on: {device}, {mic.rate} Hz mu-law, gain {gain:g} dB.')

    def _mic_failed(self, reason, now):
        self._mic_retry_at = now + 5.0
        if reason != self._mic_reported:
            self._mic_reported = reason
            self.get_logger().warn(f'Robot mic: {reason}')
            self._ack('audio_enabled', False, True, reason)

    def _on_mic_chunk(self, data):
        # From the mic's reader thread; paho's publish is thread-safe.
        mic = self._mic
        rate = mic.rate if mic is not None else 16000
        self._audio_seq += 1
        self._mqtt.publish(self._t['audio'], P.pack_audio(self._audio_seq, rate, data), qos=0)

    def _on_mic_exit(self, reason):
        with self._lock:
            self._mic_exit = reason

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

    # ── camera ───────────────────────────────────────────────────────────────

    def _color_topic(self):
        return self._color_topics['usb' if self._camera['type'] == 'usb' else 'realsense']

    def _update_camera(self):
        """Which camera is in use: the driver that is publishing wins; otherwise
        whatever is plugged in (so the Deck can say "USB webcam, driver not
        running"). Re-points the color subscription when it changes."""
        streaming = [kind for kind, topic in self._color_topics.items()
                     if self.count_publishers(topic) > 0]
        hw = detect_camera() if detect_camera is not None else None
        kind = streaming[0] if streaming else (hw.kind if hw is not None else None)
        if hw is not None and hw.kind == kind:
            model = hw.model
        else:
            model = {'realsense': 'RealSense', 'usb': 'USB camera'}.get(kind, '')
        camera = {'type': kind, 'model': model, 'depth': kind == 'realsense',
                  'streaming': bool(streaming)}
        if camera != self._camera:
            old_topic = self._color_topic()
            if camera['type'] != self._camera['type']:
                self.get_logger().info(
                    f'Camera: {model or "none"}' + (' (streaming)' if streaming else
                                                    (' (driver not running)' if kind else '')))
            self._camera = camera
            if self._color_topic() != old_topic and 'color' in self._subs:
                self.destroy_subscription(self._subs.pop('color'))   # reconcile re-creates it

    # ── settings ─────────────────────────────────────────────────────────────

    def _on_own_params(self, params):
        for prm in params:
            if prm.name == 'video_max_fps' and prm.value > 0:
                self._video_period = 1.0 / float(prm.value)
            elif prm.name == 'jpeg_quality':
                self._jpeg_quality = int(prm.value)
            elif prm.name == 'depth_min_m':
                self._depth_min_mm = float(prm.value) * 1000.0
            elif prm.name == 'depth_max_m':
                self._depth_max_mm = float(prm.value) * 1000.0
        return SetParametersResult(successful=True)

    def _params_files(self, spec):
        files = []
        for path in filter(None, spec.split(':')):
            real = os.path.realpath(path)
            if not os.path.isfile(real):
                self.get_logger().warn(f'Params file {path} not found; not saving settings there')
            elif real not in files:     # --symlink-install: install/ copy is src/
                files.append(real)
        return files

    def _remember(self, setting, value):
        if self._save_files[setting.target]:
            self._unsaved[setting.target][setting.key] = value
            self._last_change = time.monotonic()

    def save_settings(self):
        """Write changed settings into the params files (debounced; also on shutdown)."""
        for target, values in self._unsaved.items():
            if not values:
                continue
            for path in self._save_files[target]:
                try:
                    param_file.update_file(path, values)
                except Exception as exc:
                    self.get_logger().error(f'Saving settings to {path} failed: {exc}')
            self.get_logger().info(f'Saved {", ".join(f"{k}={v}" for k, v in values.items())}')
            self._unsaved[target] = {}

    def _on_armed(self, msg):
        self._armed = bool(msg.data)

    def _on_mode(self, msg):
        try:
            mode = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(mode, dict):
            self._mode = mode

    def _settings_tick(self):
        with self._lock:
            queue, self._set_queue = self._set_queue, []
        for key, value in queue:
            self._apply_setting(key, value)
        now = time.monotonic()
        for key, (value, sent) in list(self._set_pending.items()):
            if now - sent > _PARAM_CALL_TIMEOUT_S:
                del self._set_pending[key]
                self._ack(key, False, value, 'gait node did not answer — try again')
        if now - self._last_change >= _SAVE_DELAY_S:
            self.save_settings()
        if now - self._last_settings < _SETTINGS_PERIOD_S:
            return
        self._last_settings = now
        running = self._gait_params.services_are_ready()
        if not running:
            self._gait_values = {}
            self._mode = None
        self._gait_running = running
        if self._gait_get_sent is not None and now - self._gait_get_sent > _PARAM_CALL_TIMEOUT_S:
            self.get_logger().warn('gait_node did not answer a settings read; retrying.',
                                   throttle_duration_sec=10.0)
            self._gait_get_sent = None
        if not self._deck_active():
            return
        if running and self._gait_get_sent is None:
            self._gait_get_sent = now
            self._gait_get_seq += 1
            seq = self._gait_get_seq
            self._gait_params.get_parameters(
                _GAIT_SETTINGS, lambda future: self._on_gait_values(future, seq))
        self._publish_settings()

    def _on_gait_values(self, future, seq):
        if seq != self._gait_get_seq:
            return                         # a reply we already gave up on
        self._gait_get_sent = None
        try:
            values = future.result().values
        except Exception as exc:    # node went away mid-call
            self.get_logger().warn(f'Reading gait_node settings failed: {exc}',
                                   throttle_duration_sec=10.0)
            return
        got = {}
        for key, v in zip(_GAIT_SETTINGS, values):
            value = parameter_value_to_python(v)
            if value is not None:          # NOT_SET: the node doesn't have it
                got[key] = value
        self._gait_values = got
        self._publish_settings()

    def _publish_settings(self):
        values = dict(self._gait_values)
        values.update({k: self.get_parameter(k).value for k in _BRIDGE_SETTINGS})
        msg = S.payload(values, {'gait': self._gait_running, 'bridge': True})
        self._mqtt.publish(self._t['settings'], json.dumps(msg, separators=(',', ':')), qos=0)

    def _ack(self, key, ok, value=None, text=''):
        self._mqtt.publish(self._t['settings_ack'], json.dumps(
            {'key': key, 'ok': ok, 'value': value, 'text': text}, separators=(',', ':')), qos=1)
        if not ok:
            self.get_logger().warn(f'Deck setting {key} refused: {text}')

    def _apply_setting(self, key, raw):
        try:
            setting, value = S.coerce(key, raw)
        except S.SettingError as exc:
            self._ack(key, False, raw, str(exc))
            return
        if setting.kind == 'action':
            self._run_action(key)
            return
        param = Parameter(key, value=value)
        if setting.target == 'bridge':
            result = self.set_parameters([param])[0]
            if result.successful:
                self._remember(setting, value)
            self._ack(key, result.successful, value, result.reason)
            self._publish_settings()
            return
        if not self._gait_params.services_are_ready():
            self._ack(key, False, value, 'gait node not running (hold L1+R1 to start the stack)')
            return

        self._set_pending[key] = (value, time.monotonic())

        def done(future):
            if self._set_pending.get(key, (None,))[0] == value:
                del self._set_pending[key]
            try:
                result = future.result().results[0]
            except Exception as exc:
                self._ack(key, False, value, f'gait node did not answer: {exc}')
                return
            if result.successful:
                self._gait_values[key] = value
                self._remember(setting, value)
                self.get_logger().info(f'Deck set {key} = {value}')
            self._ack(key, result.successful, value, result.reason)
            self._publish_settings()

        self._gait_params.set_parameters([param], done)

    def _run_action(self, key):
        if key == 'clear_map':
            self._clear_map()
            return
        if key == 'save_map':
            self._save_map()
            return
        if key == 'load_map':
            self._load_map_into_slam()
            return
        if key != 'shutdown':
            self._ack(key, False, True, 'not implemented')
            return
        if self._gait_running and self._armed:
            # Powering off releases the servos: a standing robot would drop.
            self._ack(key, False, True, 'disarm first (hold L3+R3) — the robot would drop')
            return
        self.get_logger().warn('Shutdown requested from the Deck — powering off.')
        self._ack(key, True, True, 'Shutting down — wait ~20 s before cutting the power')

        def poweroff():
            try:
                result = subprocess.run(_SHUTDOWN_CMD, capture_output=True, check=False, text=True, timeout=15)
                err = result.stderr.strip() if result.returncode else ''
            except (OSError, subprocess.SubprocessError) as exc:
                err = str(exc)
            if err:
                self.get_logger().error(f'Shutdown failed: {err}')
                self._ack(key, False, True, f'shutdown failed: {err}')

        # A moment for the ack to reach the Deck before the network goes down.
        threading.Timer(1.0, poweroff).start()

    def _clear_map(self):
        if not self._slam_reset.service_is_ready():
            self._ack('clear_map', False, True, 'SLAM not running (hold L1+R1 to start the stack)')
            return

        def done(future):
            ok = future.exception() is None
            self._ack('clear_map', ok, True, 'Map cleared' if ok else f'reset failed: {future.exception()}')

        self._slam_reset.call_async(Reset.Request()).add_done_callback(done)
        self.get_logger().info('Deck: clearing the SLAM map.')

    def _save_map(self):
        key = 'save_map'
        if not (self._slam_save.service_is_ready() and self._slam_serialize.service_is_ready()):
            self._ack(key, False, True, 'SLAM not running (hold L1+R1 to start the stack)')
            return
        if self._saving_map:
            self._ack(key, False, True, 'already saving')
            return
        try:
            os.makedirs(self._maps_dir, exist_ok=True)
        except OSError as exc:
            self._ack(key, False, True, f'cannot create {self._maps_dir}: {exc}')
            return
        name = time.strftime('map_%Y%m%d_%H%M%S')
        base = os.path.join(self._maps_dir, name)
        self._saving_map = True

        def fail(text):
            self._saving_map = False
            self._ack(key, False, True, text)

        # slam_toolbox's save_map shells out to nav2's map_saver_cli (.pgm + .yaml);
        # serialize_map writes the pose graph. Image first: it's what the Deck shows.
        def serialized(future):
            if future.exception() is not None or future.result().result != 0:
                fail(f'map image saved, pose graph failed: {future.exception() or "write error"}')
                return
            for ext in ('.yaml', '.pgm', '.posegraph', '.data'):
                link = os.path.join(self._maps_dir, 'static_map' + ext)
                try:
                    if os.path.lexists(link):
                        os.remove(link)
                    os.symlink(name + ext, link)
                except OSError as exc:
                    self.get_logger().warn(f'Could not link {link}: {exc}')
            self._saving_map = False
            self._static_map_payload = self._load_static_map()
            self._mqtt.publish(self._t['static_map'], self._static_map_payload, qos=1, retain=True)
            self._ack(key, True, True, f'Map saved: {name}')
            self.get_logger().info(f'Deck: saved the SLAM map to {base}.*')

        def saved(future):
            if future.exception() is not None:
                fail(f'save failed: {future.exception()}')
                return
            if future.result().result != 0 or not os.path.isfile(base + '.yaml'):
                fail('save failed: no map yet' if future.result().result == 1 else 'save failed')
                return
            self._slam_serialize.call_async(
                SerializePoseGraph.Request(filename=base)).add_done_callback(serialized)

        req = SaveMap.Request()
        req.name.data = base
        self._slam_save.call_async(req).add_done_callback(saved)
        self.get_logger().info(f'Deck: saving the SLAM map to {base}.*')

    def _static_map_base(self):
        return os.path.join(self._maps_dir, 'static_map')

    def _load_static_map(self):
        """The newest saved map as a <prefix>/static_map payload (b'' if none)."""
        path = self._static_map_base() + '.yaml'
        if not os.path.exists(path):
            return b''
        try:
            w, h, res, ox, oy, cells = static_map.load(path)
        except static_map.MapLoadError as exc:
            self.get_logger().warn(f'Saved map not loaded: {exc}')
            return b''
        self.get_logger().info(f'Static map for the Deck: {os.path.realpath(path)} ({w}x{h})')
        return P.pack_map(w, h, res, ox, oy, zlib.compress(cells.tobytes(), 6))

    def _load_map_into_slam(self):
        key = 'load_map'
        base = self._static_map_base()
        if not os.path.exists(base + '.posegraph'):
            self._ack(key, False, True, 'no saved map yet (Map > Save map first)')
            return
        if not self._slam_load.service_is_ready():
            self._ack(key, False, True, 'SLAM not running (hold L1+R1 to start the stack)')
            return
        if self._gait_running and self._mode and self._mode.get('mode') == 'auto_nav':
            self._ack(key, False, True, 'leave auto nav first (double-tap A)')
            return
        req = DeserializePoseGraph.Request()
        req.filename = os.path.realpath(base + '.posegraph')[:-len('.posegraph')]
        req.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE

        def done(future):
            ok = future.exception() is None
            self._ack(key, ok, True, 'Saved map loaded — SLAM continues from it'
                      if ok else f'load failed: {future.exception()}')

        self._slam_load.call_async(req).add_done_callback(done)
        self.get_logger().info(f'Deck: loading {req.filename} into slam_toolbox.')

    def _set_robot_pose(self, x, y, yaw):
        """Reload the saved pose graph with the robot at (x, y, yaw) in the map
        frame. slam_toolbox scan-matches the first scan near that pose, so it
        only has to be roughly right. Stops any nav goal first: the map -> odom
        transform jumps."""
        key = 'set_pose'
        base = self._static_map_base()
        if not os.path.exists(base + '.posegraph'):
            self._ack(key, False, None, 'no saved map to place the robot on (Map > Save map first)')
            return
        if not self._slam_load.service_is_ready():
            self._ack(key, False, None, 'SLAM not running (hold L1+R1 to start the stack)')
            return
        self._nav_cancel_pub.publish(Empty())
        req = DeserializePoseGraph.Request()
        req.filename = os.path.realpath(base + '.posegraph')[:-len('.posegraph')]
        req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = x, y, yaw
        where = f'{x:.1f}, {y:.1f} m facing {math.degrees(yaw):.0f}°'

        def done(future):
            ok = future.exception() is None
            self._ack(key, ok, [round(x, 3), round(y, 3), round(yaw, 3)],
                      f'Robot placed at {where}' if ok else f'placing failed: {future.exception()}')

        self._slam_load.call_async(req).add_done_callback(done)
        self.get_logger().info(f'Deck: robot is at {where}; loading {req.filename} into slam_toolbox there.')

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
            with open('/sys/class/thermal/thermal_zone0/temp', encoding='utf-8') as f:
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
            'camera': self._camera,
            'gait': {'running': self._gait_running, 'armed': self._gait_running and self._armed},
            # Forwarded disarmed too: by-hand IMU calibration runs then, and the
            # mode message carries result notes ("IMU zeroed"). The Deck hides "walk".
            'mode': self._mode if self._gait_running else None,
            'nav': self._nav if self._gait_running else None,
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
        if self._mic is not None:
            self._mic.stop()
        self.save_settings()
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
