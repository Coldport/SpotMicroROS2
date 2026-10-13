"""
nav_node — auto-nav: walks the robot to a point picked on the Deck's map.

  Deck (tap the map / A at the cursor) -> deck_bridge -> goal_topic
  -> plan on the SLAM map (planner.py) -> follow it -> cmd_vel_topic -> gait_node

Only drives while gait_node is in auto_nav mode (double-tap A on the Deck, while
armed; see gait_node.py). Leaving that mode — double-A again, B, disarming,
losing the Deck — cancels the goal. Touches no hardware itself: gait_node turns
cmd_vel into strides, and stops on its own if cmd_vel goes quiet.

Planning uses the live SLAM map (/map), which is in the same frame as the
robot's pose. Load a saved map into SLAM first (Deck menu: Map > Load saved map)
to navigate a known space straight away. The path is re-planned every
`replan_s` (the map grows and shifts as the robot walks) and the robot stops if
the lidar sees something within `stop_distance` ahead of its nose; it waits for
it to clear (or the map to route round it) and gives up after
`blocked_timeout_s`.

It only walks forward and turns, never sideways: if the path bends more than
`turn_in_place_deg` it stops, turns on the spot until it faces the path
(within `align_deg`), then walks straight. The Deck's D-pad up/down steps the
speed through `speed_levels` (fractions of max_linear / max_angular).

Topics:
  goal_topic     geometry_msgs/PoseStamped  in   goal (map frame; orientation ignored)
  cancel_topic   std_msgs/Empty             in   stop and forget the goal
  speed_topic    std_msgs/Int32             in   +1 / -1: next speed level up / down
  mode_topic     std_msgs/String (JSON)     in   gait_node's mode
  cmd_vel_topic  geometry_msgs/Twist        out  forward m/s, turn rad/s
  status_topic   std_msgs/String (JSON)     out  latched; deck_bridge shows it on the Deck:
                   {"state": idle|driving|blocked|arrived|failed, "text": "...",
                    "goal": [x, y] | null, "path": [[x, y], ...], "left_m": float,
                    "speed": fraction, "speed_mps": m/s at that speed}
  path_topic     nav_msgs/Path              out  the current plan (for Foxglove)
"""

import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Int32, String

from spot_micro_move import planner

_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavNode(Node):

    def __init__(self):
        super().__init__('nav_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('goal_topic', '/spotmicro_move/nav_goal')
        self.declare_parameter('cancel_topic', '/spotmicro_move/nav_cancel')
        self.declare_parameter('mode_topic', '/spotmicro_move/mode')
        self.declare_parameter('cmd_vel_topic', '/spotmicro_move/cmd_vel')
        self.declare_parameter('speed_topic', '/spotmicro_move/nav_speed')
        self.declare_parameter('status_topic', '/spotmicro_move/nav_status')
        self.declare_parameter('path_topic', '/spotmicro_move/nav_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('replan_s', 3.0)
        self.declare_parameter('plan_resolution', 0.1)     # m, planning grid cell
        self.declare_parameter('robot_radius', 0.17)       # m, centre -> furthest foot, plus a bit
        self.declare_parameter('inflation_radius', 0.35)   # m, paths prefer this far from walls
        self.declare_parameter('allow_unknown', False)     # plan through unexplored space
        self.declare_parameter('goal_snap', 0.5)           # m, move a goal in/near a wall this far
        self.declare_parameter('goal_tolerance', 0.15)     # m, close enough = arrived
        self.declare_parameter('lookahead', 0.35)          # m, pure-pursuit carrot distance
        self.declare_parameter('max_linear', 0.08)         # m/s (the crawl walks ~0.1 m/s flat out)
        self.declare_parameter('max_angular', 0.35)        # rad/s
        self.declare_parameter('turn_in_place_deg', 20.0)  # heading error above which it turns on the spot
        self.declare_parameter('align_deg', 6.0)           # ...and keeps turning until it's within this
        self.declare_parameter('walk_turn_frac', 0.4)      # steering while walking, fraction of max_angular
        self.declare_parameter('speed_levels', [0.25, 0.5, 0.75, 1.0, 1.25])  # D-pad steps through these
        self.declare_parameter('speed_level', 3)           # starting index into speed_levels
        self.declare_parameter('stop_distance', 0.15)      # m beyond the nose: stop for obstacles
        self.declare_parameter('body_half_length', 0.18)   # m, base_link -> nose (lidar hits inside are the robot)
        self.declare_parameter('body_half_width', 0.09)
        self.declare_parameter('blocked_timeout_s', 20.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._p = p
        self._map_frame = p('map_frame')
        self._base_frame = p('base_frame')

        self._map = None           # OccupancyGrid
        self._map_seq = 0
        self._costmap = None
        self._costmap_seq = -1
        self._mode = None
        self._goal = None          # (x, y)
        self._path = None
        self._follower = None
        self._last_plan = 0.0
        self._state = 'idle'
        self._text = ''
        self._left = 0.0
        self._blocked_since = None
        self._scan = None          # (time, LaserScan)
        self._laser_yaw = None
        self._last_status = 0.0
        self._speeds = sorted(float(v) for v in p('speed_levels') if float(v) > 0.0) or [1.0]
        self._speed_i = min(max(int(p('speed_level')), 0), len(self._speeds) - 1)

        self._cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self._status_pub = self.create_publisher(String, p('status_topic'), _LATCHED)
        self._path_pub = self.create_publisher(Path, p('path_topic'), _LATCHED)
        self.create_subscription(OccupancyGrid, p('map_topic'), self._on_map, _LATCHED)
        self.create_subscription(LaserScan, p('scan_topic'), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, p('goal_topic'), self._on_goal, 10)
        self.create_subscription(Empty, p('cancel_topic'), self._on_cancel, 10)
        self.create_subscription(Int32, p('speed_topic'), self._on_speed, 10)
        self.create_subscription(String, p('mode_topic'), self._on_mode, _LATCHED)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self.create_timer(1.0 / float(p('rate_hz')), self._tick)
        self._set_state('idle', '')
        self.get_logger().info('nav_node up: waiting for auto_nav mode (double-tap A on the Deck) and a goal.')

    # ── inputs ───────────────────────────────────────────────────────────────

    def _on_map(self, msg):
        self._map = msg
        self._map_seq += 1

    def _on_scan(self, msg):
        self._scan = (time.monotonic(), msg)

    def _on_mode(self, msg):
        try:
            mode = json.loads(msg.data).get('mode')
        except (ValueError, AttributeError):
            return
        was = self._mode
        self._mode = mode
        if was == 'auto_nav' and mode != 'auto_nav' and self._goal is not None:
            self._stop('idle', 'Cancelled: left auto nav')

    def _on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self._map_frame:
            self._set_state('failed', f'goal must be in the {self._map_frame} frame')
            return
        if self._mode != 'auto_nav':
            self._set_state('failed', 'Not in auto nav — double-tap A first')
            return
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self._path = None
        self._follower = None
        self._blocked_since = None
        self._last_plan = 0.0
        self.get_logger().info(f'Nav goal: ({self._goal[0]:.2f}, {self._goal[1]:.2f})')
        self._set_state('driving', 'Planning…')

    def _on_cancel(self, _msg):
        if self._goal is not None:
            self._stop('idle', 'Stopped')

    def _on_speed(self, msg):
        step = (msg.data > 0) - (msg.data < 0)
        self._speed_i = min(max(self._speed_i + step, 0), len(self._speeds) - 1)
        if self._follower is not None:
            self._follower.speed = self._speed()
        self.get_logger().info(f'Nav speed {self._speed() * 100:.0f}%')
        self._publish_status()

    def _speed(self):
        return self._speeds[self._speed_i]

    # ── helpers ──────────────────────────────────────────────────────────────

    def _pose(self):
        try:
            t = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
        except tf2_ros.TransformException:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, _yaw(tr.rotation)

    def _costmap_now(self):
        if self._map is None:
            return None
        if self._costmap_seq != self._map_seq:
            m = self._map
            p = self._p
            self._costmap = planner.build_costmap(
                m.data, m.info.width, m.info.height, m.info.resolution,
                m.info.origin.position.x, m.info.origin.position.y,
                plan_resolution=float(p('plan_resolution')), robot_radius=float(p('robot_radius')),
                inflation_radius=float(p('inflation_radius')), allow_unknown=bool(p('allow_unknown')))
            self._costmap_seq = self._map_seq
        return self._costmap

    def _obstacle_ahead(self):
        """Distance (m) to the nearest lidar return in the corridor ahead of the
        nose, or None. Ignores returns that fall inside the robot's own outline."""
        if self._scan is None or time.monotonic() - self._scan[0] > 1.0:
            return None
        msg = self._scan[1]
        if self._laser_yaw is None:
            try:
                t = self._tf.lookup_transform(self._base_frame, msg.header.frame_id or 'laser', Time())
                self._laser_yaw = (t.transform.translation.x, t.transform.translation.y,
                                   _yaw(t.transform.rotation))
            except tf2_ros.TransformException:
                return None
        lx, ly, lyaw = self._laser_yaw
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(len(r), dtype=np.float32) * msg.angle_increment + lyaw
        ok = np.isfinite(r) & (r > msg.range_min)
        x = lx + r[ok] * np.cos(a[ok])
        y = ly + r[ok] * np.sin(a[ok])
        hl, hw = float(self._p('body_half_length')), float(self._p('body_half_width'))
        reach = hl + float(self._p('stop_distance'))
        hit = (x > hl) & (x < reach) & (np.abs(y) < hw + 0.03)
        return float(x[hit].min() - hl) if hit.any() else None

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def _set_state(self, state, text):
        changed = (state, text) != (self._state, self._text)
        self._state, self._text = state, text
        if changed:
            if text:
                self.get_logger().info(f'Nav {state}: {text}')
            self._publish_status()

    def _publish_status(self):
        path = []
        if self._path:
            path = [[round(x, 2), round(y, 2)] for x, y in self._path]
        msg = {'state': self._state, 'text': self._text,
               'goal': [round(self._goal[0], 2), round(self._goal[1], 2)] if self._goal else None,
               'path': path, 'left_m': round(self._left, 2), 'speed': round(self._speed(), 3),
               'speed_mps': round(self._speed() * float(self._p('max_linear')), 3)}
        self._status_pub.publish(String(data=json.dumps(msg, separators=(',', ':'))))
        self._last_status = time.monotonic()

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self._path or []:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)

    def _stop(self, state, text):
        self._goal = None
        self._path = None
        self._follower = None
        self._blocked_since = None
        self._left = 0.0
        self._publish_cmd(0.0, 0.0)
        self._publish_path()
        self._set_state(state, text)

    def _replan(self, pose, now):
        cm = self._costmap_now()
        self._last_plan = now
        if cm is None:
            return 'no map yet — is SLAM running?'
        path, why = planner.plan(cm, pose[:2], self._goal, goal_snap=float(self._p('goal_snap')))
        if path is None:
            return why
        p = self._p
        self._path = path
        self._follower = planner.Follower(
            path, lookahead=float(p('lookahead')), goal_tolerance=float(p('goal_tolerance')),
            max_linear=float(p('max_linear')), max_angular=float(p('max_angular')),
            turn_in_place_deg=float(p('turn_in_place_deg')), align_deg=float(p('align_deg')),
            walk_turn_frac=float(p('walk_turn_frac')), speed=self._speed())
        self._publish_path()
        return ''

    # ── control loop ─────────────────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()
        if self._goal is None:
            if now - self._last_status > 2.0:
                self._publish_status()
            return
        if self._mode != 'auto_nav':
            self._stop('idle', 'Cancelled: left auto nav')
            return
        pose = self._pose()
        if pose is None:
            self._publish_cmd(0.0, 0.0)
            self._set_state('driving', 'Waiting for the robot pose (map -> base_link)…')
            return

        if self._follower is None or now - self._last_plan >= float(self._p('replan_s')):
            why = self._replan(pose, now)
            if why and self._follower is None:
                self._stop('failed', f'No path: {why}')
                return
            # A failed re-plan keeps following the last good path.

        ahead = self._obstacle_ahead()
        if ahead is not None:
            if self._blocked_since is None:
                self._blocked_since = now
            self._publish_cmd(0.0, 0.0)
            if now - self._blocked_since > float(self._p('blocked_timeout_s')):
                self._stop('failed', 'Blocked: something stayed in the way')
                return
            self._set_state('blocked', f'Obstacle {ahead:.2f} m ahead — waiting')
            if now - self._last_plan > 1.5:
                self._last_plan = 0.0     # re-plan next tick: the map may route round it
            return
        self._blocked_since = None

        v, w, left, arrived = self._follower.step(*pose)
        self._left = left
        if arrived:
            self._stop('arrived', 'Arrived')
            return
        self._publish_cmd(v, w)
        self._set_state('driving', f'{left:.1f} m to go')
        if now - self._last_status > 0.5:
            self._publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)  # pylint: disable=protected-access
        except Exception:   # noqa: BLE001 — shutting down anyway
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
