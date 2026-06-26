"""
Simple point-to-point navigator for SpotMicro.

Subscribes:  /goal_pose  (geometry_msgs/PoseStamped)   — target in odom frame
             /odom       (nav_msgs/Odometry)            — from gait_controller
Publishes:   /cmd_vel    (geometry_msgs/Twist)

Two-phase controller:
  1. Rotate in place until heading error < threshold
  2. Drive forward; small angular correction keeps heading aligned
  Stop when distance to goal < arrival_radius.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


def _yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_wrap(a):
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


class PointNav(Node):

    LIN_SPEED      = 0.15   # m/s forward
    ANG_SPEED      = 0.6    # rad/s max rotation
    KP_ANG        = 1.8    # angular P gain
    HEADING_TOL   = 0.15   # rad — switch to drive phase
    ARRIVAL_RADIUS = 0.05   # m  — declare goal reached

    def __init__(self):
        super().__init__('point_nav')

        self._goal_x   = None
        self._goal_y   = None
        self._robot_x  = 0.0
        self._robot_y  = 0.0
        self._robot_yaw = 0.0
        self._active   = False

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_timer(0.05, self._control_loop)   # 20 Hz

        self.get_logger().info(
            'Point nav ready — publish a geometry_msgs/PoseStamped to /goal_pose')

    def _goal_cb(self, msg):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._active = True
        self.get_logger().info(
            f'New goal: ({self._goal_x:.2f}, {self._goal_y:.2f})')

    def _odom_cb(self, msg):
        self._robot_x   = msg.pose.pose.position.x
        self._robot_y   = msg.pose.pose.position.y
        self._robot_yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _control_loop(self):
        cmd = Twist()

        if not self._active or self._goal_x is None:
            self.pub_cmd.publish(cmd)
            return

        dx = self._goal_x - self._robot_x
        dy = self._goal_y - self._robot_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.ARRIVAL_RADIUS:
            self._active = False
            self.get_logger().info('Goal reached!')
            self.pub_cmd.publish(cmd)
            return

        desired_heading = math.atan2(dy, dx)
        heading_err     = _angle_wrap(desired_heading - self._robot_yaw)

        if abs(heading_err) > self.HEADING_TOL:
            # Phase 1: rotate in place
            cmd.angular.z = max(-self.ANG_SPEED,
                                min(self.ANG_SPEED,
                                    self.KP_ANG * heading_err))
        else:
            # Phase 2: drive forward with gentle heading correction
            cmd.linear.x  = self.LIN_SPEED
            cmd.angular.z = max(-self.ANG_SPEED * 0.5,
                                min(self.ANG_SPEED * 0.5,
                                    self.KP_ANG * heading_err))

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
