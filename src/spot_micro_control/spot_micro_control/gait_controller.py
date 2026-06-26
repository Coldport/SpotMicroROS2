"""
Trot gait controller for SpotMicro.

Subscribes:  /cmd_vel  (geometry_msgs/Twist)
Publishes:   /joint_states  (sensor_msgs/JointState)
             /odom          (nav_msgs/Odometry)  — dead-reckoned

Gait: diagonal trot — (FL + RR) swing while (FR + RL) stance, then swap.
Foot swing trajectory: cubic Bezier curve in the sagittal plane.
Body height is held constant via IK; cmd_vel drives step length & yaw rate.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure


# ── joint name order expected by robot_state_publisher ───────────────────────
# Must match the URDF joint names exactly.
# URDF leg order: rear_right, front_right, front_left, rear_left
# IK leg order:   leg_rightback, leg_rightfront, leg_leftfront, leg_leftback
JOINT_NAMES = [
    'rear_right_shoulder',  'rear_right_leg',  'rear_right_foot',
    'front_right_shoulder', 'front_right_leg', 'front_right_foot',
    'front_left_shoulder',  'front_left_leg',  'front_left_foot',
    'rear_left_shoulder',   'rear_left_leg',   'rear_left_foot',
]

# Trot diagonal pairs: (swing_indices_into_IK_order, stance_indices)
# IK order: 0=RB, 1=RF, 2=LF, 3=LB
# Pair A swings: RB(0) + LF(2)   Pair B swings: RF(1) + LB(3)
PAIR_A = [0, 2]   # rear_right + front_left
PAIR_B = [1, 3]   # front_right + rear_left


def bezier3(p0, p1, p2, p3, t):
    """Cubic Bezier point at parameter t in [0,1]."""
    u = 1.0 - t
    return u**3*p0 + 3*u**2*t*p1 + 3*u*t**2*p2 + t**3*p3


class GaitController(Node):

    # ── physical defaults ─────────────────────────────────────────────────────
    BODY_HEIGHT   = 0.18    # metres above ground
    STEP_HEIGHT   = 0.06    # peak foot lift during swing
    STEP_LENGTH   = 0.06    # max forward stride per step (at full speed)
    GAIT_FREQ     = 1.2     # complete cycles per second
    CTRL_HZ       = 50.0    # control loop rate

    # Nominal foot positions in global frame (x=fwd, y=up, z=right)
    # These are the default stance foot positions when the robot stands still.
    # x: ±half body length + small offset; y: ground; z: ±(hip + small offset)
    def _nominal_feet(self):
        L = self.sm.body_length / 2
        Z = self.sm.hip_length + self.sm.body_width / 2
        y = 0.0   # ground level in the IK global frame (y=0)
        return np.array([
            [-L,  y, -Z],   # leg_rightback
            [ L,  y, -Z],   # leg_rightfront
            [ L,  y,  Z],   # leg_leftfront
            [-L,  y,  Z],   # leg_leftback
        ])

    def __init__(self):
        super().__init__('gait_controller')

        self.sm = SpotMicroStickFigure(y=self.BODY_HEIGHT)

        self._cmd_vx  = 0.0   # forward velocity  m/s
        self._cmd_vy  = 0.0   # lateral velocity  m/s
        self._cmd_wz  = 0.0   # yaw rate          rad/s

        # Gait state
        self._phase     = 0.0          # 0..1 within one half-cycle
        self._pair_idx  = 0            # which pair (A=0, B=1) is currently swinging
        self._foot_pos  = self._nominal_feet().copy()  # current foot positions
        self._stance_start = self._nominal_feet().copy()  # where each foot was at stance start

        # Dead-reckoned pose
        self._x   = 0.0
        self._y_w = 0.0
        self._yaw = 0.0

        self._dt = 1.0 / self.CTRL_HZ

        self.pub_js   = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br    = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info('Gait controller ready — publish /cmd_vel to move')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg):
        self._cmd_vx = msg.linear.x
        self._cmd_vy = msg.linear.y
        self._cmd_wz = msg.angular.z

    def _control_loop(self):
        moving = (abs(self._cmd_vx) > 0.01 or
                  abs(self._cmd_vy) > 0.01 or
                  abs(self._cmd_wz) > 0.05)

        if moving:
            self._trot_step()
        else:
            self._stand_still()

        angles = self.sm.get_leg_angles()
        # IK order: RB, RF, LF, LB  →  URDF order: RR, FR, FL, RL
        flat = [*angles[0], *angles[1], *angles[2], *angles[3]]
        self._publish_joints(flat)
        self._publish_odom()

    # ── gait logic ────────────────────────────────────────────────────────────

    def _trot_step(self):
        half_period = 1.0 / (2.0 * self.GAIT_FREQ)
        d_phase = self._dt / half_period
        self._phase += d_phase

        swing_pair   = PAIR_A if self._pair_idx == 0 else PAIR_B
        stance_pair  = PAIR_B if self._pair_idx == 0 else PAIR_A

        t = min(self._phase, 1.0)

        # Step vector: how far each foot travels this half-cycle
        stride_x = self._cmd_vx * half_period * self.STEP_LENGTH / max(abs(self._cmd_vx), 0.3)
        stride_y = self._cmd_vy * half_period * self.STEP_LENGTH / max(abs(self._cmd_vy), 0.3)
        turn_r   = self._cmd_wz * half_period

        for i in swing_pair:
            nom  = self._nominal_feet()[i]
            # start of swing: current foot pos; end: nominal + stride
            p0 = self._stance_start[i].copy()
            p3 = nom.copy()
            p3[0] += stride_x * (1 if i in [1, 2] else -1) * 0.5
            p3[2] += stride_y * 0.5
            # Bezier control points — arc upward
            p1 = p0 + np.array([0.0,  self.STEP_HEIGHT * 0.5, 0.0])
            p2 = p3 + np.array([0.0,  self.STEP_HEIGHT,       0.0])
            self._foot_pos[i] = bezier3(p0, p1, p2, p3, t)

        # Stance feet push body backward (opposite of stride direction)
        push_x = -stride_x * d_phase
        push_z = -stride_y * d_phase
        dphi   = turn_r   * d_phase
        for i in stance_pair:
            self._foot_pos[i][0] += push_x
            self._foot_pos[i][2] += push_z

        # Dead-reckon world pose
        self._yaw   += self._cmd_wz * self._dt
        self._x     += math.cos(self._yaw) * self._cmd_vx * self._dt
        self._y_w   += math.sin(self._yaw) * self._cmd_vx * self._dt

        # Apply to IK model
        feet = self._foot_pos.copy()
        # Keep feet at ground level (y=0 in IK frame)
        feet[:, 1] = 0.0
        try:
            self.sm.set_absolute_foot_coordinates(feet)
        except Exception:
            pass   # IK can fail at joint limits — hold last pose

        if self._phase >= 1.0:
            self._phase = 0.0
            self._pair_idx = 1 - self._pair_idx
            self._stance_start = self._foot_pos.copy()

    def _stand_still(self):
        self._phase    = 0.0
        self._foot_pos = self._nominal_feet().copy()
        self._stance_start = self._foot_pos.copy()
        try:
            self.sm.set_absolute_foot_coordinates(self._foot_pos)
        except Exception:
            pass

    # ── publishers ────────────────────────────────────────────────────────────

    def _publish_joints(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = [float(a) for a in angles]
        self.pub_js.publish(msg)

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y_w
        t.transform.translation.z = self.BODY_HEIGHT
        half = self._yaw / 2.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self.tf_br.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y_w
        odom.pose.pose.position.z  = self.BODY_HEIGHT
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x  = self._cmd_vx
        odom.twist.twist.angular.z = self._cmd_wz
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
