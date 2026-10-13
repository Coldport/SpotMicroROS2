"""
gait_node — crawl (one leg at a time) gait, driven by the Steam Deck.

Command path (see spot_micro_deck):
  Deck app -> MQTT <prefix>/deck/input -> deck_bridge -> ROS sensor_msgs/Joy
  -> this node -> spot_micro_ik.Dog -> PCA9685 servos

Left stick:  forward/back + strafe.  Right stick X: turn.
Once armed, the sticks drive it directly (no hold-to-walk button); centring
the sticks drops it to a stand at the current body height. Hold
`height_up_button` (default r1) / `height_down_button` (default l1) to raise /
lower the body; holding both does nothing (that's the Deck's L1+R1 stack
toggle chord).

This node opens the PCA9685/I2C bus and constructs its ServoDriver at startup
— there is no simulated/dry-run mode here (see spot_micro_ik.hardware if you
want that for bench testing away from the robot). What's gated is whether it
actually WRITES to servos:

ARM/DISARM: the node starts disarmed (I2C is open, but nothing is commanded).
Holding both stick-click buttons (`arm_buttons`, default l3+r3) together for
`arm_hold_s` (default 3s) arms it. Holding the same chord again disarms:
releases all servos (goes limp). Any of these also force a disarm: `estop_button` (default b), losing
the Deck connection, or `joy_topic` going quiet longer than `joy_timeout_s` —
each requires the full arm hold again afterward, on purpose.

On arming, legs are powered up one at a time (`arm_leg_stagger_s` apart,
default 0.4s) rather than all twelve servos at once, to limit the inrush
current surge on the servo supply. The confirmation jiggle runs after the last
leg has engaged.

Nothing here is wired into spot_micro_bringup; spot_micro_deck's
full_stack.launch.py (started by holding L1+R1 on the Deck) brings this node
up alongside the sensing stack, but — per the above — that alone never moves
a servo until L3+R3 is held too.

GAIT: `gait_type` "crawl" (default) lifts one leg at a time in the order
RL -> FL -> RR -> FR, so three feet are always on the ground; each leg gets
1/4 of the cycle (body shift, then swing — see BALANCE SHIFT). "trot" is the old diagonal-pair gait (FL+RR, FR+RL),
swinging for 1/2 of the cycle.

STANCE: feet sit a little outside the hips (`stance_y`), front feet a little
ahead of the front hips (`stance_x_front`) and rear feet a little behind the
rear hips (`stance_x_rear`) — a wider, longer support polygon. The gait
strides around those neutral points.

TRIM: `trim_fl` / `trim_fr` / `trim_rl` / `trim_rr` are fixed per-leg [x, y, z]
foot offsets (mm, same per-leg IK frame as the stance params) added to every
command for that leg, to compensate for static build/calibration error (e.g.
the front sitting low under the camera's weight, or one corner sitting
forward/high). z: + = extend the leg = raise that corner. y: + = foot further
out. x: same sign convention as `stance_x_front` / `stance_x_rear`.

BALANCE SHIFT: with three feet down, the body's centre of mass sits right on
the diagonal between two of them, so lifting the fourth tips it over that
diagonal. Before each leg lifts, the body slides toward the centroid of the
three feet that stay down (away from the lifted leg). Each leg's gait slot
starts with `shift_frac` of four-feet-down time while the body moves to the
next leg's target, then the leg swings while the body holds there. The slide
also spills `shift_overlap` (fraction of a swing) into the low ends of the
neighbouring swings and follows a quintic ease, so the sway is slow and has
no jerk at its start or end.
Target = gain * (centroid of the other three feet), using `level_half_length`
/ `level_half_width` as the foot rectangle; `balance_gain_lateral` and
`balance_gain_fore_aft` scale it, `balance_max_shift` clamps it (mm). Fore/aft
uses `ik_x_forward_sign` to map body-forward onto IK x. Crawl only (trot has
two feet down on a diagonal, which a body shift can't help).

TURNING: the turn stick rotates the body about its centre, so each foot's
stride follows an arc of the circle through it (front and rear feet also
move sideways, in opposite directions) instead of plain fore/aft pushes that
skid the feet. `turn_gain` is the arc length (mm) each foot sweeps per half
stride at full stick. Translation and turn are combined per foot, and the
total horizontal offset from the neutral point is clamped to
`max_foot_offset` mm so no stick combination (or param typo) can drive a
foot out of reach.

STABLE FLAG: `stable_topic` (std_msgs/Bool) is published every control tick:
True while no foot is in the air (standing, disarmed, or the four-feet-down
part of a crawl slot). scan_gate (this package, started by bringup) uses it to feed SLAM
only lidar scans taken while the body isn't tipped by a lifted leg.

ODOMETRY: odom -> base_link is broadcast every control tick (so launch the
sensing stack with static_odom:=false, as full_stack.launch.py does). Heading
is the IMU's yaw; position is dead-reckoned from the commanded stride (a stance
foot sweeps 2 * stride per stance phase), times `odom_scale` to correct for
slip. Without a fresh IMU the heading just holds. Rough, but it gives
slam_toolbox a starting guess instead of "never moved".

ARMED FLAG: `armed_topic` (std_msgs/Bool, latched) says whether the servos are
being driven; deck_bridge forwards it to the Deck's ARMED badge.

LIVE TUNING: the gait/stance/balance/leveling parameters in LIVE_PARAMS can be
changed while the node runs — `ros2 param set /gait_node step_height 40.0`, or
the Deck's settings menu (deck_bridge relays it, and also writes Deck changes
into config/move_params.yaml so they are used next launch). gait_type can only
switch while not walking.

SETTLE: when the sticks centre, feet don't snap to the stand pose — each foot
moves toward it at no more than `settle_speed` mm/s, so a leg that was up
mid-swing comes down gently.

LEVELING: body roll/pitch from the IMU (`imu_topic`, BNO055 fused orientation)
is integrated into per-leg height offsets (front vs rear for pitch, left vs
right for roll) so the body stays level. Mount orientation is handled by
`imu_roll_sign` / `imu_pitch_sign` / `imu_swap_roll_pitch`; "level" can be
trimmed with `level_trim_roll` / `level_trim_pitch`. If the IMU goes quiet the
correction bleeds back to zero. The current IMU roll/pitch is logged on arming
so the signs/trim can be checked.

WALKING STABILISATION: with `level_enabled` and `walk_stab_enabled`, walking
also uses the IMU beyond the slow leveling integral:
  * the integral follows a low-passed tilt (`walk_level_filter_s`) while
    walking, so it doesn't chase the rock of every step;
  * a fast PD term (`walk_level_kp` on tilt, `walk_level_kd` on the gyro rate)
    adds to the leveling height offsets, mostly on the planted feet (a raised
    foot takes less of it, so its ground clearance is kept);
  * the body leans away from the side it is tipping toward
    (`walk_com_gain` mm/deg, clamped to `walk_com_max`), even with body sway off;
  * past `walk_tilt_slow_deg` of filtered tilt the steps slow down, to
    `walk_min_speed` of the normal rate at `walk_tilt_stop_deg`;
  * walking with the turn stick centred holds the IMU heading
    (`walk_heading_gain` turn per rad of drift, clamped to `walk_heading_max`;
    same turn sign as `cmd_vel_turn_sign`).

MODES (armed only; `mode_topic`, a latched std_msgs/String of JSON, says which
one is active — deck_bridge forwards it to the Deck):
  walk       the default, as above.
  sentry     double-tap `sentry_button` (default y) to enter or leave. The feet
             stay planted and the sticks pose the body instead: left stick X =
             roll, left stick Y = pitch, right stick X = yaw, up to
             `sentry_max_{roll,pitch,yaw}_deg`, slewed at `sentry_rate_deg_s`.
             R1/L1 still raise/lower it. IMU leveling holds its last correction.
             OBJECT TRACKING: pick something on the Deck's camera view (touch
             and drag a box, or D-pad + A); track_node follows it in the color
             stream and publishes its angle off-centre on `track_topic`. While
             it is locked on, yaw and pitch turn the camera toward it at
             `track_gain` deg/s per deg of error (outside `track_deadband_deg`),
             within the sentry limits and rate; roll stays on the left stick.
             X on the Deck stops tracking; leaving sentry stops it too.
  calibrate  hold `calib_buttons` (default y + dpad_up) for `calib_hold_s`
             (3s) to enter or leave. The feet go to `calib_pose` (IK x, y, z —
             the pose the calibration file was verified at: feet under the
             shoulders) with no stance/trim/leveling, so only the servo offsets
             decide how the robot stands. L1/R1 pick the leg (FL, FR, RL, RR),
             L2/R2 the joint (hip, thigh, knee); D-pad up/down changes its
             offset by `calib_step_deg`, right/left by `calib_fine_step_deg`
             (held = repeat). The new offset is live at once. Leaving the mode —
             including by disarming or estop — writes the offsets into
             `calibration_path` (servo_calibration.yaml; servo_calibration.txt
             is not touched).
             ANGLE TEST: A sends the selected joint to a set servo angle
             (before the offset; it starts where IK has the joint) and A again
             returns it. Right stick up/down changes that angle by
             `calib_angle_step_deg`, right/left by `calib_angle_fine_deg`; the
             joint slews there at `calib_angle_speed_deg_s`. Then use the
             D-pad to change the offset until the joint really sits at that
             angle (e.g. square to the body at 90). Best done with the robot
             on a stand: one joint moving tips a standing robot.
  trick      a scripted move (spot_micro_move/tricks.py), started from walk:
             tap `trick_button` (default x) once / twice / three times for
             `trick_taps` (butt shake, dance, push-ups), or hold X and press a
             D-pad direction for `trick_dpad` (up: handshake, down: sit,
             left: bow, right: pee). The robot glides to its stand pose,
             plays it, and goes back to walk. Press X again to stop early.
  auto_nav   double-tap `nav_button` (default a) to enter or leave. The robot
             walks itself to a point picked on the Deck's map: nav_node
             (this package) plans the route and publishes `cmd_vel_topic`
             (geometry_msgs/Twist: linear.x forward / linear.y left m/s,
             angular.z turn-left rad/s), which this node turns into strides —
             the inverse of the ODOMETRY estimate below, so `odom_scale`
             calibrates both. cmd_vel older than `cmd_vel_timeout_s` counts as
             stop. The left stick still drives by hand (no turning) and
             overrides cmd_vel while pushed; the right stick, D-pad and L1/R1/
             L2/R2 drive the Deck's map view, so height, sentry, tricks, IMU
             zero / calibration are off until you leave. `cmd_vel_turn_sign`
             flips the turn if the robot turns the wrong way for a +angular.z.
  imu_calib  hold `imu_calib_buttons` (default y + dpad_left) for
             `imu_chord_hold_s` (1s) to start (again to cancel): works out how
             the IMU is mounted, so roll/pitch come out on the right axes with
             the right signs. Drone style, five poses: level, left side down,
             right side down, nose down, nose up. Armed, the robot does it
             itself on flat ground, tilting `imu_calib_roll_deg` each way.
             Disarmed, the Deck prompts you through them by hand, with a live
             tilt gauge; a pose is recorded once it is IMU_CAL_TILT_DEG from
             level and from every pose already done, and held still. The
             spot_micro_imu node does the maths and saves the result to
             imu_params.yaml (see spot_micro_imu/mount.py).
Leaving sentry, calibrate or a trick glides the feet back to the stand pose
before the sticks can walk again.

IMU ZERO: hold `imu_zero_buttons` (default y + dpad_down) for
`imu_chord_hold_s` (armed or not): the robot's current attitude becomes level
and its current heading yaw 0 (spot_micro_imu's `zero` service; saved to
imu_params.yaml). Do it with the robot standing on flat ground. This
supersedes level_trim_roll / level_trim_pitch, which should stay 0.

NOTES: the mode message also carries the last result to show on the Deck
("note": {"seq", "text", "ok"}), e.g. "IMU zeroed".
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from spot_micro_deck.chord import ChordDetector, DoubleTap, Repeat, TapCounter
from spot_micro_deck.protocol import AXIS_NAMES, BUTTON_NAMES
from spot_micro_ik import Dog, load_calibration
from spot_micro_ik.calibration import JOINTS, save_offsets, with_offset
from spot_micro_ik.dog import DEFAULT_STAND_POSE
from spot_micro_move import tricks

_AX = {name: i for i, name in enumerate(AXIS_NAMES)}
_BTN = {name: i for i, name in enumerate(BUTTON_NAMES)}

# gait_type -> (per-leg phase offset in the cycle, fraction of the cycle in each leg's slot).
# In crawl, the first `shift_frac` of a slot is four-feet-down body shift, the rest swing.
# crawl: one leg in the air at a time, in the standard quadruped walk order
#        (rear then front on the same side: RL, FL, RR, FR).
# trot:  diagonal pairs swing together.
GAITS = {
    'crawl': ({'RL': 0.0, 'FL': 0.25, 'RR': 0.5, 'FR': 0.75}, 0.25),
    'trot': ({'FL': 0.0, 'RR': 0.0, 'FR': 0.5, 'RL': 0.5}, 0.5),
}
LEFT_SIDE = {"FL", "RL"}   # sign convention for the turn approximation below
FRONT = {"FL", "FR"}
LEGS = ('FL', 'FR', 'RL', 'RR')
MODES = ('walk', 'sentry', 'calibrate', 'trick', 'imu_calib', 'auto_nav')
DPAD = ('dpad_up', 'dpad_down', 'dpad_left', 'dpad_right')
# IMU axis calibration steps, in order; 'return' only when armed.
IMU_CAL_STEPS = ('level', 'left', 'right', 'front', 'back', 'return')
IMU_CAL_TILT_DEG = 12.0      # by hand: a pose counts once this far from level and every earlier pose
IMU_CAL_HOLD_S = 1.0         # ... and held still this long (level: 1.5 s)

# Parameters that can be changed while the node runs (`ros2 param set`, or the
# Deck's settings menu through deck_bridge). The rest — topics, buttons, arm
# timing, control/PWM rates, calibration — are only read at startup.
LIVE_PARAMS = frozenset([
    'height_rate', 'z_min', 'z_max', 'speed_scale', 'stick_deadzone',
    'stand_z', 'step_height', 'step_length', 'step_lateral', 'turn_gain', 'max_foot_offset',
    'gait_freq', 'gait_type', 'shift_frac', 'shift_overlap',
    'balance_enabled', 'balance_gain_lateral', 'balance_gain_fore_aft', 'balance_max_shift',
    'ik_x_forward_sign', 'stance_x_front', 'stance_x_rear', 'stance_y', 'settle_speed',
    *(f'trim_{leg.lower()}' for leg in ('FL', 'FR', 'RL', 'RR')),
    'level_enabled', 'imu_timeout_s', 'imu_roll_sign', 'imu_pitch_sign', 'imu_swap_roll_pitch',
    'level_trim_roll', 'level_trim_pitch', 'level_gain', 'level_max_deg', 'level_deadband_deg',
    'level_half_length', 'level_half_width',
    'sentry_max_roll_deg', 'sentry_max_pitch_deg', 'sentry_max_yaw_deg', 'sentry_rate_deg_s',
    'sentry_foot_speed', 'calib_step_deg', 'calib_fine_step_deg', 'trick_foot_speed',
    'calib_angle_step_deg', 'calib_angle_fine_deg', 'calib_angle_speed_deg_s',
    'track_gain', 'track_deadband_deg', 'track_timeout_s',
    'walk_stab_enabled', 'walk_level_kp', 'walk_level_kd', 'walk_level_filter_s',
    'walk_com_gain', 'walk_com_max', 'walk_tilt_slow_deg', 'walk_tilt_stop_deg', 'walk_min_speed',
    'walk_heading_gain', 'walk_heading_max',
])


def _deadzone(value, dz):
    return 0.0 if abs(value) < dz else value


def _rescaled(value, dz):
    """Deadzone that still reaches 0..1 smoothly past the dead band (for posing,
    where a jump from 0 straight to `dz` would jerk the body)."""
    if abs(value) < dz or dz >= 1.0:
        return 0.0
    return math.copysign((abs(value) - dz) / (1.0 - dz), value)


def _rotation_offset(fwd, left, up, roll, pitch, yaw):
    """(d_fwd, d_left, d_up) in the body frame of a foot at (fwd, left, up) that
    stays put on the ground while the body turns by roll/pitch/yaw (deg,
    REP-103: +roll = right side down, +pitch = nose down, +yaw = turn left).
    The foot in the new body frame is R^T p with R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    # Rz^T
    x, y, z = cy * fwd + sy * left, -sy * fwd + cy * left, up
    # Ry^T
    x, z = cp * x - sp * z, sp * x + cp * z
    # Rx^T
    y, z = cr * y + sr * z, -sr * y + cr * z
    return x - fwd, y - left, z - up


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait_node')

        self.declare_parameter('calibration_path', 'config/servo_calibration.yaml')
        self.declare_parameter('joy_topic', '/spotmicro_deck/joy')
        self.declare_parameter('height_up_button', 'r1')
        self.declare_parameter('height_down_button', 'l1')
        self.declare_parameter('height_rate', 40.0)       # mm/s while a height button is held
        self.declare_parameter('z_min', 120.0)            # mm, body height clamp
        self.declare_parameter('z_max', 220.0)
        self.declare_parameter('speed_scale', 0.5)        # scales stick input -> stride
        self.declare_parameter('estop_button', 'b')
        self.declare_parameter('arm_buttons', ['l3', 'r3'])
        self.declare_parameter('arm_hold_s', 3.0)
        self.declare_parameter('arm_leg_stagger_s', 0.4)  # delay between legs powering up on arm
        self.declare_parameter('joy_timeout_s', 0.5)
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('servo_pwm_hz', 50)         # PCA9685 PWM rate; servos see one update per PWM frame
        self.declare_parameter('stick_deadzone', 0.08)

        self.declare_parameter('stand_z', DEFAULT_STAND_POSE['z'])
        self.declare_parameter('step_height', 30.0)     # mm foot lift during swing
        self.declare_parameter('step_length', 40.0)      # mm max fore/aft stride
        self.declare_parameter('step_lateral', 20.0)      # mm max lateral stride
        self.declare_parameter('turn_gain', 25.0)         # mm of arc per foot per half stride at full turn stick
        self.declare_parameter('max_foot_offset', 60.0)   # mm clamp on each foot's horizontal stride offset
        self.declare_parameter('stable_topic', '/spotmicro_move/stable')
        self.declare_parameter('odom_scale', 1.0)         # measured / estimated distance walked
        self.declare_parameter('armed_topic', '/spotmicro_move/armed')
        self.declare_parameter('gait_freq', 1.0)          # full step cycles per second at any commanded speed
        self.declare_parameter('gait_type', 'crawl')      # 'crawl' (one leg at a time) or 'trot'
        self.declare_parameter('shift_frac', 0.35)        # fraction of each leg's slot with 4 feet down, shifting body
        self.declare_parameter('shift_overlap', 0.4)      # fraction of a swing the body slide may spill into
        self.declare_parameter('balance_enabled', True)
        self.declare_parameter('balance_gain_lateral', 1.0)   # x centroid of the 3 support feet, side to side
        self.declare_parameter('balance_gain_fore_aft', 0.0)  # x centroid, front/back (needs ik_x_forward_sign right)
        self.declare_parameter('balance_max_shift', 40.0)     # mm clamp per axis
        self.declare_parameter('ik_x_forward_sign', -1.0)     # +1 if IK +x is body-forward, -1 if backward

        # Neutral foot placement (mm, per-leg IK frame: +x forward, +y away from the body).
        self.declare_parameter('stance_x_front', 50.0)    # front feet this far ahead of their hips
        self.declare_parameter('stance_x_rear', -50.0)    # rear feet this far behind (negative = back)
        self.declare_parameter('stance_y', 15.0)          # all feet this far outward
        self.declare_parameter('settle_speed', 60.0)      # mm/s max foot speed returning to stand
        for leg in LEGS:                                  # mm [x, y, z] static per-leg foot offset
            self.declare_parameter(f'trim_{leg.lower()}', [0.0, 0.0, 0.0])

        self.declare_parameter('level_enabled', True)
        self.declare_parameter('imu_topic', '/spotmicro_imu/imu/data')
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('imu_roll_sign', 1.0)      # flip to -1 if the IMU is mounted mirrored
        self.declare_parameter('imu_pitch_sign', 1.0)
        self.declare_parameter('imu_swap_roll_pitch', False)  # IMU mounted rotated 90 deg
        self.declare_parameter('level_trim_roll', 0.0)    # deg, IMU reading that counts as level
        self.declare_parameter('level_trim_pitch', 0.0)
        self.declare_parameter('level_gain', 2.0)         # 1/s, integral gain on tilt error
        self.declare_parameter('level_max_deg', 10.0)     # clamp on the correction angle
        self.declare_parameter('level_deadband_deg', 0.5)
        self.declare_parameter('level_half_length', 93.0)  # mm, body centre -> front/rear hip
        self.declare_parameter('level_half_width', 80.0)   # mm, body centre -> foot laterally
        # Walking stabilisation (see WALKING STABILISATION); only with level_enabled.
        self.declare_parameter('walk_stab_enabled', True)
        self.declare_parameter('walk_level_kp', 0.5)       # deg correction per deg of tilt
        self.declare_parameter('walk_level_kd', 0.05)      # deg correction per deg/s of tilt rate
        self.declare_parameter('walk_level_filter_s', 0.4)  # low-pass on tilt for the slow terms
        self.declare_parameter('walk_com_gain', 1.0)       # mm of body shift per deg of tilt
        self.declare_parameter('walk_com_max', 15.0)       # mm clamp per axis
        self.declare_parameter('walk_tilt_slow_deg', 6.0)  # tilt where the step rate starts to drop
        self.declare_parameter('walk_tilt_stop_deg', 14.0)  # ... and where it bottoms out
        self.declare_parameter('walk_min_speed', 0.35)     # step rate fraction at walk_tilt_stop_deg
        self.declare_parameter('walk_heading_gain', 1.0)   # turn stick per rad of heading drift
        self.declare_parameter('walk_heading_max', 0.3)    # clamp on that turn

        self.declare_parameter('mode_topic', '/spotmicro_move/mode')
        self.declare_parameter('sentry_button', 'y')       # double-tap: sentry mode on/off
        self.declare_parameter('double_tap_s', 0.4)
        self.declare_parameter('sentry_max_roll_deg', 15.0)
        self.declare_parameter('sentry_max_pitch_deg', 15.0)
        self.declare_parameter('sentry_max_yaw_deg', 15.0)
        self.declare_parameter('sentry_rate_deg_s', 45.0)  # body pose slew rate
        self.declare_parameter('sentry_foot_speed', 200.0)  # mm/s cap on foot motion while posing
        self.declare_parameter('calib_buttons', ['y', 'dpad_up'])  # hold: calibration mode on/off
        self.declare_parameter('calib_hold_s', 3.0)
        self.declare_parameter('calib_pose', [0.0, 0.0, 120.0])     # IK [x, y, z] mm of every foot
        self.declare_parameter('calib_step_deg', 1.0)       # D-pad up/down
        self.declare_parameter('calib_fine_step_deg', 0.25)  # D-pad right/left
        self.declare_parameter('calib_max_offset_deg', 30.0)
        self.declare_parameter('calib_angle_step_deg', 5.0)      # angle test: right stick up/down
        self.declare_parameter('calib_angle_fine_deg', 1.0)      # angle test: right stick left/right
        self.declare_parameter('calib_angle_speed_deg_s', 60.0)  # joint slew rate in the angle test
        self.declare_parameter('track_topic', '/spotmicro_track/status')
        self.declare_parameter('track_gain', 1.5)            # sentry tracking: deg/s of pose per deg of error
        self.declare_parameter('track_deadband_deg', 1.5)
        self.declare_parameter('track_timeout_s', 0.5)       # older tracker status = not tracking
        self.declare_parameter('trick_button', 'x')
        self.declare_parameter('trick_taps', ['butt_shake', 'dance', 'pushups'])   # X once / twice / 3x
        self.declare_parameter('trick_dpad', ['handshake', 'sit', 'bow', 'pee'])   # X + up/down/left/right
        self.declare_parameter('trick_foot_speed', 300.0)   # mm/s safety cap on foot motion in tricks
        self.declare_parameter('imu_zero_buttons', ['y', 'dpad_down'])
        self.declare_parameter('imu_calib_buttons', ['y', 'dpad_left'])
        self.declare_parameter('imu_chord_hold_s', 1.0)
        self.declare_parameter('imu_service_ns', '/spotmicro_imu')
        self.declare_parameter('imu_calib_roll_deg', 15.0)  # armed axis calibration: body roll each way
        self.declare_parameter('nav_button', 'a')           # double-tap: auto_nav mode on/off
        self.declare_parameter('cmd_vel_topic', '/spotmicro_move/cmd_vel')
        self.declare_parameter('cmd_vel_timeout_s', 0.6)
        self.declare_parameter('cmd_vel_turn_sign', 1.0)

        p = self.get_parameter
        self._joy_timeout = float(p('joy_timeout_s').value)
        self._up_button = str(p('height_up_button').value)
        self._down_button = str(p('height_down_button').value)
        self._estop_button = str(p('estop_button').value)
        arm_buttons = list(p('arm_buttons').value)
        arm_hold_s = float(p('arm_hold_s').value)
        if str(p('gait_type').value) not in GAITS:
            raise ValueError(f"gait_type '{p('gait_type').value}' not in {list(GAITS)}")
        self._t = 0.0                    # gait clock, only advances while walking
        self._z = float(p('stand_z').value)
        self._level_roll = 0.0           # deg, accumulated correction
        self._level_pitch = 0.0
        self._apply_tunables()
        for leg, t in self._trim.items():
            if len(t) != 3:
                raise ValueError(f'trim_{leg.lower()} must be [x, y, z], got {t}')

        for name in (self._up_button, self._down_button):
            if name not in _BTN:
                raise ValueError(f"height button '{name}' not in {BUTTON_NAMES}")
        if self._estop_button not in _BTN:
            raise ValueError(f"estop_button '{self._estop_button}' not in {BUTTON_NAMES}")
        for name in arm_buttons:
            if name not in _BTN:
                raise ValueError(f"arm_buttons entry '{name}' not in {BUTTON_NAMES}")

        self._calibration_path = str(p('calibration_path').value)
        self._calibration = load_calibration(self._calibration_path)
        self._pwm_hz = int(p('servo_pwm_hz').value)
        self.dog = None
        self._open_servos()   # opens I2C now; starts disarmed below

        self._armed = False
        self._arm_chord = ChordDetector([_BTN[n] for n in arm_buttons], arm_hold_s)
        self._arm_hold_s = arm_hold_s
        self._arm_progress = 0

        # ── modes (see MODES in the module docstring) ──
        sentry_button = str(p('sentry_button').value)
        calib_buttons = list(p('calib_buttons').value)
        trick_button = str(p('trick_button').value)
        imu_zero_buttons = list(p('imu_zero_buttons').value)
        imu_calib_buttons = list(p('imu_calib_buttons').value)
        nav_button = str(p('nav_button').value)
        for name in [sentry_button, *calib_buttons, trick_button, *imu_zero_buttons, *imu_calib_buttons,
                     nav_button]:
            if name not in _BTN:
                raise ValueError(f"mode button '{name}' not in {BUTTON_NAMES}")
        self._trick_taps_map = [str(n) for n in p('trick_taps').value]
        self._trick_dpad = [str(n) for n in p('trick_dpad').value]
        if len(self._trick_dpad) != len(DPAD):
            raise ValueError(f'trick_dpad must name {len(DPAD)} tricks (up, down, left, right)')
        for name in self._trick_taps_map + self._trick_dpad:
            if name and name not in tricks.TRICKS:
                raise ValueError(f"trick '{name}' not in {sorted(tricks.TRICKS)}")
        self._calib_pose = [float(v) for v in p('calib_pose').value]
        if len(self._calib_pose) != 3:
            raise ValueError(f'calib_pose must be [x, y, z], got {self._calib_pose}')
        self._calib_max = float(p('calib_max_offset_deg').value)
        self._mode = 'walk'
        self._sentry_tap = DoubleTap(_BTN[sentry_button], float(p('double_tap_s').value))
        self._sentry_button = sentry_button
        self._nav_tap = DoubleTap(_BTN[nav_button], float(p('double_tap_s').value))
        self._nav_button = nav_button
        self._cmd_vel = (0.0, 0.0, 0.0)   # forward, left (m/s), turn left (rad/s) from nav_node
        self._cmd_vel_time = 0.0
        self._cmd_vel_timeout = float(p('cmd_vel_timeout_s').value)
        self._cmd_vel_turn_sign = 1.0 if float(p('cmd_vel_turn_sign').value) >= 0 else -1.0
        self._nav_manual = False          # left stick is overriding cmd_vel
        self._calib_chord = ChordDetector([_BTN[n] for n in calib_buttons], float(p('calib_hold_s').value))
        self._calib_buttons = calib_buttons
        self._pose = [0.0, 0.0, 0.0]      # sentry roll, pitch, yaw (deg), as commanded
        self._calib_leg = 0               # index into LEGS
        self._calib_joint = 0             # index into JOINTS
        self._calib_dirty = False
        self._calib_note = ''             # last save result, for the Deck
        self._calib_wait_release = False  # ignore D-pad until the mode chord is let go
        # D-pad and right-stick steps auto-repeat; leg/joint switches and A are one per press.
        self._calib_keys = {name: Repeat() if name.startswith(('dpad', 'rs_')) else Repeat(1e9)
                            for name in ('dpad_up', 'dpad_down', 'dpad_left', 'dpad_right',
                                         'l1', 'r1', 'l2', 'r2', 'a',
                                         'rs_up', 'rs_down', 'rs_left', 'rs_right')}
        # Angle test (calibrate mode, A): the selected joint is sent to a set servo angle.
        self._calib_angle_mode = False
        self._calib_targets = {}          # (leg, joint) -> target servo angle (deg, before offset)
        self._calib_slew = {}             # (leg, joint) -> angle commanded now, while it differs from IK
        # Sentry object tracking (track_node, picked on the Deck).
        self._track = None                # last status dict from track_node
        self._track_time = 0.0
        self._track_was_on = False        # last sentry tick followed the tracker
        self._needs_settle = False        # glide back to stand before walking again
        # Tricks: X taps, or X held + a D-pad direction.
        self._trick_button = trick_button
        self._trick_tapper = TapCounter(_BTN[trick_button], float(p('double_tap_s').value))
        self._trick_prev = {name: False for name in (trick_button, *DPAD)}
        self._trick = None                # {'name', 'frames', 't0'} while mode == 'trick'
        # IMU zero / axis calibration (the maths runs in spot_micro_imu).
        imu_hold = float(p('imu_chord_hold_s').value)
        self._imu_zero_chord = ChordDetector([_BTN[n] for n in imu_zero_buttons], imu_hold)
        self._imu_cal_chord = ChordDetector([_BTN[n] for n in imu_calib_buttons], imu_hold)
        self._imu_zero_buttons = imu_zero_buttons
        self._imu_calib_buttons = imu_calib_buttons
        self._imu_cal_roll = float(p('imu_calib_roll_deg').value)
        self._imu_cal = None              # axis calibration progress while mode == 'imu_calib'
        ns = str(p('imu_service_ns').value).rstrip('/')
        self._imu_srv = {name: self.create_client(Trigger, f'{ns}/{name}')
                         for name in ('zero', *(f'axis_calib/{s}' for s in IMU_CAL_STEPS[:-1]))}
        self._note = None                 # last result for the Deck: {'seq', 'text', 'ok'}
        self._note_seq = 0
        self._last_mode_pub = 0.0
        self._mode_pub = self.create_publisher(
            String, p('mode_topic').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self._last_joy = None            # sensor_msgs/Joy
        self._last_joy_time = 0.0
        self._last_tick = time.monotonic()

        self._engage_queue = []          # legs still to power up, one at a time, after arming
        self._engage_tick = 0
        self._jiggle_queue = []          # pending z-targets for the non-blocking arm jiggle
        self._jiggle_tick = 0

        self._feet = None                # leg -> [x, y, z] last commanded (before leveling)

        self._imu_rp = None              # (roll, pitch) deg, body frame, trim applied
        self._imu_up = None              # world up in the body frame (unit vector)
        self._imu_rate = 0.0             # rad/s, |angular velocity|
        self._imu_rp_rate = (0.0, 0.0)   # deg/s, roll / pitch rate (same signs as _imu_rp)
        self._tilt_f = (0.0, 0.0)        # low-passed roll / pitch (deg), see WALKING STABILISATION
        self._stab = (0.0, 0.0)          # fast roll / pitch correction (deg) while walking
        self._stab_w = {leg: 1.0 for leg in LEGS}   # how much of it each leg takes (swing legs less)
        self._gait_rate = 1.0            # step-rate scale from tilt
        self._heading_ref = None         # IMU yaw (rad) held while walking without turning
        self._last_imu_time = 0.0

        self._stable = True              # no foot in the air (see STABLE FLAG)
        self._odom = [0.0, 0.0, 0.0]     # x, y (m), yaw (rad) in the odom frame
        self._odom_v = (0.0, 0.0)        # body forward, left (m/s), set by _gait_tick
        self._imu_yaw = 0.0
        self._odom_scale = float(p('odom_scale').value)
        self._tf = TransformBroadcaster(self)
        self._stable_pub = self.create_publisher(Bool, p('stable_topic').value, 10)
        # Latched, so deck_bridge (and the Deck's ARMED badge) see it at any time.
        self._armed_pub = self.create_publisher(
            Bool, p('armed_topic').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._publish_armed()
        self._publish_mode()

        self.create_subscription(Joy, p('joy_topic').value, self._on_joy, 10)
        self.create_subscription(Twist, p('cmd_vel_topic').value, self._on_cmd_vel, 10)
        # Always subscribed (cheap) so leveling can be switched on live.
        self.create_subscription(Imu, p('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(String, p('track_topic').value, self._on_track, 10)
        self._control_dt = 1.0 / float(p('control_hz').value)
        self._jiggle_ticks_per_step = max(1, round(0.12 / self._control_dt))
        self._engage_ticks_per_leg = max(1, round(float(p('arm_leg_stagger_s').value) / self._control_dt))
        self.create_timer(self._control_dt, self._control_loop)

        # Registered last: rclpy runs these on every set, declare_parameter included.
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_set)

        # Deliberately NOT calling self.dog.stand() here: the driver is live (real I2C)
        # from construction, and standing would move real servos before anyone armed it.
        self.get_logger().info(
            f'gait_node up, DISARMED (I2C open, nothing commanded). '
            f'Listening on {p("joy_topic").value}. '
            f'Hold "{"+".join(arm_buttons)}" for {arm_hold_s:.0f}s to arm/disarm, '
            f'sticks walk once armed, "{self._up_button}"/"{self._down_button}" raise/lower, '
            f'"{self._estop_button}" to go limp. Once armed: double-tap "{sentry_button}" '
            f'for sentry (pose in place), hold "{"+".join(calib_buttons)}" for calibration, '
            f'tap "{trick_button}" (or hold it + D-pad) for tricks, double-tap "{nav_button}" for auto nav. '
            f'Hold "{"+".join(imu_zero_buttons)}" to zero the IMU, '
            f'"{"+".join(imu_calib_buttons)}" to calibrate its axes.')

    # ── live parameters ──────────────────────────────────────────────────────

    def _apply_tunables(self, changed=()):
        """(Re)read every LIVE_PARAMS parameter into the fields the control loop
        uses. Called at startup and after each accepted live change (from the
        Deck's settings menu, via deck_bridge, or `ros2 param set`)."""
        p = self.get_parameter
        self._height_rate = float(p('height_rate').value)
        self._z_min = float(p('z_min').value)
        self._z_max = float(p('z_max').value)
        self._speed_scale = float(p('speed_scale').value)
        self._deadzone = float(p('stick_deadzone').value)
        self._stand_z = float(p('stand_z').value)
        if 'stand_z' in changed:
            self._z = self._stand_z      # new standing height takes over from R1/L1
        self._z = min(max(self._z, self._z_min), self._z_max)   # live body height (R1/L1)
        self._step_height = float(p('step_height').value)
        self._step_length = float(p('step_length').value)
        self._step_lateral = float(p('step_lateral').value)
        self._turn_gain = float(p('turn_gain').value)
        self._max_foot_offset = max(float(p('max_foot_offset').value), 0.0)
        self._gait_freq = max(float(p('gait_freq').value), 1e-3)
        gait_type = str(p('gait_type').value)
        self._gait_offsets, self._slot_frac = GAITS[gait_type]
        # Trot keeps its old timing: no four-feet-down shift window.
        self._shift_frac = min(max(float(p('shift_frac').value), 0.0), 0.9) if gait_type == 'crawl' else 0.0
        self._shift_overlap = min(max(float(p('shift_overlap').value), 0.0), 0.5)
        self._swing_start = self._slot_frac * self._shift_frac   # leg-local phase where swing begins
        # Legs in the order they swing, for looking up the previous slot's shift target.
        self._swing_order = sorted(self._gait_offsets, key=self._gait_offsets.get)
        self._balance = bool(p('balance_enabled').value) and gait_type == 'crawl'
        self._bal_gain_y = float(p('balance_gain_lateral').value)
        self._bal_gain_x = float(p('balance_gain_fore_aft').value)
        self._bal_max = float(p('balance_max_shift').value)
        self._ik_x_sign = 1.0 if float(p('ik_x_forward_sign').value) >= 0 else -1.0
        self._stance_x_front = float(p('stance_x_front').value)
        self._stance_x_rear = float(p('stance_x_rear').value)
        self._stance_y = float(p('stance_y').value)
        self._settle_speed = max(float(p('settle_speed').value), 1.0)
        self._trim = {leg: [float(v) for v in p(f'trim_{leg.lower()}').value] for leg in LEGS}

        self._level_enabled = bool(p('level_enabled').value)
        self._imu_timeout = float(p('imu_timeout_s').value)
        self._imu_roll_sign = float(p('imu_roll_sign').value)
        self._imu_pitch_sign = float(p('imu_pitch_sign').value)
        self._imu_swap = bool(p('imu_swap_roll_pitch').value)
        self._trim_roll = float(p('level_trim_roll').value)
        self._trim_pitch = float(p('level_trim_pitch').value)
        self._level_gain = float(p('level_gain').value)
        self._level_max = float(p('level_max_deg').value)
        self._level_deadband = float(p('level_deadband_deg').value)
        self._half_length = float(p('level_half_length').value)
        self._half_width = float(p('level_half_width').value)
        if 'level_enabled' in changed and not self._level_enabled:
            self._level_roll = 0.0
            self._level_pitch = 0.0
            self._stab = (0.0, 0.0)
            self._gait_rate = 1.0

        self._sentry_max = [float(p(f'sentry_max_{a}_deg').value) for a in ('roll', 'pitch', 'yaw')]
        self._sentry_rate = max(float(p('sentry_rate_deg_s').value), 1.0)
        self._sentry_foot_speed = max(float(p('sentry_foot_speed').value), 1.0)
        self._calib_step = float(p('calib_step_deg').value)
        self._calib_fine_step = float(p('calib_fine_step_deg').value)
        self._trick_foot_speed = max(float(p('trick_foot_speed').value), 1.0)
        self._calib_angle_step = float(p('calib_angle_step_deg').value)
        self._calib_angle_fine = float(p('calib_angle_fine_deg').value)
        self._calib_angle_speed = max(float(p('calib_angle_speed_deg_s').value), 1.0)
        self._track_gain = max(float(p('track_gain').value), 0.0)
        self._track_deadband = max(float(p('track_deadband_deg').value), 0.0)
        self._track_timeout = float(p('track_timeout_s').value)

        self._walk_stab = bool(p('walk_stab_enabled').value)
        self._walk_kp = float(p('walk_level_kp').value)
        self._walk_kd = float(p('walk_level_kd').value)
        self._walk_filter_s = max(float(p('walk_level_filter_s').value), 0.0)
        self._walk_com_gain = float(p('walk_com_gain').value)
        self._walk_com_max = max(float(p('walk_com_max').value), 0.0)
        self._walk_tilt_slow = float(p('walk_tilt_slow_deg').value)
        self._walk_tilt_stop = max(float(p('walk_tilt_stop_deg').value), self._walk_tilt_slow + 0.1)
        self._walk_min_speed = min(max(float(p('walk_min_speed').value), 0.05), 1.0)
        self._walk_heading_gain = float(p('walk_heading_gain').value)
        self._walk_heading_max = max(float(p('walk_heading_max').value), 0.0)

    def _validate_params(self, params):
        """Pre-set check for live changes: only LIVE_PARAMS, with sane values, and
        no gait_type switch mid-stride (the phase tables would jump the feet)."""
        new = {prm.name: prm.value for prm in params}
        merged = {name: self.get_parameter(name).value for name in ('z_min', 'z_max')}
        merged.update({k: v for k, v in new.items() if k in merged})
        for name, value in new.items():
            if name not in LIVE_PARAMS:
                return SetParametersResult(
                    successful=False, reason=f'{name} is only read at startup; restart gait_node')
            if name == 'gait_type':
                if value not in GAITS:
                    return SetParametersResult(successful=False,
                                               reason=f'gait_type must be one of {list(GAITS)}')
                if self._t > 0.0:
                    return SetParametersResult(successful=False,
                                               reason='stop walking before changing the gait')
            if name.startswith('trim_') and len(value) != 3:
                return SetParametersResult(successful=False, reason=f'{name} must be [x, y, z]')
            if name == 'gait_freq' and value <= 0.0:
                return SetParametersResult(successful=False, reason='gait_freq must be > 0')
        if merged['z_min'] >= merged['z_max']:
            return SetParametersResult(successful=False, reason='z_min must be below z_max')
        return SetParametersResult(successful=True)

    def _on_params_set(self, params):
        names = {prm.name for prm in params}
        self._apply_tunables(names)
        self.get_logger().info('Live update: ' + ', '.join(
            f'{prm.name}={prm.value}' for prm in params))

    def _publish_armed(self):
        self._armed_pub.publish(Bool(data=self._armed))

    # ── input ────────────────────────────────────────────────────────────────

    def _on_joy(self, msg):
        self._last_joy = msg
        self._last_joy_time = time.monotonic()

    def _on_cmd_vel(self, msg):
        self._cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_vel_time = time.monotonic()

    def _on_imu(self, msg):
        q = msg.orientation
        roll = math.degrees(math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                       1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))))
        if self._imu_swap:
            roll, pitch = pitch, roll
        roll = self._imu_roll_sign * roll - self._trim_roll
        pitch = self._imu_pitch_sign * pitch - self._trim_pitch
        self._imu_rp = (roll, pitch)
        self._imu_up = (2.0 * (q.x * q.z - q.w * q.y), 2.0 * (q.y * q.z + q.w * q.x),
                        1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        w = msg.angular_velocity
        self._imu_rate = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        rr, pr = math.degrees(w.x), math.degrees(w.y)
        if self._imu_swap:
            rr, pr = pr, rr
        self._imu_rp_rate = (self._imu_roll_sign * rr, self._imu_pitch_sign * pr)
        self._imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._last_imu_time = time.monotonic()

    def _on_track(self, msg):
        try:
            track = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(track, dict):
            self._track = track
            self._track_time = time.monotonic()

    def _tracking(self):
        """None when track_node isn't following anything (idle, or its status is
        stale); (0, 0) while it is but has no fix (starting / lost: hold the
        pose); else (yaw_err, pitch_err) deg: + = target left of / below centre."""
        t = self._track
        if t is None or time.monotonic() - self._track_time > self._track_timeout:
            return None
        state = t.get('state')
        if state in ('starting', 'lost'):
            return 0.0, 0.0
        if state != 'tracking':
            return None
        try:
            yaw, pitch = (float(v) for v in t['err_deg'])
        except (KeyError, TypeError, ValueError):
            return 0.0, 0.0
        return yaw, pitch

    def _imu_fresh(self):
        return (self._imu_rp is not None
                and time.monotonic() - self._last_imu_time <= self._imu_timeout)

    def _joy_fresh(self):
        return (self._last_joy is not None
                and time.monotonic() - self._last_joy_time <= self._joy_timeout)

    # ── control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        self._stable = True   # _gait_tick clears it while a foot is up
        self._odom_v = (0.0, 0.0)
        try:
            self._control_step()
        except OSError as e:
            # I2C write failed (Errno 121 etc.) — seen when the servo load browns out
            # the bus mid-stride. Used to kill the node; now disarm and stay up.
            self._on_i2c_fault(e)
        self._stable_pub.publish(Bool(data=self._stable))
        self._publish_odom()

    def _publish_odom(self):
        """Integrate the commanded body velocity along the IMU heading (see ODOMETRY)."""
        o = self._odom
        if self._imu_fresh():
            o[2] = self._imu_yaw
        fwd, left = self._odom_v
        c, s = math.cos(o[2]), math.sin(o[2])
        o[0] += (fwd * c - left * s) * self._control_dt
        o[1] += (fwd * s + left * c) * self._control_dt
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = o[0], o[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(o[2] / 2.0), math.cos(o[2] / 2.0)
        self._tf.sendTransform(t)

    # ── I2C faults ───────────────────────────────────────────────────────────

    def _open_servos(self):
        """(Re)open the PCA9685. Returns True on success. A PCA9685 that browned
        out comes back reset (asleep, default prescaler), so after any I2C fault
        it has to be re-initialised, not just written to again."""
        try:
            self.dog = Dog(self._calibration, dry_run=False, pwm_hz=self._pwm_hz)
            return True
        except (OSError, ValueError) as e:
            self.dog = None
            self.get_logger().error(
                f'Cannot open the PCA9685 servo board on I2C ({e}). The bus or board has '
                f'no power or is wedged — check `i2cdetect -y 1` (expect 0x40 and the '
                f'IMU at 0x28/0x29). Usually needs a power-cycle of the robot. '
                f'Will retry on the next arm.')
            return False

    def _on_i2c_fault(self, err):
        was_armed = self._armed
        self._armed = False
        self._publish_armed()
        self._exit_mode()
        self._engage_queue = []
        self._jiggle_queue = []
        self._t = 0.0
        self._feet = None
        self._level_roll = 0.0
        self._level_pitch = 0.0
        self._stab = (0.0, 0.0)
        self._heading_ref = None
        try:
            if self.dog is not None:
                self.dog.release()
        except OSError:
            pass
        self.dog = None   # force a full re-init on the next arm
        self.get_logger().error(
            f'I2C error talking to the servos ({err}){" — DISARMED" if was_armed else ""}. '
            f'Likely a servo-supply brownout or loose I2C wiring. Hold the arm chord to retry.')

    def _control_step(self):
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        if not self._joy_fresh():
            self._t = 0.0
            self._arm_chord.reset()
            self._arm_progress = 0
            self._jiggle_queue = []
            self._sentry_tap.reset()
            self._nav_tap.reset()
            self._calib_chord.reset()
            self._imu_zero_chord.reset()
            self._imu_cal_chord.reset()
            self._trick_tapper.reset()
            if self._armed:
                self._disarm('lost Joy/Deck connection')
            elif self._mode == 'imu_calib':
                self._end_imu_cal('IMU calibration cancelled: lost the Deck', False)
            return

        joy = self._last_joy

        fired = self._arm_chord.update(joy.buttons, now)
        held = int(self._arm_chord.held_for(now))
        if held and held != self._arm_progress and held < self._arm_hold_s:
            what = 'disarm' if self._armed else 'arm'
            self.get_logger().info(f'Arm chord held {held}/{self._arm_hold_s:.0f}s (to {what}) ...')
        self._arm_progress = held
        if fired:
            self._toggle_armed()
        self._mode_inputs(joy, now)

        if joy.buttons[_BTN[self._estop_button]]:
            self._jiggle_queue = []
            if self._armed:
                self._disarm('estop')
            elif self._mode == 'imu_calib':
                self._end_imu_cal('IMU calibration cancelled', False)
            self._t = 0.0
            return

        if not self._armed:
            if self._mode == 'imu_calib':
                self._imu_cal_tick(now, dt)   # by hand: nothing is commanded
            return   # idle: servos released, nothing to command

        if self._engage_queue:
            self._engage_tick_step()
            return

        if self._jiggle_queue:
            self._jiggle_tick_step()
            return

        if self._mode not in ('walk', 'auto_nav'):
            self._decay_stab(dt)
        if self._mode == 'calibrate':
            self._calib_tick(joy, now, dt)
            return
        if self._mode == 'trick':
            self._trick_tick(now, dt)
            return
        if self._mode == 'imu_calib':
            self._imu_cal_tick(now, dt)
            return

        up = bool(joy.buttons[_BTN[self._up_button]])
        down = bool(joy.buttons[_BTN[self._down_button]])
        if up != down and self._mode != 'auto_nav':   # both held = Deck stack-toggle chord, leave height alone
            dz = self._height_rate * dt * (1.0 if up else -1.0)
            self._z = min(max(self._z + dz, self._z_min), self._z_max)

        if self._mode == 'sentry':
            self._sentry_tick(joy, now, dt)
            return

        # Stick-up reads as negative from the Deck, so flip it: forward = +.
        vx = -_deadzone(joy.axes[_AX['left_y']], self._deadzone)
        vy = _deadzone(joy.axes[_AX['left_x']], self._deadzone)     # strafe left = +
        wz = _deadzone(joy.axes[_AX['right_x']], self._deadzone)    # turn = +
        scale = self._speed_scale
        if self._mode == 'auto_nav':
            # The right stick moves the Deck's map cursor here, so it never turns the robot.
            wz = 0.0
            manual = vx != 0.0 or vy != 0.0
            if manual != self._nav_manual:
                self._nav_manual = manual
                self._publish_mode()
            if not manual:
                vx, vy, wz = self._cmd_vel_strides()
                scale = 1.0   # already in stride units

        walking = not self._needs_settle and (vx != 0.0 or vy != 0.0 or wz != 0.0)
        self._update_leveling(dt, walking)

        if self._needs_settle:
            # Just left sentry/calibrate: glide back to the stand pose before walking.
            self._t = 0.0
            if self._settle_tick(dt):
                self._needs_settle = False
            return

        if not walking:
            self._t = 0.0
            self._heading_ref = None
            self._settle_tick(dt)
            return

        vx *= scale
        vy *= scale
        wz *= scale
        wz = self._heading_hold(vx, vy, wz)

        self._t += dt * self._gait_rate
        self._gait_tick(vx, vy, wz)

    def _stabilising(self):
        """The walking IMU terms are on (leveling + walk_stab_enabled) and the IMU is live."""
        return self._level_enabled and self._walk_stab and self._imu_fresh()

    def _heading_hold(self, vx, vy, wz):
        """While walking with no turn asked for, steer back to the IMU heading
        held since the turn stick was let go, so a straight walk stays straight
        (feet slip unevenly, and a crawl veers). Any turn command, or stopping,
        lets go of the held heading."""
        if wz != 0.0 or (vx == 0.0 and vy == 0.0) or not self._stabilising() \
                or self._walk_heading_gain <= 0.0:
            self._heading_ref = None
            return wz
        if self._heading_ref is None:
            self._heading_ref = self._imu_yaw
            return wz
        err = math.atan2(math.sin(self._heading_ref - self._imu_yaw),
                         math.cos(self._heading_ref - self._imu_yaw))   # + = need to turn left
        lim = self._walk_heading_max
        # Same mapping as auto nav's cmd_vel: +angular.z (left) -> cmd_vel_turn_sign * wz.
        return self._cmd_vel_turn_sign * max(-lim, min(lim, self._walk_heading_gain * err))

    def _cmd_vel_strides(self):
        """nav_node's cmd_vel as stick-equivalent (vx, vy, wz), where 1.0 = a
        full stride (before speed_scale): the inverse of _gait_tick's odometry
        estimate for translation, and of _turn_offset's arc for turning."""
        if time.monotonic() - self._cmd_vel_time > self._cmd_vel_timeout:
            return 0.0, 0.0, 0.0
        fwd, left, turn = self._cmd_vel
        stance_len = 1.0 - (self._slot_frac - self._swing_start)
        k = 2e-3 * self._gait_freq / stance_len * self._odom_scale     # mm stride -> m/s
        vx = fwd / (self._step_length * k) if self._step_length > 0.0 else 0.0
        vy = left / (self._step_lateral * k) if self._step_lateral > 0.0 else 0.0
        # A stance foot sweeps 2 * turn_gain mm of arc at its radius r (as in
        # _turn_offset) per stance phase; average the front and rear radii.
        side = self._half_width + self._stance_y
        r = 0.5 * (math.hypot(self._half_length + self._ik_x_sign * self._stance_x_front, side)
                   + math.hypot(-self._half_length + self._ik_x_sign * self._stance_x_rear, side))
        w_full = 2.0 * self._turn_gain / r * self._gait_freq / stance_len   # rad/s at wz = 1
        wz = self._cmd_vel_turn_sign * turn / w_full if self._turn_gain > 0.0 else 0.0
        return (max(-1.0, min(1.0, vx)), max(-1.0, min(1.0, vy)), max(-1.0, min(1.0, wz)))

    # ── foot placement ───────────────────────────────────────────────────────

    def _neutral(self, leg):
        """Stand position for one leg: slightly out, front forward, rear back."""
        x = self._stance_x_front if leg in FRONT else self._stance_x_rear
        return [x, self._stance_y, self._z]

    def _write_feet(self, feet, raw=False):
        """Command every leg, remembering the pre-leveling target and adding
        the static per-leg xyz trim and IMU leveling height offset on top
        (unless `raw`: calibration mode wants plain IK)."""
        self._feet = {leg: list(pos) for leg, pos in feet.items()}
        for leg, (x, y, z) in feet.items():
            if raw:
                self.dog.move_leg(leg, x, y, z, joint_angles=self._calib_overrides(leg))
                continue
            tx, ty, tz = self._trim[leg]
            self.dog.move_leg(leg, x + tx, y + ty, z + tz + self._level_dz(leg))

    def _move_toward(self, targets, dt, speed, raw=False):
        """Move each foot toward targets[leg] at <= `speed` mm/s. Returns True
        once every foot is there."""
        if self._feet is None:
            self._write_feet(targets, raw)
            return True
        max_step = speed * dt
        feet = {}
        arrived = True
        for leg in LEGS:
            cur = self._feet[leg]
            tgt = targets[leg]
            d = [t - c for t, c in zip(tgt, cur)]
            dist = math.sqrt(sum(v * v for v in d))
            if dist <= max_step or dist == 0.0:
                feet[leg] = list(tgt)
            else:
                k = max_step / dist
                feet[leg] = [c + v * k for c, v in zip(cur, d)]
                arrived = False
        self._write_feet(feet, raw)
        return arrived

    def _settle_tick(self, dt):
        """Walk each foot toward its stand position at <= settle_speed, so a leg
        caught mid-swing lowers gently instead of dropping."""
        return self._move_toward({leg: self._neutral(leg) for leg in LEGS}, dt, self._settle_speed)

    # ── leveling ─────────────────────────────────────────────────────────────

    def _update_leveling(self, dt, walking=False):
        stab = walking and self._walk_stab
        if not walking:
            self._stab_w = {leg: 1.0 for leg in LEGS}
        if not (self._level_enabled and self._imu_fresh()):
            # No IMU (or leveling off): bleed the corrections back to neutral
            # rather than hold stale ones.
            decay = max(0.0, 1.0 - dt)
            self._level_roll *= decay
            self._level_pitch *= decay
            self._stab = (0.0, 0.0)
            self._gait_rate = 1.0
            if self._level_enabled:
                self.get_logger().warn('No fresh IMU data — leveling paused.', throttle_duration_sec=5.0)
            return
        roll, pitch = self._imu_rp
        a = dt / (self._walk_filter_s + dt) if self._walk_filter_s + dt > 0.0 else 1.0
        self._tilt_f = tuple(f + a * (v - f) for f, v in zip(self._tilt_f, (roll, pitch)))
        # Walking rocks the body every step; integrating that would chase it,
        # so the slow integral follows the filtered tilt then (the fast terms
        # below handle the rocking).
        i_roll, i_pitch = self._tilt_f if stab else (roll, pitch)
        if abs(i_roll) > self._level_deadband:
            self._level_roll += self._level_gain * i_roll * dt
        if abs(i_pitch) > self._level_deadband:
            self._level_pitch += self._level_gain * i_pitch * dt
        lim = self._level_max
        self._level_roll = max(-lim, min(lim, self._level_roll))
        self._level_pitch = max(-lim, min(lim, self._level_pitch))

        if stab:
            # Fast PD correction on the raw tilt and gyro rate, on top of the integral.
            rr, pr = self._imu_rp_rate
            self._stab = (max(-lim, min(lim, self._walk_kp * roll + self._walk_kd * rr)),
                          max(-lim, min(lim, self._walk_kp * pitch + self._walk_kd * pr)))
            # Tipping a lot: slow the steps down (never to a stop mid-swing).
            tilt = math.hypot(*self._tilt_f)
            over = (tilt - self._walk_tilt_slow) / (self._walk_tilt_stop - self._walk_tilt_slow)
            self._gait_rate = 1.0 - min(max(over, 0.0), 1.0) * (1.0 - self._walk_min_speed)
        else:
            self._decay_stab(dt)

    def _decay_stab(self, dt):
        """Not walking: ease the fast correction out over ~0.2 s instead of dropping it."""
        k = max(0.0, 1.0 - dt / 0.2)
        self._stab = (self._stab[0] * k, self._stab[1] * k)
        self._gait_rate = 1.0

    def _level_dz(self, leg):
        """Extra leg extension (mm) for this leg from the leveling correction.
        REP-103: +pitch = nose down -> extend front legs; +roll = right side
        down -> extend right legs."""
        if not self._level_enabled:
            return 0.0
        w = self._stab_w[leg]
        pitch = self._level_pitch + w * self._stab[1]
        roll = self._level_roll + w * self._stab[0]
        dz = self._half_length * math.sin(math.radians(pitch))
        dz *= 1.0 if leg in FRONT else -1.0
        droll = self._half_width * math.sin(math.radians(roll))
        dz += -droll if leg in LEFT_SIDE else droll
        return dz

    # ── gait ─────────────────────────────────────────────────────────────────

    def _shift_target(self, leg):
        """Body shift (bx forward, by left, mm, body frame) that puts the centre
        of mass over the three feet left down while `leg` swings: the centroid
        of the other three corners of the foot rectangle, i.e. -(corner)/3."""
        sx = 1.0 if leg in FRONT else -1.0
        sy = 1.0 if leg in LEFT_SIDE else -1.0
        half_w = self._half_width + self._stance_y
        bx = -self._bal_gain_x * sx * self._half_length / 3.0
        by = -self._bal_gain_y * sy * half_w / 3.0
        lim = self._bal_max
        return max(-lim, min(lim, bx)), max(-lim, min(lim, by))

    def _body_shift(self, phase, period):
        """Scheduled body shift for this gait phase. Each slot's slide runs
        across its four-feet-down window AND spills `shift_overlap` of a swing
        into the neighbouring swings (the last bit of the previous leg's swing,
        the first bit of this one's), where the foot is barely off the ground.
        The longer window plus a quintic (smootherstep) profile keeps the
        body's acceleration low and its jerk zero at both ends, so the sway
        eases in and out rather than lurching. The very first slot after
        starting to walk slides from zero (the body starts centred)."""
        if not self._balance:
            return 0.0, 0.0
        n = len(self._swing_order)
        k = int(phase / self._slot_frac) % n
        u = (phase - self._gait_offsets[self._swing_order[k]]) / self._slot_frac   # 0..1 in slot
        a = self._shift_frac
        ov = self._shift_overlap * (1.0 - a)                # overlap, in slot units
        first_slot = self._t < self._slot_frac * period
        if u > 1.0 - ov:
            # Tail of this leg's swing: the next slot's slide has started.
            frm = self._shift_target(self._swing_order[k])
            to = self._shift_target(self._swing_order[(k + 1) % n])
            x = (u - 1.0 + ov) / (a + 2.0 * ov)
        else:
            frm = (0.0, 0.0) if first_slot else self._shift_target(self._swing_order[(k - 1) % n])
            to = self._shift_target(self._swing_order[k])
            if first_slot:
                x = u / (a + ov) if a + ov > 0.0 else 1.0
            else:
                x = (u + ov) / (a + 2.0 * ov) if a + ov > 0.0 else 1.0
        x = min(max(x, 0.0), 1.0)
        x = x * x * x * (x * (6.0 * x - 15.0) + 10.0)      # smootherstep
        return tuple(f + (t - f) * x for f, t in zip(frm, to))

    def _turn_offset(self, leg, wz, s):
        """IK-frame (x, y) foot offset at stride position s (-1..+1) for a
        turn command wz: the neutral foot point rotated about the body centre
        by s * (half-stride yaw), so the foot traces an arc rather than a
        straight fore/aft line. Sign kept from the old tank-style turn: +wz
        moves the left feet along +IK x in stance-start, right feet -IK x."""
        if wz == 0.0:
            return 0.0, 0.0
        side = 1.0 if leg in LEFT_SIDE else -1.0
        nx = self._neutral(leg)[0]
        # Foot point in the body frame (forward, left), mm from the body centre.
        fwd = (self._half_length if leg in FRONT else -self._half_length) + self._ik_x_sign * nx
        left = side * (self._half_width + self._stance_y)
        radius = math.hypot(fwd, left)
        # Yaw that sweeps turn_gain mm of arc; chosen so the left feet's
        # forward motion matches the previous convention.
        yaw = -self._ik_x_sign * self._turn_gain * wz / radius * s
        c, sn = math.cos(yaw), math.sin(yaw)
        d_fwd = c * fwd - sn * left - fwd
        d_left = sn * fwd + c * left - left
        return self._ik_x_sign * d_fwd, side * d_left

    def _gait_tick(self, vx, vy, wz):
        period = 1.0 / self._gait_freq
        phase = (self._t % period) / period   # 0..1

        # Diagonal stick + turn would otherwise stack up to ~1.7x a single axis.
        mag = math.sqrt(vx * vx + vy * vy + wz * wz)
        if mag > 1.0:
            vx, vy, wz = vx / mag, vy / mag, wz / mag
        stride_x = self._step_length * vx
        stride_y = self._step_lateral * vy
        bx, by = self._body_shift(phase, period)

        swing_len = self._slot_frac - self._swing_start
        stance_len = 1.0 - swing_len
        k = 2e-3 * self._gait_freq * self._gait_rate / stance_len * self._odom_scale   # mm stride -> m/s
        self._odom_v = (stride_x * k, stride_y * k)
        if self._stabilising() and self._walk_com_gain != 0.0:
            # Lean the body away from the side it is tipping toward (+roll =
            # right side down -> shift left; +pitch = nose down -> shift back).
            lim = self._walk_com_max
            roll_f, pitch_f = self._tilt_f
            by += max(-lim, min(lim, self._walk_com_gain * roll_f))
            bx -= max(-lim, min(lim, self._walk_com_gain * pitch_f))

        feet = {}
        for leg in LEGS:
            side = 1.0 if leg in LEFT_SIDE else -1.0
            local = (phase - self._gait_offsets[leg]) % 1.0

            if self._swing_start <= local < self._slot_frac:
                sw = (local - self._swing_start) / swing_len   # 0..1 across the swing
                # Eased so the foot leaves and lands with zero velocity (soft
                # lift-off and touchdown): cosine ease fore/aft, sin^2 lift.
                ease = 0.5 - 0.5 * math.cos(math.pi * sw)
                s = 2.0 * ease - 1.0                           # -1 -> +1
                lift = math.sin(math.pi * sw) ** 2
                dz = -self._step_height * lift                 # lift (shorten the leg)
                self._stable = False
                # A raised foot takes little of the fast tilt correction, so the
                # correction can't eat its ground clearance; eased back in as it lands.
                self._stab_w[leg] = 1.0 - lift
            else:
                st = ((local - self._slot_frac) % 1.0) / stance_len   # 0..1 across stance
                s = 1.0 - 2.0 * st                             # +1 -> -1
                dz = 0.0
                self._stab_w[leg] = 1.0

            tx, ty = self._turn_offset(leg, wz, s)
            # IK y is per-leg "outward", so a body-frame strafe is +y on the
            # left legs and -y on the right legs. stride_x deliberately does NOT
            # go through ik_x_sign: applying it (2026-10-13) made a forward
            # stick walk backward on the real robot.
            dx = stride_x * s + tx
            dy = side * stride_y * s + ty
            r = math.hypot(dx, dy)
            if r > self._max_foot_offset:
                dx *= self._max_foot_offset / r
                dy *= self._max_foot_offset / r

            # Moving the body by (bx, by) moves every foot by (-bx, -by) relative
            # to it. Per-leg IK y is outward, so body-left (+by) is -y on left legs.
            dx -= self._ik_x_sign * bx
            dy -= side * by

            nx, ny, nz = self._neutral(leg)
            feet[leg] = [nx + dx, ny + dy, nz + dz]
        self._write_feet(feet)

    # ── modes: sentry / calibrate ────────────────────────────────────────────

    def _mode_inputs(self, joy, now):
        """Y double-tap toggles sentry, the Y+D-pad-up hold toggles calibrate,
        Y+D-pad-down zeroes the IMU, Y+D-pad-left calibrates its axes, X starts
        tricks. Fed every tick so the detectors track the buttons even while
        disarmed."""
        tap = self._sentry_tap.update(joy.buttons, now)
        nav = self._nav_tap.update(joy.buttons, now)
        calib = self._calib_chord.update(joy.buttons, now)
        imu_zero = self._imu_zero_chord.update(joy.buttons, now)
        imu_cal = self._imu_cal_chord.update(joy.buttons, now)
        trick = self._trick_input(joy, now)
        ready = self._armed and not self._engage_queue and not self._jiggle_queue

        if nav:
            if not ready:
                self.get_logger().info('Auto nav needs the robot armed (and the legs engaged).')
            elif self._mode == 'auto_nav':
                self._set_mode('walk')
            elif self._mode == 'walk':
                self._set_mode('auto_nav')
            else:
                self.get_logger().info(f'Auto nav ignored in {self._mode} mode.')
        if self._mode == 'auto_nav':
            if tap or calib or imu_zero or imu_cal or trick:
                self.get_logger().info(f'Mode buttons ignored in auto nav (double-tap '
                                       f'"{self._nav_button}" to leave it first).')
            return

        if imu_zero:
            if self._mode in ('calibrate', 'imu_calib', 'trick'):
                self.get_logger().info(f'IMU zero ignored in {self._mode} mode.')
            else:
                self._imu_zero()
        if imu_cal:
            if self._mode == 'imu_calib':
                self._end_imu_cal('IMU calibration cancelled', False)
            elif self._mode == 'walk' and (ready or not self._armed):
                self._start_imu_cal(now)
            else:
                self.get_logger().info(f'IMU calibration ignored in {self._mode} mode.')
        if trick == 'stop':
            self.get_logger().info(f'Trick "{self._trick["name"]}" stopped.')
            self._set_mode('walk')
        elif trick:
            if ready and self._mode == 'walk':
                self._start_trick(trick)
            else:
                self.get_logger().info(f'Trick "{trick}" ignored: needs armed + walk mode.')

        if not (tap or calib):
            return
        if not ready:
            self.get_logger().info('Mode buttons ignored: arm first (and wait for the legs to engage).')
            return
        if self._mode in ('trick', 'imu_calib'):
            return
        if calib:
            self._set_mode('walk' if self._mode == 'calibrate' else 'calibrate')
        elif self._mode != 'calibrate':   # Y is part of the calibrate chord; taps don't leave it
            self._set_mode('walk' if self._mode == 'sentry' else 'sentry')

    def _trick_input(self, joy, now):
        """-> a trick name, 'stop' (X pressed during a trick) or None."""
        def down(name):
            return bool(joy.buttons[_BTN[name]])

        pressed = {name: down(name) and not was for name, was in self._trick_prev.items()}
        self._trick_prev = {name: down(name) for name in self._trick_prev}
        taps = self._trick_tapper.update(joy.buttons, now)
        if self._mode == 'trick':
            if pressed[self._trick_button]:
                self._trick_tapper.cancel()
                return 'stop'
            return None
        if down(self._trick_button):
            for name, trick in zip(DPAD, self._trick_dpad):
                if pressed[name] and trick:
                    self._trick_tapper.cancel()   # this X press was for the combo
                    return trick
        if taps and self._trick_taps_map:
            return self._trick_taps_map[min(taps, len(self._trick_taps_map)) - 1] or None
        return None

    def _start_trick(self, name):
        geom = {'half_length': self._half_length,
                'half_width': self._half_width + self._stance_y,
                'max_shift': self._bal_max}
        self._trick = {'name': name, 'frames': tricks.TRICKS[name](geom), 't0': None}
        self._set_mode('trick')

    def _trick_tick(self, now, dt):
        """Glide to the stand pose, then play the trick's keyframes."""
        tr = self._trick
        if tr['t0'] is None:
            if self._settle_tick(dt):
                tr['t0'] = now
            return
        pose, done = tricks.sample(tr['frames'], now - tr['t0'])
        if any(off[2] > 1.0 for off in pose.feet.values()):
            self._stable = False
        self._move_toward(self._posed_feet(pose), dt, self._trick_foot_speed)
        if done:
            self._set_mode('walk')

    # ── IMU zero / axis calibration ──────────────────────────────────────────

    def _notify(self, text, ok):
        """Result line for the Deck (shown as a toast) and the log."""
        self._note_seq += 1
        self._note = {'seq': self._note_seq, 'text': text, 'ok': ok}
        (self.get_logger().info if ok else self.get_logger().warn)(text)
        self._publish_mode()

    def _call_imu(self, name, on_done):
        """Call one of spot_micro_imu's Trigger services; on_done(ok, message)."""
        client = self._imu_srv[name]
        if not client.service_is_ready():
            on_done(False, 'IMU node not running — is the stack up (L1+R1)?')
            return

        def done(future):
            try:
                res = future.result()
            except Exception as e:   # noqa: BLE001 — any failure is just reported
                on_done(False, f'IMU service call failed: {e}')
                return
            on_done(bool(res.success), str(res.message))

        client.call_async(Trigger.Request()).add_done_callback(done)

    def _imu_zero(self):
        self._call_imu('zero', self._notify_ok_msg)

    def _notify_ok_msg(self, ok, message):
        self._notify(message, ok)

    def _start_imu_cal(self, now):
        auto = self._armed
        self._imu_cal = {'auto': auto, 'step': 'level', 'since': now, 'still_since': None,
                         'pending': False, 'ups': {}, 'tilt': 0.0, 'hold': 0.0, 'hint': ''}
        if auto:
            self._level_roll = 0.0   # the robot rolls itself; leveling would fight it
            self._level_pitch = 0.0
        self._set_mode('imu_calib')
        self.get_logger().warn('IMU AXIS CALIBRATION ' + (
            '(robot rolls itself — keep it on flat ground)' if auto else '(by hand, disarmed)'))

    def _end_imu_cal(self, text, ok):
        self._imu_cal = None
        self._level_roll = 0.0       # earned against the old mount
        self._level_pitch = 0.0
        self._set_mode('walk')
        self._notify(text, ok)

    def _imu_cal_prompt(self):
        c = self._imu_cal
        if c is None:
            return ''
        step = c['step']
        if c['pending']:
            return 'Recording — hold still…'
        if c['auto']:
            r = self._imu_cal_roll
            return {'level': 'Standing level — keep the robot on flat ground',
                    'left': f'Rolling {r:.0f}° onto its left side',
                    'right': f'Rolling {r:.0f}° onto its right side',
                    'front': f'Tipping {r:.0f}° nose down',
                    'back': f'Tipping {r:.0f}° nose up',
                    'return': 'Done — standing back up'}[step]
        return {'level': 'Hold the robot level (or set it on a flat floor) and keep it still',
                'left': 'Tip it onto its LEFT side, ~20°, and hold still',
                'right': 'Now its RIGHT side down, ~20°, and hold still',
                'front': 'Tip the NOSE down, ~20°, and hold still',
                'back': 'Tip the TAIL down (nose up), ~20°, and hold still'}.get(step, '')

    def _imu_cal_tick(self, now, dt):
        c = self._imu_cal
        if c['pending']:
            return
        step = c['step']
        if c['auto']:
            r = self._imu_cal_roll
            target = {'left': tricks.Pose(roll=-r), 'right': tricks.Pose(roll=r),
                      'front': tricks.Pose(pitch=r), 'back': tricks.Pose(pitch=-r)}.get(step, tricks.STAND)
            arrived = self._move_toward(self._posed_feet(target), dt, self._settle_speed)
            if not arrived:
                c['still_since'] = None
                return
            if step == 'return':
                self._end_imu_cal(c['result'], True)
                return
            # Let the body (and the IMU's fusion) settle before recording.
            if c['still_since'] is None:
                c['still_since'] = now
            if now - c['still_since'] < 1.0 or self._imu_rate > 0.15:
                return
        else:
            if now - c['since'] > 60.0:
                self._end_imu_cal(f'IMU calibration timed out at the "{step}" step', False)
                return
            if not self._imu_fresh():
                c['still_since'] = None
                return

            def angle(u):
                dot = sum(a * b for a, b in zip(self._imu_up, u))
                return math.degrees(math.acos(max(-1.0, min(1.0, dot))))

            # Tilt from level, and how far from the nearest pose already recorded
            # (so the pose still held from the previous step isn't taken again).
            tilt = angle(c['ups']['level']) if step != 'level' else 0.0
            new = min((angle(u) for u in c['ups'].values()), default=180.0)
            still = self._imu_rate < 0.2
            hold_s = 1.5 if step == 'level' else IMU_CAL_HOLD_S
            ok = still and (step == 'level' or (tilt >= IMU_CAL_TILT_DEG and new >= IMU_CAL_TILT_DEG))
            if not ok:
                c['still_since'] = None
            elif c['still_since'] is None:
                c['still_since'] = now
            # Live progress for the Deck's tilt gauge.
            c['tilt'] = round(tilt, 1)
            c['hold'] = 0.0 if c['still_since'] is None else round(min((now - c['still_since']) / hold_s, 1.0), 2)
            if step != 'level' and tilt >= IMU_CAL_TILT_DEG and new < IMU_CAL_TILT_DEG:
                c['hint'] = 'That is the pose you just did — move to the new one'
            elif step != 'level' and tilt < IMU_CAL_TILT_DEG:
                c['hint'] = 'Tilt further'
            elif not still:
                c['hint'] = 'Hold still'
            else:
                c['hint'] = 'Hold it there…'
            if now - self._last_mode_pub >= 0.2:
                self._publish_mode()
            if c['still_since'] is None or now - c['still_since'] < hold_s:
                return

        up_now = self._imu_up
        c['pending'] = True
        self._publish_mode()

        def done(ok, message):
            if self._imu_cal is not c:
                return   # cancelled meanwhile
            c['pending'] = False
            if not ok:
                self._end_imu_cal(message, False)
                return
            c['ups'][step] = up_now
            nxt = IMU_CAL_STEPS[IMU_CAL_STEPS.index(step) + 1]
            if nxt == 'return' and not c['auto']:
                self._end_imu_cal(message, True)
                return
            if nxt == 'return':
                c['result'] = message
            c['step'], c['since'], c['still_since'] = nxt, time.monotonic(), None
            self._publish_mode()

        self._call_imu(f'axis_calib/{step}', done)

    def _set_mode(self, mode):
        old = self._mode
        if mode == old:
            return
        if old == 'calibrate':
            self._save_calibration()
            self._calib_angle_mode = False
            self._calib_slew = {}
        if old == 'trick':
            self._trick = None
        if old == 'imu_calib':
            self._imu_cal = None
        self._mode = mode
        self._t = 0.0
        self._pose = [0.0, 0.0, 0.0]
        self._track_was_on = False
        if mode == 'calibrate':
            self._calib_note = ''
            self._calib_wait_release = True   # the chord's buttons are still down
            leg, joint = LEGS[self._calib_leg], JOINTS[self._calib_joint]
            self.get_logger().warn(
                f'CALIBRATION mode: feet to IK {self._calib_pose}, no trim/leveling. L1/R1 leg, '
                f'L2/R2 joint, D-pad offset, A angle test (right stick sets the angle). '
                f'Now on {leg} {joint}.')
        elif mode == 'trick':
            self.get_logger().info(f'TRICK: {self._trick["name"]} (press "{self._trick_button}" to stop).')
        elif mode == 'imu_calib':
            pass   # _start_imu_cal logs it
        elif mode == 'auto_nav':
            self._nav_manual = False
            self.get_logger().info('AUTO NAV mode: walking to goals from nav_node (double-tap '
                                   f'"{self._nav_button}" to leave). Left stick overrides.')
        elif mode == 'sentry':
            self.get_logger().info('SENTRY mode: feet planted, sticks pose the body (double-tap '
                                   f'"{self._sentry_button}" to walk again).')
        else:
            self._needs_settle = True
            self.get_logger().info(f'Back to WALK mode (from {old}).')
        self._publish_mode()

    def _exit_mode(self):
        """On disarm / fault: back to walk, saving any calibration changes."""
        if self._mode == 'calibrate':
            self._save_calibration()
        self._calib_angle_mode = False
        self._calib_slew = {}
        if self._mode == 'imu_calib':
            self._note_seq += 1
            self._note = {'seq': self._note_seq, 'text': 'IMU calibration cancelled', 'ok': False}
        self._trick = None
        self._imu_cal = None
        self._mode = 'walk'
        self._pose = [0.0, 0.0, 0.0]
        self._needs_settle = False
        self._publish_mode()

    def _publish_mode(self):
        msg = {'mode': self._mode}
        if self._mode == 'sentry':
            msg['pose'] = dict(zip(('roll', 'pitch', 'yaw'), (round(a, 1) for a in self._pose)))
            msg['tracking'] = self._track_was_on
        if self._mode == 'trick' and self._trick:
            msg['trick'] = self._trick['name']
        if self._mode == 'auto_nav':
            msg['manual'] = self._nav_manual
        if self._mode == 'imu_calib' and self._imu_cal:
            c = self._imu_cal
            msg['imu_calib'] = {'step': c['step'], 'auto': c['auto'], 'prompt': self._imu_cal_prompt()}
            if not c['auto'] and not c['pending']:
                msg['imu_calib'].update(tilt=c['tilt'], need=IMU_CAL_TILT_DEG, hold=c['hold'], hint=c['hint'])
        if self._note:
            msg['note'] = self._note
        if self._mode == 'calibrate' or self._calib_note:
            leg, joint = LEGS[self._calib_leg], JOINTS[self._calib_joint]
            msg['calib'] = {
                'leg': leg, 'joint': joint,
                'offset': getattr(self._calibration[leg], joint).offset,
                'offsets': {lg: [getattr(self._calibration[lg], j).offset for j in JOINTS] for lg in LEGS},
                'unsaved': self._calib_dirty,
                'note': self._calib_note,
                'angle_mode': self._calib_angle_mode,
            }
            if self._mode == 'calibrate' and self.dog is not None:
                # Angle test readout: target, where the joint is commanded now
                # (both before the offset) and what the servo actually gets.
                key = (leg, joint)
                angle = self._calib_slew.get(key)
                if angle is None:
                    angle = self._calib_ik_angle(leg, joint)
                if angle is not None:
                    msg['calib'].update(
                        target=round(self._calib_target(leg, joint), 2), angle=round(angle, 2),
                        servo=round(self.dog.legs[leg].servo_command(joint, angle), 2))
        self._mode_pub.publish(String(data=json.dumps(msg, separators=(',', ':'))))
        self._last_mode_pub = time.monotonic()

    def _sentry_tick(self, joy, now, dt):
        """Feet stay on their stand points; the body rolls/pitches/yaws over them.
        While track_node is locked onto something picked on the Deck, yaw and
        pitch follow it instead of the sticks (roll stays on the left stick)."""
        ax = joy.axes
        dz = self._deadzone
        # Protocol convention: stick left / up = +. Left stick left leans left
        # (-roll), up dips the nose (+pitch); right stick left turns left (+yaw).
        want = [-_rescaled(ax[_AX['left_x']], dz) * self._sentry_max[0],
                _rescaled(ax[_AX['left_y']], dz) * self._sentry_max[1],
                _rescaled(ax[_AX['right_x']], dz) * self._sentry_max[2]]
        step = self._sentry_rate * dt
        tracking = self._tracking()
        if tracking is not None:
            # Turn toward the target: +yaw error (target left) -> yaw left,
            # +pitch error (target below) -> nose down. Velocity control, since
            # the error is measured from where the camera already points.
            yaw_err, pitch_err = tracking
            for i, err in ((2, yaw_err), (1, pitch_err)):
                err = math.copysign(max(abs(err) - self._track_deadband, 0.0), err)
                v = max(-self._sentry_rate, min(self._sentry_rate, self._track_gain * err))
                want[i] = max(-self._sentry_max[i], min(self._sentry_max[i], self._pose[i] + v * dt))
        if (tracking is not None) != self._track_was_on:
            self._track_was_on = tracking is not None
            self._publish_mode()
        self._pose = [p + min(max(w - p, -step), step) for p, w in zip(self._pose, want)]
        roll, pitch, yaw = self._pose
        self._move_toward(self._posed_feet(tricks.Pose(roll=roll, pitch=pitch, yaw=yaw)),
                          dt, self._sentry_foot_speed)
        if now - self._last_mode_pub >= 0.2:
            self._publish_mode()

    def _posed_feet(self, pose):
        """Foot targets (IK frame) for a tricks.Pose: the body rotated and moved
        over feet that stay planted on their stand points, plus any per-foot
        offsets (lifted paws)."""
        feet = {}
        for leg in LEGS:
            nx, ny, nz = self._neutral(leg)
            side = 1.0 if leg in LEFT_SIDE else -1.0
            fwd = (self._half_length if leg in FRONT else -self._half_length) + self._ik_x_sign * nx
            left = side * (self._half_width + ny)
            d_fwd, d_left, d_up = _rotation_offset(fwd, left, -nz, pose.roll, pose.pitch, pose.yaw)
            # Moving the body moves the planted foot the other way relative to it.
            d_fwd -= pose.fwd
            d_left -= pose.left
            d_up -= pose.up
            o_fwd, o_out, o_up = pose.feet.get(leg, (0.0, 0.0, 0.0))
            d_fwd += o_fwd
            d_left += side * o_out
            d_up += o_up
            # IK frame: x = ik_x_sign * forward, y = outward, z = extension (down).
            feet[leg] = [nx + self._ik_x_sign * d_fwd, ny + side * d_left, nz - d_up]
        return feet

    def _calib_tick(self, joy, now, dt):
        def down(name):
            return bool(joy.buttons[_BTN[name]])

        # Don't step offsets while the mode chord's buttons are still held from
        # entering (or are being held to leave): wait for them all to come up.
        if any(down(n) for n in self._calib_buttons if not n.startswith('dpad')):
            self._calib_wait_release = True
        elif self._calib_wait_release and not any(down(n) for n in self._calib_buttons):
            self._calib_wait_release = False
        keys = self._calib_keys
        live = not self._calib_wait_release
        moved = False
        for name, d_leg, d_joint in (('r1', 1, 0), ('l1', -1, 0), ('r2', 0, 1), ('l2', 0, -1)):
            if keys[name].update(down(name) and live, now):
                self._calib_leg = (self._calib_leg + d_leg) % len(LEGS)
                self._calib_joint = (self._calib_joint + d_joint) % len(JOINTS)
                moved = True
        if moved:
            leg, joint = LEGS[self._calib_leg], JOINTS[self._calib_joint]
            self.get_logger().info(
                f'Calibrating {leg} {joint} (offset {getattr(self._calibration[leg], joint).offset:+.2f} deg)')
            self._publish_mode()
        for name, delta in (('dpad_up', self._calib_step), ('dpad_down', -self._calib_step),
                            ('dpad_right', self._calib_fine_step), ('dpad_left', -self._calib_fine_step)):
            if keys[name].update(down(name) and live, now):
                self._adjust_offset(delta)
        if keys['a'].update(down('a') and live, now):
            self._calib_angle_mode = not self._calib_angle_mode
            leg, joint = LEGS[self._calib_leg], JOINTS[self._calib_joint]
            self.get_logger().info(
                f'Angle test {"ON" if self._calib_angle_mode else "OFF"} ({leg} {joint})')
            moved = True
        if self._calib_angle_mode:
            # Right stick (protocol: up / left = +): up/down coarse, right/left fine.
            rx, ry = joy.axes[_AX['right_x']], joy.axes[_AX['right_y']]
            for name, pushed, delta in (('rs_up', ry > 0.6, self._calib_angle_step),
                                        ('rs_down', ry < -0.6, -self._calib_angle_step),
                                        ('rs_right', rx < -0.6, self._calib_angle_fine),
                                        ('rs_left', rx > 0.6, -self._calib_angle_fine)):
                if keys[name].update(pushed and live, now):
                    self._adjust_angle_target(delta)
        self._calib_slew_tick(dt)
        if moved or self._calib_slew:
            if moved or now - self._last_mode_pub >= 0.2:
                self._publish_mode()
        x, y, z = self._calib_pose
        self._move_toward({leg: [x, y, z] for leg in LEGS}, dt, self._settle_speed, raw=True)

    def _calib_ik_angle(self, leg, joint):
        """Servo angle (deg, before offset) IK gives `joint` at the foot's current spot."""
        pos = self._feet[leg] if self._feet is not None else self._calib_pose
        return self.dog.legs[leg].joint_angles(*pos)[joint]

    def _calib_target(self, leg, joint):
        """Angle-test target for a joint: where it was last set, else where IK
        has it in calib_pose (where the offsets were verified)."""
        if (leg, joint) not in self._calib_targets:
            ik = self.dog.legs[leg].joint_angles(*self._calib_pose)[joint]
            self._calib_targets[(leg, joint)] = round(ik * 2.0) / 2.0 if ik is not None else 90.0
        return self._calib_targets[(leg, joint)]

    def _adjust_angle_target(self, delta):
        leg, joint = LEGS[self._calib_leg], JOINTS[self._calib_joint]
        rng = getattr(self._calibration[leg], joint).range
        cur = self._calib_target(leg, joint)
        new = round(min(max(cur + delta, 0.0), rng), 2)
        self._calib_targets[(leg, joint)] = new
        self.get_logger().info(f'{leg} {joint} angle target {cur:.1f} -> {new:.1f} deg')

    def _calib_slew_tick(self, dt):
        """Move the angle-tested joint toward its target (and any joint that
        was being tested back to its IK angle) at calib_angle_speed_deg_s, so
        nothing jumps when the test starts, stops or changes joint."""
        sel = (LEGS[self._calib_leg], JOINTS[self._calib_joint])
        if self._calib_angle_mode and sel not in self._calib_slew:
            start = self._calib_ik_angle(*sel)
            self._calib_slew[sel] = start if start is not None else self._calib_target(*sel)
        step = self._calib_angle_speed * dt
        for key in list(self._calib_slew):
            to_target = self._calib_angle_mode and key == sel
            goal = self._calib_target(*key) if to_target else self._calib_ik_angle(*key)
            if goal is None:
                continue   # IK unreachable for that joint: hold where it is
            cur = self._calib_slew[key]
            nxt = cur + min(max(goal - cur, -step), step)
            if not to_target and abs(goal - nxt) < 1e-6:
                del self._calib_slew[key]   # back on IK
            else:
                self._calib_slew[key] = nxt

    def _calib_overrides(self, leg):
        """{joint: servo angle} for this leg's joints under the angle test, or None."""
        out = {joint: a for (lg, joint), a in self._calib_slew.items() if lg == leg}
        return out or None

    def _adjust_offset(self, delta):
        leg, joint = LEGS[self._calib_leg], JOINTS[self._calib_joint]
        cur = getattr(self._calibration[leg], joint).offset
        new = round(min(max(cur + delta, -self._calib_max), self._calib_max), 2)
        if new == cur:
            return
        self._calibration[leg] = with_offset(self._calibration[leg], joint, new)
        if self.dog is not None:
            self.dog.legs[leg].cal = self._calibration[leg]   # live on the next write
        self._calib_dirty = True
        self.get_logger().info(f'{leg} {joint} offset {cur:+.2f} -> {new:+.2f} deg')
        self._publish_mode()

    def _save_calibration(self):
        if not self._calib_dirty:
            self._calib_note = 'no changes'
            return
        name = os.path.basename(self._calibration_path)
        try:
            save_offsets(self._calibration_path, self._calibration)
        except (OSError, ValueError) as e:
            self._calib_note = f'SAVE FAILED: {e}'
            self.get_logger().error(f'Saving servo offsets to {self._calibration_path} failed: {e} '
                                    f'(they stay live until gait_node restarts)')
            return
        self._calib_dirty = False
        self._calib_note = f'saved to {name}'
        self.get_logger().info(f'Servo offsets saved to {self._calibration_path}')

    # ── arm / disarm ─────────────────────────────────────────────────────────

    def _toggle_armed(self):
        if self._armed:
            self._jiggle_queue = []
            self._disarm('chord')
        else:
            if self.dog is None and not self._open_servos():
                return   # stay disarmed; _open_servos logged why
            if self._mode == 'imu_calib':
                self._end_imu_cal('IMU calibration cancelled: armed', False)
            self._armed = True
            self._publish_armed()
            self.get_logger().warn('ARMED — driving real servos over I2C.')
            self._level_roll = 0.0
            self._level_pitch = 0.0
            if self._level_enabled:
                if self._imu_fresh():
                    r, pch = self._imu_rp
                    self.get_logger().info(
                        f'IMU roll={r:+.1f} pitch={pch:+.1f} deg (after sign/trim; '
                        f'+roll = right side down, +pitch = nose down).')
                else:
                    self.get_logger().warn(
                        'Leveling enabled but no IMU data yet — is spot_micro_imu running?')
            self._engage_queue = list(LEGS)
            self._engage_tick = 0
            self._start_confirmation_jiggle()   # runs once every leg has engaged

    def _disarm(self, reason):
        try:
            if self.dog is not None:
                self.dog.release()
        except OSError as e:
            self._on_i2c_fault(e)
            return
        self._armed = False
        self._publish_armed()
        self._exit_mode()
        self._engage_queue = []
        self._t = 0.0
        self._feet = None
        self._level_roll = 0.0
        self._level_pitch = 0.0
        self._stab = (0.0, 0.0)
        self._heading_ref = None
        self.get_logger().warn(f'DISARMED ({reason}) — servos released. Hold the arm chord again to resume.')

    def _engage_tick_step(self):
        """Power up the next leg to its stand pose, then wait _engage_ticks_per_leg
        ticks before the next one, so the servos' inrush current is spread out
        instead of all twelve kicking in on the same tick. Non-blocking, same as
        the jiggle below."""
        if self._engage_tick == 0:
            leg = self._engage_queue[0]
            x, y, z = self._neutral(leg)
            tx, ty, tz = self._trim[leg]
            self.dog.move_leg(leg, x + tx, y + ty, z + tz)
            self.get_logger().info(f'Engaged {leg}.')
        self._engage_tick += 1
        if self._engage_tick >= self._engage_ticks_per_leg:
            self._engage_tick = 0
            self._engage_queue.pop(0)

    def _start_confirmation_jiggle(self):
        """Queue a quick double-bounce (all legs dip and recover twice) so arming
        has an obvious physical confirmation, not just a log line. Non-blocking —
        _jiggle_tick_step() advances it one _control_loop tick at a time (NEVER
        time.sleep() here: this node's single-threaded executor would stop
        processing incoming Joy messages for the whole sleep, which is exactly
        what silently auto-disarmed the node via joy_timeout_s the first time
        this used a blocking sleep loop, 2026-09-24)."""
        dip = 15.0   # mm, well under step_height's default swing
        self._jiggle_queue = [self._z - dip, self._z, self._z - dip, self._z]
        self._jiggle_tick = 0

    def _jiggle_tick_step(self):
        if self._jiggle_tick == 0:
            z = self._jiggle_queue[0]
            self._write_feet({leg: self._neutral(leg)[:2] + [z] for leg in LEGS})
        self._jiggle_tick += 1
        if self._jiggle_tick >= self._jiggle_ticks_per_step:
            self._jiggle_tick = 0
            self._jiggle_queue.pop(0)

    def destroy_node(self):
        if self._mode == 'calibrate':
            self._save_calibration()
        try:
            if self.dog is not None:
                self.dog.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
