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
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool

from spot_micro_deck.chord import ChordDetector
from spot_micro_deck.protocol import AXIS_NAMES, BUTTON_NAMES
from spot_micro_ik import Dog, load_calibration
from spot_micro_ik.dog import DEFAULT_STAND_POSE

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


def _deadzone(value, dz):
    return 0.0 if abs(value) < dz else value


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

        p = self.get_parameter
        self._joy_timeout = float(p('joy_timeout_s').value)
        self._up_button = str(p('height_up_button').value)
        self._down_button = str(p('height_down_button').value)
        self._height_rate = float(p('height_rate').value)
        self._z_min = float(p('z_min').value)
        self._z_max = float(p('z_max').value)
        self._speed_scale = float(p('speed_scale').value)
        self._estop_button = str(p('estop_button').value)
        arm_buttons = list(p('arm_buttons').value)
        arm_hold_s = float(p('arm_hold_s').value)
        self._deadzone = float(p('stick_deadzone').value)
        self._stand_z = float(p('stand_z').value)
        self._z = min(max(self._stand_z, self._z_min), self._z_max)   # live body height (R1/L1)
        self._step_height = float(p('step_height').value)
        self._step_length = float(p('step_length').value)
        self._step_lateral = float(p('step_lateral').value)
        self._turn_gain = float(p('turn_gain').value)
        self._max_foot_offset = max(float(p('max_foot_offset').value), 0.0)
        self._gait_freq = max(float(p('gait_freq').value), 1e-3)
        gait_type = str(p('gait_type').value)
        if gait_type not in GAITS:
            raise ValueError(f"gait_type '{gait_type}' not in {list(GAITS)}")
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
        self._trim = {}
        for leg in LEGS:
            t = [float(v) for v in p(f'trim_{leg.lower()}').value]
            if len(t) != 3:
                raise ValueError(f'trim_{leg.lower()} must be [x, y, z], got {t}')
            self._trim[leg] = t

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

        for name in (self._up_button, self._down_button):
            if name not in _BTN:
                raise ValueError(f"height button '{name}' not in {BUTTON_NAMES}")
        if self._estop_button not in _BTN:
            raise ValueError(f"estop_button '{self._estop_button}' not in {BUTTON_NAMES}")
        for name in arm_buttons:
            if name not in _BTN:
                raise ValueError(f"arm_buttons entry '{name}' not in {BUTTON_NAMES}")

        self._calibration = load_calibration(p('calibration_path').value)
        self.dog = Dog(self._calibration, dry_run=False, pwm_hz=int(p('servo_pwm_hz').value))   # opens I2C now; starts disarmed below

        self._armed = False
        self._arm_chord = ChordDetector([_BTN[n] for n in arm_buttons], arm_hold_s)
        self._arm_hold_s = arm_hold_s
        self._arm_progress = 0

        self._last_joy = None            # sensor_msgs/Joy
        self._last_joy_time = 0.0
        self._t = 0.0                    # gait clock, only advances while walking
        self._last_tick = time.monotonic()

        self._engage_queue = []          # legs still to power up, one at a time, after arming
        self._engage_tick = 0
        self._jiggle_queue = []          # pending z-targets for the non-blocking arm jiggle
        self._jiggle_tick = 0

        self._feet = None                # leg -> [x, y, z] last commanded (before leveling)

        self._imu_rp = None              # (roll, pitch) deg, body frame, trim applied
        self._last_imu_time = 0.0
        self._level_roll = 0.0           # deg, accumulated correction
        self._level_pitch = 0.0

        self._stable = True              # no foot in the air (see STABLE FLAG)
        self._stable_pub = self.create_publisher(Bool, p('stable_topic').value, 10)

        self.create_subscription(Joy, p('joy_topic').value, self._on_joy, 10)
        if self._level_enabled:
            self.create_subscription(Imu, p('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        self._control_dt = 1.0 / float(p('control_hz').value)
        self._jiggle_ticks_per_step = max(1, round(0.12 / self._control_dt))
        self._engage_ticks_per_leg = max(1, round(float(p('arm_leg_stagger_s').value) / self._control_dt))
        self.create_timer(self._control_dt, self._control_loop)

        # Deliberately NOT calling self.dog.stand() here: the driver is live (real I2C)
        # from construction, and standing would move real servos before anyone armed it.
        self.get_logger().info(
            f'gait_node up, DISARMED (I2C open, nothing commanded). '
            f'Listening on {p("joy_topic").value}. '
            f'Hold "{"+".join(arm_buttons)}" for {arm_hold_s:.0f}s to arm/disarm, '
            f'sticks walk once armed, "{self._up_button}"/"{self._down_button}" raise/lower, '
            f'"{self._estop_button}" to go limp.')

    # ── input ────────────────────────────────────────────────────────────────

    def _on_joy(self, msg):
        self._last_joy = msg
        self._last_joy_time = time.monotonic()

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
        self._last_imu_time = time.monotonic()

    def _imu_fresh(self):
        return (self._imu_rp is not None
                and time.monotonic() - self._last_imu_time <= self._imu_timeout)

    def _joy_fresh(self):
        return (self._last_joy is not None
                and time.monotonic() - self._last_joy_time <= self._joy_timeout)

    # ── control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        self._stable = True   # _gait_tick clears it while a foot is up
        self._control_step()
        self._stable_pub.publish(Bool(data=self._stable))

    def _control_step(self):
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        if not self._joy_fresh():
            self._t = 0.0
            self._arm_chord.reset()
            self._arm_progress = 0
            self._jiggle_queue = []
            if self._armed:
                self._disarm('lost Joy/Deck connection')
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

        if joy.buttons[_BTN[self._estop_button]]:
            self._jiggle_queue = []
            if self._armed:
                self._disarm('estop')
            self._t = 0.0
            return

        if not self._armed:
            return   # idle: servos released, nothing to command

        if self._engage_queue:
            self._engage_tick_step()
            return

        if self._jiggle_queue:
            self._jiggle_tick_step()
            return

        up = bool(joy.buttons[_BTN[self._up_button]])
        down = bool(joy.buttons[_BTN[self._down_button]])
        if up != down:   # both held = Deck stack-toggle chord, leave height alone
            dz = self._height_rate * dt * (1.0 if up else -1.0)
            self._z = min(max(self._z + dz, self._z_min), self._z_max)

        # Stick-up reads as negative from the Deck, so flip it: forward = +.
        vx = -_deadzone(joy.axes[_AX['left_y']], self._deadzone)
        vy = _deadzone(joy.axes[_AX['left_x']], self._deadzone)     # strafe left = +
        wz = _deadzone(joy.axes[_AX['right_x']], self._deadzone)    # turn = +

        self._update_leveling(dt)

        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            self._t = 0.0
            self._settle_tick(dt)
            return

        vx *= self._speed_scale
        vy *= self._speed_scale
        wz *= self._speed_scale

        self._t += dt
        self._gait_tick(vx, vy, wz)

    # ── foot placement ───────────────────────────────────────────────────────

    def _neutral(self, leg):
        """Stand position for one leg: slightly out, front forward, rear back."""
        x = self._stance_x_front if leg in FRONT else self._stance_x_rear
        return [x, self._stance_y, self._z]

    def _write_feet(self, feet):
        """Command every leg, remembering the pre-leveling target and adding
        the static per-leg xyz trim and IMU leveling height offset on top."""
        self._feet = {leg: list(pos) for leg, pos in feet.items()}
        for leg, (x, y, z) in feet.items():
            tx, ty, tz = self._trim[leg]
            self.dog.move_leg(leg, x + tx, y + ty, z + tz + self._level_dz(leg))

    def _settle_tick(self, dt):
        """Walk each foot toward its stand position at <= settle_speed, so a leg
        caught mid-swing lowers gently instead of dropping."""
        if self._feet is None:
            self._write_feet({leg: self._neutral(leg) for leg in LEGS})
            return
        max_step = self._settle_speed * dt
        feet = {}
        for leg in LEGS:
            cur = self._feet[leg]
            tgt = self._neutral(leg)
            d = [t - c for t, c in zip(tgt, cur)]
            dist = math.sqrt(sum(v * v for v in d))
            if dist <= max_step or dist == 0.0:
                feet[leg] = tgt
            else:
                k = max_step / dist
                feet[leg] = [c + v * k for c, v in zip(cur, d)]
        self._write_feet(feet)

    # ── leveling ─────────────────────────────────────────────────────────────

    def _update_leveling(self, dt):
        if not self._level_enabled:
            return
        if not self._imu_fresh():
            # No IMU: bleed the correction back to neutral rather than hold a stale one.
            decay = max(0.0, 1.0 - dt)
            self._level_roll *= decay
            self._level_pitch *= decay
            self.get_logger().warn('No fresh IMU data — leveling paused.', throttle_duration_sec=5.0)
            return
        roll, pitch = self._imu_rp
        if abs(roll) > self._level_deadband:
            self._level_roll += self._level_gain * roll * dt
        if abs(pitch) > self._level_deadband:
            self._level_pitch += self._level_gain * pitch * dt
        lim = self._level_max
        self._level_roll = max(-lim, min(lim, self._level_roll))
        self._level_pitch = max(-lim, min(lim, self._level_pitch))

    def _level_dz(self, leg):
        """Extra leg extension (mm) for this leg from the leveling correction.
        REP-103: +pitch = nose down -> extend front legs; +roll = right side
        down -> extend right legs."""
        if not self._level_enabled:
            return 0.0
        dz = self._half_length * math.sin(math.radians(self._level_pitch))
        dz *= 1.0 if leg in FRONT else -1.0
        droll = self._half_width * math.sin(math.radians(self._level_roll))
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
                dz = -self._step_height * math.sin(math.pi * sw) ** 2   # lift (shorten the leg)
                self._stable = False
            else:
                st = ((local - self._slot_frac) % 1.0) / stance_len   # 0..1 across stance
                s = 1.0 - 2.0 * st                             # +1 -> -1
                dz = 0.0

            tx, ty = self._turn_offset(leg, wz, s)
            # IK y is per-leg "outward", so a body-frame strafe is +y on the
            # left legs and -y on the right legs.
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

    # ── arm / disarm ─────────────────────────────────────────────────────────

    def _toggle_armed(self):
        if self._armed:
            self._jiggle_queue = []
            self._disarm('chord')
        else:
            self._armed = True
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
        self.dog.release()
        self._armed = False
        self._engage_queue = []
        self._t = 0.0
        self._feet = None
        self._level_roll = 0.0
        self._level_pitch = 0.0
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
        try:
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
