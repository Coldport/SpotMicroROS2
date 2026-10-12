"""
Robot settings the Steam Deck can change live, and how they are shown there.

deck_bridge publishes this catalog with current values on <prefix>/settings;
the Deck app draws its settings menu from that message alone (it has no list of
its own), so adding a row here is all it takes to expose another parameter.
Ranges are what the Deck lets you dial in — the bridge clamps to them, and
gait_node does its own checks too (see LIVE_PARAMS there).

target 'gait' rows are gait_node parameters (spot_micro_move); 'bridge' rows are
this bridge's own. No ROS imports, so it can be unit-tested.
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class Setting:
    key: str              # parameter name on the target node
    label: str
    group: str
    # 'float' | 'int' | 'bool' | 'enum' | 'action' (a button: no value, the Deck
    # sends true after a double-A confirm; see deck_bridge._run_action)
    kind: str
    target: str = 'gait'  # 'gait' (gait_node) | 'bridge' (deck_bridge)
    min: Optional[float] = None
    max: Optional[float] = None
    step: Optional[float] = None
    unit: str = ''
    options: List[str] = field(default_factory=list)
    help: str = ''

    def describe(self):
        d = {'key': self.key, 'label': self.label, 'group': self.group, 'type': self.kind,
             'target': self.target, 'unit': self.unit, 'help': self.help}
        if self.kind in ('float', 'int'):
            d.update(min=self.min, max=self.max, step=self.step)
        if self.kind == 'enum':
            d['options'] = list(self.options)
        return d


def _f(key, label, group, lo, hi, step, unit='', help='', target='gait'):
    return Setting(key, label, group, 'float', target, lo, hi, step, unit, help=help)


SETTINGS = [
    # ── Walk ──
    Setting('gait_type', 'Gait', 'Walk', 'enum', options=['crawl', 'trot'],
            help='crawl: one leg up at a time. trot: diagonal pairs. Stop walking to switch.'),
    _f('speed_scale', 'Speed', 'Walk', 0.1, 1.5, 0.05, 'x', 'Multiplies the sticks before they become stride.'),
    _f('gait_freq', 'Step rate', 'Walk', 0.1, 2.0, 0.05, 'Hz', 'Full step cycles per second.'),
    _f('step_length', 'Stride length', 'Walk', 0.0, 70.0, 5.0, 'mm', 'Fore/aft stride at full stick.'),
    _f('step_lateral', 'Strafe stride', 'Walk', 0.0, 50.0, 5.0, 'mm', 'Sideways stride at full stick.'),
    _f('step_height', 'Foot lift', 'Walk', 10.0, 90.0, 5.0, 'mm', 'How high each foot rises in swing.'),
    _f('turn_gain', 'Turn rate', 'Walk', 0.0, 80.0, 5.0, 'mm', 'Arc each foot sweeps per half stride at full turn.'),
    _f('max_foot_offset', 'Max foot reach', 'Walk', 20.0, 80.0, 5.0, 'mm', 'Hard clamp on each foot\'s stride offset.'),
    _f('stick_deadzone', 'Stick deadzone', 'Walk', 0.05, 0.8, 0.05, '', 'Stick travel ignored around centre.'),
    _f('settle_speed', 'Settle speed', 'Walk', 20.0, 200.0, 10.0, 'mm/s', 'Foot speed when dropping back to stand.'),
    # ── Body ──
    _f('stand_z', 'Body height', 'Body', 100.0, 220.0, 5.0, 'mm', 'Standing height (R1/L1 still nudge it).'),
    _f('height_rate', 'R1/L1 height rate', 'Body', 10.0, 80.0, 5.0, 'mm/s'),
    _f('stance_x_front', 'Front feet fore/aft', 'Body', -60.0, 60.0, 5.0, 'mm', '+ = feet further back.'),
    _f('stance_x_rear', 'Rear feet fore/aft', 'Body', -60.0, 60.0, 5.0, 'mm', '+ = feet further back.'),
    _f('stance_y', 'Stance width', 'Body', 0.0, 50.0, 5.0, 'mm', 'All feet this far outward.'),
    # ── Balance ──
    Setting('balance_enabled', 'Body sway', 'Balance', 'bool',
            help='Crawl only: shift the body over the three planted feet before each lift.'),
    _f('balance_gain_lateral', 'Sway side-to-side', 'Balance', 0.0, 1.5, 0.05, 'x'),
    _f('balance_gain_fore_aft', 'Sway front/back', 'Balance', 0.0, 1.5, 0.05, 'x'),
    _f('balance_max_shift', 'Max sway', 'Balance', 0.0, 60.0, 5.0, 'mm'),
    _f('shift_frac', 'Four-feet-down time', 'Balance', 0.0, 0.9, 0.05, '', 'Share of each leg slot spent shifting.'),
    _f('shift_overlap', 'Sway overlap', 'Balance', 0.0, 0.5, 0.05, '', 'How far the slide spills into the swing.'),
    # ── Leveling ──
    Setting('level_enabled', 'IMU leveling', 'Leveling', 'bool',
            help='Use the IMU to keep the body level.'),
    _f('level_gain', 'Leveling gain', 'Leveling', 0.0, 5.0, 0.25, '1/s'),
    _f('level_max_deg', 'Max correction', 'Leveling', 0.0, 20.0, 1.0, 'deg'),
    _f('level_trim_roll', 'Level roll trim', 'Leveling', -20.0, 20.0, 0.5, 'deg', 'IMU roll that counts as level.'),
    _f('level_trim_pitch', 'Level pitch trim', 'Leveling', -20.0, 20.0, 0.5, 'deg', 'IMU pitch that counts as level.'),
    # ── Sentry (double-tap Y) ──
    _f('sentry_max_roll_deg', 'Max roll', 'Sentry', 0.0, 30.0, 1.0, 'deg', 'Left stick X at full tilt.'),
    _f('sentry_max_pitch_deg', 'Max pitch', 'Sentry', 0.0, 30.0, 1.0, 'deg', 'Left stick Y at full tilt.'),
    _f('sentry_max_yaw_deg', 'Max yaw', 'Sentry', 0.0, 30.0, 1.0, 'deg', 'Right stick X at full tilt.'),
    _f('sentry_rate_deg_s', 'Pose speed', 'Sentry', 5.0, 120.0, 5.0, 'deg/s', 'How fast the body follows the sticks.'),
    # ── Calibration (hold Y + D-pad up) ──
    _f('calib_step_deg', 'D-pad up/down step', 'Calibration', 0.25, 5.0, 0.25, 'deg'),
    _f('calib_fine_step_deg', 'D-pad left/right step', 'Calibration', 0.05, 1.0, 0.05, 'deg'),
    # ── Video (this bridge) ──
    _f('video_max_fps', 'Video frame cap', 'Video', 2.0, 30.0, 1.0, 'fps', target='bridge'),
    Setting('jpeg_quality', 'Depth JPEG quality', 'Video', 'int', 'bridge', 20, 95, 5,
            help='Re-encoding quality of the depth view (the color view is sent as the camera encodes it).'),
    _f('depth_min_m', 'Depth near', 'Video', 0.05, 1.0, 0.05, 'm', 'Warmest colour of the depth view.', 'bridge'),
    _f('depth_max_m', 'Depth far', 'Video', 0.3, 5.0, 0.1, 'm', 'Coolest colour of the depth view.', 'bridge'),
    # ── Map ──
    Setting('clear_map', 'Clear map', 'Map', 'action', 'bridge',
            help='Wipes the SLAM map and starts a new one from where the robot stands. Press A twice.'),
    Setting('save_map', 'Save map', 'Map', 'action', 'bridge',
            help='Saves the current SLAM map on the robot (~/spotmicro_maps, newest = static_map). '
                 'Press A twice.'),
    # ── Robot ──
    Setting('shutdown', 'Shut down robot', 'Robot', 'action', 'bridge',
            help='Powers the robot\'s computer off (disarm first). Press A twice. '
                 'Wait for the green light to stop blinking before switching the power off.'),
]

BY_KEY = {s.key: s for s in SETTINGS}


class SettingError(ValueError):
    pass


def coerce(key, value):
    """Validate/clamp a value from the Deck for setting `key` -> (Setting, value)."""
    s = BY_KEY.get(key)
    if s is None:
        raise SettingError(f'unknown setting "{key}"')
    if s.kind == 'action':
        if value is not True:
            raise SettingError(f'{key} is a button: send true')
        return s, True
    if s.kind == 'bool':
        if not isinstance(value, bool):
            raise SettingError(f'{key} needs true/false')
        return s, value
    if s.kind == 'enum':
        if value not in s.options:
            raise SettingError(f'{key} must be one of {s.options}')
        return s, value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SettingError(f'{key} needs a number')
    v = min(max(float(value), s.min), s.max)
    if s.step:
        # Snap to the step grid (from min) so float noise doesn't pile up.
        v = s.min + round((v - s.min) / s.step) * s.step
        v = min(max(v, s.min), s.max)
    v = round(v, 4)
    return s, (int(round(v)) if s.kind == 'int' else v)


def payload(values, available):
    """Message for <prefix>/settings.

    values:    key -> current value (missing = unknown)
    available: target -> bool (is that node reachable right now)
    """
    items = []
    for s in SETTINGS:
        d = s.describe()
        d['value'] = values.get(s.key)
        d['available'] = bool(available.get(s.target)) and (s.kind == 'action' or s.key in values)
        items.append(d)
    return {'v': 1, 'items': items}
