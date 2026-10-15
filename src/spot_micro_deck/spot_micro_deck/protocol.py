"""
MQTT message layout shared by the robot's deck_bridge node and the Steam Deck
app. No ROS or MQTT imports here so it can be unit-tested — and
steamdeck_app/src/protocol.rs (the Deck app is written in Rust) must be kept in
sync with this file.

Everything goes through one MQTT broker (Mosquitto, running on the robot).
All topics live under a prefix, "spotmicro" by default.

Robot -> Deck
  <p>/robot/online   "1"/"0", retained. The bridge sets the "0" as its MQTT
                     last-will, so the Deck sees the robot drop off.
  <p>/video/color    JPEG bytes. Only published while a Deck is online and
  <p>/video/depth    has that view selected (encoding costs the Pi CPU).
  <p>/map            retained; u16 width, u16 height, f32 resolution (m/cell),
                     f32 origin_x, f32 origin_y, then zlib(int8 occupancy,
                     width*height, row 0 = lowest y; -1 unknown, 0..100 occupied).
  <p>/static_map    retained; the saved map (maps_dir/static_map.yaml, see
                     deck_bridge), same layout as <p>/map. Empty payload = none saved.
  <p>/scan           f32 angle_min, f32 angle_increment, f32 range_max, u16 count,
                     then count * u16 range in millimetres (0xFFFF = no return).
                     Angles are in the robot base frame (the bridge applies the laser's
                     yaw from TF), counter-clockwise from +x = front.
  <p>/audio          the robot's mic, while a Deck is online and the "Robot mic"
                     setting (audio_enabled) is on: u8 codec (1 = G.711 mu-law,
                     1 byte per sample, mono), u16 sample rate, u32 sequence
                     number (gaps = lost chunks), then the samples (~40 ms each).
  <p>/state          JSON object, ~5 Hz (see DeckBridge._build_state). Includes
                     "camera" (which camera is plugged in / streaming) and
                     "gait" (gait_node running / armed) and, while armed,
                     "mode" (gait_node's sentry / calibrate state; see
                     spot_micro_move/gait_node.py MODES).
  <p>/settings       JSON, ~1 Hz while a Deck is online: the live-tunable
                     settings with their current values (see settings.py).
                     The Deck's settings menu is drawn from this alone.
  <p>/settings/ack   JSON reply to each deck/set:
                       {"key": "step_height", "ok": true, "value": 40.0, "text": ""}
  <p>/track          JSON, sentry object tracking status, as track_node publishes
                     it (spot_micro_move/track_node.py), on every tracked frame:
                       {"state": "tracking", "box": [x, y, w, h], "err_deg": [yaw, pitch], "seq": 7}
                     box is normalised 0..1 in the camera image (x, y = top-left).

Deck -> Robot
  <p>/deck/online    "1"/"0", retained; the Deck's last-will is "0".
  <p>/deck/view      plain text: color | depth | lidar | map. Republished every
                     second as a heartbeat; the robot stops encoding video if
                     it goes quiet.
  <p>/deck/input     JSON, ~30 Hz:
                       {"seq": 12, "axes": [lx, ly, rx, ry, l2, r2],
                        "buttons": ["a", "l1"]}          (names of pressed buttons)
  <p>/deck/set       JSON, change one setting: {"key": "step_height", "value": 40.0}
  <p>/deck/nav       JSON, auto nav (gait_node must be in auto_nav mode):
                       {"cmd": "goal", "x": 1.25, "y": -0.5}   (map frame, metres)
                       {"cmd": "cancel"}
                       {"cmd": "speed", "step": 1}             (+1 / -1: next speed level)
                       {"cmd": "pose", "x": 1.0, "y": 2.0, "yaw": 1.57}
                         "the robot is here": reloads the saved map into SLAM
                         with the robot at that pose (map frame, yaw in rad
                         counter-clockwise from +x); the result comes back on
                         <p>/settings/ack with key "set_pose".
                     Progress comes back in <p>/state "nav" (nav_node's status).
  <p>/deck/track     JSON, sentry object tracking (normalised image box, see <p>/track):
                       {"cmd": "start", "box": [0.4, 0.3, 0.2, 0.25]}
                       {"cmd": "stop"}
  <p>/deck/tune      JSON, the Deck's Tune screen (gait_node modes autotune / servo_cal):
                       {"cmd": "autotune_start"}  | "autotune_stop" | "autotune_revert"
                       {"cmd": "autotune_keep"}   the bridge saves the run's changes into the
                                                  params file; result on <p>/settings/ack, key
                                                  "autotune_keep"
                       {"cmd": "autotune_clear"}  forget the last run's report
                       {"cmd": "servo_start"} | "servo_stop" (saves) | "servo_apply" | "servo_clear"
                       {"cmd": "servo_select", "leg": "FL", "joint": "hip"}
                       {"cmd": "servo_jog", "delta": 1.0}       (deg, the joint's real angle)
                       {"cmd": "servo_record", "actual": 93.5}  (deg the joint really sits at)
                     Progress comes back in <p>/state "mode": "tune" (autotune report) and
                     "servo_cal" (see gait_node _servo_cal_status).

All binary fields are little endian.
"""

import json
import math
import struct

DEFAULT_PREFIX = 'spotmicro'

# Main-display views, in the order the Deck's R2 button cycles through them (L2 goes back).
# Only the first two are camera streams; the robot sends video for those alone.
VIEW_NAMES = ['color', 'depth', 'lidar', 'map']
VIDEO_VIEWS = ('color', 'depth')

# Names follow SDL's game-controller naming, which is what the Steam Deck
# presents in game mode. paddle1..4 are the Deck's back buttons: SDL's P1/P2 are
# the right/left upper paddles (R4/L4) and P3/P4 the right/left lower (R5/L5).
BUTTON_NAMES = [
    'a', 'b', 'x', 'y',
    'l1', 'r1',
    'back', 'start', 'guide',
    'l3', 'r3',
    'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right',
    'paddle1', 'paddle2', 'paddle3', 'paddle4',
    'l2', 'r2',            # digital "pressed" copies of the trigger axes
]

# Order of "axes" in the input message and in the published Joy message.
# Sticks follow the ROS joy convention (left and up are positive, -1..1).
# Triggers are 0 (released) .. 1 (fully pressed) — NOT the inverted -1..1 range
# of ROS's xbox joy driver.
AXIS_NAMES = ['left_x', 'left_y', 'right_x', 'right_y', 'l2', 'r2']

_MAP_HEADER = struct.Struct('<HHfff')
_SCAN_HEADER = struct.Struct('<fffH')
_AUDIO_HEADER = struct.Struct('<BHI')
AUDIO_MULAW = 1


class ProtocolError(Exception):
    pass


def topics(prefix=DEFAULT_PREFIX):
    p = prefix.strip('/')
    return {
        'robot_online': f'{p}/robot/online',
        'video_color': f'{p}/video/color',
        'video_depth': f'{p}/video/depth',
        'map': f'{p}/map',
        'static_map': f'{p}/static_map',
        'scan': f'{p}/scan',
        'audio': f'{p}/audio',
        'state': f'{p}/state',
        'settings': f'{p}/settings',
        'settings_ack': f'{p}/settings/ack',
        'deck_online': f'{p}/deck/online',
        'deck_view': f'{p}/deck/view',
        'deck_input': f'{p}/deck/input',
        'deck_set': f'{p}/deck/set',
        'deck_nav': f'{p}/deck/nav',
        'deck_track': f'{p}/deck/track',
        'deck_tune': f'{p}/deck/tune',
        'track': f'{p}/track',
    }


def pack_map(width, height, resolution, origin_x, origin_y, zlib_data):
    return _MAP_HEADER.pack(width, height, resolution, origin_x, origin_y) + zlib_data


def unpack_map_header(payload):
    """-> (width, height, resolution, origin_x, origin_y, zlib_data)"""
    if len(payload) < _MAP_HEADER.size:
        raise ProtocolError('map payload too short')
    return _MAP_HEADER.unpack_from(payload) + (payload[_MAP_HEADER.size:],)


def pack_scan(angle_min, angle_increment, range_max, ranges_mm_le):
    """ranges_mm_le: bytes of little-endian u16 millimetre ranges."""
    return _SCAN_HEADER.pack(angle_min, angle_increment, range_max,
                             len(ranges_mm_le) // 2) + ranges_mm_le


def pack_audio(seq, rate, mulaw, codec=AUDIO_MULAW):
    return _AUDIO_HEADER.pack(codec, rate, seq & 0xFFFFFFFF) + mulaw


def unpack_audio(payload):
    """-> (codec, rate, seq, samples)"""
    if len(payload) < _AUDIO_HEADER.size:
        raise ProtocolError('audio payload too short')
    return _AUDIO_HEADER.unpack_from(payload) + (payload[_AUDIO_HEADER.size:],)


def unpack_scan(payload):
    """-> (angle_min, angle_increment, range_max, [range_mm, ...])"""
    if len(payload) < _SCAN_HEADER.size:
        raise ProtocolError('scan payload too short')
    angle_min, inc, range_max, count = _SCAN_HEADER.unpack_from(payload)
    body = payload[_SCAN_HEADER.size:]
    if len(body) != 2 * count:
        raise ProtocolError(f'scan length mismatch: {len(body)} bytes for {count} ranges')
    return angle_min, inc, range_max, list(struct.unpack(f'<{count}H', body))


def encode_input(sequence, axes, pressed_buttons):
    return json.dumps({'seq': sequence, 'axes': [round(a, 4) for a in axes],
                       'buttons': list(pressed_buttons)}, separators=(',', ':'))


def decode_set(payload):
    """-> (key, value) from a deck/set message."""
    try:
        msg = json.loads(payload)
        return str(msg['key']), msg['value']
    except (ValueError, KeyError, TypeError) as exc:
        raise ProtocolError(f'bad set message: {exc}') from exc


def decode_nav(payload):
    """-> ('goal', x, y), ('cancel',), ('speed', +1 | -1) or ('pose', x, y, yaw)
    from a deck/nav message."""
    try:
        msg = json.loads(payload)
        cmd = msg['cmd']
        if cmd == 'cancel':
            return ('cancel',)
        if cmd == 'goal':
            x, y = float(msg['x']), float(msg['y'])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('non-finite goal')
            return ('goal', x, y)
        if cmd == 'speed':
            step = msg['step']
            if step not in (1, -1) or isinstance(step, bool):
                raise ValueError('speed step must be 1 or -1')
            return ('speed', int(step))
        if cmd == 'pose':
            x, y, yaw = float(msg['x']), float(msg['y']), float(msg['yaw'])
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError('non-finite pose')
            return ('pose', x, y, yaw)
    except (ValueError, KeyError, TypeError) as exc:
        raise ProtocolError(f'bad nav message: {exc}') from exc
    raise ProtocolError(f'unknown nav command "{cmd}"')


def decode_track(payload):
    """-> ('start', [x, y, w, h]) or ('stop',) from a deck/track message."""
    try:
        msg = json.loads(payload)
        cmd = msg['cmd']
        if cmd == 'stop':
            return ('stop',)
        if cmd == 'start':
            box = [float(v) for v in msg['box']]
            if len(box) != 4 or not all(math.isfinite(v) for v in box):
                raise ValueError('box must be 4 finite numbers')
            if box[2] <= 0.0 or box[3] <= 0.0:
                raise ValueError('box must have a size')
            return ('start', box)
    except (ValueError, KeyError, TypeError) as exc:
        raise ProtocolError(f'bad track message: {exc}') from exc
    raise ProtocolError(f'unknown track command "{cmd}"')


TUNE_COMMANDS = ('autotune_start', 'autotune_stop', 'autotune_keep', 'autotune_revert', 'autotune_clear',
                 'servo_start', 'servo_stop', 'servo_select', 'servo_jog', 'servo_record', 'servo_apply',
                 'servo_clear')
TUNE_LEGS = ('FL', 'FR', 'RL', 'RR')
TUNE_JOINTS = ('hip', 'thigh', 'knee')


def decode_tune(payload):
    """-> a validated deck/tune command dict (only the fields that command takes)."""
    try:
        msg = json.loads(payload)
        cmd = msg['cmd']
        if cmd not in TUNE_COMMANDS:
            raise ProtocolError(f'unknown tune command "{cmd}"')
        out = {'cmd': cmd}
        if cmd == 'servo_select':
            if msg['leg'] not in TUNE_LEGS or msg['joint'] not in TUNE_JOINTS:
                raise ValueError(f'unknown joint {msg["leg"]} {msg["joint"]}')
            out.update(leg=msg['leg'], joint=msg['joint'])
        elif cmd in ('servo_jog', 'servo_record'):
            field = 'delta' if cmd == 'servo_jog' else 'actual'
            v = msg[field]
            if isinstance(v, bool):
                raise ValueError(f'{field} must be a number')
            v = float(v)
            limit = 30.0 if cmd == 'servo_jog' else 360.0
            if not math.isfinite(v) or abs(v) > limit:
                raise ValueError(f'{field} out of range')
            out[field] = v
        return out
    except (ValueError, KeyError, TypeError) as exc:
        raise ProtocolError(f'bad tune message: {exc}') from exc


def decode_input(payload):
    """-> (sequence, [6 axes], [6+ button flags in BUTTON_NAMES order])"""
    try:
        msg = json.loads(payload)
        axes = [float(a) for a in msg['axes']]
        pressed = set(msg['buttons'])
        seq = int(msg['seq'])
    except (ValueError, KeyError, TypeError) as exc:
        raise ProtocolError(f'bad input message: {exc}') from exc
    if len(axes) != len(AXIS_NAMES):
        raise ProtocolError(f'expected {len(AXIS_NAMES)} axes, got {len(axes)}')
    unknown = pressed - set(BUTTON_NAMES)
    if unknown:
        raise ProtocolError(f'unknown buttons: {sorted(unknown)}')
    return seq, axes, [1 if name in pressed else 0 for name in BUTTON_NAMES]
