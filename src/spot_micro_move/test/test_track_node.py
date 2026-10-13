import json
import math

import pytest

from spot_micro_move.track_node import angle_error, clamp_box, parse_command


def test_centred_box_has_no_error():
    yaw, pitch = angle_error((0.4, 0.4, 0.2, 0.2), (87.0, 58.0))
    assert yaw == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_signs_follow_rep103_body_pose():
    # Target on the left edge -> turn left (+yaw); at the bottom -> nose down (+pitch).
    yaw, _ = angle_error((0.0, 0.45, 0.0, 0.1), (90.0, 60.0))
    assert yaw == pytest.approx(45.0)
    _, pitch = angle_error((0.45, 0.9, 0.1, 0.1), (90.0, 60.0))
    assert pitch > 0.0
    yaw, pitch = angle_error((0.8, 0.0, 0.2, 0.2), (90.0, 60.0))
    assert yaw < 0.0 and pitch < 0.0


def test_pinhole_not_linear():
    # Halfway to the edge is less than half the half-FOV.
    yaw, _ = angle_error((0.25, 0.4, 0.0, 0.2), (90.0, 60.0))
    assert yaw == pytest.approx(math.degrees(math.atan(0.5)))


def test_clamp_box():
    assert clamp_box([0.9, 0.9, 0.3, 0.3]) == pytest.approx((0.9, 0.9, 0.1, 0.1))
    assert clamp_box([0.5, 0.5, -0.2, -0.1]) == pytest.approx((0.3, 0.4, 0.2, 0.1))
    assert clamp_box([1.2, 0.0, 0.1, 0.1]) is None
    assert clamp_box([0.1, 0.1, float('nan'), 0.1]) is None
    assert clamp_box([0.1, 0.1, 0.1]) is None


def test_parse_command():
    assert parse_command('{"cmd":"stop"}') == ('stop',)
    # 'start' always returns 2 items ('stop' returns 1)
    cmd, box = parse_command(  # pylint: disable=unbalanced-tuple-unpacking
        json.dumps({'cmd': 'start', 'box': [0.1, 0.2, 0.3, 0.4]}))
    assert cmd == 'start' and box == pytest.approx((0.1, 0.2, 0.3, 0.4))
    for bad in ('{"cmd":"go"}', '[]', '{"cmd":"start","box":[2,2,0.1,0.1]}', 'nope'):
        with pytest.raises((ValueError, KeyError, TypeError)):
            parse_command(bad)
