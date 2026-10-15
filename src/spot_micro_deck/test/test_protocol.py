import json
import zlib

import pytest

from spot_micro_deck import protocol as P


def test_topics_use_prefix():
    t = P.topics('robot1/')
    assert t['deck_input'] == 'robot1/deck/input'
    assert t['video_depth'] == 'robot1/video/depth'


def test_input_round_trip():
    axes = [0.5, -0.25, 0.0, 1.0, 0.75, 0.0]
    raw = P.encode_input(7, axes, ['a', 'l2', 'dpad_up'])
    seq, out_axes, buttons = P.decode_input(raw)
    assert seq == 7
    assert out_axes == pytest.approx(axes, abs=1e-4)
    pressed = {n for n, b in zip(P.BUTTON_NAMES, buttons) if b}
    assert pressed == {'a', 'l2', 'dpad_up'}
    assert len(buttons) == len(P.BUTTON_NAMES)


@pytest.mark.parametrize('bad', [
    'not json',
    json.dumps({'seq': 1, 'axes': [0.0] * 5, 'buttons': []}),
    json.dumps({'seq': 1, 'axes': [0.0] * 6, 'buttons': ['nope']}),
    json.dumps({'axes': [0.0] * 6, 'buttons': []}),
])
def test_bad_input_rejected(bad):
    with pytest.raises(P.ProtocolError):
        P.decode_input(bad)


def test_map_round_trip():
    grid = bytes([255, 0, 100, 50]) * 3            # int8: -1, 0, 100, 50
    payload = P.pack_map(4, 3, 0.05, -1.5, 2.0, zlib.compress(grid))
    w, h, res, ox, oy, data = P.unpack_map_header(payload)
    assert (w, h) == (4, 3)
    assert res == pytest.approx(0.05) and ox == pytest.approx(-1.5) and oy == pytest.approx(2.0)
    assert zlib.decompress(data) == grid


def test_scan_round_trip():
    ranges = [500, 65535, 12000]
    body = b''.join(r.to_bytes(2, 'little') for r in ranges)
    a0, inc, rmax, out = P.unpack_scan(P.pack_scan(-3.14, 0.01, 12.0, body))
    assert out == ranges
    assert a0 == pytest.approx(-3.14, abs=1e-5) and rmax == pytest.approx(12.0)
    assert inc == pytest.approx(0.01, abs=1e-6)


def test_scan_length_mismatch_rejected():
    payload = P.pack_scan(0.0, 0.1, 5.0, b'\x01\x00\x02\x00')[:-1]
    with pytest.raises(P.ProtocolError):
        P.unpack_scan(payload)


def test_audio_round_trip():
    payload = P.pack_audio(7, 16000, b'\x01\xff\x7f')
    assert payload.hex() == '01803e07000000' + '01ff7f'    # pinned for protocol.rs
    assert P.unpack_audio(payload) == (P.AUDIO_MULAW, 16000, 7, b'\x01\xff\x7f')
    with pytest.raises(P.ProtocolError):
        P.unpack_audio(b'\x01\x80')


def test_nav_messages():
    assert P.decode_nav('{"cmd":"goal","x":1.25,"y":-0.5}') == ('goal', 1.25, -0.5)
    assert P.decode_nav('{"cmd":"cancel"}') == ('cancel',)
    assert P.decode_nav('{"cmd":"speed","step":-1}') == ('speed', -1)
    assert P.decode_nav('{"cmd":"pose","x":1.0,"y":-2,"yaw":1.5}') == ('pose', 1.0, -2.0, 1.5)


@pytest.mark.parametrize('bad', [
    'nope', '{"cmd":"goal","x":1}', '{"cmd":"goal","x":"a","y":0}',
    '{"cmd":"goal","x":NaN,"y":0}', '{"cmd":"fly"}', '{}',
    '{"cmd":"speed","step":3}', '{"cmd":"speed","step":true}', '{"cmd":"pose","x":0,"y":0}',
    '{"cmd":"pose","x":0,"y":0,"yaw":Infinity}',
])
def test_bad_nav_rejected(bad):
    with pytest.raises(P.ProtocolError):
        P.decode_nav(bad)


def test_decode_track():
    assert P.decode_track('{"cmd":"stop"}') == ('stop',)
    assert P.decode_track('{"cmd":"start","box":[0.1,0.2,0.3,0.4]}') == ('start', [0.1, 0.2, 0.3, 0.4])
    for bad in ('{"cmd":"start","box":[0.1,0.2,0.3]}', '{"cmd":"start","box":[0.1,0.2,0,0.4]}',
                '{"cmd":"start","box":[0.1,0.2,"NaN",0.4]}', '{"cmd":"go"}', 'nope'):
        with pytest.raises(P.ProtocolError):
            P.decode_track(bad)


def test_track_topics():
    t = P.topics('robo')
    assert t['deck_track'] == 'robo/deck/track'
    assert t['track'] == 'robo/track'


def test_decode_tune():
    assert P.topics('robo')['deck_tune'] == 'robo/deck/tune'
    assert P.decode_tune('{"cmd":"autotune_start","extra":1}') == {'cmd': 'autotune_start'}
    assert P.decode_tune('{"cmd":"servo_select","leg":"RR","joint":"knee"}') == \
        {'cmd': 'servo_select', 'leg': 'RR', 'joint': 'knee'}
    assert P.decode_tune('{"cmd":"servo_jog","delta":-1}') == {'cmd': 'servo_jog', 'delta': -1.0}
    assert P.decode_tune('{"cmd":"servo_record","actual":93.5}') == {'cmd': 'servo_record', 'actual': 93.5}
    for bad in ('{"cmd":"servo_select","leg":"XX","joint":"hip"}', '{"cmd":"servo_jog","delta":true}',
                '{"cmd":"servo_jog","delta":90}', '{"cmd":"servo_record"}', '{"cmd":"dance"}', 'nope'):
        with pytest.raises(P.ProtocolError):
            P.decode_tune(bad)
