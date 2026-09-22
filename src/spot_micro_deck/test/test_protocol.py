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


def test_scan_length_mismatch_rejected():
    payload = P.pack_scan(0.0, 0.1, 5.0, b'\x01\x00\x02\x00')[:-1]
    with pytest.raises(P.ProtocolError):
        P.unpack_scan(payload)
