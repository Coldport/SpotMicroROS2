import json

import pytest

from spot_micro_deck import protocol as P
from spot_micro_deck import settings as S


def test_keys_unique_and_ranges_sane():
    assert len(S.BY_KEY) == len(S.SETTINGS)
    for s in S.SETTINGS:
        assert s.target in ('gait', 'bridge')
        if s.kind in ('float', 'int'):
            assert s.min < s.max and s.step > 0, s.key
        if s.kind == 'enum':
            assert s.options, s.key
        assert s.kind in ('float', 'int', 'bool', 'enum', 'action'), s.key


def test_coerce_clamps_and_snaps_to_step():
    assert S.coerce('step_height', 1000.0)[1] == 90.0
    assert S.coerce('step_height', 41.3)[1] == 40.0
    assert S.coerce('speed_scale', 0.3 + 0.05 + 0.05)[1] == 0.4      # float noise removed
    assert S.coerce('jpeg_quality', 71)[1] == 70 and isinstance(S.coerce('jpeg_quality', 71)[1], int)
    assert S.coerce('level_enabled', True)[1] is True
    assert S.coerce('gait_type', 'trot')[1] == 'trot'


@pytest.mark.parametrize('key,value', [
    ('nope', 1.0), ('level_enabled', 1), ('gait_type', 'gallop'), ('step_height', 'high'),
    ('step_height', True),
])
def test_coerce_rejects(key, value):
    with pytest.raises(S.SettingError):
        S.coerce(key, value)


def test_payload_marks_unreachable_targets():
    msg = S.payload({'step_height': 40.0, 'video_max_fps': 20.0}, {'gait': False, 'bridge': True})
    items = {i['key']: i for i in msg['items']}
    assert items['step_height']['value'] == 40.0 and not items['step_height']['available']
    assert items['video_max_fps']['available']
    assert items['gait_type']['options'] == ['crawl', 'crawl8', 'trot']
    json.dumps(msg)   # must be serialisable as-is


def test_set_message_round_trip():
    assert P.decode_set(b'{"key":"step_height","value":40.0}') == ('step_height', 40.0)
    with pytest.raises(P.ProtocolError):
        P.decode_set(b'{"value":1}')


def test_settings_topics():
    t = P.topics('spotmicro')
    assert t['settings'] == 'spotmicro/settings'
    assert t['settings_ack'] == 'spotmicro/settings/ack'
    assert t['deck_set'] == 'spotmicro/deck/set'


def test_shutdown_is_an_action_available_with_the_bridge_alone():
    assert S.coerce('shutdown', True)[1] is True
    with pytest.raises(S.SettingError):
        S.coerce('shutdown', False)
    items = {i['key']: i for i in S.payload({}, {'gait': False, 'bridge': True})['items']}
    assert items['shutdown']['available'] and items['shutdown']['type'] == 'action'
    assert items['shutdown']['value'] is None
