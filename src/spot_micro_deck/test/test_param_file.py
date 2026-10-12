import pytest

from spot_micro_deck import param_file as F
from spot_micro_deck import settings as S

SAMPLE = """/**:
    ros__parameters:
      # comment stays
      step_height: 60.0       # mm foot lift
      gait_type: "crawl"      # crawl/trot
      balance_enabled: true
      arm_buttons: ["l3", "r3"]  # list untouched
      jpeg_quality: 70
"""


def test_replaces_values_keeps_comments_and_columns():
    out = F.update_text(SAMPLE, {'step_height': 45.5, 'gait_type': 'trot',
                                 'balance_enabled': False, 'jpeg_quality': 85})
    lines = out.split('\n')
    assert lines[2] == '      # comment stays'
    assert lines[3] == '      step_height: 45.5       # mm foot lift'
    assert lines[4] == '      gait_type: "trot"       # crawl/trot'
    assert lines[5] == '      balance_enabled: false'
    assert lines[6] == SAMPLE.split('\n')[6]
    assert lines[7] == '      jpeg_quality: 85'


def test_missing_key_is_appended_to_block():
    out = F.update_text(SAMPLE, {'stance_y': 20.0})
    assert out.endswith('      jpeg_quality: 70\n      stance_y: 20.0\n')


def test_no_change_is_identical():
    assert F.update_text(SAMPLE, {}) == SAMPLE


def test_update_file_follows_symlink(tmp_path):
    real = tmp_path / 'real.yaml'
    real.write_text(SAMPLE)
    link = tmp_path / 'link.yaml'
    link.symlink_to(real)
    F.update_file(str(link), {'step_height': 30.0})
    assert link.is_symlink()
    assert 'step_height: 30.0 ' in real.read_text()


def test_every_setting_is_in_the_real_params_files():
    import os, yaml
    root = os.path.join(os.path.dirname(__file__), '..', '..')
    files = {'gait': 'spot_micro_move/config/move_params.yaml',
             'bridge': 'spot_micro_deck/config/deck_params.yaml'}
    for target, rel in files.items():
        with open(os.path.join(root, rel)) as f:
            text = f.read()
        params = yaml.safe_load(text)['/**']['ros__parameters']
        for s in S.SETTINGS:
            if s.target == target and s.kind != 'action':   # buttons aren't parameters
                assert s.key in params, (rel, s.key)
                # Round trip: writing the current value back changes nothing.
                assert F.update_text(text, {s.key: params[s.key]}) == text, (rel, s.key)
