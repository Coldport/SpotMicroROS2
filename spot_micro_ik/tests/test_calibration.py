"""Two-point servo calibration: fit_joint, JointCal.command and saving scale lines.
Run from the repository root: python3 -m pytest spot_micro_ik/tests"""

from pathlib import Path

import pytest

from spot_micro_ik.calibration import (JointCal, fit_joint, load_calibration, update_offsets_text,
                                       with_cal)
from spot_micro_ik.leg import Leg

CAL_FILE = Path(__file__).resolve().parents[2] / 'config' / 'servo_calibration.yaml'


def test_two_points_fit_exactly():
    # A servo that turns 10% too far and sits 3 deg off at 90.
    true = JointCal(0, 0.0, 270.0, offset=0.0)

    def really_at(servo):          # where the joint ends up for a servo command
        return 90.0 + (servo - 90.0 - 3.0) / 1.1

    pts = [(s, really_at(s)) for s in (70.0, 120.0)]
    offset, scale = fit_joint(pts)
    fitted = JointCal(true.channel, 0.0, 270.0, offset, scale)
    for want in (45.0, 90.0, 135.0):
        assert really_at(fitted.command(want)) == pytest.approx(want, abs=0.01)


def test_one_point_fits_the_offset_only():
    assert fit_joint([(95.0, 90.0)], scale=1.0) == (5.0, 1.0)
    offset, scale = fit_joint([(95.0, 100.0)], scale=1.1)
    assert scale == 1.1 and JointCal(0, 0, 270, offset, scale).command(100.0) == pytest.approx(95.0)


@pytest.mark.parametrize('pts', [[], [(90.0, 90.0), (95.0, 95.0)], [(90.0, 90.0), (150.0, 120.0)]])
def test_bad_points_are_refused(pts):
    with pytest.raises(ValueError):
        fit_joint(pts)


def test_scale_one_matches_the_old_offset_only_model():
    cal = load_calibration(CAL_FILE)
    leg = Leg('FL', cal['FL'])
    for angle in (30.0, 90.0, 150.0):
        assert leg.servo_command('hip', angle) == pytest.approx(angle + cal['FL'].hip.offset)


def test_saving_adds_and_updates_scale_lines():
    text = CAL_FILE.read_text(encoding='utf-8')
    cal = load_calibration(CAL_FILE)
    cal['RR'] = with_cal(cal['RR'], 'knee', 4.5, 1.05)
    new = update_offsets_text(text, cal)
    assert new.count('    scale:') == 1 and 'scale: 1.05' in new
    cal['RR'] = with_cal(cal['RR'], 'knee', 4.5, 0.98)
    newer = update_offsets_text(new, cal)
    assert newer.count('    scale:') == 1 and 'scale: 0.98' in newer
    # unchanged scale = 1 everywhere else: no new lines
    assert len(newer.split('\n')) == len(text.split('\n')) + 1
