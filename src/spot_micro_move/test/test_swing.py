import math

import pytest

from spot_micro_move.swing import MAX_DROP_SPEED, classic_swing, landing_share, soft_swing

SWING_S = 0.278      # crawl8 at gait_freq 0.45
N = 4000


def profile(height, peak=0.35, zone=20.0, speed=250.0, swing_s=SWING_S):
    """[(sw, z mm, fwd)] across one swing."""
    out = []
    for i in range(N + 1):
        sw = i / N
        lift, fwd = soft_swing(sw, swing_s, height, peak, zone, speed)
        out.append((sw, lift * height, fwd))
    return out


def speed_entering(prof, z_mm, swing_s=SWING_S):
    """Downward foot speed (mm/s) as it passes z_mm on the way down."""
    dt = swing_s / N
    for (sw0, z0, _), (_, z1, _) in zip(prof, prof[1:]):
        if sw0 > 0.5 and z0 > z_mm >= z1:
            return (z0 - z1) / dt
    raise AssertionError(f'never passed {z_mm} mm')


def test_classic_swing_is_the_old_curve():
    for sw in (0.0, 0.2, 0.5, 0.9, 1.0):
        lift, fwd = classic_swing(sw)
        assert lift == pytest.approx(math.sin(math.pi * sw) ** 2)
        assert fwd == pytest.approx(0.5 - 0.5 * math.cos(math.pi * sw))


@pytest.mark.parametrize('height', [30.0, 45.0, 90.0])
@pytest.mark.parametrize('peak', [0.2, 0.35, 0.5])
def test_soft_swing_shape(height, peak):
    prof = profile(height, peak)
    assert prof[0][1] == pytest.approx(0.0, abs=1e-9)
    assert prof[-1][1] == pytest.approx(0.0, abs=1e-9)
    assert prof[0][2] == pytest.approx(0.0) and prof[-1][2] == pytest.approx(1.0)
    top = max(prof, key=lambda r: r[1])
    assert top[1] == pytest.approx(height, rel=1e-3)
    assert top[0] == pytest.approx(peak, abs=2e-3)
    # Up, then only down: no dip below the floor or bounce on the way down.
    zs = [z for _, z, _ in prof]
    i = zs.index(top[1])
    assert all(a <= b + 1e-9 for a, b in zip(zs[:i], zs[1:i + 1]))
    assert all(a >= b - 1e-9 for a, b in zip(zs[i:], zs[i + 1:]))
    assert min(zs) >= -1e-9
    # Fore/aft never goes backwards.
    fs = [f for _, _, f in prof]
    assert all(a <= b + 1e-9 for a, b in zip(fs, fs[1:]))


def test_no_jumps():
    # Largest per-sample change stays small: the pieces meet smoothly.
    prof = profile(90.0)
    dt = SWING_S / N
    v = [(b[1] - a[1]) / dt for a, b in zip(prof, prof[1:])]
    assert max(abs(b - a) for a, b in zip(v, v[1:])) < 20.0     # mm/s per 70 us sample


def test_lands_softer_than_classic():
    soft = profile(90.0)
    classic = [(i / N, classic_swing(i / N)[0] * 90.0, 0.0) for i in range(N + 1)]
    for z in (20.0, 10.0, 5.0):
        assert speed_entering(soft, z) < 0.5 * speed_entering(classic, z)


def test_landing_speed_is_honoured_when_there_is_time():
    prof = profile(45.0, speed=250.0, swing_s=0.357)            # crawl8 at 0.35 Hz
    assert speed_entering(prof, 19.99, swing_s=0.357) == pytest.approx(250.0, rel=0.02)
    # and the foot is straight above its landing spot by then
    assert all(f == pytest.approx(1.0) for sw, z, f in prof if z < 19.9 and sw > 0.5)


def test_high_step_keeps_the_drop_within_reach():
    # 90 mm in a short swing: the landing gives way to the drop, not the other way.
    share = landing_share(SWING_S, 90.0, 0.35, 20.0, 100.0)
    drop_s = (1.0 - 0.35 - share) * SWING_S
    assert (90.0 - 20.0) / drop_s <= MAX_DROP_SPEED + 1e-6
    assert landing_share(SWING_S, 90.0, 0.35, 20.0, 100.0) >= 0.3 * 0.65 - 1e-9


def test_degenerate_inputs():
    assert soft_swing(0.5, SWING_S, 0.0, 0.35, 20.0, 250.0)[0] == 0.0
    # zone 0: plain rise/drop, still lands at zero
    lift, fwd = soft_swing(1.0, SWING_S, 40.0, 0.35, 0.0, 250.0)
    assert lift == pytest.approx(0.0) and fwd == pytest.approx(1.0)
    # zone above the step: whole descent is the slow landing
    prof = profile(15.0, zone=40.0)
    assert prof[-1][1] == pytest.approx(0.0, abs=1e-9)
