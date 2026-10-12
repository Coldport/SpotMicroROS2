import pytest

from spot_micro_move.tricks import STAND, TRICKS, Pose, duration, sample

GEOM = {'half_length': 93.0, 'half_width': 80.0, 'max_shift': 40.0}
_BODY = ('roll', 'pitch', 'yaw', 'fwd', 'left', 'up')


def same(a, b, tol=1e-6):
    feet = set(a.feet) | set(b.feet)
    return (all(abs(getattr(a, k) - getattr(b, k)) < tol for k in _BODY)
            and all(all(abs(p - q) < tol for p, q in zip(a.feet.get(leg, (0, 0, 0)), b.feet.get(leg, (0, 0, 0))))
                    for leg in feet))


@pytest.mark.parametrize('name', sorted(TRICKS))
def test_every_trick_starts_and_ends_standing(name):
    frames = TRICKS[name](GEOM)
    assert same(sample(frames, 0.0)[0], STAND)
    end, done = sample(frames, duration(frames) + 0.01)
    assert done and same(end, STAND)
    assert 1.0 < duration(frames) < 15.0


@pytest.mark.parametrize('name', sorted(TRICKS))
def test_poses_stay_in_a_sane_range(name):
    frames = TRICKS[name](GEOM)
    t = 0.0
    while t < duration(frames):
        p, _ = sample(frames, t)
        assert all(abs(getattr(p, k)) <= 25.0 for k in ('roll', 'pitch', 'yaw'))
        assert all(abs(getattr(p, k)) <= 45.0 for k in ('fwd', 'left', 'up'))
        assert all(abs(c) <= 70.0 for off in p.feet.values() for c in off)
        t += 0.02


def test_a_foot_only_lifts_after_the_body_has_shifted():
    frames = TRICKS['handshake'](GEOM)
    t = 0.0
    while t < duration(frames):
        p, _ = sample(frames, t)
        if p.feet.get('FR', (0, 0, 0))[2] > 1.0:
            # Over the other three feet: back (-fwd) and to the left (+left).
            assert p.fwd < -25.0 and p.left > 20.0
        t += 0.02


def test_but_merges_and_drops_feet():
    p = Pose(feet={'FR': (1, 2, 3)}).but(pitch=5.0, feet={'RL': (0, 1, 0)})
    assert p.pitch == 5.0 and set(p.feet) == {'FR', 'RL'}
    assert set(p.but(feet={'FR': None}).feet) == {'RL'}
