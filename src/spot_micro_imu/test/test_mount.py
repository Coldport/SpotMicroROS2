import math

import pytest

from spot_micro_imu.mount import (apply, calibrate_mount, describe, quat_from_matrix,
                                  quat_mul, quat_rotate, up_vector, yaw_deg, zero_mount)


def axis_angle(axis, deg):
    h = math.radians(deg) / 2.0
    n = math.sqrt(sum(a * a for a in axis))
    return (math.cos(h), *(math.sin(h) * a / n for a in axis))


def rpy(q):
    w, x, y, z = q
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
    return roll, pitch, yaw_deg(q)


# A badly mounted chip: body forward = sensor +y, body left = sensor -x, up = sensor +z
# (rotated 90 deg), plus a 7 deg tilt.
R_SB_TRUE = quat_mul(quat_from_matrix(((0, -1, 0), (1, 0, 0), (0, 0, 1))), axis_angle((1, 1, 0), 7))
Q_BS_TRUE = (R_SB_TRUE[0], -R_SB_TRUE[1], -R_SB_TRUE[2], -R_SB_TRUE[3])


def sensor_q(body_q):
    """What the chip reads when the body has world orientation body_q."""
    return quat_mul(body_q, Q_BS_TRUE)


def close(a, b, tol=0.5):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def poses(heading, tilt=15.0, swap=None):
    """World-up in the sensor frame for the five calibration poses."""
    rot = {'level': (1, 0, 0, 0),
           'left': axis_angle((1, 0, 0), -tilt), 'right': axis_angle((1, 0, 0), tilt),
           'front': axis_angle((0, 1, 0), tilt), 'back': axis_angle((0, 1, 0), -tilt)}
    if swap:
        a, b = swap
        rot[a], rot[b] = rot[b], rot[a]
    return {k: up_vector(sensor_q(quat_mul(heading, r))) for k, r in rot.items()}


def test_calibration_recovers_the_mount_and_signs():
    heading = axis_angle((0, 0, 1), 40)
    mount = calibrate_mount(poses(heading))
    # Nose-down pitch and right-down roll now read correctly through the mount.
    probe = sensor_q(quat_mul(heading, quat_mul(axis_angle((0, 1, 0), 10), axis_angle((1, 0, 0), 5))))
    r, p, _ = rpy(apply(probe, mount, 0.0))
    assert abs(r - 5) < 0.5 and abs(p - 10) < 0.5
    d = describe(mount)
    assert d.startswith('forward = sensor +y') and 'left = sensor -x' in d and 'up = sensor +z' in d


def test_calibration_tolerates_sloppy_hand_tilts():
    heading = axis_angle((0, 0, 1), -70)
    ups = poses(heading, tilt=20.0)
    # By hand the side tilts pick up some pitch and the nose tilts some roll.
    ups['left'] = up_vector(sensor_q(quat_mul(heading, quat_mul(axis_angle((0, 1, 0), 4), axis_angle((1, 0, 0), -18)))))
    ups['front'] = up_vector(sensor_q(quat_mul(heading, quat_mul(axis_angle((1, 0, 0), -5), axis_angle((0, 1, 0), 25)))))
    mount = calibrate_mount(ups)
    probe = sensor_q(quat_mul(heading, axis_angle((0, 1, 0), 10)))
    r, p, _ = rpy(apply(probe, mount, 0.0))
    assert abs(p - 10) < 1.5 and abs(r) < 1.5


def test_calibration_rejects_bad_poses():
    heading = (1, 0, 0, 0)
    ups = poses(heading)
    ups['left'] = up_vector(sensor_q(axis_angle((1, 0, 0), -2)))
    with pytest.raises(ValueError, match='tilt it further'):
        calibrate_mount(ups)
    ups = poses(heading)
    ups['left'] = ups['right']
    with pytest.raises(ValueError, match='same way'):
        calibrate_mount(ups)
    with pytest.raises(ValueError, match='wrong way round'):
        calibrate_mount(poses(heading, swap=('left', 'right')))
    with pytest.raises(ValueError, match='same axis'):
        ups = poses(heading)
        calibrate_mount(ups | {'front': ups['right'], 'back': ups['left']})   # side tilts both times
    with pytest.raises(ValueError, match='wrong way round'):
        ups = poses(heading)   # did the nose tilts when asked for the side ones and vice versa
        calibrate_mount(ups | {'left': ups['back'], 'right': ups['front'], 'front': ups['right'], 'back': ups['left']})
    with pytest.raises(ValueError, match='missing poses: back'):
        calibrate_mount({k: v for k, v in poses(heading).items() if k != 'back'})


def test_zero_makes_the_current_attitude_read_zero_and_keeps_forward():
    mount = quat_from_matrix(((0, -1, 0), (1, 0, 0), (0, 0, 1)))   # right apart from the 7 deg tilt
    now = sensor_q(quat_mul(axis_angle((0, 0, 1), 70), axis_angle((0, 1, 0), 3)))
    new, yaw0 = zero_mount(mount, now)
    assert close(rpy(apply(now, new, yaw0)), (0, 0, 0), 0.01)
    # After zeroing, a later 10 deg nose-down pitch reads as pitch, not roll.
    later = sensor_q(quat_mul(axis_angle((0, 0, 1), 70), quat_mul(axis_angle((0, 1, 0), 3), axis_angle((0, 1, 0), 10))))
    r, p, _ = rpy(apply(later, new, yaw0))
    assert abs(p - 10) < 1.0 and abs(r) < 1.5


def test_vectors_rotate_into_the_body_frame():
    v = quat_rotate((R_SB_TRUE[0], -R_SB_TRUE[1], -R_SB_TRUE[2], -R_SB_TRUE[3]),
                    quat_rotate(R_SB_TRUE, (1.0, 0.0, 0.0)))
    assert close(v, (1, 0, 0), 1e-6)
