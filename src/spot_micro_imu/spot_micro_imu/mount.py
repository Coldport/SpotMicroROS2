"""
IMU mount correction: turns the BNO055's own (sensor-frame) orientation into the
robot body's (REP-103: x forward, y left, z up). No ROS imports, so it can be
unit-tested.

Two corrections, both stored in imu_params.yaml:

  mount  q_sb, the body frame's rotation relative to the sensor (unit
         quaternion w, x, y, z). Fixes an IMU mounted tilted or rotated
         (axes swapped / flipped). Identity = the chip's axes ARE the body's.
  yaw0   heading (deg) that reads as zero yaw.

  published orientation  q_wb = Rz(-yaw0) * q_ws * q_sb
  published vectors      v_b  = rotate(conj(q_sb), v_s)

Zeroing (zero_mount) re-tilts the mount so the robot's current attitude reads
as level and takes the current heading as yaw0. Axis calibration
(calibrate_mount) builds the whole mount from five gravity readings, drone
style: level, left side down, right side down, nose down, nose up. Level gives
body z; the side tilts give body x (the axis it rolled about), the nose tilts
body y, and the two are cross-checked.
"""

import math


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-9:
        raise ValueError('zero quaternion')
    return tuple(c / n for c in q)


def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w t + cross(q.xyz, t)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def quat_from_matrix(m):
    """Unit quaternion of a 3x3 rotation matrix (rows)."""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = m
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = (0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        q = ((m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s)
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        q = ((m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        q = ((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s)
    q = quat_normalize(q)
    return q if q[0] >= 0.0 else tuple(-c for c in q)


def quat_from_yaw(yaw_deg):
    h = math.radians(yaw_deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def yaw_deg(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(v):
    n = math.sqrt(_dot(v, v))
    if n < 1e-9:
        raise ValueError('zero vector')
    return tuple(c / n for c in v)


def up_vector(q_ws):
    """World up (gravity reversed) expressed in the sensor frame."""
    return quat_rotate(quat_conj(q_ws), (0.0, 0.0, 1.0))


def mount_from_axes(z_s, x_s):
    """q_sb for a body whose up (z) and forward (x) axes are z_s / x_s in sensor
    coordinates. x_s only has to be roughly right: it is made square to z_s."""
    z = _unit(z_s)
    x = _unit(tuple(a - _dot(x_s, z) * b for a, b in zip(x_s, z)))
    y = _cross(z, x)
    # R_sb maps body coordinates to sensor coordinates: its columns are the body
    # axes in the sensor frame.
    m = ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))
    return quat_from_matrix(m)


def body_axes(q_sb):
    """(forward, left, up) body axes in sensor coordinates."""
    return tuple(quat_rotate(q_sb, e) for e in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))


def apply(q_ws, q_sb, yaw0):
    """Body orientation to publish."""
    return quat_mul(quat_conj(quat_from_yaw(yaw0)), quat_mul(q_ws, q_sb))


def zero_mount(q_sb, q_ws):
    """-> (q_sb, yaw0) that make the current attitude read roll = pitch = yaw = 0.
    Keeps the body's forward axis (just squares it to the new up)."""
    up = up_vector(q_ws)
    fwd, left, _ = body_axes(q_sb)
    if abs(_dot(fwd, _unit(up))) > 0.95:
        # Nose (nearly) straight up/down: forward can't be kept; derive it from left.
        fwd = _cross(left, up)
    new = mount_from_axes(up, fwd)
    return new, yaw_deg(quat_mul(q_ws, new))


def tilt_deg(u_a, u_b):
    c = max(-1.0, min(1.0, _dot(_unit(u_a), _unit(u_b))))
    return math.degrees(math.acos(c))


# Axis calibration poses, in the order they are asked for (drone style).
CALIB_POSES = ('level', 'left', 'right', 'front', 'back')
CALIB_POSE_TEXT = {
    'left': 'left side down', 'right': 'right side down',
    'front': 'nose down', 'back': 'nose up (tail down)',
}


def calibrate_mount(ups, min_tilt_deg=5.0):
    """q_sb from world-up (sensor frame) read in each of CALIB_POSES: level,
    left side down, right side down, nose down, nose up. Raises ValueError with
    a hint if the readings don't fit together.

    Level gives body z. Rolling right side down by theta about body +x moves
    world-up (seen from the body) toward +y, so up_right x up_level points
    along +x (and up_left the other way); tipping the nose down about +y gives
    up_front x up_level along +y. Each axis is found from its own pair of
    poses, then the two are checked against each other — square to one
    another, and right-handed with z (a pair done the wrong way round flips
    one) — and averaged."""
    missing = [k for k in CALIB_POSES if k not in ups]
    if missing:
        raise ValueError(f'missing poses: {", ".join(missing)}')
    z = _unit(ups['level'])
    for name in CALIB_POSES[1:]:
        t = tilt_deg(z, ups[name])
        if t < min_tilt_deg:
            raise ValueError(f'the {CALIB_POSE_TEXT[name]} tilt was only {t:.1f} deg — tilt it '
                             f'further (at least {min_tilt_deg:.0f} deg)')

    def axis(pos, neg, what):
        dp = [a - b for a, b in zip(_unit(ups[pos]), z)]
        dn = [a - b for a, b in zip(_unit(ups[neg]), z)]
        if _dot(dp, dn) > 0.0:
            raise ValueError(f'the {CALIB_POSE_TEXT[pos]} and {CALIB_POSE_TEXT[neg]} tilts '
                             f'went the same way — {what}')
        v = _cross([a - b for a, b in zip(_unit(ups[pos]), _unit(ups[neg]))], z)
        if math.sqrt(_dot(v, v)) < 1e-3:
            raise ValueError(f'could not find the axis from the {what}')
        return _unit(v)

    x = axis('right', 'left', 'roll it side to side about its long axis')
    y = axis('front', 'back', 'tip it nose down, then nose up')
    between = math.degrees(math.acos(max(-1.0, min(1.0, _dot(x, y)))))
    if not 60.0 <= between <= 120.0:
        raise ValueError(f'the side tilts and the nose tilts were about the same axis '
                         f'({between:.0f} deg apart) — side tilts roll it, nose tilts pitch it')
    if _dot(_cross(x, y), z) < 0.0:
        raise ValueError('one pair of tilts was the wrong way round (left/right or nose '
                         'down/up swapped) — try again')
    # Forward from the roll pair, plus forward implied by the pitch pair (y x z).
    fwd = [a + b for a, b in zip(x, _cross(y, z))]
    return mount_from_axes(z, fwd)


def describe(q_sb):
    """e.g. 'forward = sensor -y, left = sensor +x, up = sensor +z' (nearest axes)."""
    names = ('x', 'y', 'z')
    parts = []
    for label, v in zip(('forward', 'left', 'up'), body_axes(q_sb)):
        i = max(range(3), key=lambda k: abs(v[k]))  # pylint: disable=cell-var-from-loop
        off = tilt_deg(v, tuple((1.0 if k == i else 0.0) * math.copysign(1.0, v[i]) for k in range(3)))
        parts.append(f'{label} = sensor {"+" if v[i] >= 0 else "-"}{names[i]}'
                     + (f' ({off:.0f} deg off)' if off >= 2.0 else ''))
    return ', '.join(parts)
