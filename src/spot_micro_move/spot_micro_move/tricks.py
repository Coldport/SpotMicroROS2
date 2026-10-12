"""
Tricks: short scripted moves played by gait_node (no ROS imports, so it can be
unit-tested).

A trick is a list of keyframes, each (seconds, Pose): the robot eases from the
previous pose to this one over that many seconds (smootherstep, so it starts
and stops gently). Every trick starts and ends at the plain stand pose.

Pose fields, all relative to the stand pose:
  roll, pitch, yaw   body rotation, deg, REP-103 (+roll = right side down,
                     +pitch = nose down, +yaw = turn left); feet stay planted
  fwd, left, up      body translation, mm (feet stay planted)
  feet               {leg: (fwd, out, up)} mm, moves that foot off its planted
                     spot (out = away from the body, up = lift)

A foot is only lifted after the body has moved over the other three
(`_shift`), the same idea as the crawl gait's balance shift.
"""

import math
from dataclasses import dataclass, field

FRONT = {'FL', 'FR'}
LEFT_SIDE = {'FL', 'RL'}


@dataclass
class Pose:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    fwd: float = 0.0
    left: float = 0.0
    up: float = 0.0
    feet: dict = field(default_factory=dict)

    def but(self, **changes):
        """Copy with some fields changed (feet: merged, legs set to None dropped)."""
        values = {k: getattr(self, k) for k in ('roll', 'pitch', 'yaw', 'fwd', 'left', 'up')}
        feet = dict(self.feet)
        for leg, off in changes.pop('feet', {}).items():
            if off is None:
                feet.pop(leg, None)
            else:
                feet[leg] = off
        values.update(changes)
        return Pose(feet=feet, **values)


_BODY = ('roll', 'pitch', 'yaw', 'fwd', 'left', 'up')
STAND = Pose()


def _smoother(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * x * (x * (6.0 * x - 15.0) + 10.0)


def blend(a, b, x):
    """Pose a..b at fraction x (already eased)."""
    body = {k: getattr(a, k) + (getattr(b, k) - getattr(a, k)) * x for k in _BODY}
    feet = {}
    for leg in set(a.feet) | set(b.feet):
        fa = a.feet.get(leg, (0.0, 0.0, 0.0))
        fb = b.feet.get(leg, (0.0, 0.0, 0.0))
        feet[leg] = tuple(p + (q - p) * x for p, q in zip(fa, fb))
    return Pose(feet=feet, **body)


def duration(frames):
    return sum(d for d, _ in frames)


def sample(frames, t):
    """-> (Pose at t seconds into the trick, finished?)."""
    prev = STAND
    for d, pose in frames:
        if t < d:
            return blend(prev, pose, _smoother(t / d) if d > 0.0 else 1.0), False
        t -= d
        prev = pose
    return prev, True


def _shift(leg, geom, gain=1.0):
    """Body (fwd, left) that puts it over the other three feet while `leg` is up."""
    sx = 1.0 if leg in FRONT else -1.0
    sy = 1.0 if leg in LEFT_SIDE else -1.0
    lim = geom['max_shift']
    fwd = -gain * sx * geom['half_length'] / 3.0
    left = -gain * sy * geom['half_width'] / 3.0
    return max(-lim, min(lim, fwd)), max(-lim, min(lim, left))


def _wave(frames, base, field_name, amp, half_period, cycles):
    """Append `cycles` back-and-forth swings of one body field around base."""
    for _ in range(cycles):
        for sign in (1.0, -1.0):
            frames.append((half_period, base.but(**{field_name: getattr(base, field_name) + sign * amp})))


# ── the tricks ───────────────────────────────────────────────────────────────

def handshake(geom):
    """Shift onto three legs, raise the front-right paw and shake it."""
    fwd, left = _shift('FR', geom)
    lean = STAND.but(fwd=fwd, left=left, pitch=-6.0)
    paw = (40.0, 10.0, 45.0)
    frames = [(0.9, lean), (0.6, lean.but(feet={'FR': paw}))]
    for _ in range(3):
        frames += [(0.22, lean.but(feet={'FR': (paw[0], paw[1], paw[2] + 20.0)})),
                   (0.22, lean.but(feet={'FR': paw}))]
    frames += [(0.4, lean.but(feet={'FR': paw})), (0.6, lean), (0.8, STAND)]
    return frames


def pee(geom):
    """Lift the rear-left leg out to the side and hold it, with a little shiver."""
    fwd, left = _shift('RL', geom)
    lean = STAND.but(fwd=fwd, left=left, roll=4.0)
    leg_up = (-10.0, 55.0, 40.0)
    frames = [(0.9, lean), (0.7, lean.but(feet={'RL': leg_up}))]
    for _ in range(8):
        frames += [(0.12, lean.but(up=-3.0, feet={'RL': leg_up})),
                   (0.12, lean.but(up=0.0, feet={'RL': leg_up}))]
    frames += [(0.4, lean.but(feet={'RL': leg_up})), (0.6, lean), (0.8, STAND)]
    return frames


def butt_shake(geom):
    """Nose down, then wag the rear end side to side (yaw about the front hips)."""
    down = STAND.but(pitch=8.0)
    frames = [(0.5, down)]
    yaw = 8.0
    # Turning the body about its centre and sliding it the other way keeps the
    # front still and swings only the rear.
    slide = geom['half_length'] * math.sin(math.radians(yaw))
    for _ in range(5):
        frames += [(0.18, down.but(yaw=yaw, left=-slide)), (0.18, down.but(yaw=-yaw, left=slide))]
    frames += [(0.2, down), (0.5, STAND)]
    return frames


def dance(geom):
    """About 9 s to a ~130 bpm beat: bounce, twist, bob, sway, bow."""
    beat = 0.46
    frames = []
    for i in range(4):                                     # bounce with a side lean
        frames += [(beat / 2, STAND.but(up=-18.0, roll=10.0 if i % 2 else -10.0)),
                   (beat / 2, STAND.but(up=0.0))]
    low = STAND.but(up=-10.0)
    frames.append((beat / 2, low))
    _wave(frames, low, 'yaw', 12.0, beat, 3)               # twist
    _wave(frames, STAND, 'pitch', 10.0, beat / 2, 3)       # head bob
    _wave(frames, STAND, 'left', 22.0, beat, 2)            # sway
    for i in range(4):                                     # hip circle
        a = math.pi / 2 * i
        frames.append((beat / 2, STAND.but(roll=8.0 * math.cos(a), pitch=8.0 * math.sin(a))))
    frames += [(beat, STAND.but(pitch=16.0, fwd=-15.0)),   # bow
               (beat * 2, STAND.but(pitch=16.0, fwd=-15.0)),
               (beat * 1.5, STAND)]
    return frames


def sit(geom):
    """Sit back on the rear legs for a few seconds."""
    seated = STAND.but(pitch=-18.0, fwd=-20.0, up=-12.0)
    return [(1.2, seated), (3.0, seated), (1.2, STAND)]


def bow(geom):
    """Play bow: front down, rear up, a little wiggle, back up."""
    bowed = STAND.but(pitch=16.0, fwd=-15.0)
    frames = [(0.9, bowed), (0.6, bowed)]
    _wave(frames, bowed, 'yaw', 6.0, 0.2, 3)
    frames += [(0.2, bowed), (0.8, bowed), (0.9, STAND)]
    return frames


def pushups(geom):
    """Three slow push-ups."""
    frames = []
    for _ in range(3):
        frames += [(0.7, STAND.but(up=-30.0)), (0.7, STAND)]
    return frames


TRICKS = {
    'handshake': handshake,
    'pee': pee,
    'butt_shake': butt_shake,
    'dance': dance,
    'sit': sit,
    'bow': bow,
    'pushups': pushups,
}
