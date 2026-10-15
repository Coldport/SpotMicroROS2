"""
Lidar scan matching for autotune: how far the robot really moved between two
scans taken while it stood still, independent of the gait's own (commanded,
dead-reckoned) odometry. Plain numpy, no ROS, so it can be unit-tested.

  points = scan_points(ranges, angle_min, angle_increment, ...)   # Nx2, base frame
  m = match(after, before, init=(dx, dy, dyaw))
  m.dx, m.dy, m.dyaw   # pose of the robot at `after` in its frame at `before`

match() is point-to-point ICP with nearest neighbours found by brute force,
on scans thinned to MAX_POINTS (a 360-point scan pair is ~0.1 M distances per
iteration — a few ms on the Pi). Each iteration drops pairs further apart than
a shrinking gate, so walls that only one scan sees don't drag the fit. The
initial guess (from odometry / the IMU's yaw) matters for big turns: ICP only
finds the nearest local fit.
"""

import math
from dataclasses import dataclass

import numpy as np

MAX_POINTS = 360
MIN_POINTS = 40          # fewer points than this (either scan) = no match
GATES_M = (0.6, 0.4, 0.25, 0.15, 0.1, 0.08)   # outlier gate per stage, coarse to fine
ITERS_PER_GATE = 6
GOOD_RMS_M = 0.05        # residual below this, with enough inliers, = trustworthy
GOOD_INLIERS = 0.5


@dataclass
class Match:
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0          # rad
    rms: float = float('inf')  # m, over the inliers of the final stage
    inliers: float = 0.0       # fraction of `after` points matched in the final stage
    ok: bool = False

    @property
    def dist(self):
        return math.hypot(self.dx, self.dy)


def scan_points(ranges, angle_min, angle_increment, range_min=0.05, range_max=12.0,
                laser_pose=(0.0, 0.0, 0.0), body_radius=0.0, max_range=8.0):
    """LaserScan ranges -> Nx2 float array of hits in the base frame.

    laser_pose: (x, y, yaw) of the laser in the base frame. Returns closer than
    `body_radius` from the base origin (the robot's own legs) are dropped, and
    so are returns past `max_range` (sparse, noisy, and slow to match)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(range_min, 1e-3)) & (r <= min(range_max, max_range))
    r, a = r[ok], a[ok]
    lx, ly, lyaw = laser_pose
    pts = np.column_stack((lx + r * np.cos(a + lyaw), ly + r * np.sin(a + lyaw)))
    if body_radius > 0.0:
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) > body_radius]
    return pts


def thin(points, n=MAX_POINTS):
    """Every k-th point so at most `n` remain (scan order kept)."""
    if len(points) <= n:
        return points
    return points[::math.ceil(len(points) / n)]


def _transform(points, dx, dy, dyaw):
    c, s = math.cos(dyaw), math.sin(dyaw)
    return points @ np.array([[c, s], [-s, c]]) + (dx, dy)


def _best_fit(src, dst):
    """Rigid 2D transform (dx, dy, dyaw) minimising |R src + t - dst| (Kabsch)."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    h = (src - cs).T @ (dst - cd)
    dyaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(dyaw), math.sin(dyaw)
    t = cd - np.array([[c, -s], [s, c]]) @ cs
    return float(t[0]), float(t[1]), dyaw


def match(after, before, init=(0.0, 0.0, 0.0)):
    """Pose (dx, dy, dyaw) of the robot when `after` was scanned, in its frame
    when `before` was scanned, i.e. the transform that lays `after` onto
    `before`. Both are Nx2 base-frame points (scan_points)."""
    src, dst = thin(np.asarray(after, float)), thin(np.asarray(before, float))
    if len(src) < MIN_POINTS or len(dst) < MIN_POINTS:
        return Match()
    dx, dy, dyaw = (float(v) for v in init)
    rms, frac = float('inf'), 0.0
    for gate in GATES_M:
        for _ in range(ITERS_PER_GATE):
            moved = _transform(src, dx, dy, dyaw)
            d2 = ((moved[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
            nearest = d2.argmin(axis=1)
            dist = np.sqrt(d2[np.arange(len(src)), nearest])
            keep = dist < gate
            frac = float(keep.mean())
            if keep.sum() < MIN_POINTS // 2:
                return Match(dx, dy, dyaw, rms, frac, False)
            rms = float(np.sqrt((dist[keep] ** 2).mean()))
            # Incremental fit of the moved points, composed onto the estimate.
            ix, iy, iyaw = _best_fit(moved[keep], dst[nearest[keep]])
            c, s = math.cos(iyaw), math.sin(iyaw)
            dx, dy = c * dx - s * dy + ix, s * dx + c * dy + iy
            dyaw = math.atan2(math.sin(dyaw + iyaw), math.cos(dyaw + iyaw))
            if abs(ix) + abs(iy) < 1e-4 and abs(iyaw) < 1e-4:
                break
    return Match(dx, dy, dyaw, rms, frac, rms < GOOD_RMS_M and frac >= GOOD_INLIERS)


def clearance(points, direction, half_width=0.15):
    """Free distance (m) from the base origin along `direction` (rad, base
    frame: 0 = forward, pi/2 = left) within a corridor `half_width` either side
    of that line; inf if nothing is in the way."""
    if points is None or len(points) == 0:
        return float('inf')
    c, s = math.cos(direction), math.sin(direction)
    along = points[:, 0] * c + points[:, 1] * s
    across = -points[:, 0] * s + points[:, 1] * c
    hit = (along > 0.0) & (np.abs(across) <= half_width)
    return float(along[hit].min()) if hit.any() else float('inf')


def nearest(points):
    """Distance (m) to the closest point; inf if none."""
    if points is None or len(points) == 0:
        return float('inf')
    return float(np.hypot(points[:, 0], points[:, 1]).min())
