"""
Foot path through one swing (no ROS imports, so it can be unit-tested).

classic_swing is the original symmetric step: sin^2 lift, cosine ease fore/aft.
It lands with zero speed only if the floor is exactly where the gait expects
it. On three legs the body sags, so the floor is usually 10-30 mm higher, and
the foot meets it while still dropping at up to ~1 m/s.

soft_swing (gait_node `soft_landing`) shapes the descent around that:
  * up      to the top by `peak` (share of the swing)
  * drop    quickly to `zone` mm above the planned floor
  * land    the last `zone` mm slowing evenly from `speed` mm/s to 0, so an
            early floor is met at no more than `speed`
Fore/aft finishes as the landing starts, so the foot comes straight down
instead of scuffing forward along the floor.
"""

import math

# The drop to the landing zone keeps enough time to average no more than this
# (its peak is ~1.5x; the classic swing peaks at ~1 m/s at the default settings),
# so a high step leaves less time for the landing. Never less than this share
# of the descent for the landing either way.
MAX_DROP_SPEED = 1000.0    # mm/s
MIN_LANDING_SHARE = 0.3


def classic_swing(sw):
    """-> (lift 0..1, fore/aft progress 0..1) at swing fraction sw (0..1)."""
    return math.sin(math.pi * sw) ** 2, 0.5 - 0.5 * math.cos(math.pi * sw)


def landing_share(swing_s, height, peak, zone, speed):
    """Share of the swing spent in the final landing (0 if there's no zone)."""
    zone = min(max(zone, 0.0), height)
    if zone <= 0.0:
        return 0.0
    swing_s = max(swing_s, 1e-3)
    down = 1.0 - peak
    # Slowing evenly from `speed` to 0 over `zone` mm takes 2 * zone / speed s.
    want = 2.0 * zone / max(speed, 1.0) / swing_s
    room = down - (height - zone) / MAX_DROP_SPEED / swing_s
    return min(want, max(room, MIN_LANDING_SHARE * down))


def soft_swing(sw, swing_s, height, peak, zone, speed):
    """-> (lift 0..1, fore/aft progress 0..1) at swing fraction sw (0..1).

    swing_s: how long the swing lasts (s); height: step height (mm);
    peak: share of the swing spent going up; zone: landing height (mm);
    speed: foot speed (mm/s) entering the landing zone, falling to 0 at touchdown.
    """
    if height <= 0.0:
        return 0.0, 0.5 - 0.5 * math.cos(math.pi * sw)
    peak = min(max(peak, 0.1), 0.9)
    zone = min(max(zone, 0.0), height)
    app = landing_share(swing_s, height, peak, zone, speed)
    land = 1.0 - app                   # swing fraction where the landing starts
    fwd = 0.5 - 0.5 * math.cos(math.pi * min(sw / land, 1.0))
    if sw < peak:
        z = height * math.sin(0.5 * math.pi * sw / peak) ** 2
    elif sw < land:
        # Cubic Hermite from the top (at rest) to the zone, arriving at the
        # landing's entry speed so there is no jolt where they meet.
        d = land - peak
        u = (sw - peak) / d
        v_in = 2.0 * zone / app * d if app > 0.0 else 0.0   # mm per unit u
        v_in = min(v_in, 3.0 * (height - zone))             # never dips below the zone
        z = ((2 * u ** 3 - 3 * u ** 2 + 1) * height
             + (-2 * u ** 3 + 3 * u ** 2) * zone
             - (u ** 3 - u ** 2) * v_in)
    else:
        t = (1.0 - sw) / app if app > 0.0 else 0.0   # 1 -> 0 across the landing
        z = zone * t * t
    return z / height, fwd
