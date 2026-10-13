import math
import time

import numpy as np

from spot_micro_move.planner import Follower, build_costmap, path_length, plan

RES = 0.05


def room(w_m=4.0, h_m=3.0, wall_x=None, gap=None):
    """Free room with walls round the edge; optionally a vertical wall at
    wall_x (m) with a gap (y0, y1) in it. Returns the costmap inputs."""
    w, h = int(w_m / RES), int(h_m / RES)
    g = np.zeros((h, w), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    if wall_x is not None:
        c = int(wall_x / RES)
        g[:, c] = 100
        if gap:
            g[int(gap[0] / RES):int(gap[1] / RES), c] = 0
    return g.ravel(), w, h


def cm_for(*args, **kw):
    cells, w, h = room(*args, **kw)
    return build_costmap(cells, w, h, RES, 0.0, 0.0)


def clear_of_walls(cm, path, step=0.02):
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        n = max(1, int(math.dist((ax, ay), (bx, by)) / step))
        for i in range(n + 1):
            x, y = ax + (bx - ax) * i / n, ay + (by - ay) * i / n
            if cm.obstacle[cm.cell(x, y)]:
                return False
    return True


def test_open_room_is_a_straight_line():
    cm = cm_for()
    path, why = plan(cm, (0.8, 1.5), (3.2, 1.5))
    assert path, why
    assert len(path) == 2
    assert math.dist(path[-1], (3.2, 1.5)) < 0.1


def test_goes_through_the_gap():
    cm = cm_for(wall_x=2.0, gap=(2.2, 2.8))
    path, why = plan(cm, (0.8, 0.6), (3.2, 0.6))
    assert path, why
    assert clear_of_walls(cm, path)
    # where the path crosses x = 2.0, it is inside the gap
    ys = [ay + (by - ay) * (2.0 - ax) / (bx - ax)
          for (ax, ay), (bx, by) in zip(path, path[1:]) if (ax - 2.0) * (bx - 2.0) < 0]
    assert len(ys) == 1 and 2.2 <= ys[0] <= 2.8, ys
    assert path_length(path) > 4.0


def test_no_way_through_a_closed_wall():
    cm = cm_for(wall_x=2.0)
    path, why = plan(cm, (0.8, 1.5), (3.2, 1.5))
    assert path is None and why == 'no way through'


def test_goal_in_a_wall_snaps_to_free_space():
    cm = cm_for()
    path, why = plan(cm, (1.0, 1.5), (3.99, 1.5))   # on the right-hand wall
    assert path, why
    assert path[-1][0] < 3.9


def test_robot_next_to_a_wall_can_still_leave():
    cm = cm_for()
    path, why = plan(cm, (0.08, 1.5), (2.0, 1.5))   # hugging the left wall
    assert path, why


def test_unknown_is_blocked_unless_allowed():
    cells, w, h = room()
    g = cells.reshape(h, w).copy()
    g[:, int(2.0 / RES):int(2.4 / RES)] = -1
    blocked = build_costmap(g.ravel(), w, h, RES, 0.0, 0.0)
    assert plan(blocked, (0.8, 1.5), (3.2, 1.5))[0] is None
    allowed = build_costmap(g.ravel(), w, h, RES, 0.0, 0.0, allow_unknown=True)
    assert plan(allowed, (0.8, 1.5), (3.2, 1.5))[0] is not None


def test_big_map_plans_quickly():
    cells, w, h = room(20.0, 20.0, wall_x=10.0, gap=(18.0, 19.0))
    cm = build_costmap(cells, w, h, RES, 0.0, 0.0)
    t = time.monotonic()
    path, why = plan(cm, (1.0, 1.0), (19.0, 1.0))
    assert path, why
    assert time.monotonic() - t < 3.0


def simulate(path, pose, steps=4000, dt=0.1, **kw):
    f = Follower(path, **kw)
    x, y, yaw = pose
    for _ in range(steps):
        v, w, _left, done = f.step(x, y, yaw)
        if done:
            return (x, y), True
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
    return (x, y), False


def test_follower_reaches_the_goal():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.5)]
    end, done = simulate(path, (0.0, 0.0, math.pi))   # starts facing the wrong way
    assert done and math.dist(end, (2.0, 1.5)) <= 0.15


def test_follower_turns_in_place_first():
    f = Follower([(0.0, 0.0), (0.0, 1.0)])
    v, w, _, done = f.step(0.0, 0.0, 0.0)    # goal is 90 deg to the left
    assert v == 0.0 and w > 0.0 and not done


def test_follower_respects_limits():
    f = Follower([(0.0, 0.0), (3.0, 0.0)], max_linear=0.05, max_angular=0.2)
    v, w, left, _ = f.step(0.0, 0.1, 0.05)
    assert 0.0 < v <= 0.05 and abs(w) <= 0.2
    assert abs(left - 3.0) < 0.05


def test_follower_lines_up_before_walking():
    f = Follower([(0.0, 0.0), (0.0, 2.0)], turn_in_place_deg=20.0, align_deg=6.0)
    yaw = 0.0                                 # goal 90 deg to the left
    for _ in range(200):
        v, w, _, _ = f.step(0.0, 0.0, yaw)
        if v > 0.0:
            break
        yaw += w * 0.1
    # Only starts walking once within align_deg, not as soon as it's under 20 deg.
    assert abs(yaw - math.pi / 2) <= math.radians(6.0) + 1e-6


def test_follower_walks_without_much_turning():
    f = Follower([(0.0, 0.0), (3.0, 0.0)], max_angular=0.4, walk_turn_frac=0.4)
    v, w, _, _ = f.step(0.0, 0.0, math.radians(10.0))   # 10 deg off: walk, steer a little
    assert v > 0.0 and abs(w) <= 0.16 + 1e-9


def test_follower_speed_scales_both_limits():
    f = Follower([(0.0, 0.0), (3.0, 0.0)], max_linear=0.08, max_angular=0.4, speed=0.5)
    v, _, _, _ = f.step(0.0, 0.0, 0.0)
    assert 0.0 < v <= 0.04 + 1e-9
    _, w, _, _ = f.step(0.0, 0.0, math.pi / 2)          # turning on the spot
    assert abs(w) <= 0.2 + 1e-9
