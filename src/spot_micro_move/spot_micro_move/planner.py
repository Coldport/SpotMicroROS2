"""
Path planning and following for auto-nav (nav_node). No ROS imports, so it can
be unit-tested.

  build_costmap()  SLAM occupancy grid -> coarse cost grid: walls grown by the
                   robot's radius are impassable, and cost rises toward walls
                   out to `inflation_radius`, so paths keep to open space.
  plan()           A* over the cost grid, 8-connected, then shortcut to the
                   fewest straight segments that stay clear. Start and goal
                   are moved to the nearest clear cell if they sit too close
                   to a wall (a robot standing next to a wall must still be
                   able to leave it).
  Follower         pure pursuit along the path, forward and turn only (never
                   sideways): when the next point is more than
                   `turn_in_place_deg` off the nose it stops and turns on the
                   spot until it faces within `align_deg`, then walks
                   straight, steering gently toward a point `lookahead`
                   metres along the path. `speed` scales both limits.

Coordinates are metres in the map frame. Grids follow nav_msgs/OccupancyGrid:
row 0 is the lowest y; -1 unknown, 0..100 occupied probability.
"""

import heapq
import math

import cv2
import numpy as np

OCCUPIED = 50          # occupancy at or above this is a wall
_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)]


class CostMap:
    """cost[row, col] >= 1 per cell step, np.inf = impassable."""

    def __init__(self, cost, resolution, origin_x, origin_y):
        self.cost = cost
        self.res = resolution
        self.ox = origin_x
        self.oy = origin_y
        self.h, self.w = cost.shape
        self.obstacle = None   # bool grid, set by build_costmap

    def cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def centre(self, row, col):
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def inside(self, row, col):
        return 0 <= row < self.h and 0 <= col < self.w

    def passable(self, row, col):
        return self.inside(row, col) and math.isfinite(self.cost[row, col])


def build_costmap(cells, width, height, resolution, origin_x, origin_y, *,
                  plan_resolution=0.1, robot_radius=0.17, inflation_radius=0.35,
                  inflation_cost=8.0, allow_unknown=False, unknown_cost=4.0):
    grid = np.asarray(cells, dtype=np.int8).reshape(height, width)
    f = max(1, int(round(plan_resolution / resolution)))
    # Pad to a multiple of f, then pool f x f blocks: any wall makes the
    # coarse cell a wall; otherwise any known-free cell makes it free.
    ph, pw = -(-height // f) * f, -(-width // f) * f
    padded = np.full((ph, pw), -1, dtype=np.int8)
    padded[:height, :width] = grid
    blocks = padded.reshape(ph // f, f, pw // f, f)
    wall = (blocks >= OCCUPIED).any(axis=(1, 3))
    free = ((blocks >= 0) & (blocks < OCCUPIED)).any(axis=(1, 3)) & ~wall
    unknown = ~wall & ~free
    res = resolution * f

    obstacle = wall | (unknown & (not allow_unknown))
    if obstacle.any():
        # Distance (m) from each cell to the nearest obstacle cell.
        dist = cv2.distanceTransform((~obstacle).astype(np.uint8), cv2.DIST_L2, 5) * res
    else:
        dist = np.full(obstacle.shape, np.inf, dtype=np.float32)
    cost = np.ones(obstacle.shape, dtype=np.float32)
    band = max(inflation_radius - robot_radius, 1e-6)
    near = dist < inflation_radius
    cost[near] += inflation_cost * ((inflation_radius - dist[near]) / band).clip(0.0, 1.0) ** 2
    if allow_unknown:
        cost[unknown] += unknown_cost
    cost[dist < robot_radius] = np.inf
    cost[obstacle] = np.inf
    cm = CostMap(cost, res, origin_x, origin_y)
    cm.obstacle = obstacle
    return cm


def _nearest_passable(cm, row, col, max_cells):
    """Closest passable cell to (row, col) within max_cells, or None."""
    if cm.passable(row, col):
        return row, col
    r0, r1 = max(0, row - max_cells), min(cm.h, row + max_cells + 1)
    c0, c1 = max(0, col - max_cells), min(cm.w, col + max_cells + 1)
    if r0 >= r1 or c0 >= c1:
        return None
    rows, cols = np.nonzero(np.isfinite(cm.cost[r0:r1, c0:c1]))
    if len(rows) == 0:
        return None
    d2 = (rows + r0 - row) ** 2 + (cols + c0 - col) ** 2
    i = int(np.argmin(d2))
    if d2[i] > max_cells * max_cells:
        return None
    return int(rows[i] + r0), int(cols[i] + c0)


def _free_start(cm, row, col, radius_cells):
    """The robot may stand closer to a wall than the planner allows (it was
    driven there by hand): let it move through the grown-wall zone around
    itself, at a high cost, so it can back out. Real wall cells stay blocked."""
    if cm.passable(row, col) or not cm.inside(row, col):
        return cm.cost
    cost = cm.cost.copy()
    r0, r1 = max(0, row - radius_cells), min(cm.h, row + radius_cells + 1)
    c0, c1 = max(0, col - radius_cells), min(cm.w, col + radius_cells + 1)
    rr, cc = np.mgrid[r0:r1, c0:c1]
    disk = (rr - row) ** 2 + (cc - col) ** 2 <= radius_cells * radius_cells
    sub = cost[r0:r1, c0:c1]
    sub[disk & ~cm.obstacle[r0:r1, c0:c1] & ~np.isfinite(sub)] = 50.0
    cost[row, col] = min(cost[row, col], 50.0)   # SLAM noise can put the robot "in" a wall
    return cost


def _line_clear(cost, a, b):
    """Every cell on the straight line a -> b (row, col) is passable and not
    much more expensive than open floor (so shortcuts don't hug walls)."""
    (r0, c0), (r1, c1) = a, b
    n = max(abs(r1 - r0), abs(c1 - c0))
    for i in range(n + 1):
        t = i / n if n else 0.0
        r = int(round(r0 + (r1 - r0) * t))
        c = int(round(c0 + (c1 - c0) * t))
        if not cost[r, c] <= 2.0:
            return False
    return True


def _shortcut(cost, cells):
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _line_clear(cost, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


def plan(cm, start, goal, goal_snap=0.5):
    """-> (path, why). path: [(x, y), ...] from start to goal (metres), or None
    with a reason. The goal is moved to the nearest reachable spot within
    `goal_snap` metres if it's in or too close to a wall."""
    s = cm.cell(*start)
    if not cm.inside(*s):
        return None, 'robot is off the map'
    cost = _free_start(cm, *s, radius_cells=int(math.ceil(0.4 / cm.res)))
    g = cm.cell(*goal)
    if not cm.inside(*g):
        return None, 'goal is off the map'
    g = _nearest_passable(cm, *g, max_cells=int(round(goal_snap / cm.res)))
    if g is None:
        return None, 'goal is in a wall or unexplored'
    if not math.isfinite(cost[s]):
        return None, 'robot is boxed in'

    w = cm.w
    flat = cost.ravel().tolist()
    gr, gc = g

    def h(r, c):
        dr, dc = abs(r - gr), abs(c - gc)
        return (dr + dc) + (_SQRT2 - 2.0) * min(dr, dc)

    start_i, goal_i = s[0] * w + s[1], gr * w + gc
    best = {start_i: 0.0}
    came = {}
    heap = [(h(*s), 0.0, start_i)]
    while heap:
        _, d, i = heapq.heappop(heap)
        if i == goal_i:
            break
        if d > best.get(i, math.inf):
            continue
        r, c = divmod(i, w)
        for dr, dc, step in _NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < cm.h and 0 <= nc < w):
                continue
            j = nr * w + nc
            cj = flat[j]
            if cj == math.inf:
                continue
            if dr and dc and (flat[r * w + nc] == math.inf or flat[nr * w + c] == math.inf):
                continue   # don't cut a wall's corner
            nd = d + step * 0.5 * (flat[i] + cj)
            if nd < best.get(j, math.inf):
                best[j] = nd
                came[j] = i
                heapq.heappush(heap, (nd + h(nr, nc), nd, j))
    if goal_i not in best:
        return None, 'no way through'

    cells = [divmod(goal_i, w)]
    i = goal_i
    while i != start_i:
        i = came[i]
        cells.append(divmod(i, w))
    cells.reverse()
    cells = _shortcut(cost, cells)
    path = [tuple(start)] + [cm.centre(r, c) for r, c in cells[1:]]
    if len(path) == 1:
        path.append(cm.centre(*cells[0]))
    return path, ''


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Follower:
    """Pure pursuit along a fixed path. step() -> (forward m/s, turn rad/s,
    metres left, arrived)."""

    def __init__(self, path, *, lookahead=0.35, goal_tolerance=0.15, max_linear=0.08,
                 max_angular=0.35, turn_in_place_deg=20.0, align_deg=6.0, walk_turn_frac=0.4,
                 turn_gain=1.5, speed=1.0):
        self.path = [tuple(p) for p in path]
        self.lookahead = lookahead
        self.tol = goal_tolerance
        self.max_v = max_linear
        self.max_w = max_angular
        self.turn_in_place = math.radians(turn_in_place_deg)
        self.align = math.radians(min(align_deg, turn_in_place_deg))
        self.walk_turn_frac = walk_turn_frac   # turn limit while walking, as a fraction of max_angular
        self.k_turn = turn_gain
        self.speed = speed     # 0..: scales max_linear and max_angular (the Deck's D-pad)
        self._seg = 0          # progress: index of the segment the robot is on
        self._turning = False  # turning on the spot until lined up within `align`

    def _project(self, x, y):
        """Advance progress to the closest segment (never backwards) ->
        (segment index, fraction along it)."""
        best = (math.inf, self._seg, 0.0)
        for i in range(self._seg, len(self.path) - 1):
            (ax, ay), (bx, by) = self.path[i], self.path[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0.0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(ax + t * dx - x, ay + t * dy - y)
            if d < best[0] - 1e-9:
                best = (d, i, t)
        self._seg = best[1]
        return best[1], best[2]

    def _along(self, seg, t, dist):
        """Point `dist` metres further along the path from (seg, t)."""
        (ax, ay), (bx, by) = self.path[seg], self.path[seg + 1]
        px, py = ax + t * (bx - ax), ay + t * (by - ay)
        for i in range(seg, len(self.path) - 1):
            nx, ny = self.path[i + 1]
            d = math.hypot(nx - px, ny - py)
            if d >= dist:
                k = dist / d if d else 0.0
                return px + k * (nx - px), py + k * (ny - py)
            dist -= d
            px, py = nx, ny
        return self.path[-1]

    def remaining(self, x, y):
        seg, t = self._project(x, y)
        (ax, ay), (bx, by) = self.path[seg], self.path[seg + 1]
        px, py = ax + t * (bx - ax), ay + t * (by - ay)
        return math.hypot(bx - px, by - py) + path_length(self.path[seg + 1:])

    def step(self, x, y, yaw):
        if len(self.path) < 2:
            return 0.0, 0.0, 0.0, True
        gx, gy = self.path[-1]
        to_goal = math.hypot(gx - x, gy - y)
        left = self.remaining(x, y)
        if to_goal <= self.tol:
            return 0.0, 0.0, 0.0, True
        seg, t = self._project(x, y)
        tx, ty = self._along(seg, t, self.lookahead)
        err = _wrap(math.atan2(ty - y, tx - x) - yaw)
        max_w = self.max_w * self.speed
        # Hysteresis: once turning on the spot, keep at it until lined up, so it
        # never walks off on a diagonal while still swinging round.
        if abs(err) > self.turn_in_place:
            self._turning = True
        elif abs(err) <= self.align:
            self._turning = False
        if self._turning:
            # Don't let the turn fade out to nothing just before lining up.
            w = math.copysign(max(0.3 * max_w, min(max_w, self.k_turn * abs(err))), err)
            return 0.0, w, left, False
        lim = max_w * self.walk_turn_frac
        w = max(-lim, min(lim, self.k_turn * err))
        # Slow down for heading errors and on the final approach.
        v = self.max_v * self.speed * max(0.5, math.cos(err)) * max(0.4, min(1.0, to_goal / 0.4))
        return v, w, left, False
