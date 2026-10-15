import math

import numpy as np
import pytest

from spot_micro_move import scan_match
from spot_micro_move.autotune import COURSE, AutoTune, Obs, TuneConfig

# A 5 m x 4 m room with a box in it, as wall points (world frame).
_rng = np.random.default_rng(0)


def _wall(a, b, step=0.02):
    n = int(math.dist(a, b) / step)
    t = np.linspace(0.0, 1.0, n)[:, None]
    return np.asarray(a) + t * (np.asarray(b) - np.asarray(a))


ROOM = np.vstack([_wall((-1.5, -2), (3.5, -2)), _wall((3.5, -2), (3.5, 2)), _wall((3.5, 2), (-1.5, 2)),
                  _wall((-1.5, 2), (-1.5, -2)), _wall((1.0, 0.8), (1.6, 0.8)), _wall((1.6, 0.8), (1.6, 1.4))])


def scan_from(x, y, yaw):
    """Room points as the robot at (x, y, yaw) sees them (no occlusion, a bit of noise)."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = ROOM - (x, y)
    pts = np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))
    pts = pts[::3] + _rng.normal(0.0, 0.005, (len(pts[::3]), 2))
    return pts


PARAMS = {'odom_scale': 1.0, 'turn_scale': 1.0, 'walk_turn_trim': 0.0, 'cmd_vel_turn_sign': 1.0,
          'speed_scale': 0.8, 'gait_type': 'crawl', 'balance_enabled': True,
          'balance_gain_lateral': 0.35, 'balance_gain_fore_aft': 0.35, 'crawl8_shift_side': 25.0,
          'turn_model_rate': 0.5}


class SimRobot:
    """Walks `stride_eff` of what odometry thinks, veers `veer` rad/s walking
    forward, turns `turn_sign` * `turn_rate` rad/s per unit of (scaled) turn stick."""

    def __init__(self, stride_eff=0.7, veer=0.02, turn_sign=-1.0, turn_rate=0.4, rock_deg=4.0, speed=0.1):
        self.pose = [0.0, 0.0, 0.0]
        self.odom = [0.0, 0.0, 0.0]
        self.t = 0.0
        self.cycles = 0.0
        self.stopped_at = 0.0
        self.k = dict(stride_eff=stride_eff, veer=veer, turn_sign=turn_sign, turn_rate=turn_rate,
                      rock=rock_deg, speed=speed)

    def obs(self, stick=False):
        moving = self.t - self.stopped_at < 0.6
        roll = self.k['rock'] * math.sin(self.t * 6.0) if moving else 0.0
        return Obs(t=self.t, odom=tuple(self.odom), imu=(roll, 0.3 * roll, self.pose[2]), still=not moving,
                   settled=not moving, cycles=self.cycles, scan=scan_from(*self.pose), scan_t=self.t - 0.05,
                   stick=stick)

    def step(self, cmd, dt=0.05, scale=0.8):
        self.t += dt
        if cmd.drive is None:
            return
        self.stopped_at = self.t
        vx, vy, wz = (v * scale for v in cmd.drive)
        sp = self.k['speed']
        # odometry: commanded, along the IMU heading
        h = self.pose[2]
        self.odom[0] += (vx * math.cos(h) - vy * math.sin(h)) * sp * dt
        self.odom[1] += (vx * math.sin(h) + vy * math.cos(h)) * sp * dt
        self.odom[2] = h
        e = self.k['stride_eff']
        p = self.pose
        p[0] += (vx * math.cos(h) - vy * math.sin(h)) * sp * e * dt
        p[1] += (vx * math.sin(h) + vy * math.cos(h)) * sp * e * dt
        p[2] += (self.k['veer'] * vx + self.k['turn_sign'] * self.k['turn_rate'] * wz) * dt
        self.cycles += 0.45 * dt


def run(robot, tune, max_s=600.0, every=None):
    while tune.state == 'running' and robot.t < max_s:
        cmd = tune.step(robot.obs())
        tune.p.update(cmd.apply)
        if tune.want_match is not None:
            tune.on_match(scan_match.match(*tune.want_match))
            tune.want_match = None
        if every:
            every(robot, tune)
        robot.step(cmd)
    return tune


def test_scan_match_finds_the_move():
    before = scan_from(0.0, 0.0, 0.0)
    for x, y, yaw, init in [(0.9, 0.05, 0.05, (0.7, 0.0, 0.0)), (0.0, 0.03, math.pi, (0.0, 0.0, 2.9)),
                            (-0.05, 0.1, -0.1, (0.0, 0.0, 0.0))]:
        m = scan_match.match(scan_from(x, y, yaw), before, init)
        assert m.ok
        assert m.dx == pytest.approx(x, abs=0.02) and m.dy == pytest.approx(y, abs=0.02)
        assert abs(math.atan2(math.sin(m.dyaw - yaw), math.cos(m.dyaw - yaw))) < 0.02


def test_clearance_looks_down_a_corridor():
    pts = scan_from(0.0, 0.0, 0.0)
    assert scan_match.clearance(pts, 0.0) == pytest.approx(3.5, abs=0.03)
    assert scan_match.clearance(pts, math.pi) == pytest.approx(1.5, abs=0.03)
    assert scan_match.clearance(pts, math.pi / 2, 0.25) == pytest.approx(2.0, abs=0.03)


def test_whole_course_finds_scale_turn_sign_and_veer():
    robot = SimRobot()
    tune = run(robot, AutoTune(PARAMS))
    assert tune.state == 'done', tune.message
    names = [s['name'] for s in tune.segments]
    assert names == [c[0] for c in COURSE if c[3] != 'adjust']
    new = {c['key']: c['new'] for c in tune.changes}
    assert new['odom_scale'] == pytest.approx(0.7, abs=0.03)
    assert new['cmd_vel_turn_sign'] == -1.0
    # 0.4 rad/s per unit stick measured vs 0.5 predicted
    assert new['turn_scale'] == pytest.approx(0.8, abs=0.05)
    # veer 0.02 rad/s per unit vx; trim must cancel it: sign * rate * trim = -veer
    assert -1.0 * 0.4 * new['walk_turn_trim'] == pytest.approx(-0.02, abs=0.01)
    assert 'balance_gain_lateral' in new           # rocking 4 deg -> ~2.8 rms
    assert tune.original['odom_scale'] == 1.0
    turned = math.degrees(robot.pose[2])
    assert abs(turned) < 15.0, 'the two half-turns cancel out'
    rep = tune.report()
    assert rep['state'] == 'done' and rep['progress'] == 1.0
    assert rep['segments'][0]['real_m'] == pytest.approx(0.7, abs=0.03)


def test_a_steady_accurate_robot_changes_nothing():
    robot = SimRobot(stride_eff=1.0, veer=0.0, turn_sign=1.0, turn_rate=0.5, rock_deg=0.5)
    tune = run(robot, AutoTune(PARAMS))
    assert tune.state == 'done'
    assert tune.changes == []


def test_touching_a_stick_stops_it():
    robot = SimRobot()
    tune = AutoTune(PARAMS)
    for _ in range(200):
        robot.step(tune.step(robot.obs()))
    tune.step(robot.obs(stick=True))
    assert tune.state == 'failed' and 'stick' in tune.message


def test_no_room_in_front_refuses_to_start():
    robot = SimRobot()
    robot.pose = [3.0, 0.0, 0.0]          # 0.5 m from the far wall
    tune = run(robot, AutoTune(PARAMS))
    assert tune.state == 'failed' and 'room' in tune.message
    assert robot.cycles == 0.0


def test_short_room_walks_less():
    robot = SimRobot(stride_eff=1.0)
    robot.pose = [2.0, 0.0, 0.0]          # 1.5 m to the wall: room for ~1.15 m of the 1.5 asked
    tune = run(robot, AutoTune(PARAMS, TuneConfig(distance_m=1.5)))
    assert tune.state == 'done'
    assert tune.segments[0]['est'][0] < 1.2
    assert any('room' in f for f in tune.findings)


def test_wall_ahead_stops_the_walk_early():
    robot = SimRobot(stride_eff=2.0)      # really walks twice as far as odometry says
    robot.pose = [1.9, 0.0, 0.0]          # 1.6 m free: room for 1 m by the precheck

    def nearest_wall(r, t):
        assert r.pose[0] < 3.5 - 0.25, 'walked into the wall'

    tune = run(robot, AutoTune(PARAMS), every=nearest_wall)
    assert tune.state == 'done'
    assert 'stopped early' in tune.segments[0]['note']
