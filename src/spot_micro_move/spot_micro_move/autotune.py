"""
Autotune: the robot walks a fixed test course and works out a first guess for
the movement parameters from what its IMU and lidar saw. Started from the
Deck's Tune screen (gait_node mode 'autotune'). No ROS here — gait_node feeds
step() an Obs every control tick and does what the returned Cmd says, so the
sequence and the maths can be unit-tested.

The course (each test starts and ends standing still, so the lidar scans and
IMU readings it is measured with aren't tipped by a lifted leg):
  forward     walk `distance_m` forward (by the gait's own odometry)
  back        and the same distance back
  left/right  `strafe_cycles` full step cycles sideways each way
  -> adjust: odom_scale, balance
  turn_left   turn on the spot until the IMU says `turn_deg`
  turn_right  and back the other way
  -> adjust: cmd_vel_turn_sign, turn_scale, walk_turn_trim

What really happened is measured by matching the lidar scans before and after
each test (scan_match.py; the range straight ahead/behind is the fallback) and
by the IMU's yaw; how steady the body was by the IMU's roll and pitch while
walking. Heading hold is off during the tests (gait_node), so veer shows up.

Safety: the robot stops a test early if the lidar sees something closer than
`min_clearance_m` in the way, and gives up on the whole run if the body tilts
past `max_tilt_deg`, a stick is touched, or the IMU goes quiet mid-turn. B,
the arm chord and losing the Deck still disarm as always.

Every parameter change is applied live as it is made and listed with its old
value, so the Deck can keep (save) or revert them all at the end.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from spot_micro_move import scan_match

WALK_TESTS = ('forward', 'back', 'left', 'right')
TURN_TESTS = ('turn_left', 'turn_right')
# (name, label, stick (vx, vy, wz), goal kind); 'adjust' rows run the analysis.
COURSE = (
    ('forward', 'Walk forward', (1.0, 0.0, 0.0), 'dist'),
    ('back', 'Walk back', (-1.0, 0.0, 0.0), 'dist'),
    ('left', 'Steps left', (0.0, 1.0, 0.0), 'cycles'),
    ('right', 'Steps right', (0.0, -1.0, 0.0), 'cycles'),
    ('adjust_walk', 'Adjusting walk', None, 'adjust'),
    ('turn_left', 'Turn left', (0.0, 0.0, 1.0), 'turn'),
    ('turn_right', 'Turn right', (0.0, 0.0, -1.0), 'turn'),
    ('adjust_turn', 'Adjusting turns', None, 'adjust'),
)
STILL_FOR_S = 1.0        # standing still this long before a scan counts as "settled"
MATCH_TIMEOUT_S = 10.0


@dataclass
class TuneConfig:
    distance_m: float = 1.0
    strafe_cycles: float = 2.0
    turn_deg: float = 180.0
    stick: float = 1.0               # stick deflection the tests walk at (speed_scale still applies)
    min_clearance_m: float = 0.3     # stop when something is this close in the way
    corridor_m: float = 0.17         # half-width of the robot's path for that check
    turn_clearance_m: float = 0.3    # nothing this close all round before turning
    max_tilt_deg: float = 25.0       # give up past this roll / pitch
    test_timeout_s: float = 90.0


@dataclass
class Obs:
    t: float                          # monotonic seconds
    odom: tuple                       # gait_node dead reckoning (x, y m, yaw rad) in its odom frame
    imu: Optional[tuple] = None       # (roll deg, pitch deg, yaw rad); None = stale
    still: bool = False               # IMU says the body isn't moving
    settled: bool = False             # feet are at the stand pose
    cycles: float = 0.0               # gait cycles walked in the current drive
    scan: Optional[np.ndarray] = None  # newest scan, Nx2 base-frame points
    scan_t: float = -1.0              # when it arrived
    stick: bool = False               # someone moved a stick


@dataclass
class Cmd:
    drive: Optional[tuple] = None     # (vx, vy, wz) stick units, before speed_scale; None = stand
    apply: dict = field(default_factory=dict)   # parameter -> new value, set now


@dataclass
class Capture:
    t: float
    odom: tuple
    yaw: Optional[float]
    scan: Optional[np.ndarray]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _rot(dx, dy, yaw):
    """(dx, dy) in a frame rotated by yaw -> the unrotated frame's axes reversed:
    world displacement -> robot frame at heading `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


class AutoTune:

    def __init__(self, params, cfg=None):
        """params: the gait_node values the analysis reads (see gait_node
        _tune_params): odom_scale, turn_scale, walk_turn_trim, cmd_vel_turn_sign,
        speed_scale, gait_type, balance_enabled, balance_gain_lateral,
        balance_gain_fore_aft, crawl8_shift_side, turn_model_rate (rad/s at a
        full turn stick per the gait's geometry, before turn_scale)."""
        self.p = dict(params)
        self.cfg = cfg or TuneConfig()
        self.step_i = 0
        self.stage = 'settle'          # settle -> drive -> settle_end -> measure -> next test
        self.state = 'running'         # running | done | failed | stopped
        self.message = ''
        self.segments = []             # per finished test, see _finish_test
        self.changes = []              # [{'key', 'old', 'new', 'why'}]
        self.findings = []             # texts for the Deck
        self.original = {}             # key -> value before the first change
        self.want_match = None         # (after, before, init) for gait_node to run, then on_match()
        self._match = None             # (requested at, result or None)
        self._start = None             # Capture at the start of the current test
        self._end = None
        self._still_since = None
        self._settle_from = None
        self._drive = {}               # samples while driving
        self._goal = {}                # test name -> goal (m or cycles), set by the prechecks
        self._reuse = None             # a capture still valid for the next test (no motion since)
        self._init = (0.0, 0.0, 0.0)

    # ── progress for the Deck ────────────────────────────────────────────────

    @property
    def test(self):
        return COURSE[self.step_i] if self.step_i < len(COURSE) else None

    def prompt(self):
        if self.state == 'done':
            return 'Done — keep or revert the changes'
        if self.state != 'running':
            return self.message
        name, label, _stick, kind = self.test
        if kind == 'adjust':
            return label
        if self.stage == 'settle':
            return f'{label}: standing still to take a scan…'
        if self.stage == 'drive':
            d = self._drive
            if kind == 'dist':
                return f'{label}: {d.get("progress", 0.0):.2f} of {self._goal.get(name, 0.0):.2f} m'
            if kind == 'cycles':
                return f'{label}: step {min(d.get("progress", 0.0) + 1, self.cfg.strafe_cycles):.0f} ' \
                       f'of {self.cfg.strafe_cycles:.0f}'
            return f'{label}: {math.degrees(abs(d.get("progress", 0.0))):.0f} of {self.cfg.turn_deg:.0f}°'
        return f'{label}: measuring…'

    def progress(self):
        """0..1 through the course."""
        if self.state == 'done':
            return 1.0
        frac = 0.0
        if self.stage == 'drive':
            frac = 0.5 * min(self._drive.get('frac', 0.0), 1.0)
        elif self.stage in ('settle_end', 'measure'):
            frac = 0.6
        return min((self.step_i + frac) / len(COURSE), 1.0)

    def report(self):
        test = self.test
        return {
            'state': self.state,
            'test': test[0] if test else '',
            'step': min(self.step_i + 1, len(COURSE)),
            'steps': len(COURSE),
            'prompt': self.prompt(),
            'progress': round(self.progress(), 3),
            'segments': [self._seg_summary(s) for s in self.segments],
            'changes': self.changes,
            'findings': self.findings,
        }

    @staticmethod
    def _seg_summary(s):
        out = {'name': s['name'], 'label': s['label'], 'note': s.get('note', '')}
        if s.get('skipped'):
            out['skipped'] = True
            return out
        est = s['est']
        out['est_m'] = round(math.hypot(*est), 3)
        real = s.get('real')
        out['real_m'] = round(math.hypot(real[0], real[1]), 3) if real else None
        out['source'] = s.get('source', '')
        yaw = s.get('yaw')
        out['yaw_deg'] = round(math.degrees(yaw), 1) if yaw is not None else None
        out['time_s'] = round(s['duration'], 1)
        for k in ('roll_rms', 'pitch_rms', 'peak_tilt'):
            if s.get(k) is not None:
                out[k] = round(s[k], 2)
        return out

    # ── control ──────────────────────────────────────────────────────────────

    def stop(self, why='Stopped'):
        if self.state == 'running':
            self.state = 'stopped'
            self.message = why

    def _fail(self, why):
        self.state = 'failed'
        self.message = why
        return Cmd()

    def on_match(self, m):
        """Result of the scan match gait_node ran for want_match."""
        if self._match is not None:
            self._match = (self._match[0], m)

    def step(self, obs):
        """One control tick -> what gait_node should do now."""
        if self.state != 'running':
            return Cmd()
        if obs.stick:
            return self._fail('Stopped: a stick was moved')
        if obs.imu is not None and max(abs(obs.imu[0]), abs(obs.imu[1])) > self.cfg.max_tilt_deg:
            return self._fail(f'Stopped: tilted past {self.cfg.max_tilt_deg:.0f}°')
        test = self.test
        if test is None:
            self.state = 'done'
            return Cmd()
        name, _label, stick, kind = test
        if kind == 'adjust':
            apply = self._analyse_walk() if name == 'adjust_walk' else self._analyse_turn()
            self.step_i += 1
            return Cmd(apply=apply)
        if self.stage == 'settle':
            cap = self._settled(obs)
            if cap is None:
                return Cmd()
            self._start = cap
            why = self._precheck(name, kind, cap)
            if why is not None:
                return why
            self._drive = {'roll': [], 'pitch': [], 'yaw_acc': 0.0, 'yaw_prev': cap.yaw,
                           'progress': 0.0, 'frac': 0.0, 'note': '', 'cycles0': obs.cycles}
            self.stage = 'drive'
        if self.stage == 'drive':
            return self._drive_tick(obs, name, stick, kind)
        if self.stage == 'settle_end':
            cap = self._settled(obs)
            if cap is None:
                return Cmd()
            self._end = cap
            self._request_match(kind)
            self.stage = 'measure'
            return Cmd()
        if self.stage == 'measure':
            requested, result = self._match
            if result is None and obs.t - requested < MATCH_TIMEOUT_S:
                return Cmd()
            self._finish_test(name, kind, result)
            # Still standing where this test ended: that is the next one's start.
            self._reuse = self._end
            self._start = self._end = self._match = None
            self.step_i += 1
            self.stage = 'settle'
            return Cmd()
        return Cmd()

    def _settled(self, obs):
        """Capture once the feet are down, the body still for STILL_FOR_S and a
        scan has arrived since; None until then."""
        if self._reuse is not None:
            reuse, self._reuse = self._reuse, None
            return reuse
        if self._settle_from is None:
            self._settle_from = obs.t
        if not (obs.settled and (obs.still or obs.imu is None)):
            self._still_since = None
            return None
        if self._still_since is None:
            self._still_since = obs.t
        if obs.t - self._still_since < STILL_FOR_S:
            return None
        if obs.scan is not None and obs.scan_t < self._still_since:
            return None                       # wait for a scan taken standing still
        if obs.scan is None and obs.t - self._settle_from < 3.0:
            return None                       # give the lidar a moment
        self._still_since = self._settle_from = None
        return Capture(obs.t, tuple(obs.odom), obs.imu[2] if obs.imu else None, obs.scan)

    def _precheck(self, name, kind, cap):
        """Room for this test? None = go; else a Cmd (failed run), or the test skipped."""
        cfg = self.cfg
        pts = cap.scan
        if pts is None:
            if self.step_i == 0:
                return self._fail('No lidar scan — is the stack running (L1+R1)?')
            self._skip_test(name, 'no lidar scan')
            return Cmd()
        if kind == 'dist':
            direction = 0.0 if name == 'forward' else math.pi
            free = scan_match.clearance(pts, direction, cfg.corridor_m)
            goal = cfg.distance_m if name == 'forward' else self._goal.get('forward', cfg.distance_m)
            room = free - cfg.min_clearance_m - 0.05
            if room < goal:
                if room < 0.3:
                    where = 'in front' if name == 'forward' else 'behind'
                    need = goal + cfg.min_clearance_m + 0.05
                    if name == 'forward':
                        return self._fail(f'Not enough room {where}: {free:.2f} m free, '
                                          f'needs about {need:.1f} m')
                    self._skip_test(name, f'only {free:.2f} m free {where}')
                    return Cmd()
                self.findings.append(f'Only {room:.2f} m of room for "{name}", so it walked that '
                                     f'instead of {goal:.1f} m.')
                goal = room
            self._goal[name] = goal
        elif kind == 'cycles':
            side = math.pi / 2 if name == 'left' else -math.pi / 2
            free = scan_match.clearance(pts, side, 0.25)
            if free < cfg.min_clearance_m + 0.1:
                self._skip_test(name, f'only {free:.2f} m free to the side')
                return Cmd()
        elif kind == 'turn':
            if cap.yaw is None:
                self._skip_test(name, 'no IMU — turns need its heading')
                return Cmd()
            near = scan_match.nearest(pts)
            if near < cfg.turn_clearance_m:
                self._skip_test(name, f'something {near:.2f} m away — too close to turn')
                return Cmd()
        return None

    def _skip_test(self, name, why):
        label = next(c[1] for c in COURSE if c[0] == name)
        self.segments.append({'name': name, 'label': label, 'skipped': True, 'note': f'skipped: {why}'})
        self.step_i += 1
        self.stage = 'settle'
        self._reuse = self._start      # still standing where this test would have started
        self._start = None

    def _drive_tick(self, obs, name, stick, kind):
        cfg, d, start = self.cfg, self._drive, self._start
        if obs.imu is not None:
            d['roll'].append(obs.imu[0])
            d['pitch'].append(obs.imu[1])
            if d['yaw_prev'] is not None:
                d['yaw_acc'] += _wrap(obs.imu[2] - d['yaw_prev'])
            d['yaw_prev'] = obs.imu[2]
        elif kind == 'turn':
            return self._fail('Stopped: lost the IMU mid-turn')
        elapsed = obs.t - start.t
        done = False
        if kind == 'dist':
            fx, _fy = _rot(obs.odom[0] - start.odom[0], obs.odom[1] - start.odom[1], start.odom[2])
            d['progress'] = abs(fx)
            goal = self._goal[name]
            d['frac'] = d['progress'] / goal if goal > 0 else 1.0
            done = d['progress'] >= goal
        elif kind == 'cycles':
            d['progress'] = obs.cycles - d['cycles0']
            d['frac'] = d['progress'] / cfg.strafe_cycles
            done = d['progress'] >= cfg.strafe_cycles
        else:
            d['progress'] = d['yaw_acc']
            d['frac'] = abs(d['yaw_acc']) / math.radians(cfg.turn_deg)
            done = d['frac'] >= 1.0
        if not done and obs.scan is not None and obs.scan_t > start.t:
            if kind == 'turn':
                gap = scan_match.nearest(obs.scan)
                limit = cfg.turn_clearance_m * 0.75
            else:
                vx, vy = stick[0], stick[1]
                gap = scan_match.clearance(obs.scan, math.atan2(vy, vx), cfg.corridor_m)
                limit = cfg.min_clearance_m
            if gap < limit:
                d['note'] = f'stopped early: something {gap:.2f} m away'
                done = True
        if not done and elapsed > cfg.test_timeout_s:
            d['note'] = f'stopped after {cfg.test_timeout_s:.0f} s (not making progress?)'
            done = True
        if done:
            d['duration'] = elapsed
            d['cycles'] = obs.cycles - d['cycles0']
            self.stage = 'settle_end'
            return Cmd()
        return Cmd(drive=tuple(cfg.stick * v for v in stick))

    def _request_match(self, kind):
        s, e = self._start, self._end
        ex, ey = _rot(e.odom[0] - s.odom[0], e.odom[1] - s.odom[1], s.odom[2])
        if s.yaw is not None and e.yaw is not None:
            dyaw = self._drive['yaw_acc'] if kind == 'turn' else _wrap(e.yaw - s.yaw)
        else:
            dyaw = _wrap(e.odom[2] - s.odom[2])
        self._init = (ex, ey, dyaw)
        if s.scan is None or e.scan is None:
            self.want_match = None
            self._match = (e.t - MATCH_TIMEOUT_S, None)   # nothing to match: don't wait
            return
        self.want_match = (e.scan, s.scan, self._init)
        self._match = (e.t, None)

    def _finish_test(self, name, kind, m):
        s, e, d = self._start, self._end, self._drive
        label = next(c[1] for c in COURSE if c[0] == name)
        ex, ey, _ = self._init
        seg = {'name': name, 'label': label, 'est': (ex, ey), 'duration': d['duration'],
               'cycles': d['cycles'], 'note': d['note'], 'real': None, 'source': ''}
        if s.yaw is not None and e.yaw is not None:
            seg['imu_yaw'] = d['yaw_acc']
        seg['yaw'] = seg.get('imu_yaw')
        if m is not None and m.ok:
            seg['real'] = (m.dx, m.dy)
            seg['lidar_yaw'] = m.dyaw
            seg['source'] = 'lidar'
            if seg['yaw'] is None:
                seg['yaw'] = m.dyaw
        elif kind == 'dist' and s.scan is not None and e.scan is not None:
            # Fallback: how much nearer the wall ahead (behind) got.
            direction = 0.0 if name == 'forward' else math.pi
            before = scan_match.clearance(s.scan, direction, 0.1)
            after = scan_match.clearance(e.scan, direction, 0.1)
            if math.isfinite(before) and math.isfinite(after):
                moved = before - after
                seg['real'] = (moved if name == 'forward' else -moved, 0.0)
                seg['source'] = 'range'
        if d['roll']:
            roll, pitch = np.asarray(d['roll']), np.asarray(d['pitch'])
            seg.update(roll_mean=float(roll.mean()), roll_rms=float(roll.std()),
                       pitch_mean=float(pitch.mean()), pitch_rms=float(pitch.std()),
                       peak_tilt=float(max(np.abs(roll).max(), np.abs(pitch).max())))
        self.segments.append(seg)

    # ── analysis ─────────────────────────────────────────────────────────────

    def _seg(self, name):
        return next((s for s in self.segments if s['name'] == name and not s.get('skipped')), None)

    def _change(self, key, new, why):
        old = self.p.get(key)
        if old == new:
            return {}
        self.original.setdefault(key, old)
        self.p[key] = new
        self.changes.append({'key': key, 'old': old, 'new': new, 'why': why})
        return {key: new}

    def _eff(self, stick):
        return min(abs(stick) * self.cfg.stick * float(self.p.get('speed_scale', 1.0)), 1.0)

    def _analyse_walk(self):
        apply = {}
        apply.update(self._fit_odom_scale())
        apply.update(self._fit_balance())
        self._walk_findings()
        return apply

    def _fit_odom_scale(self):
        ratios = []
        for name in ('forward', 'back'):
            s = self._seg(name)
            if s is None or s['real'] is None:
                continue
            est = math.hypot(*s['est'])
            real = abs(s['real'][0])
            if est < 0.1:
                continue
            ratios.append(real / est)
        if not ratios:
            self.findings.append('Could not measure the distance walked (no lidar match), '
                                 'so odom_scale is unchanged.')
            return {}
        if len(ratios) == 2 and abs(ratios[0] - ratios[1]) > 0.25 * max(ratios):
            self.findings.append(f'Walks {abs(ratios[0] - ratios[1]) / max(ratios) * 100:.0f}% further one way '
                                 f'than the other (forward {ratios[0]:.2f}, back {ratios[1]:.2f} of the '
                                 'commanded stride) — check stance_x_front/rear and the fore/aft sway.')
        ratio = sum(ratios) / len(ratios)
        if ratio < 0.6:
            self.findings.append(f'Feet slip a lot: only {ratio * 100:.0f}% of the commanded stride '
                                 'turns into motion — try a slower step rate or more foot lift.')
        old = float(self.p.get('odom_scale', 1.0))
        new = round(min(max(old * ratio, 0.3), 2.5), 3)
        if abs(new - old) < 0.02 * old:
            return {}
        return self._change('odom_scale', new, f'Odometry said {ratio ** -1:.2f}× the real distance '
                                               f'(measured {ratio:.2f} of commanded)')

    def _tilt_stats(self):
        segs = [s for s in (self._seg(n) for n in WALK_TESTS) if s is not None and 'roll_rms' in s]
        if not segs:
            return None
        return (sum(s['roll_rms'] for s in segs) / len(segs),
                sum(s['pitch_rms'] for s in segs) / len(segs),
                max(s['peak_tilt'] for s in segs),
                sum(s['roll_mean'] for s in segs if s['name'] in ('forward', 'back')) / 2.0)

    def _fit_balance(self):
        stats = self._tilt_stats()
        if stats is None:
            self.findings.append('No IMU data while walking, so the balance settings are unchanged.')
            return {}
        roll_rms, pitch_rms, peak, _ = stats
        gait = self.p.get('gait_type', 'crawl')
        apply = {}
        rocky = 'Body rocks {:.1f}° (rms) {} while walking'
        if gait == 'crawl' and self.p.get('balance_enabled', True):
            if roll_rms > 2.5:
                old = float(self.p.get('balance_gain_lateral', 0.0))
                new = round(min(old + 0.15, 1.2), 2)
                if new > old:
                    apply.update(self._change('balance_gain_lateral', new,
                                              rocky.format(roll_rms, 'side to side') + ': more sideways sway'))
            if pitch_rms > 2.5:
                old = float(self.p.get('balance_gain_fore_aft', 0.0))
                new = round(min(old + 0.15, 1.0), 2)
                if new > old:
                    apply.update(self._change('balance_gain_fore_aft', new,
                                              rocky.format(pitch_rms, 'front to back') + ': more fore/aft sway'))
        elif gait == 'crawl8':
            if roll_rms > 2.5:
                old = float(self.p.get('crawl8_shift_side', 0.0))
                new = min(old + 5.0, 40.0)
                if new > old:
                    apply.update(self._change('crawl8_shift_side', new,
                                              rocky.format(roll_rms, 'side to side') + ': more sideways sway'))
            if pitch_rms > 2.5:
                self.findings.append(rocky.format(pitch_rms, 'front to back') +
                                     ' — try crawl8 sway forward / back (Settings > Balance).')
        elif roll_rms > 2.5 or pitch_rms > 2.5:
            self.findings.append(rocky.format(max(roll_rms, pitch_rms), '') +
                                 ' — trot has no body sway; a crawl gait is steadier.')
        if gait in ('crawl', 'crawl8') and not self.p.get('balance_enabled', True) and roll_rms > 2.5:
            self.findings.append('Body sway is off — turning it on (Settings > Balance) should steady the walk.')
        if peak > 15.0:
            self.findings.append(f'Tipped up to {peak:.0f}° while walking — a slower step rate or '
                                 'a lower body would help.')
        return apply

    def _walk_findings(self):
        stats = self._tilt_stats()
        if stats is not None and abs(stats[3]) > 3.0:
            side = 'right' if stats[3] > 0 else 'left'
            self.findings.append(f'Leans {abs(stats[3]):.1f}° to the {side} while walking — '
                                 'check the IMU zero (Y + ↓) and the leg trims.')
        for name in ('forward', 'back'):
            s = self._seg(name)
            if s is None or s['real'] is None:
                continue
            fx, fy = s['real']
            if abs(fx) > 0.2 and abs(fy) / abs(fx) > 0.1:
                way = 'left' if fy > 0 else 'right'
                self.findings.append(f'Drifts {abs(fy) * 100:.0f} cm {way} over {abs(fx):.2f} m walking {name}.')
        for name in ('left', 'right'):
            s = self._seg(name)
            if s is None or s['real'] is None:
                continue
            est, real = abs(s['est'][1]), abs(s['real'][1])
            if est > 0.02:
                self.findings.append(f'Steps {name}: moved {real * 100:.0f} cm sideways for '
                                     f'{est * 100:.0f} cm commanded ({real / est * 100:.0f}%).')
            if s['yaw'] is not None and abs(math.degrees(s['yaw'])) > 10.0:
                self.findings.append(f'Turns {abs(math.degrees(s["yaw"])):.0f}° while stepping {name}.')

    def _analyse_turn(self):
        apply = {}
        rates, signs = [], []
        for name, wz in (('turn_left', 1.0), ('turn_right', -1.0)):
            s = self._seg(name)
            if s is None or s['yaw'] is None or s['duration'] <= 0.5:
                continue
            rates.append(abs(s['yaw']) / s['duration'] / self._eff(wz))
            signs.append(math.copysign(1.0, s['yaw']) * wz)
            if s['real'] is not None and math.hypot(*s['real']) > 0.1:
                self.findings.append(f'{s["label"]}: wandered {math.hypot(*s["real"]) * 100:.0f} cm '
                                     'while turning on the spot.')
            if 'lidar_yaw' in s and 'imu_yaw' in s:
                # The lidar match is mod 360; compare the half-turn's remainder.
                diff = math.degrees(_wrap(s['imu_yaw'] - s['lidar_yaw']))
                if abs(diff) > 8.0:
                    self.findings.append(f'{s["label"]}: IMU heading and lidar disagree by {abs(diff):.0f}° '
                                         '— redo the IMU axis calibration (Y + ←).')
        if not rates:
            self.findings.append('Turns not measured, so the turn settings are unchanged.')
            return apply
        if len(signs) == 2 and signs[0] != signs[1]:
            self.findings.append('It turned the same way for both turn directions — check the turn '
                                 'settings and ik_x_forward_sign; turn settings left unchanged.')
            return apply
        sign = signs[0]
        if sign != float(self.p.get('cmd_vel_turn_sign', 1.0)):
            apply.update(self._change('cmd_vel_turn_sign', sign,
                                      'A left turn on the stick turns it right' if sign < 0 else
                                      'A left turn on the stick turns it left'))
        if len(rates) == 2 and abs(rates[0] - rates[1]) > 0.2 * max(rates):
            self.findings.append(f'Turns {abs(rates[0] - rates[1]) / max(rates) * 100:.0f}% faster to the '
                                 f'{"left" if rates[0] > rates[1] else "right"}.')
        k = sum(rates) / len(rates)                  # rad/s per unit of (scaled) turn stick
        model = float(self.p.get('turn_model_rate', 0.0))
        if model > 0.0:
            old = float(self.p.get('turn_scale', 1.0))
            new = round(min(max(k / model, 0.2), 3.0), 2)
            if abs(new - old) >= 0.03:
                apply.update(self._change('turn_scale', new,
                                          f'Turns at {new:.2f}× the rate its leg geometry predicts '
                                          f'({math.degrees(k):.0f}°/s at full stick)'))
        apply.update(self._fit_veer(k, sign))
        return apply

    def _fit_veer(self, k, sign):
        """walk_turn_trim: turn stick per unit of forward stick that cancels the
        veer seen walking straight (heading hold off)."""
        trims = []
        for name, vx in (('forward', 1.0), ('back', -1.0)):
            s = self._seg(name)
            if s is None or s['yaw'] is None or s['duration'] <= 1.0:
                continue
            rate = s['yaw'] / s['duration']           # rad/s, + = drifted left
            if abs(math.degrees(s['yaw'])) > 3.0:
                way = 'left' if s['yaw'] > 0 else 'right'
                self.findings.append(f'Veered {abs(math.degrees(s["yaw"])):.0f}° {way} walking {name}.')
            trims.append(-rate / (sign * k) / (vx * self._eff(vx)))
        if not trims:
            return {}
        if len(trims) == 2 and trims[0] * trims[1] < 0 and min(abs(t) for t in trims) > 0.02:
            self.findings.append('It veers the same way walking forward and back — a turn trim can\'t '
                                 'fix that (look for a slipping foot); only the forward veer is trimmed.')
            trims = trims[:1]
        delta = sum(trims) / len(trims)
        old = float(self.p.get('walk_turn_trim', 0.0))
        new = round(min(max(old + delta, -0.4), 0.4), 3)
        if abs(new - old) < 0.01:
            return {}
        return self._change('walk_turn_trim', new, 'Cancels the veer when walking straight')
