"""Usage: python3 docs/tools/make_wiki_graphics.py [SCREENSHOT_DIR] [OUT_DIR]

Wiki graphics: the robot (side + top view, proportions from
config/spot_micro_config.yaml), the install flow, the Wi-Fi-over-Bluetooth
flow, the Deck controls and an annotated app screenshot. Colours match the
Deck app (steamdeck_app/src/ui.rs). Drawn at 2x and downsampled.

SCREENSHOT_DIR needs camera.png from the app, e.g.
  SDL_VIDEODRIVER=offscreen spotmicro-deck --screenshot DIR/camera.png
OUT_DIR defaults to docs/images."""
import math
import os
import sys

import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else None
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'docs', 'images')
FONT_DIR = os.path.join(ROOT, 'steamdeck_app', 'assets')
SS = 2

BG = (10, 12, 17)
PANEL = (17, 22, 31)
PANEL2 = (24, 31, 43)
LINE = (52, 62, 78)
TRACK = (40, 48, 62)
ACCENT = (76, 194, 255)
GOOD = (61, 220, 132)
WARN = (255, 181, 71)
BAD = (255, 92, 92)
TEXT = (232, 238, 246)
MUTED = (140, 152, 170)
BODY = (214, 222, 234)
BODY_SHADE = (150, 162, 180)
SERVO = (46, 54, 68)
PURPLE = (180, 140, 255)

_fonts = {}


def font(px, bold=False):
    key = (px, bold)
    if key not in _fonts:
        name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
        _fonts[key] = ImageFont.truetype(os.path.join(FONT_DIR, name), px * SS)
    return _fonts[key]


class Canvas:
    """Coordinates in output pixels; drawn at SS x."""

    def __init__(self, w, h, bg=BG):
        self.w, self.h = w, h
        self.img = Image.new('RGB', (w * SS, h * SS), bg)
        self.d = ImageDraw.Draw(self.img)

    def s(self, *v):
        return [x * SS for x in v]

    def rect(self, x, y, w, h, fill=None, outline=None, r=12, width=2):
        self.d.rounded_rectangle(self.s(x, y, x + w, y + h), r * SS, fill=fill, outline=outline,
                                 width=width * SS if outline else 0)

    def ellipse(self, cx, cy, rx, ry, fill=None, outline=None, width=2):
        self.d.ellipse(self.s(cx - rx, cy - ry, cx + rx, cy + ry), fill=fill, outline=outline,
                       width=width * SS if outline else 0)

    def circle(self, cx, cy, r, **kw):
        self.ellipse(cx, cy, r, r, **kw)

    def line(self, pts, fill=LINE, width=2):
        self.d.line([(x * SS, y * SS) for x, y in pts], fill=fill, width=width * SS, joint='curve')

    def poly(self, pts, fill=None, outline=None):
        self.d.polygon([(x * SS, y * SS) for x, y in pts], fill=fill, outline=outline)

    def text(self, x, y, s, px=16, bold=False, fill=TEXT, anchor='la'):
        self.d.text((x * SS, y * SS), s, font=font(px, bold), fill=fill, anchor=anchor)

    def text_w(self, s, px=16, bold=False):
        return self.d.textlength(s, font=font(px, bold)) / SS

    def lines(self, x, y, rows, px=16, gap=6, anchor='la'):
        """rows: [(text, colour, bold)]; returns the y below the last one."""
        for t, c, b in rows:
            self.text(x, y, t, px, b, c, anchor)
            y += px + gap
        return y

    def arrow(self, a, b, fill=MUTED, width=2, head=10):
        self.line([a, b], fill, width)
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        p1 = (b[0] - head * math.cos(ang - 0.45), b[1] - head * math.sin(ang - 0.45))
        p2 = (b[0] - head * math.cos(ang + 0.45), b[1] - head * math.sin(ang + 0.45))
        self.poly([b, p1, p2], fill=fill)

    def keycap(self, x, y, label, px=14, fill=TRACK, fg=TEXT):
        w = self.text_w(label, px, True) + 16
        self.rect(x, y, w, px + 12, fill=fill, r=6)
        self.text(x + w / 2, y + (px + 12) / 2, label, px, True, fg, 'mm')
        return w

    def callout(self, x, y, title, body, target, color=ACCENT, w=None, align='l', elbow=False):
        """A label card at (x, y) with a leader line to `target` (elbow: across, then down)."""
        tw = max([self.text_w(title, 16, True)] + [self.text_w(b, 14) for b in body])
        w = w or tw + 28
        h = 34 + 20 * len(body)
        if align == 'r':
            x -= w
        # leader from the card's nearest edge
        ax = x if target[0] < x else x + w
        if x <= target[0] <= x + w:
            ax = target[0]
            ay = y if target[1] < y else y + h
        else:
            ay = y + h / 2
        if elbow and not x <= target[0] <= x + w:
            self.line([(ax, ay), (target[0], ay), target], fill=color, width=2)
        else:
            self.line([(ax, ay), target], fill=color, width=2)
        self.circle(target[0], target[1], 5, fill=color)
        self.rect(x, y, w, h, fill=PANEL, outline=LINE, r=10)
        self.rect(x, y + 8, 4, h - 16, fill=color, r=2)
        self.text(x + 16, y + 10, title, 16, True, TEXT)
        for i, b in enumerate(body):
            self.text(x + 16, y + 34 + i * 20, b, 14, False, MUTED)

    def title(self, t, sub=None):
        self.text(40, 32, t, 30, True, TEXT)
        if sub:
            self.text(40, 74, sub, 16, False, MUTED)

    def save(self, name):
        os.makedirs(OUT, exist_ok=True)
        path = os.path.join(OUT, name)
        self.img.resize((self.w, self.h), Image.LANCZOS).save(path, optimize=True)
        print('wrote', path)


def geometry():
    with open(os.path.join(ROOT, 'config', 'spot_micro_config.yaml')) as f:
        cfg = yaml.safe_load(f)['spot_micro']
    return cfg['kinematics'], cfg['visual']


def leg_2d(hip, foot, l1, l2):
    """Knee position for a planar 2-link leg, knee bent backwards (towards -x)."""
    dx, dy = foot[0] - hip[0], foot[1] - hip[1]
    d = min(math.hypot(dx, dy), l1 + l2 - 1e-6)
    a = math.atan2(dy, dx)
    b = math.acos((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d))
    # screen y grows down; +b puts the knee behind (left of) the hip-foot line
    return hip[0] + l1 * math.cos(a + b), hip[1] + l1 * math.sin(a + b)


def servo(c, x, y, ang=0, size=18):
    c.circle(x, y, size, fill=SERVO, outline=(90, 102, 122), width=2)
    c.circle(x, y, size * 0.35, fill=ACCENT)


# ── robot side view ───────────────────────────────────────────────────────────

def robot_side():
    k, v = geometry()
    c = Canvas(1600, 960)
    c.title('SpotMicro — what\'s on the robot',
            'Side view, to scale (config/spot_micro_config.yaml). Front is on the right.')
    S = 2300  # px per metre
    ground = 860
    cx = 760
    body_h = v['body_height'] * S
    body_top = ground - k['default_body_height'] * S - body_h / 2
    mid_y = body_top + body_h / 2
    half = v['body_length'] * S / 2
    front, rear = v['front_length'] * S, v['rear_length'] * S
    # ground
    c.line([(140, ground), (1460, ground)], fill=TRACK, width=3)
    for gx in range(150, 1460, 40):
        c.line([(gx, ground + 2), (gx - 14, ground + 16)], fill=(28, 34, 44), width=2)

    l1, l2 = k['upper_leg_length'] * S, k['lower_leg_length'] * S
    hips = [cx + k['body_length'] * S / 2, cx - k['body_length'] * S / 2]
    toe_r = v['toe_radius'] * S / 2
    knees = []
    for far, shade in ((True, BODY_SHADE), (False, BODY)):
        for hx in hips:
            off = 22 if far else 0
            hip = (hx - off, mid_y + 6 - off * 0.3)
            foot = (hx - off + 6, ground - toe_r - off * 0.3)
            knee = leg_2d(hip, foot, l1, l2)
            col = (70, 80, 98) if far else shade
            c.line([hip, knee], fill=col, width=30)
            c.line([knee, foot], fill=col, width=20)
            c.circle(foot[0], foot[1], toe_r, fill=(60, 66, 78) if far else (34, 38, 46))
            if not far:
                knees.append((knee, foot, hip))
                servo(c, knee[0], knee[1], size=15)
    # body
    c.rect(cx - half, body_top, 2 * half, body_h, fill=BODY, r=18)
    c.poly([(cx + half - 4, body_top), (cx + half + front, body_top + body_h * 0.22),
            (cx + half + front, body_top + body_h * 0.78), (cx + half - 4, body_top + body_h)], fill=BODY)
    c.poly([(cx - half + 4, body_top), (cx - half - rear, body_top + body_h * 0.3),
            (cx - half - rear, body_top + body_h * 0.7), (cx - half + 4, body_top + body_h)], fill=BODY)
    # side cut-out showing the electronics
    win = (cx - half + 30, body_top + 22, 2 * half - 60, body_h - 44)
    c.rect(*win, fill=(30, 36, 48), r=10)
    pi = (win[0] + 14, win[1] + 12, 130, win[3] - 24)
    c.rect(*pi, fill=(32, 120, 70), r=6)
    c.text(pi[0] + pi[2] / 2, pi[1] + pi[3] / 2, 'Pi 5', 15, True, TEXT, 'mm')
    pca = (pi[0] + pi[2] + 14, win[1] + 12, 110, win[3] - 24)
    c.rect(*pca, fill=(40, 70, 140), r=6)
    c.text(pca[0] + pca[2] / 2, pca[1] + pca[3] / 2, 'PCA9685', 13, True, TEXT, 'mm')
    imu = (pca[0] + pca[2] + 14, win[1] + win[3] / 2 - 18, 50, 36)
    c.rect(*imu, fill=(120, 60, 150), r=5)
    c.text(imu[0] + 25, imu[1] + 18, 'IMU', 12, True, TEXT, 'mm')
    bat = (cx - half + 40, body_top + body_h - 18, 2 * half - 80, 12)
    # shoulder (hip) + thigh servos on the near legs
    for knee, foot, hip in knees:
        servo(c, hip[0], hip[1], size=22)
    # camera on the nose
    cam_x = cx + half + front - 6
    c.rect(cam_x - 6, mid_y - 22, 30, 44, fill=(40, 44, 54), r=6)
    c.circle(cam_x + 18, mid_y, 11, fill=(20, 24, 30), outline=ACCENT, width=3)
    # lidar on top
    lx = cx - 10
    c.rect(lx - 70, body_top - 26, 140, 26, fill=(36, 40, 50), r=4)
    c.rect(lx - 52, body_top - 66, 104, 42, fill=(48, 54, 66), r=14)
    c.rect(lx - 52, body_top - 54, 104, 10, fill=(20, 24, 30), r=3)
    # front marker
    c.arrow((1330, 150), (1460, 150), fill=ACCENT, width=4, head=16)
    c.text(1330, 118, 'FRONT', 16, True, ACCENT)

    knee_f, foot_f, hip_f = knees[0]
    knee_r, foot_r, hip_r = knees[1]
    c.callout(60, 290, 'RPLIDAR A2M12', ['360° scan, 10 Hz', '→ /scan → SLAM map'], (lx - 52, body_top - 45), ACCENT, elbow=True)
    c.callout(1180, 380, 'Camera (front)', ['RealSense D405 (colour + depth)', 'or any USB webcam'], (cam_x + 18, mid_y), ACCENT)
    c.callout(60, 140, 'Raspberry Pi 5', ['ROS 2 Jazzy, MQTT broker,', 'deck_bridge + gait_node'], (pi[0] + 20, pi[1] + 8), GOOD, elbow=True)
    c.callout(740, 120, 'PCA9685 servo driver', ['I2C → 12 servo channels'], (pca[0] + 80, pca[1] + 8), (110, 150, 255))
    c.callout(1100, 220, 'BNO055 IMU', ['roll / pitch / heading', 'levels the body while walking'], (imu[0] + 25, imu[1]), PURPLE, elbow=True)
    c.callout(1200, 560, 'Hip + thigh servos', ['"Hip" swings the leg out,', '"Thigh" swings it fore/aft'], (hip_f[0] + 10, hip_f[1] + 10), WARN)
    c.callout(1200, 700, 'Knee servo', ['bends backwards, like a dog'], (knee_f[0] + 8, knee_f[1]), WARN)
    c.callout(60, 560, 'Rear legs', ['same three joints:', 'Hip · Thigh · Knee'], (knee_r[0] - 8, knee_r[1]), WARN)
    c.callout(60, 720, 'Feet', ['rubber toes; the gait keeps', '3–4 on the ground'], (foot_r[0] - 10, foot_r[1]), MUTED)
    c.save('robot_side.png')


# ── robot top view ───────────────────────────────────────────────────────────

def robot_top():
    k, v = geometry()
    c = Canvas(1600, 900)
    c.title('Leg names and which way is which',
            'Top view. The names match the app\'s servo calibration grid; the arrows show what the sticks do.')
    S = 2400
    cx, cy = 640, 500
    L, W = k['body_length'] * S, k['body_width'] * S
    hip_len = k['hip_length'] * S
    bw = v['body_width'] * S
    half = v['body_length'] * S / 2
    # legs (drawn under the body)
    legs = {'Front left': (1, -1), 'Front right': (1, 1), 'Rear left': (-1, -1), 'Rear right': (-1, 1)}
    pos = {}
    for name, (fx, sy) in legs.items():
        hx, hy = cx + fx * L / 2, cy + sy * W / 2
        tx, ty = hx, hy + sy * (hip_len + 18)
        c.line([(hx, hy), (tx, ty)], fill=BODY_SHADE, width=26)
        c.line([(tx, ty), (tx + 26, ty)], fill=BODY, width=30)
        c.circle(tx + 34, ty, 13, fill=(34, 38, 46))
        servo(c, hx, hy, size=17)
        pos[name] = (tx + 34, ty)
    c.rect(cx - half, cy - bw / 2, 2 * half, bw, fill=BODY, r=16)
    c.poly([(cx + half - 2, cy - bw / 2), (cx + half + v['front_length'] * S, cy - bw * 0.3),
            (cx + half + v['front_length'] * S, cy + bw * 0.3), (cx + half - 2, cy + bw / 2)], fill=BODY)
    c.poly([(cx - half + 2, cy - bw / 2), (cx - half - v['rear_length'] * S, cy - bw * 0.35),
            (cx - half - v['rear_length'] * S, cy + bw * 0.35), (cx - half + 2, cy + bw / 2)], fill=BODY)
    c.circle(cx - 10, cy, 62, fill=(48, 54, 66))
    c.circle(cx - 10, cy, 30, fill=(20, 24, 30))
    c.text(cx - 10, cy, 'lidar', 13, True, MUTED, 'mm')
    nose = cx + half + v['front_length'] * S
    c.rect(nose - 8, cy - 26, 22, 52, fill=(40, 44, 54), r=5)
    c.circle(nose + 14, cy, 9, fill=ACCENT)
    # names
    for name, (px, py) in pos.items():
        up = py < cy
        c.text(px, py + (-34 if up else 34), name, 18, True, TEXT, 'mm')
    # axes
    c.arrow((cx, cy), (cx + 330, cy), fill=GOOD, width=3, head=14)
    c.text(cx + 340, cy - 10, 'x  forward', 15, True, GOOD)
    c.arrow((cx, cy), (cx, cy - 300), fill=BAD, width=3, head=14)
    c.text(cx + 10, cy - 310, 'y  left', 15, True, BAD)
    c.text(cx, 840, 'ROS frame base_link: x forward, y left, z up (yaw is counter-clockwise).', 15, False, MUTED, 'mm')

    # stick legend
    x0, y0 = 1160, 150
    c.rect(x0, y0, 400, 620, fill=PANEL, outline=LINE, r=14)
    c.text(x0 + 24, y0 + 22, 'When armed (walk mode)', 18, True, TEXT)

    def stick(sx, sy, label, arrows, notes):
        c.circle(sx, sy, 54, fill=(30, 36, 48), outline=LINE, width=2)
        c.circle(sx, sy, 24, fill=(70, 80, 98))
        for (dx, dy, col) in arrows:
            c.arrow((sx + dx * 30, sy + dy * 30), (sx + dx * 74, sy + dy * 74), fill=col, width=4, head=12)
        c.text(sx, sy + 92, label, 15, True, TEXT, 'mm')
        c.lines(x0 + 200, sy - 40, notes, 14, 6)

    stick(x0 + 100, y0 + 160, 'Left stick', [(0, -1, GOOD), (0, 1, GOOD), (-1, 0, BAD), (1, 0, BAD)],
          [('↑ ↓  walk forward / back', GOOD, True), ('← →  strafe left / right', BAD, True),
           ('', MUTED, False), ('Sentry: tilt the body', MUTED, False)])
    stick(x0 + 100, y0 + 380, 'Right stick', [(-1, 0, ACCENT), (1, 0, ACCENT)],
          [('← →  turn on the spot', ACCENT, True), ('', MUTED, False), ('Auto nav: moves the', MUTED, False),
           ('map cursor', MUTED, False)])
    c.lines(x0 + 24, y0 + 520, [('R1 / L1 held: raise / lower body', TEXT, False),
                                ('Sticks centred: settles into a stand', MUTED, False),
                                ('B: go limp (emergency stop)', BAD, True)], 14, 8)
    c.save('robot_top.png')


# ── install flow ─────────────────────────────────────────────────────────────

def install_flow():
    c = Canvas(1600, 760)
    c.title('Installation at a glance',
            'Three places, done once. Afterwards the robot starts its side on boot and the app is a Steam game.')
    cols = [
        ('1  The robot (Raspberry Pi 5)', GOOD, [
            ('Ubuntu 24.04 + ROS 2 Jazzy', ['flash, boot, install ROS 2 Jazzy']),
            ('git clone the repo', ['github.com/Coldport/SpotMicroROS2']),
            ('./setup.sh', ['drivers, udev rules, Mosquitto, BlueZ,', 'systemd services']),
            ('colcon build', ['then source install/setup.bash']),
            ('Reboot', ['the Deck bridge + Bluetooth Wi-Fi', 'services start on their own']),
        ]),
        ('2  Build the app (Pi or any Linux PC)', ACCENT, [
            ('Install Rust', ['rustup.rs — the build script adds', 'the x86_64 target and fetches Zig']),
            ('cd steamdeck_app', []),
            ('./deploy.sh --push deck@<deck-ip>', ['cross-builds, copies the installer over', 'ssh and runs it on the Deck']),
            ('…or skip building', ['copy dist/spotmicro-deck-setup.sh', '(prebuilt, in the repo) to the Deck']),
        ]),
        ('3  The Steam Deck', WARN, [
            ('Desktop Mode → Konsole', ['set a password (passwd), then', 'sudo systemctl start sshd (for --push)']),
            ('bash spotmicro-deck-setup.sh <robot-ip>', ['installs the app, saves the robot IP', '(deploy.sh --push does this for you)']),
            ('Add a Non-Steam Game → SpotMicro', ['Steam → Games menu']),
            ('Controller layout: Gamepad', ['not Keyboard / Mouse']),
            ('Gaming Mode → launch SpotMicro', ['green Link + Robot pills = connected']),
        ]),
    ]
    cw, gap, top, ch = 480, 40, 120, 500
    for i, (head, col, steps) in enumerate(cols):
        x = 40 + i * (cw + gap)
        c.rect(x, top, cw, ch, fill=PANEL, outline=LINE, r=16)
        c.rect(x + 1, top + 1, cw - 2, 54, fill=PANEL2, r=15)
        c.rect(x + 1, top + 40, cw - 2, 16, fill=PANEL2, r=0)
        c.text(x + 20, top + 28, head, 19, True, col, 'lm')
        yy = top + 80
        for n, (t, sub) in enumerate(steps, 1):
            if n > 1:
                c.line([(x + 36, yy - 22), (x + 36, yy - 2)], fill=LINE, width=2)
            c.circle(x + 36, yy + 14, 15, fill=col)
            c.text(x + 36, yy + 14, str(n), 14, True, BG, 'mm')
            c.text(x + 62, yy + 3, t, 16, True, TEXT)
            for k, line in enumerate(sub):
                c.text(x + 62, yy + 28 + k * 19, line, 13, False, MUTED)
            yy += 50 + 19 * len(sub)
        if i < 2:
            c.arrow((x + cw + 4, top + ch / 2), (x + cw + gap - 4, top + ch / 2), fill=col, width=4, head=14)
    c.rect(40, 650, 1520, 80, fill=(20, 34, 30), outline=GOOD, r=14)
    c.text(64, 690, 'First drive', 18, True, GOOD, 'lm')
    x = 220
    for key, what in [('L1+R1', 'hold 3 s: start the stack'), ('L3+R3', 'hold 3 s: arm'),
                      ('L stick', 'walk'), ('B', 'go limp')]:
        w = c.keycap(x, 676, key, 15)
        c.text(x + w + 10, 690, what, 15, False, TEXT, 'lm')
        x += w + c.text_w(what, 15) + 56
    c.save('install_flow.png')


# ── Wi-Fi over Bluetooth ─────────────────────────────────────────────────────

def wifi_flow():
    c = Canvas(1600, 860)
    c.title('Wi-Fi setup over Bluetooth',
            'Settings (☰) → Connection → Wi-Fi setup over Bluetooth, then A twice. Works connected or not.')

    def box(x, y, w, h, title, sub, col):
        c.rect(x, y, w, h, fill=PANEL, outline=col, r=14, width=2)
        c.text(x + w / 2, y + 26, title, 17, True, TEXT, 'mm')
        for i, s in enumerate(sub):
            c.text(x + w / 2, y + 54 + i * 22, s, 14, False, MUTED, 'mm')

    # lanes
    c.text(60, 130, 'STEAM DECK', 14, True, ACCENT)
    c.text(60, 560, 'ROBOT', 14, True, GOOD)
    c.line([(40, 520), (1560, 520)], fill=TRACK, width=2)
    w, h = 260, 110
    xs = [60, 370, 680, 990]
    box(xs[0], 160, w, h, '☰ → Wi-Fi setup', ['A, then A again', '(two presses on purpose)'], ACCENT)
    box(xs[1], 160, w, h, 'Find "SpotMicro"', ['paired devices first,', 'else a 12 s scan'], ACCENT)
    box(xs[2], 160, w, h, 'Pair + connect', ['no PIN ("just works")', 'RFCOMM channel 22'], ACCENT)
    box(xs[3], 160, w, h, 'Same Wi-Fi?', ['compares the robot\'s', 'network with the Deck\'s'], WARN)
    for a, b in zip(xs, xs[1:]):
        c.arrow((a + w + 4, 215), (b - 4, 215), fill=ACCENT, width=3)
    # robot lane
    box(xs[2], 580, w, h, 'spotmicro-wifi-provision', ['sends hello:', 'its Wi-Fi name + IP'], GOOD)
    c.arrow((xs[2] + 90, 274), (xs[2] + 90, 576), fill=ACCENT, width=3)
    c.arrow((xs[2] + 170, 576), (xs[2] + 170, 274), fill=GOOD, width=3)
    # yes / no
    box(1310, 160, 250, h, 'Use its IP', ['already connected?', 'then just a ✓ toast'], GOOD)
    c.arrow((xs[3] + w + 4, 215), (1306, 215), fill=GOOD, width=3)
    c.text(xs[3] + w + 16, 186, 'yes', 14, True, GOOD)
    c.arrow((1435, 274), (1435, 576), fill=GOOD, width=3)
    c.text(1445, 420, 'not', 14, True, GOOD)
    c.text(1445, 440, 'connected', 14, True, GOOD)
    box(xs[3], 380, w, 120, 'No', ['sends the Deck\'s SSID', '+ password (NetworkManager)'], WARN)
    c.arrow((xs[3] + w / 2, 274), (xs[3] + w / 2, 376), fill=WARN, width=3)
    c.text(xs[3] + w / 2 + 10, 310, 'no', 14, True, WARN)
    box(xs[3], 580, w, 140, 'Robot joins it', ['writes netplan, joins,', 'replies with its new IP;', 'a failed join is undone'], GOOD)
    c.arrow((xs[3] + w / 2, 504), (xs[3] + w / 2, 576), fill=WARN, width=3)
    box(1310, 580, 250, 140, 'App restarts', ['saves the IP in', '~/.config/spotmicro-deck/host', 'and reconnects'], ACCENT)
    c.arrow((xs[3] + w + 4, 650), (1306, 650), fill=GOOD, width=3)
    c.text(60, 780, 'Pairing a new Deck only works in the robot\'s first 10 minutes after boot, or while it has no Wi-Fi.', 15, False, MUTED)
    c.text(60, 806, 'A Deck that has paired once can run the setup at any time.', 15, False, MUTED)
    c.save('wifi_bluetooth.png')


# ── Deck controls ────────────────────────────────────────────────────────────

def deck_controls():
    c = Canvas(1600, 900)
    c.title('Steam Deck controls', 'Robot buttons work once the stack is running (L1+R1) and the robot is armed (L3+R3).')
    x0, y0, w, h = 300, 280, 1000, 380
    c.rect(x0, y0, w, h, fill=(38, 42, 50), r=170)
    c.rect(x0 + 270, y0 + 40, 460, 290, fill=(14, 18, 26), r=12)
    c.text(x0 + 500, y0 + 170, 'camera · depth · lidar · map', 18, False, MUTED, 'mm')
    # shoulder buttons
    for lab, x, y in [('L2', x0 + 70, y0 - 40), ('L1', x0 + 150, y0 - 40), ('R1', x0 + 790, y0 - 40), ('R2', x0 + 870, y0 - 40)]:
        c.rect(x - 34, y, 68, 32, fill=(60, 66, 78), r=8)
        c.text(x, y + 16, lab, 15, True, TEXT, 'mm')
    sticks = {'L3': (x0 + 150, y0 + 100), 'R3': (x0 + 850, y0 + 100)}
    for lab, (sx, sy) in sticks.items():
        c.circle(sx, sy, 46, fill=(28, 32, 40))
        c.circle(sx, sy, 30, fill=(70, 78, 92))
        c.text(sx, sy, lab, 13, True, TEXT, 'mm')
    # d-pad
    dx, dy = x0 + 130, y0 + 250
    c.rect(dx - 14, dy - 44, 28, 88, fill=(70, 78, 92), r=4)
    c.rect(dx - 44, dy - 14, 88, 28, fill=(70, 78, 92), r=4)
    # face buttons
    face = {'Y': (0, -1, (230, 180, 40)), 'X': (-1, 0, (80, 130, 230)), 'B': (1, 0, (220, 80, 80)), 'A': (0, 1, (60, 170, 100))}
    fx, fy = x0 + 870, y0 + 250
    for lab, (ox, oy, col) in face.items():
        c.circle(fx + ox * 34, fy + oy * 34, 19, fill=col)
        c.text(fx + ox * 34, fy + oy * 34, lab, 14, True, TEXT, 'mm')
    # menu buttons
    c.rect(x0 + 230, y0 + 20, 26, 14, fill=(70, 78, 92), r=5)
    c.rect(x0 + 744, y0 + 20, 26, 14, fill=(70, 78, 92), r=5)
    # back grips (drawn as tabs below)
    for x in (x0 + 110, x0 + 890):
        c.rect(x - 40, y0 + h - 6, 80, 22, fill=(60, 66, 78), r=8)

    c.callout(40, 110, 'L2 / R2', ['previous / next view', '(calibration: pick joint; nav: zoom)'], (x0 + 70, y0 - 40), ACCENT)
    c.callout(420, 110, 'L1 + R1 held 3 s', ['start / stop the robot stack', 'held alone: lower (L1) / raise (R1) body'], (x0 + 150, y0 - 40), GOOD)
    c.callout(1560, 110, 'L3 + R3 held 3 s', ['click both sticks: ARM / disarm'], (x0 + 850, y0 + 60), BAD, align='r')
    c.callout(40, 330, 'Left stick', ['walk + strafe', '(sentry: tilt body)'], (sticks['L3'][0] - 46, sticks['L3'][1]), TEXT)
    c.callout(1560, 280, 'Right stick', ['turn', '(sentry: twist; nav: cursor)'], (sticks['R3'][0] + 46, sticks['R3'][1]), TEXT, align='r')
    c.callout(40, 520, 'D-pad', ['menus · trick combos with X', 'Y + ↑ servo cal · Y + ↓ zero IMU'], (dx - 44, dy), MUTED)
    c.callout(1560, 420, 'Face buttons', ['B  e-stop: go limp', 'X  tricks (tap 1–3×)', 'Y Y  sentry · A A  auto nav'], (fx + 53, fy), WARN, align='r')
    c.callout(330, 720, 'View', ['every control on one screen', '(then A: Tune screen)'], (x0 + 243, y0 + 34), ACCENT)
    c.callout(900, 720, 'Start ☰', ['settings menu, including', 'Connection → Wi-Fi setup over Bluetooth'], (x0 + 757, y0 + 34), ACCENT)
    c.callout(1560, 560, 'Back grips L4 R4 L5 R5', ['open the Tune & calibrate', 'screen'], (x0 + 930, y0 + h + 4), PURPLE, align='r')
    c.save('deck_controls.png')


# ── annotated screenshot ─────────────────────────────────────────────────────

def annotated_main():
    if not SHOTS:
        return
    shot = Image.open(os.path.join(SHOTS, 'camera.png')).convert('RGB')
    pad_l, pad_t = 40, 110
    c = Canvas(1280 + 2 * pad_l + 360, 800 + pad_t + 40)
    c.title('The main screen', 'Camera view with the robot connected. R2 / L2 switch to depth, lidar and map.')
    c.img.paste(shot.resize((1280 * SS, 800 * SS)), (pad_l * SS, pad_t * SS))

    def mark(n, x, y):
        c.circle(pad_l + x, pad_t + y, 16, fill=WARN)
        c.text(pad_l + x, pad_t + y, str(n), 15, True, BG, 'mm')

    marks = [
        (1, 330, 52, 'View tabs', 'L2 / R2 move between Camera, Depth, Lidar, Map'),
        (2, 860, 52, 'Camera pill', 'which camera the robot has'),
        (3, 1030, 50, 'Gait pill', 'Gait off → Disarmed → ARMED'),
        (4, 1230, 50, 'Robot + Link', 'green = robot online / MQTT connected'),
        (5, 330, 90, 'Status card', 'IMU horizon, pose, nearest obstacle, rates, CPU'),
        (6, 290, 545, 'Minimap', 'the live SLAM map, robot as an arrow'),
        (7, 620, 735, 'Hint bar', 'the buttons that matter right now'),
        (8, 1250, 625, 'Controller card', 'your sticks + triggers as the robot sees them'),
    ]
    x = pad_l + 1280 + 24
    y = pad_t
    for n, mx, my, t, sub in marks:
        mark(n, mx, my)
        c.circle(x + 16, y + 16, 16, fill=WARN)
        c.text(x + 16, y + 16, str(n), 15, True, BG, 'mm')
        c.text(x + 44, y + 4, t, 17, True, TEXT)
        lines = []
        cur = ''
        for word in sub.split():
            if c.text_w(cur + ' ' + word, 14) > 300:
                lines.append(cur)
                cur = word
            else:
                cur = (cur + ' ' + word).strip()
        lines.append(cur)
        for i, ln in enumerate(lines):
            c.text(x + 44, y + 30 + i * 20, ln, 14, False, MUTED)
        y += 52 + 20 * len(lines)
    c.save('app_main_annotated.png')


if __name__ == '__main__':
    robot_side()
    robot_top()
    install_flow()
    wifi_flow()
    deck_controls()
    annotated_main()
