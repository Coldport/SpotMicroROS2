"""Usage: python3 tools/make_steam_art.py assets/DejaVuSans-Bold.ttf assets/steam

Steam library artwork for the SpotMicro Deck app: cover, wide capsule, hero,
logo and icon. Colours match the app (ui.rs). Everything is drawn at 3x and
downsampled for anti-aliasing."""
import math
import sys

from PIL import Image, ImageDraw, ImageFilter, ImageFont

FONT = sys.argv[1]           # DejaVuSans-Bold.ttf
OUT = sys.argv[2]
SS = 3

BG = (10, 12, 17)
BG2 = (18, 26, 40)
ACCENT = (76, 194, 255)
BODY = (226, 232, 240)
BODY_DARK = (120, 132, 150)
JOINT = (40, 48, 62)
WARN = (255, 181, 71)


def gradient(w, h, top, bottom):
    img = Image.new('RGB', (w, h), top)
    d = ImageDraw.Draw(img)
    for y in range(h):
        t = y / max(h - 1, 1)
        d.line([(0, y), (w, y)], fill=tuple(int(a + (b - a) * t) for a, b in zip(top, bottom)))
    return img


def glow(img, cx, cy, r, color, alpha=150):
    """Radial falloff (smooth, no blur edge artefacts)."""
    n = 256
    m = Image.new('L', (n, n), 0)
    px = m.load()
    for y in range(n):
        for x in range(n):
            d = math.hypot(x - n / 2 + 0.5, y - n / 2 + 0.5) / (n / 2)
            px[x, y] = int(alpha * max(0.0, 1.0 - d) ** 2)
    m = m.resize((int(2 * r), int(2 * r)), Image.BICUBIC)
    layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
    spot = Image.new('RGBA', m.size, color + (0,))
    spot.putalpha(m)
    layer.paste(spot, (int(cx - r), int(cy - r)))
    img.alpha_composite(layer)


def floor_grid(img, horizon, color, alpha=60):
    w, h = img.size
    layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    lw = max(1, w // 600)
    for i in range(-24, 25):          # lines converging on the vanishing point
        x = w / 2 + i * w / 10
        d.line([(w / 2 + i * w / 60, horizon), (x, h)], fill=color + (alpha,), width=lw)
    y, step = horizon, h * 0.012
    while y < h:                      # receding cross lines
        d.line([(0, y), (w, y)], fill=color + (alpha,), width=lw)
        step *= 1.35
        y += step
    fade = Image.new('L', img.size, 0)
    fd = ImageDraw.Draw(fade)
    for yy in range(int(horizon), h):
        fd.line([(0, yy), (w, yy)], fill=int(255 * min(1.0, (yy - horizon) / (h - horizon) * 1.6)))
    layer.putalpha(Image.composite(layer.getchannel('A'), Image.new('L', img.size, 0), fade))
    img.alpha_composite(layer)


def scan_rings(img, cx, cy, r0, color, n=3):
    layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    for i in range(n):
        r = r0 * (1 + 0.55 * i)
        d.arc([cx - r, cy - r * 0.32, cx + r, cy + r * 0.32], 200, 340,
              fill=color + (int(150 / (1 + i)),), width=max(2, int(r0 / 40)))
    img.alpha_composite(layer)


def leg(d, hip, thigh_deg, knee_deg, length, width, color, joint):
    """Upper and lower leg from `hip`; angles are deg from straight down (+ = forward)."""
    def seg(p, ang):
        a = math.radians(ang)
        return (p[0] + math.sin(a) * length, p[1] + math.cos(a) * length)
    knee = seg(hip, thigh_deg)
    foot = seg(knee, knee_deg)
    d.line([hip, knee], fill=color, width=int(width))
    d.line([knee, foot], fill=color, width=int(width * 0.8))
    for p, r in ((hip, width * 0.75), (knee, width * 0.6)):
        d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=joint, outline=color, width=int(width * 0.18))
    r = width * 0.45
    d.ellipse([foot[0] - r, foot[1] - r, foot[0] + r, foot[1] + r], fill=color)
    return foot


def robot(img, cx, cy, s, facing=1):
    """SpotMicro side view; body centre (cx, cy), body length s. Head toward +x if facing=1."""
    layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    bl, bh = s, s * 0.22
    L = s * 0.34
    w = s * 0.07
    f = facing
    front_hip = (cx + f * bl * 0.36, cy + bh * 0.25)
    rear_hip = (cx - f * bl * 0.36, cy + bh * 0.25)
    # far legs (darker, a bit behind)
    leg(d, (front_hip[0] - f * s * 0.03, front_hip[1] - s * 0.01), f * -12, f * 32, L, w * 0.9, BODY_DARK, JOINT)
    leg(d, (rear_hip[0] - f * s * 0.03, rear_hip[1] - s * 0.01), f * -38, f * 8, L, w * 0.9, BODY_DARK, JOINT)
    # body
    d.rounded_rectangle([cx - bl / 2, cy - bh / 2, cx + bl / 2, cy + bh / 2], radius=bh * 0.35, fill=BODY)
    d.rounded_rectangle([cx - bl * 0.42, cy - bh * 0.12, cx + bl * 0.42, cy + bh * 0.12],
                        radius=bh * 0.1, fill=JOINT)
    d.rounded_rectangle([cx - bl * 0.40, cy - bh * 0.05, cx + bl * 0.40, cy + bh * 0.05],
                        radius=bh * 0.05, fill=ACCENT)
    # lidar puck
    pw, ph = s * 0.17, s * 0.075
    d.rounded_rectangle([cx - pw / 2, cy - bh / 2 - ph, cx + pw / 2, cy - bh / 2 + 2], radius=ph * 0.3, fill=JOINT)
    d.rounded_rectangle([cx - pw * 0.42, cy - bh / 2 - ph * 0.62, cx + pw * 0.42, cy - bh / 2 - ph * 0.38],
                        radius=ph * 0.1, fill=ACCENT)
    # head with camera
    hx = cx + f * bl / 2
    hw, hh = s * 0.2, bh * 0.95
    box = [hx - (0 if f > 0 else hw), cy - hh / 2 - bh * 0.1, hx + (hw if f > 0 else 0), cy + hh / 2 - bh * 0.1]
    d.rounded_rectangle(box, radius=hh * 0.3, fill=BODY)
    ex = hx + f * hw * 0.62
    ey = cy - bh * 0.12
    for r, col in ((hh * 0.34, JOINT), (hh * 0.22, ACCENT), (hh * 0.08, (255, 255, 255))):
        d.ellipse([ex - r, ey - r, ex + r, ey + r], fill=col)
    # near legs
    leg(d, front_hip, f * 18, f * -22, L, w, BODY, JOINT)
    leg(d, rear_hip, f * -8, f * 30, L, w, BODY, JOINT)
    # soft shadow under it
    sh = Image.new('RGBA', img.size, (0, 0, 0, 0))
    floor_y = cy + bh * 0.25 + 2 * L * 0.93
    ImageDraw.Draw(sh).ellipse([cx - bl * 0.75, floor_y - s * 0.03, cx + bl * 0.75, floor_y + s * 0.05],
                               fill=(0, 0, 0, 150))
    img.alpha_composite(sh.filter(ImageFilter.GaussianBlur(s * 0.03)))
    img.alpha_composite(layer)
    return floor_y


def text(img, xy, s, size, color, anchor='mm', spacing=0.0):
    d = ImageDraw.Draw(img)
    font = ImageFont.truetype(FONT, int(size))
    if not spacing:
        d.text(xy, s, font=font, fill=color, anchor=anchor)
        return
    # letter-spaced: lay out by hand
    widths = [d.textlength(c, font=font) for c in s]
    total = sum(widths) + spacing * size * (len(s) - 1)
    x = xy[0] - (total / 2 if anchor[0] == 'm' else 0)
    for c, cw in zip(s, widths):
        d.text((x, xy[1]), c, font=font, fill=color, anchor='l' + anchor[1])
        x += cw + spacing * size


def scene(w, h, robot_xy, robot_s, horizon, rings=True):
    W, H = w * SS, h * SS
    img = gradient(W, H, BG2, BG).convert('RGBA')
    glow(img, robot_xy[0] * SS, robot_xy[1] * SS, robot_s * SS * 0.8, ACCENT, 70)
    floor_grid(img, horizon * SS, ACCENT, 55)
    if rings:
        scan_rings(img, robot_xy[0] * SS, (robot_xy[1] + robot_s * 0.66) * SS, robot_s * SS * 0.9, ACCENT)
    robot(img, robot_xy[0] * SS, robot_xy[1] * SS, robot_s * SS)
    return img


def title(img, cx, y, size, sub=True):
    text(img, (cx * SS, y * SS), 'SPOTMICRO', size * SS, BODY, spacing=0.12)
    if sub:
        text(img, (cx * SS, (y + size * 0.95) * SS), 'DECK CONTROL', size * 0.36 * SS, ACCENT, spacing=0.45)


def done(img, w, h, name):
    img.resize((w, h), Image.LANCZOS).save(f'{OUT}/{name}')


# Cover (portrait capsule) 600x900
img = scene(600, 900, (300, 470), 330, 560)
title(img, 300, 140, 64)
done(img, 600, 900, 'cover.png')

# Wide capsule 920x430
img = scene(920, 430, (690, 190), 230, 290)
text(img, (52 * SS, 185 * SS), 'SPOTMICRO', 58 * SS, BODY, anchor='lm')
text(img, (56 * SS, 238 * SS), 'DECK CONTROL', 21 * SS, ACCENT, anchor='lm')
done(img, 920, 430, 'wide.png')

# Hero 1920x620 (no text: Steam puts the logo over it, on the left)
img = scene(1920, 620, (1400, 230), 400, 380)
done(img, 1920, 620, 'hero.png')

# Logo (transparent) 1280x360
logo = Image.new('RGBA', (1280 * SS, 360 * SS), (0, 0, 0, 0))
text(logo, (640 * SS, 150 * SS), 'SPOTMICRO', 170 * SS, BODY, spacing=0.10)
text(logo, (640 * SS, 290 * SS), 'DECK CONTROL', 58 * SS, ACCENT, spacing=0.45)
shadow = logo.getchannel('A').filter(ImageFilter.GaussianBlur(6 * SS))
out = Image.new('RGBA', logo.size, (0, 0, 0, 0))
out.paste((0, 0, 0, 170), mask=shadow)
out.alpha_composite(logo)
done(out, 1280, 360, 'logo.png')

# Icon 256x256: robot on an accent-ringed dark disc
S = 256 * SS
ic = Image.new('RGBA', (S, S), (0, 0, 0, 0))
mask = Image.new('L', (S, S), 0)
ImageDraw.Draw(mask).rounded_rectangle([0, 0, S - 1, S - 1], radius=S * 0.22, fill=255)
bg = gradient(S, S, BG2, BG).convert('RGBA')
glow(bg, S / 2, S * 0.45, S * 0.42, ACCENT, 90)
robot(bg, S * 0.47, S * 0.36, S * 0.62)
ic.paste(bg, mask=mask)
ImageDraw.Draw(ic).rounded_rectangle([SS, SS, S - 1 - SS, S - 1 - SS], radius=S * 0.22, outline=ACCENT, width=4 * SS)
done(ic, 256, 256, 'icon.png')
print('ok')
