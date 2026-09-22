//! Rendering: the main view (camera / depth / lidar / map), the HUD panels and
//! the lower-left minimap. Everything is drawn with SDL2's 2D renderer at a
//! fixed 1280x800 logical size (the Steam Deck's native resolution); SDL scales
//! it to whatever display the app is on.

use std::sync::Arc;

use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::{Point, Rect};
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::{Window, WindowContext};

use crate::input::InputState;
use crate::protocol::{MapData, ScanData, BUTTON_NAMES};
use crate::shared::{Frame, RobotState, View};

pub const LOGICAL_W: u32 = 1280;
pub const LOGICAL_H: u32 = 800;

const BG: Color = Color::RGB(9, 11, 16);
const PANEL_BG: Color = Color::RGBA(6, 9, 14, 200);
const ACCENT: Color = Color::RGB(0, 214, 255);
const GOOD: Color = Color::RGB(80, 232, 128);
const BAD: Color = Color::RGB(255, 86, 86);
const WARN: Color = Color::RGB(255, 196, 64);
const TEXT: Color = Color::RGB(226, 234, 244);
const DIM: Color = Color::RGB(128, 140, 158);

const MINIMAP: u32 = 264;
const MARGIN: i32 = 16;

/// Everything the UI needs for one frame, snapshotted from the shared state.
pub struct Snapshot {
    pub view: View,
    pub host: String,
    pub broker_connected: bool,
    pub robot_online: Option<bool>,
    pub last_error: String,
    pub state: Option<RobotState>,
    pub state_age: f32,
    pub map: Option<Arc<MapData>>,
    pub map_version: u64,
    pub scan: Option<Arc<ScanData>>,
    pub scan_age: f32,
    pub frame: Option<Arc<Frame>>,
    pub frame_age: f32,
    pub video_fps: f32,
    pub decode_ms: f32,
    pub input: InputState,
    pub gamepad_connected: bool,
}

struct Font {
    atlas: Texture,
}

impl Font {
    fn new(tc: &TextureCreator<WindowContext>) -> Result<Font, String> {
        // One row of 128 glyphs, 8x8 each, white on transparent.
        let mut rgba = vec![0u8; 128 * 8 * 8 * 4];
        for (ch, glyph) in font8x8::legacy::BASIC_LEGACY.iter().enumerate() {
            for (row, bits) in glyph.iter().enumerate() {
                for col in 0..8 {
                    if bits & (1 << col) != 0 {
                        let o = (row * 128 * 8 + ch * 8 + col) * 4;
                        rgba[o..o + 4].copy_from_slice(&[255, 255, 255, 255]);
                    }
                }
            }
        }
        let mut atlas = tc
            .create_texture_static(PixelFormatEnum::RGBA32, 128 * 8, 8)
            .map_err(|e| e.to_string())?;
        atlas.update(None, &rgba, 128 * 8 * 4).map_err(|e| e.to_string())?;
        atlas.set_blend_mode(BlendMode::Blend);
        Ok(Font { atlas })
    }

    fn draw(&mut self, canvas: &mut Canvas<Window>, s: &str, x: i32, y: i32, scale: u32, color: Color) {
        self.atlas.set_color_mod(color.r, color.g, color.b);
        self.atlas.set_alpha_mod(color.a);
        let mut cx = x;
        for ch in s.chars() {
            let idx = if (ch as u32) < 128 { ch as u32 } else { b'?' as u32 };
            let src = Rect::new(idx as i32 * 8, 0, 8, 8);
            let dst = Rect::new(cx, y, 8 * scale, 8 * scale);
            let _ = canvas.copy(&self.atlas, src, dst);
            cx += (8 * scale) as i32;
        }
    }
}

fn text_width(s: &str, scale: u32) -> i32 {
    (s.chars().count() as u32 * 8 * scale) as i32
}

struct VideoTex {
    tex: Texture,
    w: u32,
    h: u32,
    seq: u64,
}

struct MapTex {
    tex: Texture,
    version: u64,
}

pub struct Ui {
    font: Font,
    video: Option<VideoTex>,
    map: Option<MapTex>,
}

/// Google's polynomial approximation of the Turbo colormap (the same map the
/// robot uses for the depth image), t in 0..=1.
fn turbo(t: f32) -> Color {
    let t = t.clamp(0.0, 1.0);
    let r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
    let g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
    let b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
    let c = |v: f32| (v.clamp(0.0, 1.0) * 255.0) as u8;
    Color::RGB(c(r), c(g), c(b))
}

fn with_alpha(c: Color, a: u8) -> Color {
    Color::RGBA(c.r, c.g, c.b, a)
}

impl Ui {
    pub fn new(tc: &TextureCreator<WindowContext>) -> Result<Ui, String> {
        Ok(Ui { font: Font::new(tc)?, video: None, map: None })
    }

    pub fn render(
        &mut self,
        canvas: &mut Canvas<Window>,
        tc: &TextureCreator<WindowContext>,
        snap: &Snapshot,
    ) {
        canvas.set_blend_mode(BlendMode::Blend);
        canvas.set_draw_color(BG);
        canvas.clear();

        match snap.view {
            View::Color | View::Depth => self.draw_video(canvas, tc, snap),
            View::Lidar => self.draw_lidar(canvas, snap),
            View::Map => {
                let area = Rect::new(MARGIN, 60, LOGICAL_W - 2 * MARGIN as u32, LOGICAL_H - 76);
                self.draw_map(canvas, tc, snap, area, 4);
            }
        }

        self.draw_top_bar(canvas, snap);
        self.draw_status_panel(canvas, snap);
        if snap.view != View::Map {
            let r = Rect::new(MARGIN, (LOGICAL_H - MINIMAP) as i32 - MARGIN, MINIMAP, MINIMAP);
            self.draw_minimap(canvas, tc, snap, r);
        }
        self.draw_controls(canvas, snap);
        self.draw_overlay(canvas, snap);
    }

    // ── primitives ───────────────────────────────────────────────────────────

    fn panel(&mut self, canvas: &mut Canvas<Window>, r: Rect, title: &str) {
        canvas.set_draw_color(PANEL_BG);
        let _ = canvas.fill_rect(r);
        canvas.set_draw_color(with_alpha(ACCENT, 170));
        let _ = canvas.draw_rect(r);
        // corner brackets, game-HUD style
        let l = 12;
        for (cx, cy, dx, dy) in [
            (r.x, r.y, 1, 1),
            (r.x + r.w - 1, r.y, -1, 1),
            (r.x, r.y + r.h - 1, 1, -1),
            (r.x + r.w - 1, r.y + r.h - 1, -1, -1),
        ] {
            canvas.set_draw_color(ACCENT);
            let _ = canvas.draw_line(Point::new(cx, cy), Point::new(cx + dx * l, cy));
            let _ = canvas.draw_line(Point::new(cx, cy), Point::new(cx, cy + dy * l));
        }
        if !title.is_empty() {
            let w = text_width(title, 2) + 12;
            canvas.set_draw_color(with_alpha(BG, 240));
            let _ = canvas.fill_rect(Rect::new(r.x + 14, r.y - 1, w as u32, 4));
            self.font.draw(canvas, title, r.x + 20, r.y - 7, 2, ACCENT);
        }
    }

    fn dot(&mut self, canvas: &mut Canvas<Window>, x: i32, y: i32, color: Color) {
        canvas.set_draw_color(color);
        let _ = canvas.fill_rect(Rect::new(x, y, 12, 12));
    }

    // ── main views ───────────────────────────────────────────────────────────

    fn draw_video(&mut self, canvas: &mut Canvas<Window>, tc: &TextureCreator<WindowContext>, snap: &Snapshot) {
        let frame = match &snap.frame {
            Some(f) if f.view == snap.view => f.clone(),
            _ => return,
        };
        let stale = match &self.video {
            Some(v) => v.w != frame.width || v.h != frame.height,
            None => true,
        };
        if stale {
            // The scale-quality hint is read when a texture is created.
            sdl2::hint::set("SDL_RENDER_SCALE_QUALITY", "linear");
            let created = tc.create_texture_streaming(PixelFormatEnum::RGB24, frame.width, frame.height);
            sdl2::hint::set("SDL_RENDER_SCALE_QUALITY", "nearest");
            match created {
                Ok(tex) => self.video = Some(VideoTex { tex, w: frame.width, h: frame.height, seq: 0 }),
                Err(_) => return,
            }
        }
        let v = self.video.as_mut().unwrap();
        if v.seq != frame.seq {
            let _ = v.tex.update(None, &frame.rgb, (frame.width * 3) as usize);
            v.seq = frame.seq;
        }
        let scale = (LOGICAL_W as f32 / frame.width as f32).min(LOGICAL_H as f32 / frame.height as f32);
        let (dw, dh) = ((frame.width as f32 * scale) as u32, (frame.height as f32 * scale) as u32);
        let dst = Rect::new(
            ((LOGICAL_W - dw) / 2) as i32,
            ((LOGICAL_H - dh) / 2) as i32,
            dw,
            dh,
        );
        let _ = canvas.copy(&v.tex, None, dst);

        if snap.view == View::Depth {
            self.draw_depth_legend(canvas, snap);
        }
    }

    fn draw_depth_legend(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let (near, far) = snap
            .state
            .as_ref()
            .and_then(|s| s.depth_range_m)
            .map(|r| (r[0], r[1]))
            .unwrap_or((0.1, 2.0));
        let (x, y, w, h) = (LOGICAL_W as i32 - 70, 110, 22, 340);
        self.panel(canvas, Rect::new(x - 12, y - 30, 60 + 24, h as u32 + 78), "");
        for i in 0..h {
            // top = far (cool), bottom = near (warm), matching the robot's colouring
            let t = i as f32 / (h - 1) as f32;
            canvas.set_draw_color(turbo(t));
            let _ = canvas.draw_line(Point::new(x, y + i), Point::new(x + w, y + i));
        }
        self.font.draw(canvas, &format!("{far:.1}m"), x - 8, y - 22, 1, TEXT);
        self.font.draw(canvas, &format!("{near:.1}m"), x - 8, y + h + 8, 1, TEXT);
        self.font.draw(canvas, "no data", x - 8, y + h + 22, 1, DIM);
    }

    fn draw_lidar(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let (cx, cy) = (LOGICAL_W as i32 / 2, LOGICAL_H as i32 / 2 + 20);
        let ppm = 64.0f32; // pixels per metre
        // range rings
        for m in 1..=5 {
            let radius = (m as f32 * ppm) as i32;
            canvas.set_draw_color(with_alpha(ACCENT, if m % 2 == 0 { 70 } else { 36 }));
            let mut prev: Option<Point> = None;
            for step in 0..=96 {
                let a = step as f32 / 96.0 * std::f32::consts::TAU;
                let p = Point::new(cx + (a.cos() * radius as f32) as i32, cy + (a.sin() * radius as f32) as i32);
                if let Some(q) = prev {
                    let _ = canvas.draw_line(q, p);
                }
                prev = Some(p);
            }
            self.font.draw(canvas, &format!("{m}m"), cx + 4, cy - radius - 10, 1, DIM);
        }
        canvas.set_draw_color(with_alpha(ACCENT, 36));
        let _ = canvas.draw_line(Point::new(cx, 60), Point::new(cx, LOGICAL_H as i32 - 10));
        let _ = canvas.draw_line(Point::new(40, cy), Point::new(LOGICAL_W as i32 - 40, cy));

        if let Some(scan) = &snap.scan {
            // Robot frame: +x forward (drawn up), +y left (drawn left).
            let mut buckets: [Vec<Point>; 4] = Default::default();
            for (i, &mm) in scan.ranges_mm.iter().enumerate() {
                if mm == 0xFFFF {
                    continue;
                }
                let r = mm as f32 / 1000.0;
                let a = scan.angle_min + i as f32 * scan.angle_increment;
                let (x, y) = (r * a.cos(), r * a.sin());
                let (sx, sy) = (cx as f32 - y * ppm, cy as f32 - x * ppm);
                if sx < 0.0 || sy < 0.0 || sx > LOGICAL_W as f32 || sy > LOGICAL_H as f32 {
                    continue;
                }
                let b = ((r / 4.0) * 4.0).min(3.0) as usize;
                buckets[b].push(Point::new(sx as i32, sy as i32));
            }
            let colors = [BAD, WARN, GOOD, ACCENT];
            for (b, pts) in buckets.iter().enumerate() {
                canvas.set_draw_color(colors[b]);
                for p in pts {
                    let _ = canvas.fill_rect(Rect::new(p.x() - 1, p.y() - 1, 3, 3));
                }
            }
        }
        // robot marker
        canvas.set_draw_color(WARN);
        let _ = canvas.draw_line(Point::new(cx, cy - 14), Point::new(cx - 9, cy + 10));
        let _ = canvas.draw_line(Point::new(cx, cy - 14), Point::new(cx + 9, cy + 10));
        let _ = canvas.draw_line(Point::new(cx - 9, cy + 10), Point::new(cx + 9, cy + 10));
        self.font.draw(canvas, "FRONT", cx - 20, cy - 5 * ppm as i32 - 32, 1, DIM);
    }

    // ── map (used for both the minimap and the full-screen map view) ─────────

    fn draw_map(
        &mut self,
        canvas: &mut Canvas<Window>,
        tc: &TextureCreator<WindowContext>,
        snap: &Snapshot,
        area: Rect,
        scan_dot: u32,
    ) {
        let map = match &snap.map {
            Some(m) if m.width > 0 && m.height > 0 => m.clone(),
            _ => {
                let msg = "NO MAP YET";
                let (tx, ty) = (area.x + (area.w as i32 - text_width(msg, 2)) / 2, area.y + area.h as i32 / 2 - 8);
                self.font.draw(canvas, msg, tx, ty, 2, DIM);
                return;
            }
        };
        let rebuild = self.map.as_ref().map(|m| m.version != snap.map_version).unwrap_or(true)
            || self.map.as_ref().map(|m| m.tex.query().width != map.width || m.tex.query().height != map.height).unwrap_or(true);
        if rebuild {
            if let Ok(mut tex) = tc.create_texture_streaming(PixelFormatEnum::RGBA32, map.width, map.height) {
                tex.set_blend_mode(BlendMode::Blend);
                let (w, h) = (map.width as usize, map.height as usize);
                let mut px = vec![0u8; w * h * 4];
                for row in 0..h {
                    for col in 0..w {
                        // grid row 0 is the lowest y; texture row 0 is the top, so flip.
                        let v = map.cells[row * w + col];
                        let rgba: [u8; 4] = if v < 0 {
                            [0, 0, 0, 0] // unknown: leave the panel showing through
                        } else if v >= 50 {
                            [96, 255, 168, 255] // wall
                        } else {
                            [44, 62, 84, 235] // known free space
                        };
                        let o = ((h - 1 - row) * w + col) * 4;
                        px[o..o + 4].copy_from_slice(&rgba);
                    }
                }
                let _ = tex.update(None, &px, w * 4);
                self.map = Some(MapTex { tex, version: snap.map_version });
            }
        }
        let Some(mt) = &self.map else { return };

        let scale = ((area.w as f32 - 8.0) / map.width as f32).min((area.h as f32 - 8.0) / map.height as f32);
        let (dw, dh) = ((map.width as f32 * scale) as u32, (map.height as f32 * scale) as u32);
        let dst = Rect::new(
            area.x + (area.w - dw as i32) / 2,
            area.y + (area.h - dh as i32) / 2,
            dw.max(1),
            dh.max(1),
        );
        let _ = canvas.copy(&mt.tex, None, dst);

        // world (map frame, metres) -> screen
        let to_screen = |x: f32, y: f32| -> (f32, f32) {
            (
                dst.x as f32 + (x - map.origin_x) / map.resolution * scale,
                dst.y as f32 + (map.height as f32 - (y - map.origin_y) / map.resolution) * scale,
            )
        };

        let Some(pose) = snap.state.as_ref().and_then(|s| s.pose.clone()) else { return };
        let yaw = pose.yaw_deg.to_radians();

        // live lidar overlay on the map, like a radar sweep
        if let Some(scan) = &snap.scan {
            if snap.scan_age < 1.0 {
                canvas.set_draw_color(with_alpha(ACCENT, 200));
                let (s, c) = yaw.sin_cos();
                for (i, &mm) in scan.ranges_mm.iter().enumerate().step_by(2) {
                    if mm == 0xFFFF {
                        continue;
                    }
                    let r = mm as f32 / 1000.0;
                    let a = scan.angle_min + i as f32 * scan.angle_increment;
                    let (lx, ly) = (r * a.cos(), r * a.sin());
                    let (wx, wy) = (pose.x + c * lx - s * ly, pose.y + s * lx + c * ly);
                    let (sx, sy) = to_screen(wx, wy);
                    if sx >= area.x as f32 && sy >= area.y as f32 && sx < area.right() as f32 && sy < area.bottom() as f32 {
                        let _ = canvas.fill_rect(Rect::new(sx as i32, sy as i32, scan_dot / 2 + 1, scan_dot / 2 + 1));
                    }
                }
            }
        }

        // robot arrow
        let (px, py) = to_screen(pose.x, pose.y);
        let (dx, dy) = (yaw.cos(), -yaw.sin()); // screen y points down
        let (nx, ny) = (-dy, dx);
        let size = (scan_dot as f32 * 2.2).max(7.0);
        let tip = Point::new((px + dx * size * 1.5) as i32, (py + dy * size * 1.5) as i32);
        let left = Point::new((px - dx * size + nx * size * 0.8) as i32, (py - dy * size + ny * size * 0.8) as i32);
        let right = Point::new((px - dx * size - nx * size * 0.8) as i32, (py - dy * size - ny * size * 0.8) as i32);
        canvas.set_draw_color(WARN);
        let _ = canvas.draw_line(tip, left);
        let _ = canvas.draw_line(tip, right);
        let _ = canvas.draw_line(left, right);
        let _ = canvas.fill_rect(Rect::new(px as i32 - 2, py as i32 - 2, 5, 5));
    }

    fn draw_minimap(
        &mut self,
        canvas: &mut Canvas<Window>,
        tc: &TextureCreator<WindowContext>,
        snap: &Snapshot,
        r: Rect,
    ) {
        self.panel(canvas, r, "MAP");
        let inner = Rect::new(r.x + 4, r.y + 12, (r.w - 8) as u32, (r.h - 16) as u32);
        canvas.set_clip_rect(inner);
        self.draw_map(canvas, tc, snap, inner, 2);
        canvas.set_clip_rect(None);
        if let Some(m) = &snap.map {
            let label = format!("{:.1}x{:.1}m", m.width as f32 * m.resolution, m.height as f32 * m.resolution);
            self.font.draw(canvas, &label, r.x + 8, r.bottom() - 14, 1, DIM);
        }
    }

    // ── HUD ──────────────────────────────────────────────────────────────────

    fn draw_top_bar(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        canvas.set_draw_color(Color::RGBA(4, 6, 10, 215));
        let _ = canvas.fill_rect(Rect::new(0, 0, LOGICAL_W, 44));
        canvas.set_draw_color(with_alpha(ACCENT, 200));
        let _ = canvas.draw_line(Point::new(0, 44), Point::new(LOGICAL_W as i32, 44));

        self.font.draw(canvas, "SPOTMICRO", 16, 14, 2, ACCENT);
        let label = format!("[L2] {}", snap.view.label());
        self.font.draw(canvas, &label, 210, 14, 2, TEXT);
        // position dots for each view
        for (i, v) in [View::Color, View::Depth, View::Lidar, View::Map].iter().enumerate() {
            canvas.set_draw_color(if *v == snap.view { ACCENT } else { with_alpha(DIM, 120) });
            let _ = canvas.fill_rect(Rect::new(440 + i as i32 * 22, 18, 12, 12));
        }

        let mut x = LOGICAL_W as i32 - 16;
        let robot = match snap.robot_online {
            Some(true) => (GOOD, "ROBOT"),
            _ => (BAD, "ROBOT"),
        };
        for (color, label) in [robot, if snap.broker_connected { (GOOD, "LINK") } else { (BAD, "LINK") }] {
            x -= text_width(label, 2);
            self.font.draw(canvas, label, x, 14, 2, TEXT);
            x -= 22;
            self.dot(canvas, x, 16, color);
            x -= 24;
        }
        if snap.video_fps > 0.0 && snap.view.is_video() && snap.frame_age < 2.0 {
            let s = format!("{:.0} FPS", snap.video_fps);
            x -= text_width(&s, 2);
            self.font.draw(canvas, &s, x, 14, 2, DIM);
        }
    }

    fn draw_status_panel(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let r = Rect::new(MARGIN, 68, 330, 262);
        self.panel(canvas, r, "STATUS");
        let mut y = r.y + 18;
        let mut line = |ui: &mut Ui, canvas: &mut Canvas<Window>, label: &str, value: String, color: Color| {
            ui.font.draw(canvas, label, r.x + 14, y, 2, DIM);
            ui.font.draw(canvas, &value, r.x + 14 + 8 * 2 * 6, y, 2, color);
            y += 22;
        };

        let fresh = snap.state_age < 2.0 && snap.state.is_some();
        let st = snap.state.clone().unwrap_or_default();
        let none = "--".to_string();
        let val = |ok: bool, s: String| if ok { (s, TEXT) } else { ("--".to_string(), DIM) };

        let (v, c) = val(fresh && st.imu.is_some(), st.imu.as_ref().map(|i| format!("{:+.1}", i.roll)).unwrap_or_default());
        line(self, canvas, "ROLL", v, c);
        let (v, c) = val(fresh && st.imu.is_some(), st.imu.as_ref().map(|i| format!("{:+.1}", i.pitch)).unwrap_or_default());
        line(self, canvas, "PITCH", v, c);
        let (v, c) = val(fresh && st.imu.is_some(), st.imu.as_ref().map(|i| format!("{:+.1}", i.yaw)).unwrap_or_default());
        line(self, canvas, "YAW", v, c);
        let (v, c) = val(fresh && st.pose.is_some(), st.pose.as_ref().map(|p| format!("{:.2},{:.2}m", p.x, p.y)).unwrap_or_default());
        line(self, canvas, "POS", v, c);
        let (v, c) = match (&st.nearest, fresh) {
            (Some(n), true) => (
                format!("{:.2}m@{:+.0}", n.range_m, n.angle_deg),
                if n.range_m < 0.4 { BAD } else if n.range_m < 0.8 { WARN } else { TEXT },
            ),
            _ => (none.clone(), DIM),
        };
        line(self, canvas, "NEAR", v, c);
        let rate = |k: &str| st.rates.get(k).copied().unwrap_or(0.0);
        line(self, canvas, "SCAN", if fresh { format!("{:.0}Hz", rate("scan")) } else { none.clone() }, if fresh { TEXT } else { DIM });
        line(self, canvas, "IMU", if fresh { format!("{:.0}Hz", rate("imu")) } else { none.clone() }, if fresh { TEXT } else { DIM });
        let cpu = st.sys.cpu_temp_c.map(|t| format!("{t:.0}C")).unwrap_or_else(|| "?".into());
        let load = st.sys.load1.map(|l| format!("{l:.1}")).unwrap_or_else(|| "?".into());
        let hot = st.sys.cpu_temp_c.map(|t| t > 75.0).unwrap_or(false);
        line(self, canvas, "CPU", if fresh { format!("{cpu}/{load}") } else { none.clone() }, if hot { BAD } else if fresh { TEXT } else { DIM });
        let map_s = match (&st.map, fresh) {
            (Some(m), true) => format!("{}x{} {:.0}s", m.w, m.h, m.age_s),
            _ => none.clone(),
        };
        line(self, canvas, "MAP", map_s, if fresh { TEXT } else { DIM });
        let up = if fresh { format!("{}:{:02}", st.uptime_s / 60, st.uptime_s % 60) } else { none };
        line(self, canvas, "UP", up, if fresh { TEXT } else { DIM });
    }

    fn draw_controls(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let r = Rect::new(LOGICAL_W as i32 - 296 - MARGIN, LOGICAL_H as i32 - 138 - MARGIN, 296, 138);
        self.panel(canvas, r, "INPUT");
        // two stick indicators
        for (i, (label, ax, ay)) in [("L", 0usize, 1usize), ("R", 2, 3)].iter().enumerate() {
            let bx = r.x + 16 + i as i32 * 84;
            let by = r.y + 20;
            canvas.set_draw_color(with_alpha(DIM, 160));
            let _ = canvas.draw_rect(Rect::new(bx, by, 72, 72));
            canvas.set_draw_color(with_alpha(DIM, 60));
            let _ = canvas.draw_line(Point::new(bx + 36, by), Point::new(bx + 36, by + 72));
            let _ = canvas.draw_line(Point::new(bx, by + 36), Point::new(bx + 72, by + 36));
            // ROS convention (left/up positive) -> screen
            let px = bx + 36 - (snap.input.axes[*ax] * 32.0) as i32;
            let py = by + 36 - (snap.input.axes[*ay] * 32.0) as i32;
            canvas.set_draw_color(ACCENT);
            let _ = canvas.fill_rect(Rect::new(px - 4, py - 4, 9, 9));
            self.font.draw(canvas, label, bx + 2, by + 2, 1, DIM);
        }
        // triggers
        for (i, (label, ai)) in [("L2", 4usize), ("R2", 5)].iter().enumerate() {
            let bx = r.x + 190 + i as i32 * 46;
            let by = r.y + 20;
            canvas.set_draw_color(with_alpha(DIM, 160));
            let _ = canvas.draw_rect(Rect::new(bx, by, 36, 72));
            let fill = (snap.input.axes[*ai] * 70.0) as u32;
            canvas.set_draw_color(WARN);
            let _ = canvas.fill_rect(Rect::new(bx + 1, by + 71 - fill as i32, 34, fill));
            self.font.draw(canvas, label, bx + 4, by + 76, 1, DIM);
        }
        let pressed: Vec<&str> = BUTTON_NAMES
            .iter()
            .zip(snap.input.buttons.iter())
            .filter(|(n, &d)| d && **n != "l2" && **n != "r2")
            .map(|(n, _)| *n)
            .collect();
        let text = if !snap.gamepad_connected {
            "NO CONTROLLER".to_string()
        } else if pressed.is_empty() {
            "L2: NEXT VIEW".to_string()
        } else {
            pressed.join(" ").to_uppercase()
        };
        let color = if snap.gamepad_connected { TEXT } else { WARN };
        let shown: String = text.chars().take(34).collect();
        self.font.draw(canvas, &shown, r.x + 14, r.bottom() - 18, 1, color);
    }

    fn draw_overlay(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let message: Option<(String, String, Color)> = if !snap.broker_connected {
            let detail = if snap.last_error.is_empty() {
                "Is the robot on and the MQTT broker running?".to_string()
            } else {
                snap.last_error.chars().take(60).collect()
            };
            Some((format!("CONNECTING TO {}", snap.host), detail, WARN))
        } else if snap.robot_online != Some(true) {
            Some(("ROBOT OFFLINE".into(), "Waiting for the robot's deck_bridge node...".into(), BAD))
        } else if snap.view.is_video() {
            let have = snap.frame.as_ref().map(|f| f.view == snap.view).unwrap_or(false);
            if !have || snap.frame_age > 2.0 {
                Some((format!("WAITING FOR {} VIDEO", snap.view.label()), "No frames yet - is the camera running?".into(), WARN))
            } else {
                None
            }
        } else {
            None
        };
        if let Some((title, detail, color)) = message {
            let w = (text_width(&title, 3).max(text_width(&detail, 2)) + 60) as u32;
            let r = Rect::new((LOGICAL_W - w) as i32 / 2, LOGICAL_H as i32 / 2 - 60, w, 110);
            self.panel(canvas, r, "");
            self.font.draw(canvas, &title, r.x + (r.w as i32 - text_width(&title, 3)) / 2, r.y + 24, 3, color);
            self.font.draw(canvas, &detail, r.x + (r.w as i32 - text_width(&detail, 2)) / 2, r.y + 68, 2, DIM);
        }
    }
}
