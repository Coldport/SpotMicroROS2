//! Rendering: the main view (camera / depth / lidar / map), the HUD (top bar,
//! status card, minimap, controller card), the settings drawer, the controls
//! reference and toasts. In auto nav mode the whole screen is the nav map
//! instead (draw_nav): the saved map in black on paper, everything live in colour.
//! Everything is drawn with SDL2's 2D renderer at a fixed 1280x800 logical
//! size (the Steam Deck's native resolution); SDL scales it to whatever display
//! the app is on. Text comes from text.rs; rounded shapes are built from one
//! anti-aliased disc texture.

use std::sync::Arc;

use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::{Point, Rect};
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::{Window, WindowContext};

use crate::bt::Provision;
use crate::input::InputState;
use crate::menu::{format_value, fraction, Menu};
use crate::navview::{NavCam, Place};
use crate::protocol::{MapData, ScanData, BUTTON_NAMES};
use crate::shared::{CameraInfo, Frame, ModeInfo, RobotState, SettingItem, TrackStatus, View};
use crate::track::{self, TrackPick};
use crate::text::{Fonts, Weight};

pub const LOGICAL_W: u32 = 1280;
pub const LOGICAL_H: u32 = 800;

const BG: Color = Color::RGB(10, 12, 17);
const PANEL: Color = Color::RGBA(15, 19, 27, 226);
const HAIRLINE: Color = Color::RGBA(255, 255, 255, 20);
const SURFACE: Color = Color::RGBA(255, 255, 255, 16);
const TRACK: Color = Color::RGB(40, 48, 62);
const ACCENT: Color = Color::RGB(76, 194, 255);
const ON_ACCENT: Color = Color::RGB(8, 18, 28);
const GOOD: Color = Color::RGB(61, 220, 132);
const WARN: Color = Color::RGB(255, 181, 71);
const BAD: Color = Color::RGB(255, 92, 92);
const TEXT: Color = Color::RGB(232, 238, 246);
const MUTED: Color = Color::RGB(140, 152, 170);
const FAINT: Color = Color::RGB(84, 94, 110);

const TOP_H: i32 = 56;
const M: i32 = 16;
const MINIMAP: u32 = 264;
const RADIUS: i32 = 14;
const DRAWER_W: u32 = 548;

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
    pub static_map: Option<Arc<MapData>>,
    pub static_map_version: u64,
    /// Some while in auto nav mode: draw the nav view with this camera.
    pub nav_cam: Option<NavCam>,
    /// A single A is waiting to become a goal.
    pub nav_a_pending: bool,
    /// The robot's mic is coming in (and playing).
    pub audio_live: bool,
    /// Sentry object tracking status from the robot (fresh only).
    pub track: Option<TrackStatus>,
    /// Some while a tracking target can be picked (sentry, camera view).
    pub track_pick: Option<TrackPick>,
    pub scan: Option<Arc<ScanData>>,
    pub scan_age: f32,
    pub frame: Option<Arc<Frame>>,
    pub frame_age: f32,
    pub video_fps: f32,
    pub decode_ms: f32,
    pub input: InputState,
    pub gamepad_connected: bool,
    pub settings: Vec<SettingItem>,
    pub settings_age: f32,
    /// Wi-Fi setup over Bluetooth (bt.rs).
    pub provision: Provision,
}

impl Snapshot {
    fn fresh_state(&self) -> Option<&RobotState> {
        self.state.as_ref().filter(|_| self.state_age < 2.0)
    }
    pub fn camera(&self) -> Option<&CameraInfo> {
        self.fresh_state().and_then(|s| s.camera.as_ref())
    }
    /// gait_node's sentry / calibrate mode, while armed (None = walking or disarmed).
    pub fn mode(&self) -> Option<&ModeInfo> {
        self.fresh_state().and_then(|s| s.mode.as_ref()).filter(|m| m.mode != "walk")
    }
    /// False only when the robot says its camera has no depth stream.
    pub fn depth_available(&self) -> bool {
        self.camera().map(|c| c.depth || c.kind.is_none()).unwrap_or(true)
    }
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

// Nav view palette: the saved map is ink on paper; live data is in colour.
const PAPER: Color = Color::RGB(226, 223, 214);
const PAPER_FREE: [u8; 4] = [250, 249, 245, 255];
const INK: [u8; 4] = [22, 22, 26, 255];
const LIVE_WALL: [u8; 4] = [0, 132, 255, 215];
const NAV_PATH: Color = Color::RGB(21, 101, 192);
const NAV_GOAL: Color = Color::RGB(22, 163, 74);
const NAV_CURSOR: Color = Color::RGB(219, 39, 119);
const NAV_ROBOT: Color = Color::RGB(234, 88, 12);
const NAV_PLACE: Color = Color::RGB(124, 58, 237);

/// Lidar dots by range, picked to read on the paper background:
/// red (near) -> orange -> teal -> blue (far).
fn range_color(r: f32) -> Color {
    const STOPS: [(f32, [f32; 3]); 4] =
        [(0.0, [214.0, 40.0, 57.0]), (1.0, [247.0, 127.0, 0.0]), (2.5, [0.0, 150.0, 136.0]), (5.0, [40.0, 80.0, 200.0])];
    let mut prev = STOPS[0];
    for st in STOPS {
        if r <= st.0 {
            let t = if st.0 > prev.0 { (r - prev.0) / (st.0 - prev.0) } else { 0.0 };
            let c = |i: usize| (prev.1[i] + (st.1[i] - prev.1[i]) * t) as u8;
            return Color::RGB(c(0), c(1), c(2));
        }
        prev = st;
    }
    let c = STOPS[3].1;
    Color::RGB(c[0] as u8, c[1] as u8, c[2] as u8)
}

/// The live map's walls that aren't within a cell of a wall in the saved map
/// (same grid as `live`; everything else -1).
fn live_only(live: &MapData, saved: &MapData) -> MapData {
    let wall_at = |x: f32, y: f32| -> bool {
        let col = ((x - saved.origin_x) / saved.resolution).floor() as i32;
        let row = ((y - saved.origin_y) / saved.resolution).floor() as i32;
        (-1..=1).any(|dr| {
            (-1..=1).any(|dc| {
                let (r, c) = (row + dr, col + dc);
                r >= 0 && c >= 0 && (r as u32) < saved.height && (c as u32) < saved.width
                    && saved.cells[(r as u32 * saved.width + c as u32) as usize] >= 50
            })
        })
    };
    let w = live.width as usize;
    let cells = live
        .cells
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            if v < 50 {
                return -1;
            }
            let (row, col) = (i / w, i % w);
            let x = live.origin_x + (col as f32 + 0.5) * live.resolution;
            let y = live.origin_y + (row as f32 + 0.5) * live.resolution;
            if wall_at(x, y) { -1 } else { 100 }
        })
        .collect();
    MapData { cells, ..*live }
}

/// RGBA texture of a map, one texel per cell, top row = highest y.
fn map_texture(tc: &TextureCreator<WindowContext>, map: &MapData, color: impl Fn(i8) -> [u8; 4]) -> Option<Texture> {
    let mut tex = tc.create_texture_streaming(PixelFormatEnum::RGBA32, map.width, map.height).ok()?;
    tex.set_blend_mode(BlendMode::Blend);
    let (w, h) = (map.width as usize, map.height as usize);
    let mut px = vec![0u8; w * h * 4];
    for row in 0..h {
        for col in 0..w {
            let o = ((h - 1 - row) * w + col) * 4;
            px[o..o + 4].copy_from_slice(&color(map.cells[row * w + col]));
        }
    }
    tex.update(None, &px, w * 4).ok()?;
    Some(tex)
}

pub struct Ui {
    font: Fonts,
    disc: Texture,
    ring: Texture,
    video: Option<VideoTex>,
    map: Option<MapTex>,
    /// Nav view: base map in ink-on-paper (the saved map, or the live one if
    /// none is saved; `version` tells which) and the live map's walls in colour.
    nav_base: Option<(MapTex, bool)>,
    nav_live: Option<MapTex>,
    /// Settings list scroll position (pixels), eased toward its target.
    scroll: f32,
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

/// White anti-aliased disc (or ring of the given relative thickness), 256 px.
fn round_texture(tc: &TextureCreator<WindowContext>, ring: Option<f32>) -> Result<Texture, String> {
    const N: usize = 256;
    let mut px = vec![0u8; N * N * 4];
    let r = N as f32 / 2.0;
    for y in 0..N {
        for x in 0..N {
            let d = ((x as f32 + 0.5 - r).powi(2) + (y as f32 + 0.5 - r).powi(2)).sqrt();
            let mut a = (r - d).clamp(0.0, 1.0);
            if let Some(t) = ring {
                a *= (d - (r - t)).clamp(0.0, 1.0);
            }
            let o = (y * N + x) * 4;
            px[o..o + 4].copy_from_slice(&[255, 255, 255, (a * 255.0) as u8]);
        }
    }
    sdl2::hint::set("SDL_RENDER_SCALE_QUALITY", "linear");
    let created = tc.create_texture_static(PixelFormatEnum::RGBA32, N as u32, N as u32);
    sdl2::hint::set("SDL_RENDER_SCALE_QUALITY", "nearest");
    let mut tex = created.map_err(|e| e.to_string())?;
    tex.update(None, &px, N * 4).map_err(|e| e.to_string())?;
    tex.set_blend_mode(BlendMode::Blend);
    Ok(tex)
}

impl Ui {
    pub fn new(tc: &TextureCreator<WindowContext>) -> Result<Ui, String> {
        Ok(Ui {
            font: Fonts::new(tc)?,
            disc: round_texture(tc, None)?,
            ring: round_texture(tc, Some(14.0))?,
            video: None,
            map: None,
            nav_base: None,
            nav_live: None,
            scroll: 0.0,
        })
    }

    pub fn render(
        &mut self,
        canvas: &mut Canvas<Window>,
        tc: &TextureCreator<WindowContext>,
        snap: &Snapshot,
        menu: &Menu,
    ) {
        canvas.set_blend_mode(BlendMode::Blend);
        canvas.set_draw_color(BG);
        canvas.clear();

        if let Some(cam) = &snap.nav_cam {
            self.draw_nav(canvas, tc, snap, cam);
            self.draw_top_bar(canvas, snap);
            if menu.open {
                self.draw_settings(canvas, snap, menu);
            } else if menu.controls_open {
                self.draw_controls_help(canvas);
            } else {
                self.draw_nav_panel(canvas, snap, cam);
            }
            if !menu.controls_open {
                self.draw_overlay(canvas, snap, menu.open, true);
            }
            self.draw_toast(canvas, menu, TOP_H + M + 60);
            return;
        }

        match snap.view {
            View::Color | View::Depth => self.draw_video(canvas, tc, snap),
            View::Lidar => self.draw_lidar(canvas, snap),
            View::Map => {
                let area = Rect::new(M, TOP_H + M, LOGICAL_W - 2 * M as u32, LOGICAL_H - (TOP_H + 2 * M) as u32);
                self.draw_map(canvas, tc, snap, area, 4);
            }
        }

        self.draw_top_bar(canvas, snap);
        self.draw_status(canvas, snap);
        if snap.view != View::Map {
            let r = Rect::new(M, (LOGICAL_H - MINIMAP) as i32 - M, MINIMAP, MINIMAP);
            self.draw_minimap(canvas, tc, snap, r);
        }
        if menu.open {
            self.draw_settings(canvas, snap, menu);
        } else if menu.controls_open {
            self.draw_controls_help(canvas);
        } else {
            self.draw_controls(canvas, snap);
            match snap.mode().cloned() {
                Some(m) => self.draw_mode(canvas, &m, snap),
                None => self.draw_hints(canvas),
            }
        }
        if !menu.controls_open {
            self.draw_overlay(canvas, snap, menu.open, false);
        }
        self.draw_toast(canvas, menu, LOGICAL_H as i32 - M - 38 - 56);
    }

    // ── primitives ───────────────────────────────────────────────────────────

    fn fill_round(&mut self, canvas: &mut Canvas<Window>, r: Rect, radius: i32, color: Color) {
        let rad = radius.min(r.width() as i32 / 2).min(r.height() as i32 / 2).max(0);
        canvas.set_draw_color(color);
        if rad == 0 {
            let _ = canvas.fill_rect(r);
            return;
        }
        let (x, y, w, h) = (r.x(), r.y(), r.width() as i32, r.height() as i32);
        let rects = [
            Rect::new(x + rad, y, (w - 2 * rad).max(0) as u32, h as u32),
            Rect::new(x, y + rad, rad as u32, (h - 2 * rad).max(0) as u32),
            Rect::new(x + w - rad, y + rad, rad as u32, (h - 2 * rad).max(0) as u32),
        ];
        for rc in rects {
            if rc.width() > 0 && rc.height() > 0 {
                let _ = canvas.fill_rect(rc);
            }
        }
        self.disc.set_color_mod(color.r, color.g, color.b);
        self.disc.set_alpha_mod(color.a);
        let q = 128;
        let rr = rad as u32;
        for (sx, sy, dx, dy) in [(0, 0, x, y), (q, 0, x + w - rad, y), (0, q, x, y + h - rad), (q, q, x + w - rad, y + h - rad)] {
            let _ = canvas.copy(&self.disc, Rect::new(sx, sy, q as u32, q as u32), Rect::new(dx, dy, rr, rr));
        }
    }

    fn circle(&mut self, canvas: &mut Canvas<Window>, cx: i32, cy: i32, radius: i32, color: Color, ring: bool) {
        let tex = if ring { &mut self.ring } else { &mut self.disc };
        tex.set_color_mod(color.r, color.g, color.b);
        tex.set_alpha_mod(color.a);
        let d = (radius * 2).max(1) as u32;
        let _ = canvas.copy(tex, None, Rect::new(cx - radius, cy - radius, d, d));
    }

    /// Card background with a soft top highlight.
    fn card(&mut self, canvas: &mut Canvas<Window>, r: Rect, title: &str) {
        self.fill_round(canvas, r, RADIUS, PANEL);
        canvas.set_draw_color(HAIRLINE);
        let _ = canvas.draw_line(Point::new(r.x() + RADIUS, r.y()), Point::new(r.right() - RADIUS, r.y()));
        if !title.is_empty() {
            self.font.draw(canvas, &title.to_uppercase(), r.x() + 16, r.y() + 12, Weight::Bold, 12, MUTED);
        }
    }

    fn text(&mut self, canvas: &mut Canvas<Window>, s: &str, x: i32, y: i32, w: Weight, px: u32, c: Color) -> i32 {
        self.font.draw(canvas, s, x, y, w, px, c)
    }

    fn text_right(&mut self, canvas: &mut Canvas<Window>, s: &str, right: i32, y: i32, w: Weight, px: u32, c: Color) {
        let tw = self.font.width(s, w, px);
        self.font.draw(canvas, s, right - tw, y, w, px, c);
    }

    fn text_center(&mut self, canvas: &mut Canvas<Window>, s: &str, cx: i32, y: i32, w: Weight, px: u32, c: Color) {
        let tw = self.font.width(s, w, px);
        self.font.draw(canvas, s, cx - tw / 2, y, w, px, c);
    }

    /// Small key cap ("L2", "☰") followed by a label; returns the total width.
    fn keycap(&mut self, canvas: &mut Canvas<Window>, key: &str, label: &str, x: i32, y: i32) -> i32 {
        let kw = self.font.width(key, Weight::Bold, 12).max(12) + 14;
        self.fill_round(canvas, Rect::new(x, y, kw as u32, 22), 6, TRACK);
        self.text_center(canvas, key, x + kw / 2, y + 3, Weight::Bold, 12, TEXT);
        if label.is_empty() {
            return kw;
        }
        kw + 6 + self.text(canvas, label, x + kw + 6, y + 3, Weight::Regular, 14, MUTED)
    }

    /// Rounded status pill with a coloured dot; drawn so it ends at `right`. Returns its left edge.
    fn pill(&mut self, canvas: &mut Canvas<Window>, right: i32, label: &str, dot: Color, fill: Color, text: Color) -> i32 {
        let w = self.font.width(label, Weight::Bold, 14) + 38;
        let x = right - w;
        self.fill_round(canvas, Rect::new(x, 13, w as u32, 30), 15, fill);
        self.circle(canvas, x + 15, 28, 4, dot, false);
        self.text(canvas, label, x + 26, 19, Weight::Bold, 14, text);
        x
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
        let dst = track::video_rect(frame.width, frame.height);
        let _ = canvas.copy(&v.tex, None, dst);
        if snap.view == View::Color {
            self.draw_track_overlay(canvas, snap, dst);
        }

        if snap.view == View::Depth {
            self.draw_depth_legend(canvas, snap);
        }
    }

    /// Sentry tracking on the camera view: the tracked box (green, amber while
    /// lost), and while picking the reticle and any box being dragged out.
    fn draw_track_overlay(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot, dst: Rect) {
        let thick = |canvas: &mut Canvas<Window>, r: Rect, c: Color| {
            canvas.set_draw_color(c);
            for i in 0..3 {
                let _ = canvas.draw_rect(Rect::new(r.x() - i, r.y() - i, r.width() + 2 * i as u32, r.height() + 2 * i as u32));
            }
        };
        let active = snap.track.as_ref().filter(|t| t.active());
        if let Some(t) = active {
            if let Some(b) = t.r#box {
                let r = track::to_screen(dst, b);
                let (c, label) = match t.state.as_str() {
                    "tracking" => (GOOD, "TRACKING".to_string()),
                    "lost" => (WARN, "LOST — looking".to_string()),
                    _ => (ACCENT, "STARTING".to_string()),
                };
                thick(canvas, r, c);
                let tw = self.font.width(&label, Weight::Bold, 12) + 14;
                let ty = (r.y() - 24).max(TOP_H + 4);
                self.fill_round(canvas, Rect::new(r.x() - 2, ty, tw as u32, 20), 6, c);
                self.text(canvas, &label, r.x() + 5, ty + 2, Weight::Bold, 12, ON_ACCENT);
            }
        }
        let Some(pick) = &snap.track_pick else { return };
        if let Some(b) = pick.drag_box() {
            thick(canvas, track::to_screen(dst, b), ACCENT);
        } else if active.is_none() {
            // Reticle: corner brackets.
            let r = track::to_screen(dst, pick.reticle(dst.width() as f32 / dst.height() as f32));
            let l = (r.width().min(r.height()) / 4) as i32;
            canvas.set_draw_color(ACCENT);
            for (cx, cy, sx, sy) in [(r.left(), r.top(), 1, 1), (r.right(), r.top(), -1, 1),
                                     (r.left(), r.bottom(), 1, -1), (r.right(), r.bottom(), -1, -1)] {
                for i in 0..3 {
                    let _ = canvas.draw_line(Point::new(cx, cy + sy * i), Point::new(cx + sx * l, cy + sy * i));
                    let _ = canvas.draw_line(Point::new(cx + sx * i, cy), Point::new(cx + sx * i, cy + sy * l));
                }
            }
            let (mx, my) = (r.x() + r.width() as i32 / 2, r.y() + r.height() as i32 / 2);
            let _ = canvas.draw_line(Point::new(mx - 6, my), Point::new(mx + 6, my));
            let _ = canvas.draw_line(Point::new(mx, my - 6), Point::new(mx, my + 6));
        }
    }

    fn draw_depth_legend(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let (near, far) = snap.state.as_ref().and_then(|s| s.depth_range_m).map(|r| (r[0], r[1])).unwrap_or((0.1, 2.0));
        let (x, y, w, h) = (LOGICAL_W as i32 - M - 76, TOP_H + 90, 14, 300);
        self.card(canvas, Rect::new(x - 18, y - 40, 76, h as u32 + 104), "");
        self.text(canvas, "DEPTH", x - 6, y - 30, Weight::Bold, 12, MUTED);
        for i in 0..h {
            // top = far (cool), bottom = near (warm), matching the robot's colouring
            canvas.set_draw_color(turbo(i as f32 / (h - 1) as f32));
            let _ = canvas.draw_line(Point::new(x, y + i), Point::new(x + w, y + i));
        }
        self.text(canvas, &format!("{far:.1} m"), x + w + 6, y - 6, Weight::Regular, 12, TEXT);
        self.text(canvas, &format!("{near:.1} m"), x + w + 6, y + h - 10, Weight::Regular, 12, TEXT);
        canvas.set_draw_color(Color::RGB(0, 0, 0));
        let _ = canvas.fill_rect(Rect::new(x, y + h + 18, w as u32, 14));
        self.text(canvas, "none", x + w + 6, y + h + 18, Weight::Regular, 12, MUTED);
    }

    fn draw_lidar(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let (cx, cy) = (LOGICAL_W as i32 / 2, LOGICAL_H as i32 / 2 + 24);
        let ppm = 64.0f32; // pixels per metre
        for m in 1..=5 {
            let radius = (m as f32 * ppm) as i32;
            canvas.set_draw_color(with_alpha(ACCENT, if m % 2 == 0 { 80 } else { 44 }));
            let mut prev: Option<Point> = None;
            for step in 0..=128 {
                let a = step as f32 / 128.0 * std::f32::consts::TAU;
                let p = Point::new(cx + (a.cos() * radius as f32) as i32, cy + (a.sin() * radius as f32) as i32);
                if let Some(q) = prev {
                    let _ = canvas.draw_line(q, p);
                }
                prev = Some(p);
            }
            self.text(canvas, &format!("{m} m"), cx + 6, cy - radius - 16, Weight::Regular, 12, FAINT);
        }
        canvas.set_draw_color(with_alpha(ACCENT, 28));
        let _ = canvas.draw_line(Point::new(cx, TOP_H + 8), Point::new(cx, LOGICAL_H as i32 - 8));
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
                for p in pts {
                    self.circle(canvas, p.x(), p.y(), 2, colors[b], false);
                }
            }
        }
        self.robot_marker(canvas, cx as f32, cy as f32, std::f32::consts::FRAC_PI_2, 12.0);
        self.text_center(canvas, "FRONT", cx, cy - 5 * ppm as i32 - 36, Weight::Bold, 12, MUTED);
    }

    /// Filled-looking arrow head: outline triangle plus a centre dot.
    fn robot_marker(&mut self, canvas: &mut Canvas<Window>, px: f32, py: f32, yaw: f32, size: f32) {
        let (dx, dy) = (yaw.cos(), -yaw.sin()); // screen y points down
        let (nx, ny) = (-dy, dx);
        let tip = Point::new((px + dx * size * 1.5) as i32, (py + dy * size * 1.5) as i32);
        let left = Point::new((px - dx * size + nx * size * 0.8) as i32, (py - dy * size + ny * size * 0.8) as i32);
        let right = Point::new((px - dx * size - nx * size * 0.8) as i32, (py - dy * size - ny * size * 0.8) as i32);
        canvas.set_draw_color(WARN);
        // a few nested outlines read as a solid shape at this size
        for k in 0..3 {
            let f = 1.0 - k as f32 * 0.25;
            let s = |p: Point| Point::new((px + (p.x() as f32 - px) * f) as i32, (py + (p.y() as f32 - py) * f) as i32);
            let _ = canvas.draw_line(s(tip), s(left));
            let _ = canvas.draw_line(s(tip), s(right));
            let _ = canvas.draw_line(s(left), s(right));
        }
        self.circle(canvas, px as i32, py as i32, 3, WARN, false);
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
                let px = if area.width() > 600 { 22 } else { 16 };
                let y = area.y() + area.height() as i32 / 2 - 10;
                self.text_center(canvas, "No map yet", area.x() + area.width() as i32 / 2, y, Weight::Bold, px, FAINT);
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
                            [120, 230, 255, 255] // wall
                        } else {
                            [40, 56, 78, 235] // known free space
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

        let scale = ((area.width() as f32 - 8.0) / map.width as f32).min((area.height() as f32 - 8.0) / map.height as f32);
        let (dw, dh) = ((map.width as f32 * scale) as u32, (map.height as f32 * scale) as u32);
        let dst = Rect::new(
            area.x() + (area.width() as i32 - dw as i32) / 2,
            area.y() + (area.height() as i32 - dh as i32) / 2,
            dw.max(1),
            dh.max(1),
        );
        let _ = canvas.copy(&mt.tex, None, dst);

        // world (map frame, metres) -> screen
        let to_screen = |x: f32, y: f32| -> (f32, f32) {
            (
                dst.x() as f32 + (x - map.origin_x) / map.resolution * scale,
                dst.y() as f32 + (map.height as f32 - (y - map.origin_y) / map.resolution) * scale,
            )
        };

        let Some(pose) = snap.state.as_ref().and_then(|s| s.pose.clone()) else { return };
        let yaw = pose.yaw_deg.to_radians();

        // live lidar overlay on the map, like a radar sweep
        if let Some(scan) = &snap.scan {
            if snap.scan_age < 1.0 {
                canvas.set_draw_color(with_alpha(WARN, 210));
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
                    if sx >= area.x() as f32 && sy >= area.y() as f32 && sx < area.right() as f32 && sy < area.bottom() as f32 {
                        let _ = canvas.fill_rect(Rect::new(sx as i32, sy as i32, scan_dot / 2 + 1, scan_dot / 2 + 1));
                    }
                }
            }
        }
        let (px, py) = to_screen(pose.x, pose.y);
        self.robot_marker(canvas, px, py, yaw, (scan_dot as f32 * 2.2).max(7.0));
    }

    fn draw_minimap(&mut self, canvas: &mut Canvas<Window>, tc: &TextureCreator<WindowContext>, snap: &Snapshot, r: Rect) {
        self.card(canvas, r, "Map");
        let inner = Rect::new(r.x() + 8, r.y() + 32, r.width() - 16, r.height() - 58);
        canvas.set_clip_rect(inner);
        self.draw_map(canvas, tc, snap, inner, 2);
        canvas.set_clip_rect(None);
        if let Some(m) = &snap.map {
            let label = format!("{:.1} × {:.1} m", m.width as f32 * m.resolution, m.height as f32 * m.resolution);
            self.text(canvas, &label, r.x() + 16, r.bottom() - 22, Weight::Regular, 12, MUTED);
        }
    }

    // ── auto nav view ────────────────────────────────────────────────────────

    /// Line `width` px thick (parallel offsets; fine for the few lines drawn here).
    fn thick_line(&mut self, canvas: &mut Canvas<Window>, a: (f32, f32), b: (f32, f32), width: f32, color: Color) {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len = (dx * dx + dy * dy).sqrt().max(1e-3);
        let (nx, ny) = (-dy / len, dx / len);
        canvas.set_draw_color(color);
        let n = width.max(1.0) as i32;
        for i in 0..n {
            let o = i as f32 - (n - 1) as f32 / 2.0;
            let _ = canvas.draw_line(
                Point::new((a.0 + nx * o) as i32, (a.1 + ny * o) as i32),
                Point::new((b.0 + nx * o) as i32, (b.1 + ny * o) as i32),
            );
        }
    }

    /// Draw a map texture where `cam` puts it, rotated with the view.
    fn draw_map_tex(canvas: &mut Canvas<Window>, tex: &Texture, map: &MapData, cam: &NavCam) {
        let (wm, hm) = (map.width as f32 * map.resolution, map.height as f32 * map.resolution);
        let (sx, sy) = cam.world_to_screen(map.origin_x + wm / 2.0, map.origin_y + hm / 2.0);
        let (dw, dh) = ((wm * cam.zoom).max(1.0), (hm * cam.zoom).max(1.0));
        let dst = Rect::new((sx - dw / 2.0).round() as i32, (sy - dh / 2.0).round() as i32, dw.round() as u32, dh.round() as u32);
        // SDL rotates clockwise; the view's rotation is counter-clockwise.
        let _ = canvas.copy_ex(tex, None, dst, -(cam.rot.to_degrees() as f64), None, false, false);
    }

    fn draw_nav(&mut self, canvas: &mut Canvas<Window>, tc: &TextureCreator<WindowContext>, snap: &Snapshot, cam: &NavCam) {
        canvas.set_draw_color(PAPER);
        let _ = canvas.fill_rect(Rect::new(0, TOP_H, LOGICAL_W, LOGICAL_H - TOP_H as u32));

        // 1 m grid, rotated with the map.
        let corners = [(0.0, TOP_H as f32), (LOGICAL_W as f32, TOP_H as f32), (0.0, LOGICAL_H as f32), (LOGICAL_W as f32, LOGICAL_H as f32)]
            .map(|(x, y)| cam.screen_to_world(x, y));
        let (x0, x1) = corners.iter().fold((f32::MAX, f32::MIN), |(a, b), c| (a.min(c.0), b.max(c.0)));
        let (y0, y1) = corners.iter().fold((f32::MAX, f32::MIN), |(a, b), c| (a.min(c.1), b.max(c.1)));
        let step = if cam.zoom < 30.0 { 5.0 } else { 1.0 };
        canvas.set_draw_color(Color::RGBA(0, 0, 0, 22));
        let mut gx = (x0 / step).floor() * step;
        while gx <= x1 {
            let (a, b) = (cam.world_to_screen(gx, y0), cam.world_to_screen(gx, y1));
            let _ = canvas.draw_line(Point::new(a.0 as i32, a.1 as i32), Point::new(b.0 as i32, b.1 as i32));
            gx += step;
        }
        let mut gy = (y0 / step).floor() * step;
        while gy <= y1 {
            let (a, b) = (cam.world_to_screen(x0, gy), cam.world_to_screen(x1, gy));
            let _ = canvas.draw_line(Point::new(a.0 as i32, a.1 as i32), Point::new(b.0 as i32, b.1 as i32));
            gy += step;
        }

        // Saved map (or the live one when nothing is saved) in ink on paper.
        let saved = snap.static_map.as_ref().filter(|m| m.width > 0 && m.height > 0);
        let (base, from_saved, version) = match saved {
            Some(m) => (Some(m.clone()), true, snap.static_map_version),
            None => (snap.map.clone(), false, snap.map_version),
        };
        let ink = |v: i8| if v < 0 { [0, 0, 0, 0] } else if v >= 50 { INK } else { PAPER_FREE };
        if let Some(m) = &base {
            let stale = !matches!(&self.nav_base, Some((t, s)) if t.version == version && *s == from_saved);
            if stale {
                self.nav_base = map_texture(tc, m, ink).map(|tex| (MapTex { tex, version }, from_saved));
            }
            if let Some((t, _)) = &self.nav_base {
                Self::draw_map_tex(canvas, &t.tex, m, cam);
            }
        }
        // Live SLAM walls that the saved map doesn't have (something new), in colour.
        if let (true, Some(m), Some(saved)) = (from_saved, &snap.map, saved) {
            {
                let key = snap.map_version ^ (snap.static_map_version << 32);
                if self.nav_live.as_ref().map(|t| t.version != key).unwrap_or(true) {
                    let new_walls = live_only(m, saved);
                    self.nav_live = map_texture(tc, &new_walls, |v| if v >= 50 { LIVE_WALL } else { [0, 0, 0, 0] })
                        .map(|tex| MapTex { tex, version: key });
                }
                if let Some(t) = &self.nav_live {
                    Self::draw_map_tex(canvas, &t.tex, m, cam);
                }
            }
        }

        let st = snap.fresh_state();
        let pose = st.and_then(|s| s.pose.clone());
        let nav = st.and_then(|s| s.nav.clone());

        // Planned path and goal.
        if let Some(n) = &nav {
            let pts: Vec<(f32, f32)> = n.path.iter().map(|p| cam.world_to_screen(p[0], p[1])).collect();
            for w in pts.windows(2) {
                self.thick_line(canvas, w[0], w[1], 5.0, with_alpha(NAV_PATH, 200));
            }
            for p in pts.iter().skip(1) {
                self.circle(canvas, p.0 as i32, p.1 as i32, 5, NAV_PATH, false);
            }
            if let Some([gx, gy]) = n.goal {
                let (sx, sy) = cam.world_to_screen(gx, gy);
                self.circle(canvas, sx as i32, sy as i32, 16, with_alpha(NAV_GOAL, 60), false);
                self.circle(canvas, sx as i32, sy as i32, 16, NAV_GOAL, true);
                self.circle(canvas, sx as i32, sy as i32, 6, NAV_GOAL, false);
            }
        }

        // Live lidar, coloured by range.
        if let (Some(scan), Some(p)) = (&snap.scan, &pose) {
            if snap.scan_age < 1.0 {
                let yaw = p.yaw_deg.to_radians();
                let (s, c) = yaw.sin_cos();
                let dot = if cam.zoom > 150.0 { 3 } else { 2 };
                for (i, &mm) in scan.ranges_mm.iter().enumerate() {
                    if mm == 0xFFFF {
                        continue;
                    }
                    let r = mm as f32 / 1000.0;
                    let a = scan.angle_min + i as f32 * scan.angle_increment;
                    let (lx, ly) = (r * a.cos(), r * a.sin());
                    let (sx, sy) = cam.world_to_screen(p.x + c * lx - s * ly, p.y + s * lx + c * ly);
                    if sx < 0.0 || sy < TOP_H as f32 || sx > LOGICAL_W as f32 || sy > LOGICAL_H as f32 {
                        continue;
                    }
                    self.circle(canvas, sx as i32, sy as i32, dot, range_color(r), false);
                }
            }
        }

        // Placing the robot: the chosen spot, and the way it faces (toward the cursor).
        let (cx, cy) = cam.cursor;
        if let Some(Place::Facing(px, py)) = cam.place {
            let (sx, sy) = cam.world_to_screen(px, py);
            self.circle(canvas, sx as i32, sy as i32, 18, with_alpha(NAV_PLACE, 60), false);
            match cam.facing() {
                Some((_, _, yaw)) => {
                    self.thick_line(canvas, (sx, sy), (cx, cy), 2.0, with_alpha(NAV_PLACE, 160));
                    self.robot_marker_color(canvas, sx, sy, yaw + cam.rot, 12.0, NAV_PLACE);
                }
                None => self.circle(canvas, sx as i32, sy as i32, 8, NAV_PLACE, false),
            }
        }

        // Cursor: crosshair, with where it is and how far from the robot.
        let col = if cam.place.is_some() {
            NAV_PLACE
        } else if snap.nav_a_pending {
            NAV_GOAL
        } else {
            NAV_CURSOR
        };
        self.circle(canvas, cx as i32, cy as i32, 14, col, true);
        for (dx, dy) in [(-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)] {
            self.thick_line(canvas, (cx + dx * 8.0, cy + dy * 8.0), (cx + dx * 22.0, cy + dy * 22.0), 2.0, col);
        }
        let (wx, wy) = cam.screen_to_world(cx, cy);
        let mut label = format!("{wx:.1}, {wy:.1} m");
        if let Some(p) = &pose {
            label = format!("{label}  ·  {:.1} m away", ((wx - p.x).powi(2) + (wy - p.y).powi(2)).sqrt());
        }
        match (cam.place, cam.facing()) {
            (Some(Place::Spot), _) => label = format!("Robot here?  {wx:.1}, {wy:.1} m"),
            (Some(Place::Facing(..)), Some((_, _, yaw))) => label = format!("Facing {:.0}°", yaw.to_degrees()),
            (Some(Place::Facing(..)), None) => label = "Move the cursor the way it faces".to_string(),
            (None, _) if snap.nav_a_pending => label = "Going here…".to_string(),
            _ => {}
        }
        let tw = self.font.width(&label, Weight::Bold, 13) + 16;
        let (lx, ly) = ((cx as i32 + 20).min(LOGICAL_W as i32 - tw - 8), cy as i32 + 18);
        self.fill_round(canvas, Rect::new(lx, ly, tw as u32, 22), 11, Color::RGBA(20, 22, 28, 200));
        self.text(canvas, &label, lx + 8, ly + 3, Weight::Bold, 13, Color::RGB(250, 250, 250));

        // Robot, on top of the cursor (which starts on it).
        if let Some(p) = &pose {
            let (sx, sy) = cam.world_to_screen(p.x, p.y);
            self.circle(canvas, sx as i32, sy as i32, 15, with_alpha(NAV_ROBOT, 50), false);
            self.robot_marker_color(canvas, sx, sy, p.yaw_deg.to_radians() + cam.rot, 11.0, NAV_ROBOT);
        }

        self.draw_compass(canvas, cam);
    }

    fn robot_marker_color(&mut self, canvas: &mut Canvas<Window>, px: f32, py: f32, yaw: f32, size: f32, color: Color) {
        let (dx, dy) = (yaw.cos(), -yaw.sin());
        let (nx, ny) = (-dy, dx);
        let tip = (px + dx * size * 1.6, py + dy * size * 1.6);
        let left = (px - dx * size + nx * size * 0.9, py - dy * size + ny * size * 0.9);
        let right = (px - dx * size - nx * size * 0.9, py - dy * size - ny * size * 0.9);
        // Filled by drawing lines from the tip to points along the back edge.
        canvas.set_draw_color(color);
        for i in 0..=24 {
            let t = i as f32 / 24.0;
            let b = (left.0 + (right.0 - left.0) * t, left.1 + (right.1 - left.1) * t);
            let _ = canvas.draw_line(Point::new(tip.0 as i32, tip.1 as i32), Point::new(b.0 as i32, b.1 as i32));
        }
        self.circle(canvas, px as i32, py as i32, 3, Color::RGB(255, 255, 255), false);
    }

    /// North arrow (map +y) and a scale bar, top right.
    fn draw_compass(&mut self, canvas: &mut Canvas<Window>, cam: &NavCam) {
        let (cx, cy) = (LOGICAL_W as i32 - M - 44, TOP_H + M + 44);
        self.circle(canvas, cx, cy, 40, Color::RGBA(15, 19, 27, 220), false);
        let a = cam.rot + std::f32::consts::FRAC_PI_2; // map +y on screen
        let (dx, dy) = (a.cos(), -a.sin());
        let tip = (cx as f32 + dx * 26.0, cy as f32 + dy * 26.0);
        let tail = (cx as f32 - dx * 20.0, cy as f32 - dy * 20.0);
        self.thick_line(canvas, (cx as f32, cy as f32), tail, 3.0, MUTED);
        self.thick_line(canvas, (cx as f32, cy as f32), tip, 3.0, BAD);
        self.circle(canvas, tip.0 as i32, tip.1 as i32, 4, BAD, false);
        self.text_center(canvas, "N", (cx as f32 + dx * 36.0) as i32, (cy as f32 + dy * 36.0) as i32 - 8, Weight::Bold, 12, TEXT);

        // Scale bar: the largest "nice" length that fits in ~150 px.
        let metres = [0.25f32, 0.5, 1.0, 2.0, 5.0, 10.0].into_iter().rev().find(|m| m * cam.zoom <= 150.0).unwrap_or(0.25);
        let px = (metres * cam.zoom) as i32;
        let (x, y) = (LOGICAL_W as i32 - M - px - 8, cy + 62);
        self.fill_round(canvas, Rect::new(x - 10, y - 8, (px + 28) as u32, 34), 10, Color::RGBA(15, 19, 27, 200));
        self.thick_line(canvas, (x as f32, y as f32 + 4.0), ((x + px) as f32, y as f32 + 4.0), 3.0, TEXT);
        let label = if metres < 1.0 { format!("{:.0} cm", metres * 100.0) } else { format!("{metres:.0} m") };
        self.text_center(canvas, &label, x + px / 2, y + 7, Weight::Bold, 12, TEXT);
    }

    /// Bottom card in auto nav: what the planner is doing, the legend and the controls.
    fn draw_nav_panel(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot, cam: &NavCam) {
        let h = 108;
        let r = Rect::new(M, LOGICAL_H as i32 - M - h, LOGICAL_W - 2 * M as u32, h as u32);
        self.card(canvas, r, "Auto nav");
        let st = snap.fresh_state();
        let nav = st.and_then(|s| s.nav.clone()).unwrap_or_default();
        let manual = st.and_then(|s| s.mode.as_ref()).map(|m| m.manual).unwrap_or(false);
        let (text, color) = if let Some(place) = cam.place {
            let step = match place {
                Place::Spot => "Where is the robot? Move the cursor there, A or tap",
                Place::Facing(..) => "Which way does it face? Point the cursor, A or tap",
            };
            (step.to_string(), NAV_PLACE)
        } else if manual {
            ("Manual — the left stick is driving".to_string(), WARN)
        } else {
            match nav.state.as_str() {
                "driving" => (nav.text.clone(), ACCENT),
                "blocked" => (nav.text.clone(), WARN),
                "arrived" => ("Arrived".to_string(), GOOD),
                "failed" => (nav.text.clone(), BAD),
                _ if !nav.text.is_empty() => (nav.text.clone(), MUTED),
                _ => ("Pick a spot: move the cursor and press A, or tap the map".to_string(), TEXT),
            }
        };
        let text = self.font.fit(&text, Weight::Bold, 18, r.width() as i32 / 2);
        self.text(canvas, &text, r.x() + 16, r.y() + 34, Weight::Bold, 18, color);

        // Speed (D-pad up/down), next to the card title.
        if let Some(speed) = nav.speed {
            let mut label = format!("Speed {:.0}%", speed * 100.0);
            if let Some(mps) = nav.speed_mps {
                label = format!("{label} · {:.0} cm/s", mps * 100.0);
            }
            let tx = r.x() + 16 + self.font.width("Auto nav", Weight::Bold, 14) + 16;
            self.text(canvas, &label, tx, r.y() + 12, Weight::Bold, 13, ACCENT);
        }

        // Legend, right.
        let mut x = r.right() - 16;
        let y = r.y() + 36;
        let saved = snap.static_map.is_some();
        // (label, colour, drawn as ink on a paper swatch)
        let items: Vec<(&str, Color, bool)> = if saved {
            vec![("Saved map", Color::RGB(22, 22, 26), true), ("New since saved", Color::RGB(0, 132, 255), false), ("Lidar", range_color(1.0), false)]
        } else {
            vec![("Live map (nothing saved yet: Map ▸ Save map)", Color::RGB(22, 22, 26), true), ("Lidar", range_color(1.0), false)]
        };
        for (label, c, ink) in items.into_iter().rev() {
            let tw = self.font.width(label, Weight::Regular, 13);
            x -= tw;
            self.text(canvas, label, x, y, Weight::Regular, 13, MUTED);
            x -= 20;
            if ink {
                let p = PAPER_FREE;
                self.fill_round(canvas, Rect::new(x, y + 2, 14, 14), 3, Color::RGB(p[0], p[1], p[2]));
                canvas.set_draw_color(c);
                let _ = canvas.fill_rect(Rect::new(x, y + 7, 14, 4));
            } else {
                self.fill_round(canvas, Rect::new(x, y + 2, 14, 14), 3, c);
            }
            x -= 18;
        }

        let mut x = r.x() + 16;
        let y = r.bottom() - 36;
        let keys: &[(&str, &str)] = if cam.place.is_some() {
            &[("R-stick", "Cursor"), ("A", "Set"), ("Y / X", "Cancel"), ("L1/R1", "Rotate"), ("L2/R2", "Zoom"), ("←/→", "Re-centre")]
        } else {
            &[("R-stick", "Cursor"), ("A", "Go"), ("X", "Stop"), ("Y", "Robot is here"), ("↑/↓", "Speed"), ("L1/R1", "Rotate"), ("L2/R2", "Zoom"), ("←/→", "Re-centre"), ("A×2", "Exit"), ("B", "Go limp")]
        };
        for (k, l) in keys {
            x += self.keycap(canvas, k, l, x, y) + 12;
        }
    }

    // ── HUD ──────────────────────────────────────────────────────────────────

    fn draw_top_bar(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        canvas.set_draw_color(Color::RGBA(11, 14, 20, 238));
        let _ = canvas.fill_rect(Rect::new(0, 0, LOGICAL_W, TOP_H as u32));
        canvas.set_draw_color(HAIRLINE);
        let _ = canvas.draw_line(Point::new(0, TOP_H), Point::new(LOGICAL_W as i32, TOP_H));

        self.circle(canvas, 26, 28, 7, ACCENT, false);
        self.circle(canvas, 26, 28, 12, with_alpha(ACCENT, 60), true);
        self.text(canvas, "SpotMicro", 46, 15, Weight::Bold, 22, TEXT);

        // view switcher: [L2] ( Camera | Depth | Lidar | Map ); in auto nav L2 zooms instead.
        if snap.nav_cam.is_some() {
            let tw = self.font.width("Auto nav map", Weight::Bold, 14);
            let hw = self.font.width("A×2 to exit", Weight::Regular, 14);
            self.fill_round(canvas, Rect::new(186, 12, (tw + hw + 44) as u32, 32), 16, SURFACE);
            self.text(canvas, "Auto nav map", 186 + 16, 19, Weight::Bold, 14, TEXT);
            self.text(canvas, "A×2 to exit", 186 + 28 + tw, 19, Weight::Regular, 14, MUTED);
        } else {
            self.draw_view_switcher(canvas, snap, 186);
        }

        // status pills, right to left
        let mut right = LOGICAL_W as i32 - M;
        let (c, label) = if snap.broker_connected { (GOOD, "Link") } else { (BAD, "No link") };
        right = self.pill(canvas, right, label, c, SURFACE, TEXT) - 8;
        let (c, label) = match snap.robot_online {
            Some(true) => (GOOD, "Robot"),
            Some(false) => (BAD, "Robot offline"),
            None => (FAINT, "Robot"),
        };
        right = self.pill(canvas, right, label, c, SURFACE, TEXT) - 8;

        let gait = snap.fresh_state().and_then(|s| s.gait.clone());
        right = match gait {
            Some(g) if g.armed => self.pill(canvas, right, "ARMED", Color::RGB(255, 255, 255), with_alpha(BAD, 210), TEXT),
            Some(g) if g.running => self.pill(canvas, right, "Disarmed", GOOD, SURFACE, TEXT),
            Some(_) => self.pill(canvas, right, "Gait off", FAINT, SURFACE, MUTED),
            None => right + 8,
        } - 8;
        match snap.mode().map(|m| m.mode.as_str()) {
            Some("sentry") => right = self.pill(canvas, right, "SENTRY", ON_ACCENT, ACCENT, ON_ACCENT) - 8,
            Some("calibrate") => right = self.pill(canvas, right, "CALIBRATING", ON_ACCENT, WARN, ON_ACCENT) - 8,
            Some("trick") => right = self.pill(canvas, right, "TRICK", ON_ACCENT, ACCENT, ON_ACCENT) - 8,
            Some("imu_calib") => right = self.pill(canvas, right, "IMU CAL", ON_ACCENT, WARN, ON_ACCENT) - 8,
            Some("auto_nav") => right = self.pill(canvas, right, "AUTO NAV", ON_ACCENT, GOOD, ON_ACCENT) - 8,
            _ => {}
        }
        if snap.audio_live {
            right = self.pill(canvas, right, "Mic", ACCENT, SURFACE, TEXT) - 8;
        }

        if let Some(cam) = snap.camera().cloned() {
            let (dot, label) = match cam.kind.as_deref() {
                Some("realsense") => {
                    let model = cam.model.trim_start_matches("RealSense ").to_string();
                    (if cam.streaming { GOOD } else { WARN }, format!("{model} · depth"))
                }
                Some(_) => {
                    let name = self.font.fit(&cam.model, Weight::Bold, 14, 150);
                    (if cam.streaming { GOOD } else { WARN }, format!("USB · {name}"))
                }
                None => (FAINT, "No camera".to_string()),
            };
            right = self.pill(canvas, right, &label, dot, SURFACE, TEXT) - 8;
        }
        if snap.video_fps > 0.0 && snap.view.is_video() && snap.frame_age < 2.0 {
            let s = format!("{:.0} fps · {:.0} ms", snap.video_fps, snap.decode_ms);
            self.text_right(canvas, &s, right - 4, 19, Weight::Regular, 14, MUTED);
        }
    }

    fn draw_view_switcher(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot, mut x: i32) {
        x += self.keycap(canvas, "L2", "", x, 17) + 8;
        let depth_ok = snap.depth_available();
        let widths: Vec<i32> = View::ALL.iter().map(|v| self.font.width(v.label(), Weight::Bold, 14) + 28).collect();
        let total: i32 = widths.iter().sum::<i32>() + 4;
        self.fill_round(canvas, Rect::new(x, 12, total as u32, 32), 16, SURFACE);
        let mut tx = x + 2;
        for (v, w) in View::ALL.iter().zip(&widths) {
            let active = *v == snap.view;
            if active {
                self.fill_round(canvas, Rect::new(tx, 14, *w as u32, 28), 14, ACCENT);
            }
            let color = if active {
                ON_ACCENT
            } else if *v == View::Depth && !depth_ok {
                FAINT
            } else {
                MUTED
            };
            self.text_center(canvas, v.label(), tx + w / 2, 19, Weight::Bold, 14, color);
            tx += w;
        }
    }

    fn draw_horizon(&mut self, canvas: &mut Canvas<Window>, r: Rect, roll: Option<f32>, pitch: Option<f32>) {
        self.fill_round(canvas, r, 10, Color::RGBA(0, 0, 0, 90));
        let (cx, cy) = (r.x() + r.width() as i32 / 2, r.y() + r.height() as i32 / 2);
        canvas.set_clip_rect(Rect::new(r.x() + 2, r.y() + 2, r.width() - 4, r.height() - 4));
        if let (Some(roll), Some(pitch)) = (roll, pitch) {
            let a = roll.to_radians();
            let off = (pitch * 1.6).clamp(-60.0, 60.0);
            let (s, c) = a.sin_cos();
            // horizon + a couple of pitch ladder lines
            for (k, alpha, len) in [(0.0f32, 255u8, 400.0f32), (-16.0, 70, 40.0), (16.0, 70, 40.0)] {
                let (ox, oy) = (cx as f32 - s * (off + k), cy as f32 + c * (off + k));
                canvas.set_draw_color(with_alpha(ACCENT, alpha));
                for t in 0..2 {
                    let t = t as f32;
                    let _ = canvas.draw_line(
                        Point::new((ox - c * len) as i32, (oy - s * len + t) as i32),
                        Point::new((ox + c * len) as i32, (oy + s * len + t) as i32),
                    );
                }
            }
        } else {
            canvas.set_clip_rect(None);
            self.text_center(canvas, "No IMU data", cx, cy - 9, Weight::Regular, 14, FAINT);
            return;
        }
        canvas.set_clip_rect(None);
        // fixed aircraft reference
        canvas.set_draw_color(WARN);
        for t in 0..2 {
            let _ = canvas.draw_line(Point::new(cx - 34, cy + t), Point::new(cx - 12, cy + t));
            let _ = canvas.draw_line(Point::new(cx + 12, cy + t), Point::new(cx + 34, cy + t));
        }
        self.circle(canvas, cx, cy, 3, WARN, false);
    }

    fn draw_status(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let r = Rect::new(M, TOP_H + M, 300, 374);
        self.card(canvas, r, "Status");
        let st = snap.fresh_state().cloned();
        let fresh = st.is_some();
        let st = st.unwrap_or_default();

        let imu = st.imu.clone();
        self.draw_horizon(canvas, Rect::new(r.x() + 14, r.y() + 36, r.width() - 28, 84), imu.as_ref().map(|i| i.roll), imu.as_ref().map(|i| i.pitch));

        let mut y = r.y() + 132;
        let left = r.x() + 16;
        let right = r.right() - 16;
        let mut row = |ui: &mut Ui, canvas: &mut Canvas<Window>, label: &str, value: Option<String>, color: Color| {
            ui.text(canvas, label, left, y, Weight::Regular, 14, MUTED);
            match value {
                Some(v) => ui.text_right(canvas, &v, right, y - 1, Weight::Bold, 16, color),
                None => ui.text_right(canvas, "—", right, y - 1, Weight::Bold, 16, FAINT),
            }
            y += 25;
        };
        let deg = |v: f32| format!("{v:+.1}°");
        row(self, canvas, "Roll", imu.as_ref().map(|i| deg(i.roll)), TEXT);
        row(self, canvas, "Pitch", imu.as_ref().map(|i| deg(i.pitch)), TEXT);
        row(self, canvas, "Heading", imu.as_ref().map(|i| format!("{:.0}°", i.yaw)), TEXT);
        row(self, canvas, "Position", st.pose.as_ref().map(|p| format!("{:.2}, {:.2} m", p.x, p.y)), TEXT);
        let (near, nc) = match &st.nearest {
            Some(n) => (
                Some(format!("{:.2} m  {:+.0}°", n.range_m, n.angle_deg)),
                if n.range_m < 0.4 { BAD } else if n.range_m < 0.8 { WARN } else { TEXT },
            ),
            None => (None, TEXT),
        };
        row(self, canvas, "Nearest", near, nc);
        let rate = |k: &str| st.rates.get(k).copied().unwrap_or(0.0);
        row(self, canvas, "Lidar / IMU", fresh.then(|| format!("{:.0} / {:.0} Hz", rate("scan"), rate("imu"))), TEXT);
        let hot = st.sys.cpu_temp_c.map(|t| t > 75.0).unwrap_or(false);
        let cpu = st.sys.cpu_temp_c.map(|t| format!("{t:.0}°C  load {:.1}", st.sys.load1.unwrap_or(0.0)));
        row(self, canvas, "CPU", cpu, if hot { BAD } else { TEXT });
        row(self, canvas, "Map", st.map.as_ref().map(|m| format!("{}×{}  {:.0}s ago", m.w, m.h, m.age_s)), TEXT);
        let up = fresh.then(|| format!("{}:{:02}", st.uptime_s / 60, st.uptime_s % 60));
        row(self, canvas, "Uptime", up, TEXT);
    }

    fn draw_controls(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot) {
        let r = Rect::new(LOGICAL_W as i32 - 300 - M, LOGICAL_H as i32 - 178 - M, 300, 178);
        self.card(canvas, r, "Controller");
        // sticks
        for (i, (ax, ay)) in [(0usize, 1usize), (2, 3)].iter().enumerate() {
            let (cx, cy) = (r.x() + 56 + i as i32 * 98, r.y() + 82);
            self.circle(canvas, cx, cy, 38, SURFACE, false);
            self.circle(canvas, cx, cy, 38, with_alpha(MUTED, 90), true);
            // ROS convention (left/up positive) -> screen
            let px = cx - (snap.input.axes[*ax] * 28.0) as i32;
            let py = cy - (snap.input.axes[*ay] * 28.0) as i32;
            let moved = snap.input.axes[*ax] != 0.0 || snap.input.axes[*ay] != 0.0;
            self.circle(canvas, px, py, 11, if moved { ACCENT } else { with_alpha(ACCENT, 150) }, false);
        }
        // triggers
        for (i, (label, ai)) in [("L2", 4usize), ("R2", 5)].iter().enumerate() {
            let (bx, by) = (r.x() + 220 + i as i32 * 34, r.y() + 44);
            self.fill_round(canvas, Rect::new(bx, by, 18, 76), 9, TRACK);
            let fill = (snap.input.axes[*ai] * 76.0) as i32;
            if fill > 2 {
                self.fill_round(canvas, Rect::new(bx, by + 76 - fill, 18, fill as u32), 9, WARN);
            }
            self.text_center(canvas, label, bx + 9, by + 80, Weight::Bold, 12, MUTED);
        }
        let pressed: Vec<String> = BUTTON_NAMES
            .iter()
            .zip(snap.input.buttons.iter())
            .filter(|(n, &d)| d && **n != "l2" && **n != "r2")
            .map(|(n, _)| n.to_uppercase())
            .collect();
        let (text, color) = if !snap.gamepad_connected {
            ("No controller".to_string(), WARN)
        } else if pressed.is_empty() {
            ("Ready".to_string(), MUTED)
        } else {
            (pressed.join("  "), TEXT)
        };
        let text = self.font.fit(&text, Weight::Bold, 14, r.width() as i32 - 32);
        self.text(canvas, &text, r.x() + 16, r.bottom() - 30, Weight::Bold, 14, color);
    }

    fn draw_hints(&mut self, canvas: &mut Canvas<Window>) {
        let hints = [("☰", "Settings"), ("View", "Controls"), ("L1+R1", "Start"), ("L3+R3", "Arm"), ("X", "Tricks"), ("B", "Stop")];
        let widths: i32 = hints
            .iter()
            .map(|(k, l)| self.font.width(k, Weight::Bold, 12).max(12) + 14 + 6 + self.font.width(l, Weight::Regular, 14) + 18)
            .sum();
        let w = widths + 14;
        let x0 = (MINIMAP as i32 + M + (LOGICAL_W as i32 - 300 - M)) / 2 - w / 2;
        let y = LOGICAL_H as i32 - M - 38;
        self.fill_round(canvas, Rect::new(x0, y, w as u32, 38), 19, PANEL);
        let mut x = x0 + 16;
        for (k, l) in hints {
            x += self.keycap(canvas, k, l, x, y + 8) + 18;
        }
    }

    /// Bottom-centre card for sentry / calibrate / trick / IMU-calibration mode
    /// (replaces the hints bar).
    fn draw_mode(&mut self, canvas: &mut Canvas<Window>, m: &ModeInfo, snap: &Snapshot) {
        let left = MINIMAP as i32 + 2 * M;
        let right = LOGICAL_W as i32 - 300 - 2 * M;
        let w = right - left;
        if m.mode == "trick" {
            let h = 104;
            let r = Rect::new(left, LOGICAL_H as i32 - M - h, w as u32, h as u32);
            self.card(canvas, r, "Trick");
            let name = trick_label(m.trick.as_deref().unwrap_or(""));
            self.text(canvas, &name, r.x() + 16, r.y() + 34, Weight::Bold, 22, TEXT);
            let mut x = r.x() + 16;
            let y = r.bottom() - 34;
            x += self.keycap(canvas, "X", "Stop", x, y) + 18;
            self.keycap(canvas, "B", "Go limp", x, y);
            return;
        }
        if m.mode == "imu_calib" {
            let Some(c) = m.imu_calib.clone() else { return };
            let gauge = !c.auto && c.tilt.is_some();
            let h = if gauge { 196 } else { 150 };
            let r = Rect::new(left, LOGICAL_H as i32 - M - h, w as u32, h as u32);
            self.card(canvas, r, if c.auto { "IMU axes — robot tilts itself" } else { "IMU axes — by hand" });
            // Step trail, the current one highlighted, done ones green.
            let steps = [("level", "Level"), ("left", "Left ↓"), ("right", "Right ↓"), ("front", "Nose ↓"), ("back", "Nose ↑")];
            let current = steps.iter().position(|(k, _)| *k == c.step).unwrap_or(steps.len());
            let mut x = r.x() + 16;
            for (i, (_, label)) in steps.iter().enumerate() {
                let (fill, color) = if i == current {
                    (ACCENT, ON_ACCENT)
                } else if i < current {
                    (SURFACE, GOOD)
                } else {
                    (SURFACE, MUTED)
                };
                let tw = self.font.width(label, Weight::Bold, 14) + 24;
                self.fill_round(canvas, Rect::new(x, r.y() + 36, tw as u32, 26), 13, fill);
                self.text_center(canvas, label, x + tw / 2, r.y() + 40, Weight::Bold, 14, color);
                x += tw + 10;
            }
            let prompt = self.font.fit(&c.prompt, Weight::Bold, 18, w - 32);
            self.text(canvas, &prompt, r.x() + 16, r.y() + 74, Weight::Bold, 18, TEXT);
            if gauge {
                // Tilt gauge (fills to the needed tilt, green once there) and hold-still bar.
                let tilt = c.tilt.unwrap_or(0.0);
                let need = c.need.unwrap_or(12.0).max(1.0);
                let hold = c.hold.unwrap_or(0.0).clamp(0.0, 1.0);
                let (bx, bw) = (r.x() + 16 + 70, (w - 32 - 70 - 90) / 2);
                let y = r.y() + 112;
                for (i, (label, frac, done, value)) in [
                    ("Tilt", if c.step == "level" { 1.0 } else { (tilt / need).min(1.0) }, c.step == "level" || tilt >= need,
                     if c.step == "level" { "level".to_string() } else { format!("{tilt:.0}° / {need:.0}°") }),
                    ("Still", hold, hold >= 1.0, format!("{:.0}%", hold * 100.0)),
                ].into_iter().enumerate() {
                    let yy = y + i as i32 * 26;
                    self.text(canvas, label, r.x() + 16, yy - 4, Weight::Regular, 14, MUTED);
                    let track = Rect::new(bx, yy, (bw * 2) as u32, 10);
                    self.fill_round(canvas, track, 5, TRACK);
                    let fw = ((bw * 2) as f32 * frac) as i32;
                    if fw > 0 {
                        self.fill_round(canvas, Rect::new(bx, yy, fw as u32, 10), 5, if done { GOOD } else { ACCENT });
                    }
                    self.text(canvas, &value, bx + bw * 2 + 12, yy - 4, Weight::Bold, 14, TEXT);
                }
                if !c.hint.is_empty() {
                    let hint = self.font.fit(&c.hint, Weight::Regular, 14, w / 2);
                    self.text_right(canvas, &hint, r.right() - 16, r.bottom() - 30, Weight::Regular, 14, WARN);
                }
            }
            let mut x = r.x() + 16;
            let y = r.bottom() - 34;
            x += self.keycap(canvas, "Y+←", "Cancel", x, y) + 18;
            self.keycap(canvas, "B", if c.auto { "Go limp" } else { "Cancel" }, x, y);
            return;
        }
        if m.mode == "sentry" {
            let h = 150;
            let r = Rect::new(left, LOGICAL_H as i32 - M - h, w as u32, h as u32);
            let following = snap.track.as_ref().filter(|t| t.active());
            self.card(canvas, r, if following.is_some() { "Sentry — tracking" } else { "Sentry — feet planted" });
            let status = match following.map(|t| t.state.as_str()) {
                Some("tracking") => {
                    let off = following.and_then(|t| t.err_deg).map(|e| e[0].hypot(e[1])).unwrap_or(0.0);
                    // m.tracking: gait_node is turning the body after it (not just the tracker running).
                    (format!("{} target · {off:.0}° off centre", if m.tracking { "Following" } else { "Tracking" }), GOOD)
                }
                Some("lost") => ("Target lost — holding, still looking".to_string(), WARN),
                Some(_) => ("Locking on…".to_string(), ACCENT),
                None if snap.track_pick.is_some() => ("Tap or drag on the picture to track something".to_string(), MUTED),
                None => ("Switch to the camera view (L2) to track something".to_string(), MUTED),
            };
            let st = self.font.fit(&status.0, Weight::Regular, 14, w / 2);
            self.text_right(canvas, &st, r.right() - 16, r.y() + 11, Weight::Regular, 14, status.1);
            let pose = m.pose.clone().unwrap_or_default();
            let col_w = (w - 32) / 3;
            for (i, (label, v, max)) in [("Roll", pose.roll, 30.0f32), ("Pitch", pose.pitch, 30.0), ("Yaw", pose.yaw, 30.0)].into_iter().enumerate() {
                let x = r.x() + 16 + i as i32 * col_w;
                self.text(canvas, label, x, r.y() + 40, Weight::Regular, 14, MUTED);
                self.text_right(canvas, &format!("{v:+.1}°"), x + col_w - 16, r.y() + 38, Weight::Bold, 18, TEXT);
                // centred bar
                let bw = col_w - 16;
                let bar = Rect::new(x, r.y() + 68, bw as u32, 8);
                self.fill_round(canvas, bar, 4, TRACK);
                let half = bw / 2;
                let fill = ((v / max).clamp(-1.0, 1.0) * half as f32) as i32;
                if fill != 0 {
                    let (fx, fw) = if fill > 0 { (x + half, fill) } else { (x + half + fill, -fill) };
                    self.fill_round(canvas, Rect::new(fx, r.y() + 68, fw as u32, 8), 4, ACCENT);
                }
                canvas.set_draw_color(MUTED);
                let _ = canvas.draw_line(Point::new(x + half, r.y() + 64), Point::new(x + half, r.y() + 80));
            }
            let mut x = r.x() + 16;
            let y = r.bottom() - 40;
            if following.is_some() {
                x += self.keycap(canvas, "L-stick ←→", "Roll", x, y) + 18;
                x += self.keycap(canvas, "X", "Stop tracking", x, y) + 18;
            } else {
                x += self.keycap(canvas, "L-stick", "Roll / pitch", x, y) + 18;
                x += self.keycap(canvas, "R-stick", "Yaw", x, y) + 18;
                if snap.track_pick.is_some() {
                    x += self.keycap(canvas, "D-pad+A", "Track", x, y) + 18;
                }
            }
            x += self.keycap(canvas, "R1/L1", "Height", x, y) + 18;
            self.keycap(canvas, "Y×2", "Walk", x, y);
            return;
        }

        let Some(c) = m.calib.clone() else { return };
        let h = 268 + 26;
        let r = Rect::new(left, LOGICAL_H as i32 - M - h, w as u32, h as u32);
        self.card(canvas, r, "Calibration — servo offsets");
        let (status, sc) = if c.unsaved {
            ("Unsaved — hold Y+↑ to save & exit".to_string(), WARN)
        } else if !c.note.is_empty() {
            (c.note.clone(), if c.note.starts_with("SAVE FAILED") { BAD } else { GOOD })
        } else {
            ("No changes yet".to_string(), MUTED)
        };
        let status = self.font.fit(&status, Weight::Regular, 14, w / 2);
        self.text_right(canvas, &status, r.right() - 16, r.y() + 11, Weight::Regular, 14, sc);

        let joints = ["hip", "thigh", "knee"];
        let label_w = 120;
        let col_w = (w - 32 - label_w) / 3;
        let (gx, gy) = (r.x() + 16, r.y() + 38);
        for (j, name) in ["Hip", "Thigh", "Knee"].iter().enumerate() {
            let cx = gx + label_w + j as i32 * col_w + col_w / 2;
            let color = if joints[j] == c.joint { ACCENT } else { MUTED };
            self.text_center(canvas, name, cx, gy, Weight::Bold, 13, color);
        }
        let row_h = 32;
        for (i, (leg, name)) in [("FL", "Front left"), ("FR", "Front right"), ("RL", "Rear left"), ("RR", "Rear right")].iter().enumerate() {
            let y = gy + 22 + i as i32 * row_h;
            let active = *leg == c.leg;
            if active {
                self.fill_round(canvas, Rect::new(gx - 6, y - 2, (w - 20) as u32, (row_h - 4) as u32), 8, SURFACE);
            }
            self.text(canvas, name, gx + 4, y + 4, if active { Weight::Bold } else { Weight::Regular }, 15, if active { TEXT } else { MUTED });
            let vals = c.offsets.get(*leg).copied().unwrap_or_default();
            for (j, v) in vals.iter().enumerate() {
                let cx = gx + label_w + j as i32 * col_w + col_w / 2;
                let selected = active && joints[j] == c.joint;
                let s = format!("{v:+.2}°");
                if selected {
                    let tw = self.font.width(&s, Weight::Bold, 16) + 20;
                    self.fill_round(canvas, Rect::new(cx - tw / 2, y - 1, tw as u32, (row_h - 6) as u32), 8, ACCENT);
                    self.text_center(canvas, &s, cx, y + 3, Weight::Bold, 16, ON_ACCENT);
                } else {
                    self.text_center(canvas, &s, cx, y + 3, Weight::Regular, 16, TEXT);
                }
            }
        }
        // Angle test line: what the selected joint is sent to, and what the servo gets.
        let ay = gy + 22 + 4 * row_h + 6;
        if c.angle_mode {
            let target = c.target.unwrap_or(0.0);
            let now = c.angle.unwrap_or(target);
            let moving = (now - target).abs() > 0.05;
            let tw = self.font.width("ANGLE TEST", Weight::Bold, 12) + 16;
            self.fill_round(canvas, Rect::new(gx - 2, ay, tw as u32, 22), 6, WARN);
            self.text(canvas, "ANGLE TEST", gx + 6, ay + 3, Weight::Bold, 12, ON_ACCENT);
            let s = format!("{} {} at {target:.1}°{}  ·  servo gets {:.1}°", c.leg, c.joint,
                            if moving { format!(" (moving, {now:.1}°)") } else { String::new() },
                            c.servo.unwrap_or(0.0));
            let s = self.font.fit(&s, Weight::Bold, 15, w - 48 - tw);
            self.text(canvas, &s, gx + tw + 10, ay + 2, Weight::Bold, 15, TEXT);
        } else {
            let s = match c.angle {
                Some(a) => format!("A: send {} {} to a set angle (now {a:.1}°), then fix its offset", c.leg, c.joint),
                None => "A: send the joint to a set angle, then fix its offset".to_string(),
            };
            let s = self.font.fit(&s, Weight::Regular, 14, w - 32);
            self.text(canvas, &s, gx, ay + 3, Weight::Regular, 14, MUTED);
        }
        let mut x = r.x() + 16;
        let y = r.bottom() - 40;
        if c.angle_mode {
            x += self.keycap(canvas, "R-stick ↑↓", "Angle ±5°", x, y) + 16;
            x += self.keycap(canvas, "←→", "±1°", x, y) + 16;
            x += self.keycap(canvas, "D-pad", "Offset", x, y) + 16;
            self.keycap(canvas, "A", "Back to pose", x, y);
            return;
        }
        x += self.keycap(canvas, "L1/R1", "Leg", x, y) + 16;
        x += self.keycap(canvas, "L2/R2", "Joint", x, y) + 16;
        x += self.keycap(canvas, "↑↓", "±1°", x, y) + 16;
        x += self.keycap(canvas, "←→", "Fine", x, y) + 16;
        x += self.keycap(canvas, "A", "Angle", x, y) + 16;
        self.keycap(canvas, "Y+↑ 3s", "Save", x, y);
    }

    // ── settings drawer ──────────────────────────────────────────────────────

    fn draw_settings(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot, menu: &Menu) {
        let r = Rect::new(LOGICAL_W as i32 - DRAWER_W as i32 - M, TOP_H + M, DRAWER_W, LOGICAL_H - (TOP_H + 2 * M) as u32);
        self.fill_round(canvas, r, RADIUS, Color::RGBA(15, 19, 27, 244));
        canvas.set_draw_color(HAIRLINE);
        let _ = canvas.draw_line(Point::new(r.x() + RADIUS, r.y()), Point::new(r.right() - RADIUS, r.y()));
        self.text(canvas, "Settings", r.x() + 24, r.y() + 18, Weight::Bold, 28, TEXT);

        let items = &snap.settings;
        let gait = snap.fresh_state().and_then(|s| s.gait.clone());
        let (sub, sc) = if items.is_empty() || snap.settings_age > 5.0 {
            ("Waiting for the robot…".to_string(), WARN)
        } else if gait.as_ref().map(|g| g.running).unwrap_or(false) {
            ("Live — changes apply instantly, until the stack restarts".to_string(), GOOD)
        } else {
            ("Gait node off — hold L1+R1 to start the stack".to_string(), WARN)
        };
        self.circle(canvas, r.x() + 29, r.y() + 66, 4, sc, false);
        self.text(canvas, &sub, r.x() + 40, r.y() + 58, Weight::Regular, 14, MUTED);

        // list area
        let list = Rect::new(r.x() + 8, r.y() + 90, r.width() - 16, r.height() - 90 - 124);
        const HEADER_H: i32 = 34;
        const ROW_H: i32 = 46;
        // layout: (y offset, a group header or an item index)
        enum Entry {
            Header(String),
            Item(usize),
        }
        let mut entries: Vec<(i32, Entry)> = Vec::new();
        let mut y = 0;
        let (mut sel_top, mut sel_y) = (0, 0);
        for (i, it) in items.iter().enumerate() {
            let new_group = i == 0 || it.group != items[i - 1].group;
            if i == menu.selected {
                sel_top = y; // includes the group header, so it scrolls into view too
            }
            if new_group {
                entries.push((y, Entry::Header(it.group.to_uppercase())));
                y += HEADER_H;
            }
            if i == menu.selected {
                sel_y = y;
            }
            entries.push((y, Entry::Item(i)));
            y += ROW_H;
        }
        let total_h = y;
        let view_h = list.height() as i32;
        let mut target = self.scroll;
        if (sel_top as f32) < target {
            target = sel_top as f32;
        }
        if (sel_y + ROW_H) as f32 > target + view_h as f32 {
            target = (sel_y + ROW_H - view_h) as f32;
        }
        target = target.clamp(0.0, (total_h - view_h).max(0) as f32);
        self.scroll += (target - self.scroll) * 0.35;
        if (target - self.scroll).abs() < 0.5 {
            self.scroll = target;
        }
        let scroll = self.scroll as i32;

        canvas.set_clip_rect(list);
        for (ey, entry) in &entries {
            let top = list.y() + ey - scroll;
            if top > list.bottom() || top + ROW_H < list.y() {
                continue;
            }
            match entry {
                Entry::Header(g) => {
                    self.text(canvas, g, list.x() + 16, top + 12, Weight::Bold, 12, ACCENT);
                }
                Entry::Item(i) => {
                    let row = Rect::new(list.x(), top, list.width(), ROW_H as u32 - 4);
                    self.draw_setting_row(canvas, &items[*i], *i == menu.selected, menu, row);
                }
            }
        }
        canvas.set_clip_rect(None);
        // scroll indicator
        if total_h > view_h {
            let bar_h = (view_h * view_h / total_h).max(30);
            let bar_y = list.y() + (scroll * (view_h - bar_h)) / (total_h - view_h).max(1);
            self.fill_round(canvas, Rect::new(r.right() - 7, bar_y, 3, bar_h as u32), 2, with_alpha(MUTED, 90));
        }

        // footer: help for the selected row + key hints
        let fy = r.bottom() - 120;
        canvas.set_draw_color(HAIRLINE);
        let _ = canvas.draw_line(Point::new(r.x() + 16, fy), Point::new(r.right() - 16, fy));
        if let Some(it) = items.get(menu.selected) {
            let mut help = it.help.clone();
            if let (Some(lo), Some(hi)) = (it.min, it.max) {
                let range = format!("Range {} – {}", format_value(it, &lo.into()), format_value(it, &hi.into()));
                help = if help.is_empty() { range } else { format!("{help}  ·  {range}") };
            }
            for (k, line) in self.font.wrap(&help, Weight::Regular, 14, r.width() as i32 - 48).iter().take(2).enumerate() {
                self.text(canvas, line, r.x() + 24, fy + 14 + k as i32 * 20, Weight::Regular, 14, MUTED);
            }
        }
        let mut x = r.x() + 24;
        let hy = r.bottom() - 40;
        for (k, l) in [("↑↓", "Select"), ("←→", "Adjust"), ("A", "Toggle"), ("☰", "Close")] {
            x += self.keycap(canvas, k, l, x, hy) + 20;
        }
    }

    fn draw_setting_row(&mut self, canvas: &mut Canvas<Window>, it: &SettingItem, selected: bool, menu: &Menu, r: Rect) {
        if selected {
            self.fill_round(canvas, r, 10, with_alpha(ACCENT, 34));
            self.fill_round(canvas, Rect::new(r.x(), r.y() + 10, 3, r.height() - 20), 2, ACCENT);
        }
        let avail = it.available;
        let label_c = if avail { TEXT } else { FAINT };
        let cy = r.y() + r.height() as i32 / 2;
        self.text(canvas, &it.label, r.x() + 18, cy - 10, if selected { Weight::Bold } else { Weight::Regular }, 16, label_c);

        let (value, pending) = menu.shown_value(it);
        let right = r.right() - 16;
        let value_c = if !avail {
            FAINT
        } else if pending {
            WARN
        } else {
            TEXT
        };
        match it.kind.as_str() {
            "action" => {
                // A red button; after the first A it asks for the second.
                let armed = menu.action_armed(&it.key);
                let label = if armed { "Press A again" } else { "A × 2" };
                let w = self.font.width(label, Weight::Bold, 14) + 28;
                let fill = if !avail { TRACK } else if armed { BAD } else { with_alpha(BAD, 70) };
                self.fill_round(canvas, Rect::new(right - w, cy - 14, w as u32, 28), 14, fill);
                self.text_center(canvas, label, right - w / 2, cy - 9, Weight::Bold, 14, if avail { TEXT } else { FAINT });
            }
            "bool" => {
                let on = value.as_bool().unwrap_or(false);
                let (sw, sh) = (46, 26);
                let sx = right - sw;
                let fill = if !avail { TRACK } else if on { ACCENT } else { TRACK };
                self.fill_round(canvas, Rect::new(sx, cy - sh / 2, sw as u32, sh as u32), 13, fill);
                let kx = if on { sx + sw - 13 } else { sx + 13 };
                self.circle(canvas, kx, cy, 9, if avail { Color::RGB(245, 248, 252) } else { MUTED }, false);
                let t = if value.is_null() { "—" } else if on { "On" } else { "Off" };
                self.text_right(canvas, t, sx - 10, cy - 9, Weight::Regular, 14, if pending { WARN } else { MUTED });
            }
            "enum" => {
                let cur = value.as_str().unwrap_or("").to_string();
                let mut x = right;
                for opt in it.options.iter().rev() {
                    let w = self.font.width(opt, Weight::Bold, 14) + 22;
                    x -= w;
                    let active = *opt == cur;
                    if active {
                        let fill = if !avail { TRACK } else if pending { WARN } else { ACCENT };
                        self.fill_round(canvas, Rect::new(x, cy - 14, w as u32, 28), 14, fill);
                    }
                    let c = if active && avail { ON_ACCENT } else if avail { MUTED } else { FAINT };
                    self.text_center(canvas, opt, x + w / 2, cy - 9, Weight::Bold, 14, c);
                    x -= 4;
                }
            }
            _ => {
                let s = if value.is_null() { "—".to_string() } else { format_value(it, &value) };
                let vw = 96;
                self.text_right(canvas, &s, right, cy - 10, Weight::Bold, 16, value_c);
                if selected && avail {
                    self.text(canvas, "‹", right - vw - 6, cy - 11, Weight::Bold, 18, ACCENT);
                    self.text_right(canvas, "›", right + 12, cy - 11, Weight::Bold, 18, ACCENT);
                }
                let (tw, tx) = (130, right - vw - 22 - 130);
                self.fill_round(canvas, Rect::new(tx, cy - 3, tw as u32, 6), 3, TRACK);
                if let Some(f) = fraction(it, &value) {
                    let fw = (f * tw as f32) as i32;
                    let c = if avail { if pending { WARN } else { ACCENT } } else { FAINT };
                    if fw > 3 {
                        self.fill_round(canvas, Rect::new(tx, cy - 3, fw as u32, 6), 3, c);
                    }
                    self.circle(canvas, tx + fw, cy, 7, if avail { Color::RGB(245, 248, 252) } else { MUTED }, false);
                }
            }
        }
    }

    // ── controls reference ───────────────────────────────────────────────────

    /// Full-screen cheat sheet (View button): every robot control and trick
    /// trigger. Mirrors gait_node / stack_launcher defaults (move_params.yaml).
    fn draw_controls_help(&mut self, canvas: &mut Canvas<Window>) {
        let r = Rect::new(M, TOP_H + M, LOGICAL_W - 2 * M as u32, LOGICAL_H - (TOP_H + 2 * M) as u32);
        self.fill_round(canvas, r, RADIUS, Color::RGBA(15, 19, 27, 250));
        canvas.set_draw_color(HAIRLINE);
        let _ = canvas.draw_line(Point::new(r.x() + RADIUS, r.y()), Point::new(r.right() - RADIUS, r.y()));
        self.text(canvas, "Controls", r.x() + 24, r.y() + 18, Weight::Bold, 28, TEXT);
        self.text(canvas, "Robot buttons work once the stack is running and the robot is armed.", r.x() + 24, r.y() + 58, Weight::Regular, 14, MUTED);

        // (keycap, what it does, extra detail)
        type Row = (&'static str, &'static str, &'static str);
        let columns: [&[(&str, &[Row])]; 3] = [
            &[
                ("Start up", &[
                    ("L1+R1", "Start / stop the robot stack", "hold 3 s"),
                    ("L3+R3", "Arm / disarm the servos", "hold 3 s — disarmed = limp"),
                    ("B", "Emergency stop", "goes limp; re-arm with L3+R3"),
                ]),
                ("Walking", &[
                    ("L stick", "Walk forward / back, strafe", ""),
                    ("R stick ←→", "Turn", ""),
                    ("R1", "Raise body", "hold"),
                    ("L1", "Lower body", "hold"),
                ]),
            ],
            &[
                ("Tricks — armed, walking", &[
                    ("X", "Butt shake", ""),
                    ("X ×2", "Dance", ""),
                    ("X ×3", "Push-ups", ""),
                    ("X + ↑↓←→", "Handshake / sit / bow / pee", "hold X, press a direction"),
                    ("X", "Stop the trick early", "while one is playing"),
                ]),
                ("Auto nav — armed", &[
                    ("A ×2", "Auto nav on / off", "the screen becomes a map"),
                    ("A / tap", "Walk to the cursor / the spot", "R stick moves the cursor"),
                    ("X", "Stop walking there", ""),
                    ("Y", "Set where the robot is", "A/tap its spot, then the way it faces"),
                    ("↑ / ↓", "Faster / slower", ""),
                    ("L1/R1 L2/R2", "Rotate / zoom the map", "← / → re-centres on the robot"),
                ]),
            ],
            &[
                ("Modes", &[
                    ("Y ×2", "Sentry: sticks tilt the body", "L tilt, R twist; Y ×2 again to leave"),
                    ("Tap / drag", "Sentry: track an object", "camera view; or D-pad + A; X stops"),
                    ("Y + ↑", "Servo calibration", "hold 3 s; L1/R1 leg, L2/R2 joint"),
                    ("A", "Calibration: angle test", "R stick sets the angle, D-pad the offset"),
                    ("Y + ↓", "Zero the IMU", "hold 1 s, standing on flat ground"),
                    ("Y + ←", "Calibrate IMU axes", "hold 1 s; follow the prompts"),
                ]),
                ("This app", &[
                    ("L2", "Next view", "camera, depth, lidar, map"),
                    ("☰", "Settings", "D-pad + A"),
                    ("View", "Show / hide this screen", ""),
                    ("X", "Wi-Fi setup over Bluetooth", "only while not connected"),
                ]),
            ],
        ];
        const GAP: i32 = 24;
        const KEY_W: i32 = 112;
        let col_w = (r.width() as i32 - 2 * 24 - 2 * GAP) / 3;
        for (c, sections) in columns.iter().enumerate() {
            let x0 = r.x() + 24 + c as i32 * (col_w + GAP);
            let mut y = r.y() + 96;
            for (title, rows) in sections.iter() {
                self.text(canvas, &title.to_uppercase(), x0, y, Weight::Bold, 12, ACCENT);
                y += 26;
                for (key, label, detail) in rows.iter() {
                    self.keycap(canvas, key, "", x0, y);
                    let lw = col_w - KEY_W;
                    let label = self.font.fit(label, Weight::Bold, 16, lw);
                    self.text(canvas, &label, x0 + KEY_W, y + 1, Weight::Bold, 16, TEXT);
                    y += 26;
                    if !detail.is_empty() {
                        let d = self.font.fit(detail, Weight::Regular, 14, lw);
                        self.text(canvas, &d, x0 + KEY_W, y - 2, Weight::Regular, 14, MUTED);
                        y += 20;
                    }
                    y += 8;
                }
                y += 14;
            }
        }

        let fy = r.bottom() - 46;
        canvas.set_draw_color(HAIRLINE);
        let _ = canvas.draw_line(Point::new(r.x() + 16, fy - 10), Point::new(r.right() - 16, fy - 10));
        self.text(canvas, "Trick buttons are the defaults (trick_taps / trick_dpad in move_params.yaml).", r.x() + 24, fy + 4, Weight::Regular, 14, FAINT);
        self.keycap(canvas, "View", "Close", r.right() - 120, fy);
    }

    // ── messages ─────────────────────────────────────────────────────────────

    /// `nav`: the auto-nav map is up, so camera problems don't matter.
    fn draw_overlay(&mut self, canvas: &mut Canvas<Window>, snap: &Snapshot, menu_open: bool, nav: bool) {
        let bt = &snap.provision;
        // Key + label drawn under the message, e.g. ("X", "Set up Wi-Fi over Bluetooth").
        let mut action: Option<(&str, &str)> = None;
        let message: Option<(String, String, Color)> = if bt.running {
            Some(("Wi-Fi setup over Bluetooth".into(), bt.step.clone(), ACCENT))
        } else if !snap.broker_connected {
            if let Some(err) = &bt.error {
                action = Some(("X", "Try again"));
                Some(("Bluetooth setup failed".into(), err.clone(), BAD))
            } else if let Some(ip) = &bt.host {
                Some((format!("Reconnecting to {ip}"), bt.step.clone(), ACCENT))
            } else {
                action = Some(("X", "Not on the same Wi-Fi? Set it up over Bluetooth"));
                let detail = if snap.last_error.is_empty() {
                    "Is the robot on and the MQTT broker running?".to_string()
                } else {
                    snap.last_error.clone()
                };
                Some((format!("Connecting to {}", snap.host), detail, WARN))
            }
        } else if snap.robot_online != Some(true) {
            Some(("Robot offline".into(), "Waiting for the robot's deck_bridge node…".into(), BAD))
        } else if nav {
            None
        } else if snap.view == View::Depth && !snap.depth_available() {
            let model = snap.camera().map(|c| c.model.clone()).unwrap_or_default();
            Some((
                "No depth on this camera".into(),
                format!("{model} is a color-only USB webcam — plug in the RealSense D405 for depth."),
                WARN,
            ))
        } else if snap.view.is_video() {
            let have = snap.frame.as_ref().map(|f| f.view == snap.view).unwrap_or(false);
            if !have || snap.frame_age > 2.0 {
                let detail = match snap.camera() {
                    Some(c) if c.kind.is_none() => "No camera is plugged into the robot.".to_string(),
                    Some(c) if !c.streaming => {
                        format!("{} is plugged in but its driver isn't running — hold L1+R1 to start the stack.", c.model)
                    }
                    _ => "No frames yet — is the camera running?".to_string(),
                };
                Some((format!("No {} video yet", snap.view.label().to_lowercase()), detail, WARN))
            } else {
                None
            }
        } else {
            None
        };
        let Some((title, detail, color)) = message else { return };
        let area_right = if menu_open { LOGICAL_W as i32 - DRAWER_W as i32 - M } else { LOGICAL_W as i32 };
        let area_left = 300 + M;
        let max_w = (area_right - area_left - 40).min(640);
        // Smaller title when the space is narrow (settings drawer open).
        let title_px = if self.font.width(&title, Weight::Bold, 28) > max_w - 64 { 22 } else { 28 };
        let title = self.font.fit(&title, Weight::Bold, title_px, max_w - 48);
        let lines = self.font.wrap(&detail, Weight::Regular, 16, max_w - 64);
        let w = (self.font.width(&title, Weight::Bold, title_px).max(lines.iter().map(|l| self.font.width(l, Weight::Regular, 16)).max().unwrap_or(0)) + 64).min(max_w);
        let w = match action {
            Some((k, l)) => w.max(self.font.width(k, Weight::Bold, 12) + self.font.width(l, Weight::Regular, 14) + 100).min(max_w),
            None => w,
        };
        let h = 96 + lines.len() as i32 * 22 + if action.is_some() { 40 } else { 0 };
        let cx = (area_left + area_right) / 2;
        let r = Rect::new(cx - w / 2, LOGICAL_H as i32 / 2 - h / 2, w as u32, h as u32);
        self.fill_round(canvas, r, 18, Color::RGBA(15, 19, 27, 240));
        self.fill_round(canvas, Rect::new(r.x(), r.y(), 4, r.height()), 2, color);
        self.text_center(canvas, &title, cx, r.y() + 26 + (28 - title_px as i32) / 2, Weight::Bold, title_px, color);
        for (i, line) in lines.iter().enumerate() {
            self.text_center(canvas, line, cx, r.y() + 70 + i as i32 * 22, Weight::Regular, 16, MUTED);
        }
        if let Some((key, label)) = action {
            let kw = self.font.width(key, Weight::Bold, 12).max(12) + 14 + 6 + self.font.width(label, Weight::Regular, 14);
            self.keycap(canvas, key, label, cx - kw / 2, r.bottom() - 46);
        }
    }

    fn draw_toast(&mut self, canvas: &mut Canvas<Window>, menu: &Menu, y: i32) {
        let Some(t) = menu.toast.as_ref().filter(|t| t.alive()) else { return };
        let a = (t.alpha() * 255.0) as u8;
        let (icon, color) = if t.ok { ("✓", GOOD) } else { ("✕", BAD) };
        let text = self.font.fit(&t.text, Weight::Regular, 16, 640);
        let w = self.font.width(&text, Weight::Regular, 16) + 64;
        let cx = if menu.open { (300 + M + LOGICAL_W as i32 - DRAWER_W as i32 - M) / 2 } else { LOGICAL_W as i32 / 2 };
        let r = Rect::new(cx - w / 2, y, w as u32, 42);
        self.fill_round(canvas, r, 21, Color::RGBA(22, 28, 38, (a as f32 * 0.96) as u8));
        self.circle(canvas, r.x() + 24, r.y() + 21, 11, with_alpha(color, a), false);
        self.text_center(canvas, icon, r.x() + 24, r.y() + 12, Weight::Bold, 14, Color::RGBA(10, 14, 20, a));
        self.text(canvas, &text, r.x() + 44, r.y() + 11, Weight::Regular, 16, with_alpha(TEXT, a));
    }
}

/// "butt_shake" -> "Butt shake".
fn trick_label(name: &str) -> String {
    let words = name.replace('_', " ");
    let mut chars = words.chars();
    match chars.next() {
        Some(c) => c.to_uppercase().chain(chars).collect(),
        None => "Trick".to_string(),
    }
}
