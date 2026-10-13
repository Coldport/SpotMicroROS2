//! Sentry object tracking: picking a target on the camera view. The robot's
//! track_node follows it and gait_node turns the body to keep it centred
//! (see spot_micro_move/track_node.py). Rendering is in ui.rs.
//!
//! Controls (Deck, sentry mode, camera view):
//!   touch + drag  draw a box round the target (a tap = a box of the reticle's size)
//!   D-pad         move the reticle
//!   A             track what's in the reticle
//!   X             stop tracking
//!
//! Boxes go to the robot normalised to the camera image: [x, y, w, h], 0..1,
//! x/y the top-left corner, +y down.

use sdl2::rect::Rect;

use crate::input::Dir;
use crate::ui::{LOGICAL_H, LOGICAL_W};

/// Reticle size, as a fraction of the image height (it is drawn square).
const RETICLE: f32 = 0.22;
/// D-pad step, as a fraction of the image.
const STEP: f32 = 0.04;
/// A drag shorter than this (logical px) is a tap.
const TAP_PX: f32 = 18.0;

/// Where a frame of this size is drawn on screen: scaled to fit, centred.
pub fn video_rect(fw: u32, fh: u32) -> Rect {
    let scale = (LOGICAL_W as f32 / fw as f32).min(LOGICAL_H as f32 / fh as f32);
    let (dw, dh) = ((fw as f32 * scale) as u32, (fh as f32 * scale) as u32);
    Rect::new(((LOGICAL_W - dw) / 2) as i32, ((LOGICAL_H - dh) / 2) as i32, dw, dh)
}

/// Screen point -> normalised image point, if it's on the picture.
fn to_image(r: Rect, x: f32, y: f32) -> Option<(f32, f32)> {
    let u = (x - r.x() as f32) / r.width() as f32;
    let v = (y - r.y() as f32) / r.height() as f32;
    ((0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v)).then_some((u, v))
}

/// Normalised box -> screen rect for drawing.
pub fn to_screen(r: Rect, b: [f32; 4]) -> Rect {
    let x = r.x() as f32 + b[0] * r.width() as f32;
    let y = r.y() as f32 + b[1] * r.height() as f32;
    Rect::new(x as i32, y as i32, ((b[2] * r.width() as f32) as u32).max(2), ((b[3] * r.height() as f32) as u32).max(2))
}

fn clamp_box(b: [f32; 4]) -> [f32; 4] {
    let w = b[2].clamp(0.01, 1.0);
    let h = b[3].clamp(0.01, 1.0);
    [b[0].clamp(0.0, 1.0 - w), b[1].clamp(0.0, 1.0 - h), w, h]
}

#[derive(Clone, Debug)]
pub struct TrackPick {
    /// Reticle centre, normalised.
    pub cursor: (f32, f32),
    /// Drag in progress: start and current point, normalised.
    drag: Option<((f32, f32), (f32, f32))>,
    /// Screen position where the drag started (to tell a tap from a drag).
    drag_px: (f32, f32),
}

impl Default for TrackPick {
    fn default() -> Self {
        TrackPick { cursor: (0.5, 0.5), drag: None, drag_px: (0.0, 0.0) }
    }
}

impl TrackPick {
    /// Reticle box for a frame of this aspect (w / h).
    pub fn reticle(&self, aspect: f32) -> [f32; 4] {
        let (h, w) = (RETICLE, RETICLE / aspect.max(0.1));
        clamp_box([self.cursor.0 - w / 2.0, self.cursor.1 - h / 2.0, w, h])
    }

    pub fn nudge(&mut self, d: Dir) {
        let (dx, dy) = match d {
            Dir::Up => (0.0, -STEP),
            Dir::Down => (0.0, STEP),
            Dir::Left => (-STEP, 0.0),
            Dir::Right => (STEP, 0.0),
        };
        self.cursor = ((self.cursor.0 + dx).clamp(0.0, 1.0), (self.cursor.1 + dy).clamp(0.0, 1.0));
    }

    /// The box being dragged out, normalised (None when not dragging).
    pub fn drag_box(&self) -> Option<[f32; 4]> {
        let ((x0, y0), (x1, y1)) = self.drag?;
        Some([x0.min(x1), y0.min(y1), (x1 - x0).abs(), (y1 - y0).abs()])
    }

    pub fn press(&mut self, r: Rect, x: f32, y: f32) {
        self.drag = to_image(r, x, y).map(|p| (p, p));
        self.drag_px = (x, y);
    }

    pub fn motion(&mut self, r: Rect, x: f32, y: f32) {
        if let Some((start, _)) = self.drag {
            let u = ((x - r.x() as f32) / r.width() as f32).clamp(0.0, 1.0);
            let v = ((y - r.y() as f32) / r.height() as f32).clamp(0.0, 1.0);
            self.drag = Some((start, (u, v)));
        }
    }

    /// Finger up: the box to track, if the press started on the picture.
    /// A tap tracks a reticle-sized box there (and moves the reticle to it).
    pub fn release(&mut self, r: Rect, x: f32, y: f32) -> Option<[f32; 4]> {
        self.motion(r, x, y);
        let b = self.drag_box()?;
        let (start, _) = self.drag.take()?;
        let moved = (x - self.drag_px.0).hypot(y - self.drag_px.1);
        if moved < TAP_PX {
            self.cursor = start;
            return Some(self.reticle(r.width() as f32 / r.height() as f32));
        }
        (b[2] > 0.02 && b[3] > 0.02).then(|| clamp_box(b))
    }

    pub fn cancel_drag(&mut self) {
        self.drag = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn video_rect_letterboxes_4_3() {
        let r = video_rect(640, 480);
        assert_eq!((r.width(), r.height()), (1066, 800));
        assert_eq!(r.x(), 107);
    }

    #[test]
    fn drag_gives_a_normalised_box() {
        let r = video_rect(640, 480);
        let mut t = TrackPick::default();
        let (x0, y0) = (r.x() as f32 + 0.25 * r.width() as f32, 0.5 * 800.0);
        t.press(r, x0, y0);
        t.motion(r, x0 + 100.0, y0 + 50.0);
        let b = t.release(r, x0 + 0.25 * r.width() as f32, y0 + 0.25 * 800.0).unwrap();
        assert!((b[0] - 0.25).abs() < 1e-3 && (b[1] - 0.5).abs() < 1e-3);
        assert!((b[2] - 0.25).abs() < 1e-3 && (b[3] - 0.25).abs() < 1e-3);
        assert!(t.drag_box().is_none());
    }

    #[test]
    fn tap_tracks_a_reticle_box_there() {
        let r = video_rect(640, 480);
        let mut t = TrackPick::default();
        let (x, y) = (r.x() as f32 + 0.5 * r.width() as f32, 0.5 * 800.0);
        t.press(r, x, y);
        let b = t.release(r, x + 3.0, y).unwrap();
        assert!((b[0] + b[2] / 2.0 - 0.5).abs() < 0.01 && (b[3] - RETICLE).abs() < 1e-6);
        // Square on screen: w / h = 1 / aspect.
        assert!((b[2] * 4.0 / 3.0 - b[3]).abs() < 1e-3);
    }

    #[test]
    fn press_off_the_picture_does_nothing() {
        let r = video_rect(640, 480);
        let mut t = TrackPick::default();
        t.press(r, 10.0, 400.0); // in the left letterbox bar
        assert!(t.release(r, 300.0, 500.0).is_none());
    }

    #[test]
    fn reticle_stays_inside() {
        let mut t = TrackPick::default();
        for _ in 0..40 {
            t.nudge(Dir::Right);
            t.nudge(Dir::Up);
        }
        let b = t.reticle(4.0 / 3.0);
        assert!(b[0] + b[2] <= 1.0 + 1e-6 && b[1] >= 0.0);
    }
}
