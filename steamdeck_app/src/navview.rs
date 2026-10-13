//! Auto-nav map view: the camera (pan / zoom / rotate), the cursor, and turning
//! controller input into goals. Shown full screen while gait_node is in
//! auto_nav mode (double-tap A on the robot side). Rendering is in ui.rs.
//!
//! Controls (Deck):
//!   right stick   move the cursor; pushing it against the screen edge pans
//!   A             walk to the cursor (sent once no second A follows — A×2
//!                 leaves auto nav on the robot, and must not set a goal too)
//!   tap screen    walk to that point straight away
//!   X             stop (cancel the goal)
//!   Y             place the robot ("it's here"): A / tap where it stands,
//!                 then A / tap the way it faces. Y or X again cancels.
//!   D-pad up/down speed up / down
//!   D-pad l/r     re-centre on the robot and follow it again
//!   L1 / R1       rotate the map (hold)
//!   L2 / R2       zoom out / in (analog)

use std::time::{Duration, Instant};

use crate::input::{Dir, InputState};
use crate::protocol::{BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_L1, BTN_R1};

/// Screen area the map fills: everything below the top bar (logical pixels).
pub const AREA_TOP: f32 = 56.0;
pub const SCREEN_W: f32 = 1280.0;
pub const SCREEN_H: f32 = 800.0;
/// Must be longer than the robot's double_tap_s (0.4 s), so the first A of
/// an A×2 never becomes a goal.
pub const A_CONFIRM_WAIT: Duration = Duration::from_millis(450);

const ZOOM_MIN: f32 = 15.0; // px per metre
const ZOOM_MAX: f32 = 600.0;
const ZOOM_RATE: f32 = 1.6; // e-folds per second at full trigger
const ROTATE_RATE: f32 = 1.6; // rad/s while L1/R1 held
const CURSOR_SPEED: f32 = 700.0; // px/s at full stick
const EDGE: f32 = 24.0; // cursor margin before panning
/// Placing the robot: the facing point must be at least this far from the spot (px).
pub const MIN_AIM_PX: f32 = 24.0;

/// Placing the robot on the map: first where it stands, then which way it faces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Place {
    /// Moving the cursor to where the robot stands.
    Spot,
    /// Standing here (map frame, m); the cursor shows which way it faces.
    Facing(f32, f32),
}

#[derive(Clone, Debug)]
pub struct NavCam {
    /// World point (map frame, m) at the centre of the view.
    pub centre: (f32, f32),
    /// Pixels per metre.
    pub zoom: f32,
    /// Map rotation on screen, radians counter-clockwise (0 = map +x to the right).
    pub rot: f32,
    /// Cursor, screen pixels.
    pub cursor: (f32, f32),
    /// Keep the robot in the centre (until the view is panned by hand).
    pub follow: bool,
    /// Some while placing the robot (Y).
    pub place: Option<Place>,
    pending_a: Option<Instant>,
}

impl Default for NavCam {
    fn default() -> Self {
        let (cx, cy) = Self::screen_centre();
        NavCam { centre: (0.0, 0.0), zoom: 90.0, rot: 0.0, cursor: (cx, cy), follow: true, place: None, pending_a: None }
    }
}

/// What the view wants sent to the robot this frame.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum NavCmd {
    Goal(f32, f32),
    Cancel,
    /// One speed level up (+1) or down (-1).
    Speed(i32),
    /// The robot is at (x, y) facing yaw (map frame, rad CCW from +x).
    Pose(f32, f32, f32),
}

impl NavCam {
    pub fn screen_centre() -> (f32, f32) {
        (SCREEN_W / 2.0, (AREA_TOP + SCREEN_H) / 2.0)
    }

    pub fn world_to_screen(&self, x: f32, y: f32) -> (f32, f32) {
        let (cx, cy) = Self::screen_centre();
        let (dx, dy) = (x - self.centre.0, y - self.centre.1);
        let (s, c) = self.rot.sin_cos();
        let (rx, ry) = (c * dx - s * dy, s * dx + c * dy);
        (cx + rx * self.zoom, cy - ry * self.zoom)
    }

    pub fn screen_to_world(&self, sx: f32, sy: f32) -> (f32, f32) {
        let (cx, cy) = Self::screen_centre();
        let (rx, ry) = ((sx - cx) / self.zoom, -(sy - cy) / self.zoom);
        let (s, c) = self.rot.sin_cos();
        (self.centre.0 + c * rx + s * ry, self.centre.1 - s * rx + c * ry)
    }

    /// A screen-space vector (pixels) as a world-space one (metres).
    fn screen_delta_to_world(&self, dx: f32, dy: f32) -> (f32, f32) {
        let (rx, ry) = (dx / self.zoom, -dy / self.zoom);
        let (s, c) = self.rot.sin_cos();
        (c * rx + s * ry, -s * rx + c * ry)
    }

    /// Per frame: sticks, triggers and shoulder buttons move the view.
    /// `robot` = the robot's map-frame position, if known.
    pub fn update(&mut self, dt: f32, input: &InputState, robot: Option<(f32, f32)>) {
        let dt = dt.clamp(0.0, 0.1);
        let b = &input.buttons;
        if b[BTN_L1] != b[BTN_R1] {
            self.rot += if b[BTN_L1] { ROTATE_RATE } else { -ROTATE_RATE } * dt;
            self.rot = self.rot.rem_euclid(std::f32::consts::TAU);
        }
        let zoom_in = input.axes[5] - input.axes[4];
        if zoom_in != 0.0 {
            // Zoom about the cursor, so what's under it stays put.
            let before = self.screen_to_world(self.cursor.0, self.cursor.1);
            self.zoom = (self.zoom * (ZOOM_RATE * zoom_in * dt).exp()).clamp(ZOOM_MIN, ZOOM_MAX);
            if !self.follow {
                let after = self.screen_to_world(self.cursor.0, self.cursor.1);
                self.centre.0 += before.0 - after.0;
                self.centre.1 += before.1 - after.1;
            }
        }
        if b[BTN_DPAD_LEFT] || b[BTN_DPAD_RIGHT] {
            self.follow = true;
            let (cx, cy) = Self::screen_centre();
            self.cursor = (cx, cy);
        }

        // Right stick (ROS convention: left / up positive) -> cursor.
        let (sx, sy) = (-input.axes[2], -input.axes[3]);
        if sx != 0.0 || sy != 0.0 {
            let mut nx = self.cursor.0 + sx * CURSOR_SPEED * dt;
            let mut ny = self.cursor.1 + sy * CURSOR_SPEED * dt;
            let (lo_x, hi_x) = (EDGE, SCREEN_W - EDGE);
            let (lo_y, hi_y) = (AREA_TOP + EDGE, SCREEN_H - EDGE);
            // Past the edge: keep the cursor there and slide the map instead.
            let over = (
                if nx < lo_x { nx - lo_x } else if nx > hi_x { nx - hi_x } else { 0.0 },
                if ny < lo_y { ny - lo_y } else if ny > hi_y { ny - hi_y } else { 0.0 },
            );
            nx = nx.clamp(lo_x, hi_x);
            ny = ny.clamp(lo_y, hi_y);
            if over != (0.0, 0.0) {
                self.follow = false;
                let (wx, wy) = self.screen_delta_to_world(over.0, over.1);
                self.centre.0 += wx;
                self.centre.1 += wy;
            }
            self.cursor = (nx, ny);
        }
        if self.follow {
            if let Some(r) = robot {
                self.centre = r;
            }
        }
    }

    /// A pressed. The goal is only sent from poll() once A_CONFIRM_WAIT has
    /// passed without a second press: a second A within that time is the
    /// robot's "leave auto nav" double-tap.
    pub fn press_a(&mut self, now: Instant) {
        self.pending_a = match self.pending_a {
            Some(t) if now.duration_since(t) < A_CONFIRM_WAIT => None,
            _ => Some(now),
        };
    }

    /// True while a single A is waiting to become a goal.
    pub fn a_pending(&self) -> bool {
        self.pending_a.is_some()
    }

    /// Screen tap (or click): move the cursor there and act on it straight away.
    pub fn tap(&mut self, sx: f32, sy: f32) -> Option<NavCmd> {
        if sy < AREA_TOP {
            return None;
        }
        self.cursor = (sx, sy);
        self.pending_a = None;
        self.confirm()
    }

    pub fn poll(&mut self, now: Instant) -> Option<NavCmd> {
        let t = self.pending_a?;
        if now.duration_since(t) < A_CONFIRM_WAIT {
            return None;
        }
        self.pending_a = None;
        self.confirm()
    }

    /// A (or a tap) at the cursor: a goal, or the next step of placing the robot.
    fn confirm(&mut self) -> Option<NavCmd> {
        let (x, y) = self.screen_to_world(self.cursor.0, self.cursor.1);
        match self.place {
            None => Some(NavCmd::Goal(x, y)),
            Some(Place::Spot) => {
                self.place = Some(Place::Facing(x, y));
                None
            }
            Some(Place::Facing(px, py)) => {
                let yaw = self.facing()?.2;
                self.place = None;
                Some(NavCmd::Pose(px, py, yaw))
            }
        }
    }

    /// While choosing which way the placed robot faces: (x, y, yaw toward the
    /// cursor), or None if the cursor is still on the spot.
    pub fn facing(&self) -> Option<(f32, f32, f32)> {
        let Some(Place::Facing(px, py)) = self.place else { return None };
        let (sx, sy) = self.world_to_screen(px, py);
        if (self.cursor.0 - sx).hypot(self.cursor.1 - sy) < MIN_AIM_PX {
            return None;
        }
        let (cx, cy) = self.screen_to_world(self.cursor.0, self.cursor.1);
        Some((px, py, (cy - py).atan2(cx - px)))
    }

    /// Y: start placing the robot, or give up on it.
    pub fn toggle_place(&mut self) {
        self.place = if self.place.is_some() { None } else { Some(Place::Spot) };
        self.pending_a = None;
    }

    /// X: cancels placing the robot if that's under way, otherwise stops it.
    pub fn press_x(&mut self) -> Option<NavCmd> {
        if self.place.take().is_some() {
            self.pending_a = None;
            return None;
        }
        Some(NavCmd::Cancel)
    }

    /// D-pad up / down: speed.
    pub fn dpad(&self, d: Dir) -> Option<NavCmd> {
        match d {
            Dir::Up => Some(NavCmd::Speed(1)),
            Dir::Down => Some(NavCmd::Speed(-1)),
            _ => None,
        }
    }

    /// Leaving auto nav: drop a half-finished A or placement.
    pub fn reset_pending(&mut self) {
        self.pending_a = None;
        self.place = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: (f32, f32), b: (f32, f32)) -> bool {
        (a.0 - b.0).abs() < 1e-3 && (a.1 - b.1).abs() < 1e-3
    }

    #[test]
    fn screen_and_world_round_trip_at_any_rotation() {
        let mut cam = NavCam { centre: (1.0, -2.0), zoom: 80.0, ..Default::default() };
        for rot in [0.0f32, 0.7, 2.5, 4.0] {
            cam.rot = rot;
            let s = cam.world_to_screen(3.0, 0.5);
            assert!(close(cam.screen_to_world(s.0, s.1), (3.0, 0.5)));
        }
    }

    #[test]
    fn unrotated_map_has_x_right_and_y_up() {
        let cam = NavCam::default();
        let (cx, cy) = NavCam::screen_centre();
        let east = cam.world_to_screen(1.0, 0.0);
        let north = cam.world_to_screen(0.0, 1.0);
        assert!(east.0 > cx && (east.1 - cy).abs() < 1e-3);
        assert!(north.1 < cy && (north.0 - cx).abs() < 1e-3);
        // rotated 90° counter-clockwise: east now points up the screen
        let cam = NavCam { rot: std::f32::consts::FRAC_PI_2, ..Default::default() };
        let east = cam.world_to_screen(1.0, 0.0);
        assert!(east.1 < cy && (east.0 - cx).abs() < 1e-3);
    }

    #[test]
    fn single_a_waits_double_a_never_sends() {
        let t0 = Instant::now();
        let mut cam = NavCam::default();
        cam.press_a(t0);
        assert_eq!(cam.poll(t0 + Duration::from_millis(200)), None);
        assert!(matches!(cam.poll(t0 + Duration::from_millis(500)), Some(NavCmd::Goal(..))));
        assert_eq!(cam.poll(t0 + Duration::from_millis(600)), None, "only once");

        cam.press_a(t0);
        cam.press_a(t0 + Duration::from_millis(250)); // A×2: leave auto nav
        assert_eq!(cam.poll(t0 + Duration::from_secs(2)), None);
    }

    #[test]
    fn triggers_zoom_and_shoulders_rotate() {
        let mut cam = NavCam::default();
        let mut input = InputState::default();
        input.axes[5] = 1.0; // R2: zoom in
        let z0 = cam.zoom;
        cam.update(0.1, &input, None);
        assert!(cam.zoom > z0);
        input.axes[5] = 0.0;
        input.axes[4] = 1.0; // L2: zoom out
        cam.update(0.1, &input, None);
        cam.update(0.1, &input, None);
        assert!(cam.zoom < z0);
        input.axes[4] = 0.0;
        input.buttons[BTN_L1] = true;
        cam.update(0.1, &input, None);
        assert!(cam.rot > 0.0);
        input.buttons[BTN_R1] = true; // both: no rotation (that's the stack chord)
        let r = cam.rot;
        cam.update(0.1, &input, None);
        assert_eq!(cam.rot, r);
    }

    #[test]
    fn cursor_pushing_the_edge_pans_and_stops_following() {
        let mut cam = NavCam::default();
        let mut input = InputState::default();
        input.axes[2] = -1.0; // right stick pushed right
        for _ in 0..20 {
            cam.update(0.1, &input, Some((0.0, 0.0)));
        }
        assert!(cam.cursor.0 <= SCREEN_W - EDGE + 1e-3);
        assert!(!cam.follow && cam.centre.0 > 0.5, "map slid right-to-left: centre moved +x");
        input.axes[2] = 0.0;
        input.buttons[BTN_DPAD_LEFT] = true; // re-centre
        cam.update(0.1, &input, Some((2.0, 3.0)));
        assert!(cam.follow && close(cam.centre, (2.0, 3.0)));
    }

    #[test]
    fn tap_goes_to_the_tapped_point() {
        let mut cam = NavCam { centre: (5.0, 5.0), ..Default::default() };
        let s = cam.world_to_screen(6.0, 4.0);
        match cam.tap(s.0, s.1) {
            Some(NavCmd::Goal(x, y)) => assert!(close((x, y), (6.0, 4.0))),
            other => panic!("{other:?}"),
        }
        assert_eq!(cam.tap(100.0, 10.0), None, "top bar isn't the map");
    }

    #[test]
    fn placing_the_robot_takes_a_spot_then_a_heading() {
        let t0 = Instant::now();
        let mut cam = NavCam { centre: (1.0, 1.0), ..Default::default() };
        cam.toggle_place();
        let spot = cam.world_to_screen(2.0, 1.0);
        assert_eq!(cam.tap(spot.0, spot.1), None, "first the spot");
        assert!(matches!(cam.place, Some(Place::Facing(x, y)) if close((x, y), (2.0, 1.0))));
        // A on the spot itself: no heading yet, still placing.
        cam.press_a(t0);
        assert_eq!(cam.poll(t0 + Duration::from_millis(500)), None);
        assert!(cam.place.is_some());
        // Cursor north of the spot (map +y): facing +90°.
        let north = cam.world_to_screen(2.0, 2.0);
        cam.cursor = north;
        cam.press_a(t0 + Duration::from_secs(1));
        match cam.poll(t0 + Duration::from_millis(1500)) {
            Some(NavCmd::Pose(x, y, yaw)) => {
                assert!(close((x, y), (2.0, 1.0)));
                assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-3);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(cam.place, None, "done");
    }

    #[test]
    fn x_cancels_placing_before_it_stops_the_robot() {
        let mut cam = NavCam::default();
        cam.toggle_place();
        assert_eq!(cam.press_x(), None);
        assert_eq!(cam.place, None);
        assert_eq!(cam.press_x(), Some(NavCmd::Cancel));
        cam.toggle_place();
        cam.toggle_place(); // Y twice: cancelled
        assert_eq!(cam.place, None);
    }

    #[test]
    fn dpad_up_down_is_speed() {
        let cam = NavCam::default();
        assert_eq!(cam.dpad(Dir::Up), Some(NavCmd::Speed(1)));
        assert_eq!(cam.dpad(Dir::Down), Some(NavCmd::Speed(-1)));
        assert_eq!(cam.dpad(Dir::Left), None);
    }
}
