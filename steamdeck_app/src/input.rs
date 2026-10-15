//! Steam Deck controller state, kept in the layout the robot expects
//! (see protocol.rs / protocol.py): 6 axes and 21 buttons.

use std::time::{Duration, Instant};

use sdl2::controller::{Axis, Button};
use sdl2::event::Event;

use crate::protocol::{BTN_L2, BTN_R2};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct InputState {
    /// left_x, left_y, right_x, right_y (ROS convention: left/up positive), l2, r2 (0..1)
    pub axes: [f32; 6],
    pub buttons: [bool; 21],
}

/// Things the UI should react to, beyond the raw input state.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Action {
    /// R2: next view.
    NextView,
    /// L2: previous view.
    PrevView,
    /// Start / menu button: open or close the settings menu.
    ToggleMenu,
    /// View (⧉) button: open or close the controls & tricks reference.
    ToggleControls,
    Nav(Dir),
    /// A: toggle / cycle the selected setting.
    Confirm,
    /// X: tricks on the robot; stop / revert / clear in auto nav, sentry and the Tune screen.
    /// (Wi-Fi setup over Bluetooth is a settings-menu row, so X can't start it by accident.)
    X,
    /// Y: in auto nav, start / cancel placing the robot on the map; back on the Tune screen.
    Place,
    /// Any back grip (L4 / R4 / L5 / R5): open or close the Tune screen.
    ToggleTune,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Dir {
    Up,
    Down,
    Left,
    Right,
}

/// Auto-repeat for a held D-pad direction, like a keyboard: one step on press,
/// then after DELAY a step every RATE until released.
#[derive(Default)]
pub struct Repeater {
    held: Option<(Dir, Instant)>,
}

impl Repeater {
    const DELAY: Duration = Duration::from_millis(380);
    const RATE: Duration = Duration::from_millis(85);

    pub fn press(&mut self, dir: Dir, now: Instant) {
        self.held = Some((dir, now + Self::DELAY));
    }
    pub fn release(&mut self, dir: Dir) {
        if matches!(self.held, Some((d, _)) if d == dir) {
            self.held = None;
        }
    }
    /// Repeats due by `now`.
    pub fn poll(&mut self, now: Instant) -> Option<Dir> {
        let (dir, next) = self.held?;
        if now < next {
            return None;
        }
        self.held = Some((dir, next.max(now - Self::RATE) + Self::RATE));
        Some(dir)
    }
}

fn dpad_dir(b: Button) -> Option<Dir> {
    Some(match b {
        Button::DPadUp => Dir::Up,
        Button::DPadDown => Dir::Down,
        Button::DPadLeft => Dir::Left,
        Button::DPadRight => Dir::Right,
        _ => return None,
    })
}

// A real Steam Deck stick rests at about 0.08 when untouched, so the deadzone has to clear that.
const STICK_DEADZONE: f32 = 0.10;
// Trigger hysteresis so a trigger resting near the threshold doesn't chatter.
const TRIGGER_PRESS: f32 = 0.6;
const TRIGGER_RELEASE: f32 = 0.3;

#[derive(Default)]
pub struct Gamepad {
    pub state: InputState,
    pub repeat: Repeater,
}

fn stick(raw: i16) -> f32 {
    let f = raw as f32 / 32767.0;
    if f.abs() < STICK_DEADZONE {
        0.0
    } else {
        f.clamp(-1.0, 1.0)
    }
}

fn button_index(b: Button) -> Option<usize> {
    Some(match b {
        Button::A => 0,
        Button::B => 1,
        Button::X => 2,
        Button::Y => 3,
        Button::LeftShoulder => 4,
        Button::RightShoulder => 5,
        Button::Back => 6,
        Button::Start => 7,
        Button::Guide => 8,
        Button::LeftStick => 9,
        Button::RightStick => 10,
        Button::DPadUp => 11,
        Button::DPadDown => 12,
        Button::DPadLeft => 13,
        Button::DPadRight => 14,
        Button::Paddle1 => 15,
        Button::Paddle2 => 16,
        Button::Paddle3 => 17,
        Button::Paddle4 => 18,
        _ => return None,
    })
}

impl Gamepad {
    /// Feed an SDL event; returns an action if it should change the UI.
    pub fn handle(&mut self, event: &Event) -> Option<Action> {
        match *event {
            Event::ControllerAxisMotion { axis, value, .. } => self.axis(axis, value),
            Event::ControllerButtonDown { button, .. } => {
                if let Some(i) = button_index(button) {
                    self.state.buttons[i] = true;
                }
                match button {
                    Button::Start => Some(Action::ToggleMenu),
                    Button::Back => Some(Action::ToggleControls),
                    Button::A => Some(Action::Confirm),
                    Button::X => Some(Action::X),
                    Button::Y => Some(Action::Place),
                    Button::Paddle1 | Button::Paddle2 | Button::Paddle3 | Button::Paddle4 => Some(Action::ToggleTune),
                    b => dpad_dir(b).map(|d| {
                        self.repeat.press(d, Instant::now());
                        Action::Nav(d)
                    }),
                }
            }
            Event::ControllerButtonUp { button, .. } => {
                if let Some(i) = button_index(button) {
                    self.state.buttons[i] = false;
                }
                if let Some(d) = dpad_dir(button) {
                    self.repeat.release(d);
                }
                None
            }
            _ => None,
        }
    }

    fn axis(&mut self, axis: Axis, value: i16) -> Option<Action> {
        // SDL: left and up are negative. ROS joy convention: left and up positive.
        // (0.0 - x rather than -x, so an idle stick is +0.0 and not -0.0.)
        match axis {
            Axis::LeftX => self.state.axes[0] = 0.0 - stick(value),
            Axis::LeftY => self.state.axes[1] = 0.0 - stick(value),
            Axis::RightX => self.state.axes[2] = 0.0 - stick(value),
            Axis::RightY => self.state.axes[3] = 0.0 - stick(value),
            Axis::TriggerLeft => {
                let f = (value as f32 / 32767.0).clamp(0.0, 1.0);
                self.state.axes[4] = f;
                let was_down = self.state.buttons[BTN_L2];
                if !was_down && f > TRIGGER_PRESS {
                    self.state.buttons[BTN_L2] = true;
                    return Some(Action::PrevView);
                } else if was_down && f < TRIGGER_RELEASE {
                    self.state.buttons[BTN_L2] = false;
                }
            }
            Axis::TriggerRight => {
                let f = (value as f32 / 32767.0).clamp(0.0, 1.0);
                self.state.axes[5] = f;
                let was_down = self.state.buttons[BTN_R2];
                if !was_down && f > TRIGGER_PRESS {
                    self.state.buttons[BTN_R2] = true;
                    return Some(Action::NextView);
                } else if was_down && f < TRIGGER_RELEASE {
                    self.state.buttons[BTN_R2] = false;
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_event(axis: Axis, value: i16) -> Event {
        Event::ControllerAxisMotion { timestamp: 0, which: 0, axis, value }
    }

    #[test]
    fn r2_press_changes_view_once_per_pull() {
        let mut pad = Gamepad::default();
        assert_eq!(pad.handle(&axis_event(Axis::TriggerRight, 10_000)), None); // light touch
        assert_eq!(pad.handle(&axis_event(Axis::TriggerRight, 25_000)), Some(Action::NextView));
        // still held / wobbling above the release threshold: no repeat
        assert_eq!(pad.handle(&axis_event(Axis::TriggerRight, 32_767)), None);
        assert_eq!(pad.handle(&axis_event(Axis::TriggerRight, 15_000)), None);
        assert!(pad.state.buttons[BTN_R2]);
        // release, then a second pull switches again
        assert_eq!(pad.handle(&axis_event(Axis::TriggerRight, 0)), None);
        assert!(!pad.state.buttons[BTN_R2]);
        assert_eq!(pad.handle(&axis_event(Axis::TriggerRight, 32_767)), Some(Action::NextView));
    }

    #[test]
    fn l2_press_goes_back_a_view() {
        let mut pad = Gamepad::default();
        assert_eq!(pad.handle(&axis_event(Axis::TriggerLeft, 25_000)), Some(Action::PrevView));
        assert_eq!(pad.handle(&axis_event(Axis::TriggerLeft, 32_767)), None);
        assert!(pad.state.buttons[BTN_L2]);
        assert_eq!(pad.handle(&axis_event(Axis::TriggerLeft, 0)), None);
        assert!(!pad.state.buttons[BTN_L2]);
    }

    #[test]
    fn sticks_use_ros_sign_convention_and_deadzone() {
        let mut pad = Gamepad::default();
        pad.handle(&axis_event(Axis::LeftX, -32_767)); // pushed left
        pad.handle(&axis_event(Axis::LeftY, -32_767)); // pushed up
        assert!((pad.state.axes[0] - 1.0).abs() < 1e-3, "left must be positive");
        assert!((pad.state.axes[1] - 1.0).abs() < 1e-3, "up must be positive");
        pad.handle(&axis_event(Axis::RightX, 1_000)); // inside the deadzone
        assert_eq!(pad.state.axes[2], 0.0);
        assert!(pad.state.axes[2].is_sign_positive(), "idle stick must be +0.0, not -0.0");
        pad.handle(&axis_event(Axis::RightY, (0.0806 * 32767.0) as i16)); // resting Deck-stick noise
        assert_eq!(pad.state.axes[3], 0.0, "a real Deck stick's resting noise must be inside the deadzone");
    }

    #[test]
    fn buttons_map_to_protocol_slots() {
        let mut pad = Gamepad::default();
        pad.handle(&Event::ControllerButtonDown { timestamp: 0, which: 0, button: Button::A });
        pad.handle(&Event::ControllerButtonDown { timestamp: 0, which: 0, button: Button::DPadUp });
        assert!(pad.state.buttons[0] && pad.state.buttons[11]);
        pad.handle(&Event::ControllerButtonUp { timestamp: 0, which: 0, button: Button::A });
        assert!(!pad.state.buttons[0]);
    }

    #[test]
    fn start_dpad_and_a_produce_menu_actions() {
        let mut pad = Gamepad::default();
        let down = |b| Event::ControllerButtonDown { timestamp: 0, which: 0, button: b };
        assert_eq!(pad.handle(&down(Button::Start)), Some(Action::ToggleMenu));
        assert_eq!(pad.handle(&down(Button::Back)), Some(Action::ToggleControls));
        assert_eq!(pad.handle(&down(Button::DPadDown)), Some(Action::Nav(Dir::Down)));
        assert_eq!(pad.handle(&down(Button::A)), Some(Action::Confirm));
        assert!(pad.state.buttons[7] && pad.state.buttons[12], "still forwarded to the robot");
        for b in [Button::Paddle1, Button::Paddle2, Button::Paddle3, Button::Paddle4] {
            assert_eq!(pad.handle(&down(b)), Some(Action::ToggleTune));
        }
    }

    #[test]
    fn held_dpad_repeats_after_a_delay() {
        let t0 = Instant::now();
        let mut r = Repeater::default();
        r.press(Dir::Right, t0);
        assert_eq!(r.poll(t0 + Duration::from_millis(100)), None);
        assert_eq!(r.poll(t0 + Duration::from_millis(400)), Some(Dir::Right));
        assert_eq!(r.poll(t0 + Duration::from_millis(420)), None);
        assert_eq!(r.poll(t0 + Duration::from_millis(480)), Some(Dir::Right));
        r.release(Dir::Left); // a different direction doesn't stop it
        assert_eq!(r.poll(t0 + Duration::from_millis(600)), Some(Dir::Right));
        r.release(Dir::Right);
        assert_eq!(r.poll(t0 + Duration::from_millis(900)), None);
    }

    #[test]
    fn view_cycle_visits_all_views() {
        let mut v = crate::shared::View::Color;
        let mut seen = vec![v];
        for _ in 0..4 {
            v = v.next();
            seen.push(v);
        }
        assert_eq!(seen[4], seen[0]);
        assert_eq!(seen.len(), 5);
        for v in crate::shared::View::ALL {
            assert_eq!(v.next().prev(), v);
        }
    }
}
