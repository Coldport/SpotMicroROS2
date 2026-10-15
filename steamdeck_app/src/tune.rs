//! The Tune screen (a back grip button, L4/R4/L5/R5): autotune and two-point
//! servo calibration, both run by the robot's gait_node (modes "autotune" /
//! "servo_cal") and driven from here with <p>/deck/tune commands. This module
//! is the screen's state and button handling; ui.rs draws it.
//!
//!   Home    pick Auto-tune or Servo calibration (D-pad, A); Y closes.
//!   Auto    A starts / stops a run; afterwards A keeps (saves) the changes,
//!           X reverts them; Y back.
//!   Servo   grid of legs x joints: D-pad picks a joint (it moves to its test
//!           angle), A works on it, X clears its points, Y back (saves).
//!           On a joint: ↑↓ move it, ←→ set the angle it REALLY sits at,
//!           A records a point, X applies the fit from two points, Y back.
//!
//! While the screen is open the robot only gets B (estop), L3/R3 (arm) and
//! L1/R1 (stack / height) — and the sticks during a run, so touching one stops it.

use serde_json::{json, Value};

use crate::input::{Action, Dir};
use crate::shared::{ServoCalInfo, TuneReport};

pub const LEGS: [&str; 4] = ["FL", "FR", "RL", "RR"];
pub const JOINTS: [&str; 3] = ["hip", "thigh", "knee"];
/// D-pad steps on a joint: move it, and the "really at" reading.
pub const JOG_DEG: f32 = 1.0;
pub const ACTUAL_STEP_DEG: f32 = 0.5;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub enum Page {
    #[default]
    Home,
    Auto,
    Servo,
}

#[derive(Default, Clone)]
pub struct TuneScreen {
    pub open: bool,
    pub page: Page,
    /// Home: 0 = auto-tune, 1 = servo calibration.
    pub home_sel: usize,
    /// Servo page: working on the selected joint (else picking one in the grid).
    pub on_joint: bool,
    pub leg: usize,
    pub joint: usize,
    /// "Really at" = the test angle + this (deg); reset after each recorded point.
    pub actual_delta: f32,
}

/// What the screen needs from the robot's state.
#[derive(Default, Clone)]
pub struct TuneCtx {
    pub mode: String,
    pub armed: bool,
    pub tune: Option<TuneReport>,
    pub servo: Option<ServoCalInfo>,
}

impl TuneCtx {
    pub fn running(&self) -> bool {
        self.mode == "autotune" || self.tune.as_ref().is_some_and(|t| t.running())
    }
}

fn cmd(name: &str) -> Value {
    json!({ "cmd": name })
}

impl TuneScreen {
    /// Open / close; leaving servo calibration saves it on the robot.
    pub fn toggle(&mut self, ctx: &TuneCtx) -> Vec<Value> {
        self.open = !self.open;
        if self.open {
            self.page = Page::Home;
            return vec![];
        }
        self.leave_servo(ctx)
    }

    fn leave_servo(&mut self, ctx: &TuneCtx) -> Vec<Value> {
        self.on_joint = false;
        if ctx.mode == "servo_cal" {
            vec![cmd("servo_stop")]
        } else {
            vec![]
        }
    }

    fn select(&self) -> Value {
        json!({ "cmd": "servo_select", "leg": LEGS[self.leg], "joint": JOINTS[self.joint] })
    }

    /// "Really at" reading for the selected joint (deg), once the robot reports its test angle.
    pub fn actual(&self, ctx: &TuneCtx) -> Option<f32> {
        ctx.servo.as_ref().and_then(|s| s.target).map(|t| t + self.actual_delta)
    }

    /// A button / D-pad press while the screen is open -> commands for the robot.
    pub fn handle(&mut self, action: Action, ctx: &TuneCtx) -> Vec<Value> {
        match self.page {
            Page::Home => match action {
                Action::Nav(Dir::Up | Dir::Left) => {
                    self.home_sel = 0;
                    vec![]
                }
                Action::Nav(Dir::Down | Dir::Right) => {
                    self.home_sel = 1;
                    vec![]
                }
                Action::Confirm if self.home_sel == 0 => {
                    self.page = Page::Auto;
                    vec![]
                }
                Action::Confirm => {
                    self.page = Page::Servo;
                    self.on_joint = false;
                    self.actual_delta = 0.0;
                    vec![cmd("servo_start"), self.select()]
                }
                Action::Place => {
                    self.open = false;
                    vec![]
                }
                _ => vec![],
            },
            Page::Auto => {
                let report = ctx.tune.as_ref();
                match action {
                    Action::Confirm if ctx.running() => vec![cmd("autotune_stop")],
                    Action::Confirm if report.is_some_and(|t| t.undecided()) => vec![cmd("autotune_keep")],
                    Action::Confirm => vec![cmd("autotune_start")],
                    Action::X if report.is_some_and(|t| t.undecided()) => vec![cmd("autotune_revert")],
                    Action::Place => {
                        self.page = Page::Home;
                        vec![]
                    }
                    _ => vec![],
                }
            }
            Page::Servo => self.handle_servo(action, ctx),
        }
    }

    fn handle_servo(&mut self, action: Action, ctx: &TuneCtx) -> Vec<Value> {
        if action == Action::Place {
            if self.on_joint {
                self.on_joint = false;
                return vec![];
            }
            self.page = Page::Home;
            return self.leave_servo(ctx);
        }
        if ctx.mode != "servo_cal" {
            // Not (yet) in the mode — e.g. the robot wasn't armed when the page opened.
            return if action == Action::Confirm { vec![cmd("servo_start"), self.select()] } else { vec![] };
        }
        if !self.on_joint {
            return match action {
                Action::Nav(d) => {
                    match d {
                        Dir::Up => self.leg = (self.leg + LEGS.len() - 1) % LEGS.len(),
                        Dir::Down => self.leg = (self.leg + 1) % LEGS.len(),
                        Dir::Left => self.joint = (self.joint + JOINTS.len() - 1) % JOINTS.len(),
                        Dir::Right => self.joint = (self.joint + 1) % JOINTS.len(),
                    }
                    self.actual_delta = 0.0;
                    vec![self.select()]
                }
                Action::Confirm => {
                    self.on_joint = true;
                    vec![]
                }
                Action::X => vec![cmd("servo_clear")],
                _ => vec![],
            };
        }
        match action {
            Action::Nav(Dir::Up) => vec![json!({ "cmd": "servo_jog", "delta": JOG_DEG })],
            Action::Nav(Dir::Down) => vec![json!({ "cmd": "servo_jog", "delta": -JOG_DEG })],
            Action::Nav(Dir::Right) => {
                self.actual_delta += ACTUAL_STEP_DEG;
                vec![]
            }
            Action::Nav(Dir::Left) => {
                self.actual_delta -= ACTUAL_STEP_DEG;
                vec![]
            }
            Action::Confirm => match self.actual(ctx) {
                Some(a) if !ctx.servo.as_ref().is_some_and(|s| s.moving) => {
                    self.actual_delta = 0.0;
                    vec![json!({ "cmd": "servo_record", "actual": (a * 100.0).round() / 100.0 })]
                }
                _ => vec![],
            },
            Action::X => vec![cmd("servo_apply")],
            _ => vec![],
        }
    }
}

/// What the joint's angle means, for the "really at" reading.
pub fn joint_hint(joint: &str) -> &'static str {
    match joint {
        "hip" => "90° = leg hangs straight down; less = foot further out from the body.",
        "thigh" => "Upper leg from straight down: 0° = pointing down, 90° = level, toward the knee.",
        _ => "Inside angle at the knee, between upper and lower leg: 180° = leg straight.",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx(mode: &str) -> TuneCtx {
        TuneCtx { mode: mode.into(), armed: true, ..Default::default() }
    }

    #[test]
    fn home_opens_pages_and_servo_starts_the_mode() {
        let mut t = TuneScreen::default();
        assert!(t.toggle(&ctx("walk")).is_empty() && t.open);
        assert_eq!(t.handle(Action::Confirm, &ctx("walk")), Vec::<Value>::new());
        assert_eq!(t.page, Page::Auto);
        t.handle(Action::Place, &ctx("walk"));
        t.handle(Action::Nav(Dir::Down), &ctx("walk"));
        let out = t.handle(Action::Confirm, &ctx("walk"));
        assert_eq!(out[0], json!({"cmd": "servo_start"}));
        assert_eq!(out[1], json!({"cmd": "servo_select", "leg": "FL", "joint": "hip"}));
        // closing the screen from the servo page saves (servo_stop)
        assert_eq!(t.toggle(&ctx("servo_cal")), vec![json!({"cmd": "servo_stop"})]);
    }

    #[test]
    fn auto_page_start_stop_keep_revert() {
        let mut t = TuneScreen { open: true, page: Page::Auto, ..Default::default() };
        assert_eq!(t.handle(Action::Confirm, &ctx("walk")), vec![json!({"cmd": "autotune_start"})]);
        assert_eq!(t.handle(Action::Confirm, &ctx("autotune")), vec![json!({"cmd": "autotune_stop"})]);
        let mut done = ctx("walk");
        done.tune = Some(TuneReport {
            state: "done".into(),
            changes: vec![crate::shared::TuneChange { key: "odom_scale".into(), ..Default::default() }],
            ..Default::default()
        });
        assert_eq!(t.handle(Action::Confirm, &done), vec![json!({"cmd": "autotune_keep"})]);
        assert_eq!(t.handle(Action::X, &done), vec![json!({"cmd": "autotune_revert"})]);
        done.tune.as_mut().unwrap().outcome = "kept".into();
        assert_eq!(t.handle(Action::X, &done), Vec::<Value>::new());
        assert_eq!(t.handle(Action::Confirm, &done), vec![json!({"cmd": "autotune_start"})], "a new run");
    }

    #[test]
    fn servo_grid_and_joint_controls() {
        let mut t = TuneScreen { open: true, page: Page::Servo, ..Default::default() };
        let mut c = ctx("servo_cal");
        assert_eq!(t.handle(Action::Nav(Dir::Up), &c), vec![json!({"cmd": "servo_select", "leg": "RR", "joint": "hip"})]);
        assert_eq!(t.handle(Action::Nav(Dir::Left), &c)[0]["joint"], "knee");
        t.handle(Action::Confirm, &c);
        assert!(t.on_joint);
        assert_eq!(t.handle(Action::Nav(Dir::Up), &c), vec![json!({"cmd": "servo_jog", "delta": 1.0})]);
        t.handle(Action::Nav(Dir::Left), &c);
        t.handle(Action::Nav(Dir::Left), &c);
        // no test angle from the robot yet: nothing to record
        assert_eq!(t.handle(Action::Confirm, &c), Vec::<Value>::new());
        c.servo = Some(ServoCalInfo { leg: "RR".into(), joint: "knee".into(), target: Some(60.0), ..Default::default() });
        assert_eq!(t.handle(Action::Confirm, &c), vec![json!({"cmd": "servo_record", "actual": 59.0})]);
        assert_eq!(t.actual_delta, 0.0, "reading resets after a point");
        assert_eq!(t.handle(Action::X, &c), vec![json!({"cmd": "servo_apply"})]);
        t.handle(Action::Place, &c);
        assert!(!t.on_joint);
        assert_eq!(t.handle(Action::Place, &c), vec![json!({"cmd": "servo_stop"})]);
        assert_eq!(t.page, Page::Home);
    }

    #[test]
    fn servo_page_before_the_mode_is_on_only_retries_start() {
        let mut t = TuneScreen { open: true, page: Page::Servo, ..Default::default() };
        assert_eq!(t.handle(Action::Nav(Dir::Down), &ctx("walk")), Vec::<Value>::new());
        assert_eq!(t.handle(Action::Confirm, &ctx("walk"))[0], json!({"cmd": "servo_start"}));
    }
}
