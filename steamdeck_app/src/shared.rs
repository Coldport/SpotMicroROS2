//! State shared between the network threads and the UI thread.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use serde::Deserialize;

use crate::protocol::{MapData, ScanData};

/// Main-display views, in the order R2 cycles through them (L2 goes back).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum View {
    Color,
    Depth,
    Lidar,
    Map,
}

impl View {
    pub fn next(self) -> View {
        match self {
            View::Color => View::Depth,
            View::Depth => View::Lidar,
            View::Lidar => View::Map,
            View::Map => View::Color,
        }
    }
    pub fn prev(self) -> View {
        match self {
            View::Color => View::Map,
            View::Depth => View::Color,
            View::Lidar => View::Depth,
            View::Map => View::Lidar,
        }
    }
    /// Name used on the wire (deck/view topic).
    pub fn wire_name(self) -> &'static str {
        match self {
            View::Color => "color",
            View::Depth => "depth",
            View::Lidar => "lidar",
            View::Map => "map",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            View::Color => "Camera",
            View::Depth => "Depth",
            View::Lidar => "Lidar",
            View::Map => "Map",
        }
    }
    pub const ALL: [View; 4] = [View::Color, View::Depth, View::Lidar, View::Map];
    pub fn is_video(self) -> bool {
        matches!(self, View::Color | View::Depth)
    }
    pub fn to_u8(self) -> u8 {
        self as u8
    }
    pub fn from_u8(v: u8) -> View {
        match v {
            1 => View::Depth,
            2 => View::Lidar,
            3 => View::Map,
            _ => View::Color,
        }
    }
}

#[derive(Deserialize, Clone, Default, Debug)]
pub struct Imu {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub yaw_deg: f32,
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct Nearest {
    pub range_m: f32,
    pub angle_deg: f32,
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct MapInfo {
    pub w: u32,
    pub h: u32,
    pub age_s: f32,
}
/// Which camera the robot has: the RealSense D405 (color + depth) or a USB webcam.
#[derive(Deserialize, Clone, Default, Debug)]
pub struct CameraInfo {
    /// "realsense" | "usb" | None (nothing plugged in)
    #[serde(rename = "type", default)]
    pub kind: Option<String>,
    #[serde(default)]
    pub model: String,
    #[serde(default)]
    pub depth: bool,
    /// The driver is running and publishing.
    #[serde(default)]
    pub streaming: bool,
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct GaitInfo {
    #[serde(default)]
    pub running: bool,
    #[serde(default)]
    pub armed: bool,
}
/// Sentry body pose (deg).
#[derive(Deserialize, Clone, Default, Debug)]
pub struct BodyPose {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}
/// Calibration-mode cursor and the servo offsets (deg), per leg [hip, thigh, knee].
#[derive(Deserialize, Clone, Default, Debug)]
pub struct CalibInfo {
    pub leg: String,
    pub joint: String,
    #[serde(default)]
    pub offsets: HashMap<String, [f32; 3]>,
    #[serde(default)]
    pub unsaved: bool,
    /// Result of the last save ("saved to …", "SAVE FAILED: …").
    #[serde(default)]
    pub note: String,
    /// Angle test (A): the selected joint is held at `target` (servo deg, before
    /// the offset); `angle` is where it is commanded now, `servo` what the servo gets.
    #[serde(default)]
    pub angle_mode: bool,
    #[serde(default)]
    pub target: Option<f32>,
    #[serde(default)]
    pub angle: Option<f32>,
    #[serde(default)]
    pub servo: Option<f32>,
}
/// IMU axis calibration progress: step "level" | "left" | "right" | "front" | "back" | "return".
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ImuCalibInfo {
    pub step: String,
    /// true: armed, the robot tilts itself; false: tilt it by hand.
    #[serde(default)]
    pub auto: bool,
    #[serde(default)]
    pub prompt: String,
    /// By hand only: current tilt from level (deg), the tilt needed, and how far
    /// through the hold-still time (0..1); hint = what's missing.
    #[serde(default)]
    pub tilt: Option<f32>,
    #[serde(default)]
    pub need: Option<f32>,
    #[serde(default)]
    pub hold: Option<f32>,
    #[serde(default)]
    pub hint: String,
}
/// A result from gait_node to show once as a toast; `seq` goes up for each new one.
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ModeNote {
    pub seq: u64,
    pub text: String,
    #[serde(default)]
    pub ok: bool,
}
/// nav_node's progress (auto nav): state "idle" | "driving" | "blocked" | "arrived" | "failed".
#[derive(Deserialize, Clone, Default, Debug)]
pub struct NavStatus {
    #[serde(default)]
    pub state: String,
    #[serde(default)]
    pub text: String,
    /// Map frame, metres.
    #[serde(default)]
    pub goal: Option<[f32; 2]>,
    #[serde(default)]
    pub path: Vec<[f32; 2]>,
    /// Speed level (fraction of full, set with the D-pad) and what that is in m/s.
    #[serde(default)]
    pub speed: Option<f32>,
    #[serde(default)]
    pub speed_mps: Option<f32>,
}
/// One test of an autotune run (gait_node autotune.py _seg_summary).
#[derive(Deserialize, Clone, Default, Debug)]
pub struct TuneSegment {
    pub name: String,
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub skipped: bool,
    /// Distance the gait's odometry thinks it walked / the lidar measured (m).
    #[serde(default)]
    pub est_m: Option<f32>,
    #[serde(default)]
    pub real_m: Option<f32>,
    #[serde(default)]
    pub yaw_deg: Option<f32>,
    #[serde(default)]
    pub time_s: Option<f32>,
}
/// A parameter autotune changed (already live on the robot).
#[derive(Deserialize, Clone, Default, Debug)]
pub struct TuneChange {
    pub key: String,
    #[serde(default)]
    pub old: serde_json::Value,
    #[serde(default)]
    pub new: serde_json::Value,
    #[serde(default)]
    pub why: String,
}
/// The current / last autotune run: state "running" | "done" | "failed" | "stopped".
#[derive(Deserialize, Clone, Default, Debug)]
pub struct TuneReport {
    pub state: String,
    /// Test running now (autotune.py COURSE name).
    #[serde(default)]
    pub test: String,
    /// 1-based position in the course (autotune.py COURSE).
    #[serde(default)]
    pub step: u32,
    #[serde(default)]
    pub prompt: String,
    #[serde(default)]
    pub progress: f32,
    #[serde(default)]
    pub segments: Vec<TuneSegment>,
    #[serde(default)]
    pub changes: Vec<TuneChange>,
    #[serde(default)]
    pub findings: Vec<String>,
    /// "" | "kept" | "reverted"
    #[serde(default)]
    pub outcome: String,
}
impl TuneReport {
    pub fn running(&self) -> bool {
        self.state == "running"
    }
    /// Finished with changes nobody has kept or reverted yet.
    pub fn undecided(&self) -> bool {
        !self.running() && self.outcome.is_empty() && !self.changes.is_empty()
    }
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ServoPoint {
    /// Angle written to the servo, and the joint's real angle (deg) at it.
    pub servo: f32,
    pub actual: f32,
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ServoFit {
    #[serde(default)]
    pub offset: Option<f32>,
    #[serde(default)]
    pub scale: Option<f32>,
    #[serde(default)]
    pub error: Option<String>,
}
/// Two-point servo calibration (mode "servo_cal"). Angles are the joint's real
/// angle (the same for left and right legs): hip 90 = straight down, thigh 0 =
/// straight down, knee = inside angle (180 = straight).
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ServoCalInfo {
    pub leg: String,
    pub joint: String,
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub unsaved: bool,
    /// leg -> [hip, thigh, knee] of [offset, scale]
    #[serde(default)]
    pub cal: HashMap<String, [[f32; 2]; 3]>,
    #[serde(default)]
    pub points: Vec<ServoPoint>,
    /// "FL hip" etc.: joints with two points recorded.
    #[serde(default)]
    pub done: Vec<String>,
    /// Test angle the joint is sent to, and what the servo gets for it now.
    #[serde(default)]
    pub target: Option<f32>,
    #[serde(default)]
    pub servo: Option<f32>,
    #[serde(default)]
    pub moving: bool,
    #[serde(default)]
    pub fit: Option<ServoFit>,
}
/// gait_node's mode: "walk" | "sentry" | "calibrate" | "trick" | "imu_calib" | "auto_nav"
/// | "autotune" | "servo_cal".
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ModeInfo {
    pub mode: String,
    #[serde(default)]
    pub pose: Option<BodyPose>,
    #[serde(default)]
    pub calib: Option<CalibInfo>,
    /// Trick being played (mode "trick").
    #[serde(default)]
    pub trick: Option<String>,
    #[serde(default)]
    pub imu_calib: Option<ImuCalibInfo>,
    #[serde(default)]
    pub note: Option<ModeNote>,
    /// auto_nav: the left stick is driving by hand, overriding the planner.
    #[serde(default)]
    pub manual: bool,
    /// sentry: the body is following a tracked object (not the sticks).
    #[serde(default)]
    pub tracking: bool,
    /// The running or last autotune run (in any mode, until cleared).
    #[serde(default)]
    pub tune: Option<TuneReport>,
    #[serde(default)]
    pub servo_cal: Option<ServoCalInfo>,
}
/// Sentry object tracking status from the robot's track_node (<p>/track).
#[derive(Deserialize, Clone, Default, Debug)]
pub struct TrackStatus {
    /// "idle" | "starting" | "tracking" | "lost"
    #[serde(default)]
    pub state: String,
    /// Normalised camera-image box [x, y, w, h].
    #[serde(default)]
    pub r#box: Option<[f32; 4]>,
    #[serde(default)]
    pub err_deg: Option<[f32; 2]>,
}
impl TrackStatus {
    /// Following something (or looking for it again).
    pub fn active(&self) -> bool {
        matches!(self.state.as_str(), "starting" | "tracking" | "lost")
    }
}
#[derive(Deserialize, Clone, Default, Debug)]
pub struct Sys {
    pub cpu_temp_c: Option<f32>,
    pub load1: Option<f32>,
}

/// The robot's JSON state summary (deck/state topic). Every field is optional so
/// an older/newer robot node doesn't break the app.
#[derive(Deserialize, Clone, Default, Debug)]
pub struct RobotState {
    #[serde(default)]
    pub uptime_s: u64,
    #[serde(default)]
    pub imu: Option<Imu>,
    #[serde(default)]
    pub pose: Option<Pose>,
    #[serde(default)]
    pub nearest: Option<Nearest>,
    #[serde(default)]
    pub map: Option<MapInfo>,
    #[serde(default)]
    pub rates: HashMap<String, f32>,
    #[serde(default)]
    pub sys: Sys,
    #[serde(default)]
    pub depth_range_m: Option<[f32; 2]>,
    #[serde(default)]
    pub camera: Option<CameraInfo>,
    #[serde(default)]
    pub gait: Option<GaitInfo>,
    /// Only while armed.
    #[serde(default)]
    pub mode: Option<ModeInfo>,
    #[serde(default)]
    pub nav: Option<NavStatus>,
}

/// One live-tunable robot setting, as the robot describes it on the settings
/// topic (spot_micro_deck/settings.py), plus the app's own rows (menu::with_local_items).
#[derive(Deserialize, Clone, Debug, Default)]
pub struct SettingItem {
    pub key: String,
    pub label: String,
    #[serde(default)]
    pub group: String,
    /// "float" | "int" | "bool" | "enum"
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default)]
    pub min: Option<f64>,
    #[serde(default)]
    pub max: Option<f64>,
    #[serde(default)]
    pub step: Option<f64>,
    #[serde(default)]
    pub unit: String,
    #[serde(default)]
    pub options: Vec<String>,
    #[serde(default)]
    pub help: String,
    #[serde(default)]
    pub value: serde_json::Value,
    #[serde(default)]
    pub available: bool,
}

#[derive(Deserialize)]
pub struct SettingsMsg {
    #[serde(default)]
    pub items: Vec<SettingItem>,
}

/// The robot's reply to a deck/set.
#[derive(Deserialize, Clone, Debug)]
pub struct SettingAck {
    pub key: String,
    pub ok: bool,
    #[serde(default)]
    pub value: serde_json::Value,
    #[serde(default)]
    pub text: String,
}

pub struct Frame {
    pub seq: u64,
    pub view: View,
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
}

#[derive(Default)]
pub struct Shared {
    pub broker_connected: bool,
    pub last_error: String,
    pub robot_online: Option<bool>,
    pub state: Option<RobotState>,
    pub state_at: Option<Instant>,
    pub map: Option<Arc<MapData>>,
    pub map_version: u64,
    /// The robot's saved map (Map > Save map); None if nothing is saved.
    pub static_map: Option<Arc<MapData>>,
    pub static_map_version: u64,
    pub scan: Option<Arc<ScanData>>,
    pub scan_at: Option<Instant>,
    pub frame: Option<Arc<Frame>>,
    pub frame_at: Option<Instant>,
    pub video_fps: f32,
    pub decode_ms: f32,
    pub settings: Vec<SettingItem>,
    pub settings_at: Option<Instant>,
    /// Replies not yet handled by the UI thread.
    pub acks: Vec<SettingAck>,
    pub track: Option<TrackStatus>,
    pub track_at: Option<Instant>,
    /// Robot mic samples not yet handed to the audio device, their rate, and
    /// when the last chunk came in.
    pub audio: Vec<i16>,
    pub audio_rate: u32,
    pub audio_at: Option<Instant>,
}

pub type SharedRef = Arc<Mutex<Shared>>;

#[cfg(test)]
mod tests {
    use super::*;

    // Shape produced by deck_bridge (settings.payload / _build_state).
    #[test]
    fn parses_robot_settings_and_state() {
        let msg: SettingsMsg = serde_json::from_str(
            r#"{"v":1,"items":[{"key":"gait_type","label":"Gait","group":"Walk","type":"enum",
            "target":"gait","unit":"","help":"h","options":["crawl","trot"],"value":"crawl","available":true},
            {"key":"step_height","label":"Foot lift","group":"Walk","type":"float","target":"gait",
            "unit":"mm","help":"","min":10.0,"max":90.0,"step":5.0,"value":null,"available":false}]}"#,
        )
        .unwrap();
        assert_eq!(msg.items.len(), 2);
        assert_eq!(msg.items[0].options, vec!["crawl", "trot"]);
        assert_eq!(msg.items[1].step, Some(5.0));
        assert!(msg.items[1].value.is_null() && !msg.items[1].available);

        let st: RobotState = serde_json::from_str(
            r#"{"uptime_s":3,"camera":{"type":"usb","model":"Anker","depth":false,"streaming":true},
            "gait":{"running":true,"armed":false},"sys":{}}"#,
        )
        .unwrap();
        let cam = st.camera.unwrap();
        assert_eq!(cam.kind.as_deref(), Some("usb"));
        assert!(!cam.depth && cam.streaming);
        assert!(st.gait.unwrap().running);
        assert!(st.mode.is_none());

        let st: RobotState = serde_json::from_str(
            r#"{"mode":{"mode":"calibrate","calib":{"leg":"FR","joint":"thigh","offset":7.75,
            "offsets":{"FL":[0,3,5],"FR":[-5,7.75,6]},"unsaved":true,"note":""}}}"#,
        )
        .unwrap();
        let c = st.mode.unwrap().calib.unwrap();
        assert_eq!((c.leg.as_str(), c.joint.as_str()), ("FR", "thigh"));
        assert_eq!(c.offsets["FR"], [-5.0, 7.75, 6.0]);

        let st: RobotState = serde_json::from_str(
            r#"{"mode":{"mode":"imu_calib","imu_calib":{"step":"right","auto":false,"prompt":"Tilt","tilt":8.5,"need":12.0,"hold":0.0,"hint":"Tilt further"},
            "note":{"seq":3,"text":"IMU zeroed","ok":true}}}"#,
        )
        .unwrap();
        let m = st.mode.unwrap();
        assert_eq!(m.imu_calib.unwrap().step, "right");
        assert_eq!(m.note.unwrap().seq, 3);

        // nav_node status, as forwarded by deck_bridge
        let st: RobotState = serde_json::from_str(
            r#"{"mode":{"mode":"auto_nav","manual":false},"nav":{"state":"driving","text":"1.2 m to go",
            "goal":[1.5,-0.25],"path":[[0,0],[1.5,-0.25]],"left_m":1.2}}"#,
        )
        .unwrap();
        let nav = st.nav.unwrap();
        assert_eq!(nav.goal, Some([1.5, -0.25]));
        assert_eq!(nav.path.len(), 2);
        assert!(!st.mode.unwrap().manual);
    }

    #[test]
    fn parses_track_status_and_angle_test() {
        let t: TrackStatus = serde_json::from_str(
            r#"{"state":"tracking","seq":4,"box":[0.1,0.2,0.3,0.4],"err_deg":[-3.5,1.25]}"#,
        )
        .unwrap();
        assert!(t.active());
        assert_eq!(t.r#box, Some([0.1, 0.2, 0.3, 0.4]));
        let t: TrackStatus = serde_json::from_str(r#"{"state":"idle","seq":5,"box":null,"err_deg":null}"#).unwrap();
        assert!(!t.active() && t.r#box.is_none());

        let st: RobotState = serde_json::from_str(
            r#"{"mode":{"mode":"calibrate","calib":{"leg":"FL","joint":"hip","offset":1.0,"offsets":{},
            "unsaved":true,"note":"","angle_mode":true,"target":95.0,"angle":93.5,"servo":94.5}}}"#,
        )
        .unwrap();
        let c = st.mode.unwrap().calib.unwrap();
        assert!(c.angle_mode);
        assert_eq!((c.target, c.angle, c.servo), (Some(95.0), Some(93.5), Some(94.5)));
        let st: RobotState = serde_json::from_str(r#"{"mode":{"mode":"sentry","pose":{"roll":0,"pitch":1,"yaw":2},"tracking":true}}"#).unwrap();
        assert!(st.mode.unwrap().tracking);
    }

    #[test]
    fn parses_tune_report_and_servo_cal() {
        let st: RobotState = serde_json::from_str(
            r#"{"mode":{"mode":"walk","tune":{"state":"done","test":"","step":8,"steps":8,"prompt":"Done",
            "progress":1.0,"segments":[{"name":"forward","label":"Walk forward","note":"","est_m":1.0,"real_m":0.71,
            "source":"lidar","yaw_deg":-3.2,"time_s":24.1,"roll_rms":2.9,"pitch_rms":1.1,"peak_tilt":7.0},
            {"name":"left","label":"Steps left","skipped":true,"note":"skipped: no room"}],
            "changes":[{"key":"odom_scale","old":1.0,"new":0.71,"why":"w"}],"findings":["Veered 3 deg"],"outcome":""}}}"#,
        )
        .unwrap();
        let t = st.mode.unwrap().tune.unwrap();
        assert!(t.undecided() && !t.running());
        assert_eq!(t.segments[0].real_m, Some(0.71));
        assert!(t.segments[1].skipped);
        assert_eq!(t.changes[0].new, serde_json::json!(0.71));

        let st: RobotState = serde_json::from_str(
            r#"{"mode":{"mode":"servo_cal","servo_cal":{"leg":"RR","joint":"knee","note":"point 1 recorded",
            "unsaved":true,"cal":{"FL":[[-1,1],[30,1],[10,1.05]]},"points":[{"servo":123.5,"actual":58.0}],
            "done":["FL hip"],"target":60.0,"angle":60.0,"servo":125.0,"moving":false,"fit":{"offset":3.5,"scale":1.0}}}}"#,
        )
        .unwrap();
        let c = st.mode.unwrap().servo_cal.unwrap();
        assert_eq!(c.cal["FL"][2], [10.0, 1.05]);
        assert_eq!(c.points[0].actual, 58.0);
        assert_eq!(c.fit.unwrap().offset, Some(3.5));
    }

    #[test]
    fn old_robot_state_without_new_fields_still_parses() {
        let st: RobotState = serde_json::from_str(r#"{"uptime_s":1,"sys":{}}"#).unwrap();
        assert!(st.camera.is_none() && st.gait.is_none());
    }
}
