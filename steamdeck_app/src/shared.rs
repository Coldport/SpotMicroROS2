//! State shared between the network threads and the UI thread.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use serde::Deserialize;

use crate::protocol::{MapData, ScanData};

/// Main-display views, in the order L2 cycles through them.
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
}
/// IMU axis calibration progress: step "level" | "right" | "left" | "return".
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ImuCalibInfo {
    pub step: String,
    /// true: armed, the robot rolls itself; false: tilt it by hand.
    #[serde(default)]
    pub auto: bool,
    #[serde(default)]
    pub prompt: String,
}
/// A result from gait_node to show once as a toast; `seq` goes up for each new one.
#[derive(Deserialize, Clone, Default, Debug)]
pub struct ModeNote {
    pub seq: u64,
    pub text: String,
    #[serde(default)]
    pub ok: bool,
}
/// gait_node's mode: "walk" | "sentry" | "calibrate" | "trick" | "imu_calib".
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
}

/// One live-tunable robot setting, as the robot describes it on the settings
/// topic (spot_micro_deck/settings.py). The menu is built from these alone.
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
            r#"{"mode":{"mode":"imu_calib","imu_calib":{"step":"right","auto":false,"prompt":"Tilt"},
            "note":{"seq":3,"text":"IMU zeroed","ok":true}}}"#,
        )
        .unwrap();
        let m = st.mode.unwrap();
        assert_eq!(m.imu_calib.unwrap().step, "right");
        assert_eq!(m.note.unwrap().seq, 3);
    }

    #[test]
    fn old_robot_state_without_new_fields_still_parses() {
        let st: RobotState = serde_json::from_str(r#"{"uptime_s":1,"sys":{}}"#).unwrap();
        assert!(st.camera.is_none() && st.gait.is_none());
    }
}
