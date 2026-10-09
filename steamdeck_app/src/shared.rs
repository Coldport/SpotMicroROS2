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
            View::Color => "CAMERA",
            View::Depth => "DEPTH",
            View::Lidar => "LIDAR",
            View::Map => "MAP",
        }
    }
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
}

pub type SharedRef = Arc<Mutex<Shared>>;
