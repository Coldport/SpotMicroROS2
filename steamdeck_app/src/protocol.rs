//! MQTT topics and payload formats shared with the robot's deck_bridge node.
//! Keep in sync with src/spot_micro_deck/spot_micro_deck/protocol.py — the
//! tests below parse bytes produced by that Python module.

use std::io::Read;

pub const DEFAULT_PREFIX: &str = "spotmicro";

/// Button names in the order of Joy.buttons on the robot side.
pub const BUTTON_NAMES: [&str; 21] = [
    "a", "b", "x", "y", "l1", "r1", "back", "start", "guide", "l3", "r3", "dpad_up", "dpad_down",
    "dpad_left", "dpad_right", "paddle1", "paddle2", "paddle3", "paddle4", "l2", "r2",
];
pub const BTN_A: usize = 0;
pub const BTN_L1: usize = 4;
pub const BTN_R1: usize = 5;
pub const BTN_DPAD_UP: usize = 11;
pub const BTN_DPAD_DOWN: usize = 12;
pub const BTN_DPAD_LEFT: usize = 13;
pub const BTN_DPAD_RIGHT: usize = 14;
pub const BTN_L2: usize = 19;
pub const BTN_R2: usize = 20;

pub struct Topics {
    pub robot_online: String,
    pub video_color: String,
    pub video_depth: String,
    pub map: String,
    pub static_map: String,
    pub scan: String,
    pub state: String,
    pub settings: String,
    pub settings_ack: String,
    pub deck_online: String,
    pub deck_view: String,
    pub deck_input: String,
    pub deck_set: String,
    pub deck_nav: String,
    pub deck_track: String,
    pub track: String,
}

impl Topics {
    pub fn new(prefix: &str) -> Self {
        let p = prefix.trim_matches('/');
        let t = |s: &str| format!("{p}/{s}");
        Topics {
            robot_online: t("robot/online"),
            video_color: t("video/color"),
            video_depth: t("video/depth"),
            map: t("map"),
            static_map: t("static_map"),
            scan: t("scan"),
            state: t("state"),
            settings: t("settings"),
            settings_ack: t("settings/ack"),
            deck_online: t("deck/online"),
            deck_view: t("deck/view"),
            deck_input: t("deck/input"),
            deck_set: t("deck/set"),
            deck_nav: t("deck/nav"),
            deck_track: t("deck/track"),
            track: t("track"),
        }
    }
}

pub struct MapData {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    /// Row 0 is the lowest y. -1 unknown, 0..=100 occupancy probability.
    pub cells: Vec<i8>,
}

pub struct ScanData {
    pub angle_min: f32,
    pub angle_increment: f32,
    pub range_max: f32,
    /// Millimetres; 0xFFFF means no return.
    pub ranges_mm: Vec<u16>,
}

fn f32_at(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

pub fn parse_map(payload: &[u8]) -> Result<MapData, String> {
    if payload.len() < 16 {
        return Err("map payload too short".into());
    }
    let width = u16::from_le_bytes([payload[0], payload[1]]) as u32;
    let height = u16::from_le_bytes([payload[2], payload[3]]) as u32;
    let mut raw = Vec::with_capacity((width * height) as usize);
    flate2::read::ZlibDecoder::new(&payload[16..])
        .read_to_end(&mut raw)
        .map_err(|e| format!("map zlib: {e}"))?;
    if raw.len() != (width * height) as usize {
        return Err(format!("map size mismatch: {} bytes for {}x{}", raw.len(), width, height));
    }
    Ok(MapData {
        width,
        height,
        resolution: f32_at(payload, 4),
        origin_x: f32_at(payload, 8),
        origin_y: f32_at(payload, 12),
        cells: raw.into_iter().map(|b| b as i8).collect(),
    })
}

pub fn parse_scan(payload: &[u8]) -> Result<ScanData, String> {
    if payload.len() < 14 {
        return Err("scan payload too short".into());
    }
    let count = u16::from_le_bytes([payload[12], payload[13]]) as usize;
    let body = &payload[14..];
    if body.len() != count * 2 {
        return Err(format!("scan length mismatch: {} bytes for {} ranges", body.len(), count));
    }
    Ok(ScanData {
        angle_min: f32_at(payload, 0),
        angle_increment: f32_at(payload, 4),
        range_max: f32_at(payload, 8),
        ranges_mm: body.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect(),
    })
}

pub fn encode_input(seq: u32, axes: &[f32; 6], pressed: &[bool; 21]) -> String {
    let names: Vec<&str> = BUTTON_NAMES
        .iter()
        .zip(pressed.iter())
        .filter(|(_, &down)| down)
        .map(|(n, _)| *n)
        .collect();
    let rounded: Vec<f32> = axes.iter().map(|a| (a * 10000.0).round() / 10000.0).collect();
    serde_json::json!({ "seq": seq, "axes": rounded, "buttons": names }).to_string()
}

/// deck/set: change one robot setting.
pub fn encode_set(key: &str, value: &serde_json::Value) -> String {
    serde_json::json!({ "key": key, "value": value }).to_string()
}

/// deck/nav: walk to (x, y) in the map frame (auto nav mode only).
pub fn encode_nav_goal(x: f32, y: f32) -> String {
    let r = |v: f32| ((v * 1000.0).round() / 1000.0) as f64;
    serde_json::json!({ "cmd": "goal", "x": r(x), "y": r(y) }).to_string()
}

/// deck/nav: stop and forget the goal.
pub fn encode_nav_cancel() -> String {
    serde_json::json!({ "cmd": "cancel" }).to_string()
}

/// deck/track: follow what's in this box (normalised camera-image x, y, w, h; sentry only).
pub fn encode_track_start(b: [f32; 4]) -> String {
    let r = |v: f32| (v as f64 * 10000.0).round() / 10000.0;
    serde_json::json!({ "cmd": "start", "box": [r(b[0]), r(b[1]), r(b[2]), r(b[3])] }).to_string()
}

/// deck/track: stop following.
pub fn encode_track_stop() -> String {
    serde_json::json!({ "cmd": "stop" }).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    // Produced by the Python reference implementation (protocol.py).
    const MAP_HEX: &str = "04000300cdcc4c3d0000c0bf00000040789cfbcf9062c4c090f29f4b444e030019c5035d";
    const SCAN_HEX: &str = "d00f49c039fa8e3c000040410400f401ffffe02eb80b";

    #[test]
    fn parses_map_from_python() {
        let m = parse_map(&unhex(MAP_HEX)).unwrap();
        assert_eq!((m.width, m.height), (4, 3));
        assert!((m.resolution - 0.05).abs() < 1e-6);
        assert!((m.origin_x + 1.5).abs() < 1e-6 && (m.origin_y - 2.0).abs() < 1e-6);
        assert_eq!(m.cells, vec![-1, 0, 100, 50, 0, 0, 100, -1, 10, 20, 30, 40]);
    }

    #[test]
    fn parses_scan_from_python() {
        let s = parse_scan(&unhex(SCAN_HEX)).unwrap();
        assert_eq!(s.ranges_mm, vec![500, 65535, 12000, 3000]);
        assert!((s.angle_min + 3.14159).abs() < 1e-5 && (s.range_max - 12.0).abs() < 1e-6);
    }

    #[test]
    fn rejects_truncated_payloads() {
        assert!(parse_map(&unhex(MAP_HEX)[..10]).is_err());
        let scan = unhex(SCAN_HEX);
        assert!(parse_scan(&scan[..scan.len() - 1]).is_err());
    }

    #[test]
    fn set_matches_python_decoder() {
        assert_eq!(
            encode_set("step_height", &serde_json::json!(45.0)),
            r#"{"key":"step_height","value":45.0}"#
        );
    }

    #[test]
    fn nav_matches_python_decoder() {
        assert_eq!(encode_nav_goal(1.25, -0.5), r#"{"cmd":"goal","x":1.25,"y":-0.5}"#);
        assert_eq!(encode_nav_cancel(), r#"{"cmd":"cancel"}"#);
    }

    #[test]
    fn track_matches_python_decoder() {
        assert_eq!(encode_track_start([0.1, 0.25, 0.3, 0.4]), r#"{"box":[0.1,0.25,0.3,0.4],"cmd":"start"}"#);
        assert_eq!(encode_track_stop(), r#"{"cmd":"stop"}"#);
        let t = Topics::new("spotmicro");
        assert_eq!((t.deck_track.as_str(), t.track.as_str()), ("spotmicro/deck/track", "spotmicro/track"));
    }

    #[test]
    fn input_matches_python_format() {
        let mut pressed = [false; 21];
        pressed[0] = true; // a
        pressed[BTN_L2] = true;
        pressed[11] = true; // dpad_up
        let json = encode_input(7, &[0.5, -0.25, 0.0, 1.0, 0.75, 0.0], &pressed);
        assert_eq!(
            json,
            r#"{"axes":[0.5,-0.25,0.0,1.0,0.75,0.0],"buttons":["a","dpad_up","l2"],"seq":7}"#
        );
    }
}
