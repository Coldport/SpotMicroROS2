//! Settings menu state: which row is selected, values sent but not yet
//! confirmed by the robot, and the result toast. The rows themselves come from
//! the robot (SettingItem list on the settings topic); this module only knows
//! how to step a value of each type.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::input::Dir;
use crate::shared::{SettingAck, SettingItem};

/// A sent value is shown until the robot confirms it, or this long.
const PENDING_TIMEOUT: Duration = Duration::from_secs(3);
const TOAST_FOR: Duration = Duration::from_millis(2600);
/// An "action" row (e.g. Shut down robot) needs a second A within this long.
const CONFIRM_WITHIN: Duration = Duration::from_secs(3);

pub struct Toast {
    pub text: String,
    pub ok: bool,
    pub at: Instant,
}

impl Toast {
    pub fn alive(&self) -> bool {
        self.at.elapsed() < TOAST_FOR
    }
    /// 0..1, for fading out over the last 400 ms.
    pub fn alpha(&self) -> f32 {
        let left = TOAST_FOR.saturating_sub(self.at.elapsed()).as_secs_f32();
        (left / 0.4).clamp(0.0, 1.0)
    }
}

#[derive(Default)]
pub struct Menu {
    pub open: bool,
    /// The controls & tricks reference (View button); only one of the two is open.
    pub controls_open: bool,
    pub selected: usize,
    pending: HashMap<String, (Value, Instant)>,
    pub toast: Option<Toast>,
    /// Action row waiting for its confirming second A press.
    armed_action: Option<(String, Instant)>,
}

/// A value change to send to the robot.
pub type Change = (String, Value);

/// Rows the app adds itself (not from the robot) start with this; main.rs
/// handles their Change instead of sending it.
pub const LOCAL_PREFIX: &str = "deck.";
/// "Wi-Fi setup over Bluetooth": an A×2 action row, so it can't start by accident.
pub const BT_WIFI_KEY: &str = "deck.bt_wifi";

/// The robot's settings plus the app's own Connection rows, which are there
/// even with no link (that's when they're needed most).
pub fn with_local_items(robot: &[SettingItem]) -> Vec<SettingItem> {
    let mut items = robot.to_vec();
    items.push(SettingItem {
        key: BT_WIFI_KEY.into(),
        label: "Wi-Fi setup over Bluetooth".into(),
        group: "Connection".into(),
        kind: "action".into(),
        help: "Finds the robot over Bluetooth and puts it on the Deck's Wi-Fi (or just learns its \
               address). Works whether or not the app is connected."
            .into(),
        available: true,
        ..Default::default()
    });
    items
}

/// True if the robot itself hasn't sent any settings (only the app's rows are there).
pub fn robot_items_missing(items: &[SettingItem]) -> bool {
    items.iter().all(|i| i.key.starts_with(LOCAL_PREFIX))
}

fn decimals(step: f64) -> usize {
    if step >= 1.0 {
        0
    } else if step >= 0.1 {
        1
    } else {
        2
    }
}

/// Human-readable value with unit, e.g. "45 mm", "0.35 Hz", "On", "crawl".
pub fn format_value(item: &SettingItem, v: &Value) -> String {
    match item.kind.as_str() {
        "bool" => match v.as_bool() {
            Some(true) => "On".into(),
            Some(false) => "Off".into(),
            None => "—".into(),
        },
        "enum" => v.as_str().map(str::to_string).unwrap_or_else(|| "—".into()),
        "action" => String::new(),
        _ => match v.as_f64() {
            Some(f) => {
                let n = format!("{:.*}", decimals(item.step.unwrap_or(1.0)), f);
                if item.unit.is_empty() {
                    n
                } else if item.unit == "x" {
                    format!("{n}×")
                } else {
                    format!("{n} {}", item.unit)
                }
            }
            None => "—".into(),
        },
    }
}

/// Next value for `item` one step in `dir` (+1 / -1) from `cur`, or None if it
/// can't move (already at the end, unknown value).
pub fn stepped(item: &SettingItem, cur: &Value, dir: i32) -> Option<Value> {
    match item.kind.as_str() {
        "bool" => cur.as_bool().map(|b| Value::Bool(!b)),
        "enum" => {
            let n = item.options.len();
            if n == 0 {
                return None;
            }
            let i = cur.as_str().and_then(|c| item.options.iter().position(|o| o == c)).unwrap_or(0);
            let j = (i as i32 + dir).rem_euclid(n as i32) as usize;
            (j != i).then(|| Value::from(item.options[j].clone()))
        }
        "float" | "int" => {
            let c = cur.as_f64()?;
            let step = item.step.unwrap_or(1.0);
            let (lo, hi) = (item.min.unwrap_or(f64::MIN), item.max.unwrap_or(f64::MAX));
            let mut v = (c + dir as f64 * step).clamp(lo, hi);
            if item.min.is_some() {
                v = lo + ((v - lo) / step).round() * step; // snap to the step grid
            }
            v = (v.clamp(lo, hi) * 10_000.0).round() / 10_000.0;
            if (v - c).abs() < 1e-9 {
                return None;
            }
            Some(if item.kind == "int" { Value::from(v.round() as i64) } else { Value::from(v) })
        }
        _ => None,
    }
}

/// Where the value sits in its range, 0..1 (for the slider).
pub fn fraction(item: &SettingItem, v: &Value) -> Option<f32> {
    let (lo, hi, f) = (item.min?, item.max?, v.as_f64()?);
    (hi > lo).then(|| ((f - lo) / (hi - lo)).clamp(0.0, 1.0) as f32)
}

impl Menu {
    pub fn toggle(&mut self) {
        self.open = !self.open;
        self.controls_open = false;
    }

    pub fn toggle_controls(&mut self) {
        self.controls_open = !self.controls_open;
        self.open = false;
    }

    /// The value to show: what we just sent, until the robot catches up.
    pub fn shown_value(&self, item: &SettingItem) -> (Value, bool) {
        match self.pending.get(&item.key) {
            Some((v, at)) if at.elapsed() < PENDING_TIMEOUT && *v != item.value => (v.clone(), true),
            _ => (item.value.clone(), false),
        }
    }

    pub fn navigate(&mut self, dir: Dir, items: &[SettingItem]) -> Option<Change> {
        if items.is_empty() {
            return None;
        }
        self.selected = self.selected.min(items.len() - 1);
        match dir {
            Dir::Up => {
                self.selected = if self.selected == 0 { items.len() - 1 } else { self.selected - 1 };
                None
            }
            Dir::Down => {
                self.selected = (self.selected + 1) % items.len();
                None
            }
            Dir::Left => self.adjust(&items[self.selected], -1),
            Dir::Right => self.adjust(&items[self.selected], 1),
        }
    }

    /// A: flip a switch / cycle an option list / press an action button (twice,
    /// within CONFIRM_WITHIN). Numbers are changed with left/right.
    pub fn confirm(&mut self, items: &[SettingItem]) -> Option<Change> {
        let item = items.get(self.selected)?;
        match item.kind.as_str() {
            "bool" | "enum" => self.adjust(item, 1),
            "action" => self.press_action(item),
            _ => None,
        }
    }

    /// True while `key`'s action is waiting for its second A.
    pub fn action_armed(&self, key: &str) -> bool {
        matches!(&self.armed_action, Some((k, at)) if k == key && at.elapsed() < CONFIRM_WITHIN)
    }

    fn press_action(&mut self, item: &SettingItem) -> Option<Change> {
        if !item.available {
            self.notify(format!("{}: robot not reachable", item.label), false);
            return None;
        }
        if self.action_armed(&item.key) {
            self.armed_action = None;
            return Some((item.key.clone(), Value::Bool(true)));
        }
        self.armed_action = Some((item.key.clone(), Instant::now()));
        self.notify(format!("Press A again to confirm: {}", item.label), false);
        None
    }

    fn adjust(&mut self, item: &SettingItem, dir: i32) -> Option<Change> {
        if !item.available {
            self.notify(format!("{}: not available — is the gait node running?", item.label), false);
            return None;
        }
        let (cur, _) = self.shown_value(item);
        let next = stepped(item, &cur, dir)?;
        self.pending.insert(item.key.clone(), (next.clone(), Instant::now()));
        Some((item.key.clone(), next))
    }

    /// Robot's reply to a change.
    pub fn on_ack(&mut self, ack: &SettingAck, items: &[SettingItem]) {
        let item = items.iter().find(|i| i.key == ack.key);
        let label = item.map(|i| i.label.clone()).unwrap_or_else(|| ack.key.clone());
        if ack.ok && item.is_some_and(|i| i.kind == "action") {
            self.notify(if ack.text.is_empty() { format!("{label}: done") } else { ack.text.clone() }, true);
        } else if ack.ok {
            let shown = item.map(|i| format_value(i, &ack.value)).unwrap_or_else(|| ack.value.to_string());
            self.notify(format!("{label} → {shown}"), true);
        } else {
            self.pending.remove(&ack.key);
            let why = if ack.text.is_empty() { "refused".to_string() } else { ack.text.clone() };
            self.notify(format!("{label}: {why}"), false);
        }
    }

    pub fn notify(&mut self, text: String, ok: bool) {
        self.toast = Some(Toast { text, ok, at: Instant::now() });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn num(key: &str, value: f64, min: f64, max: f64, step: f64) -> SettingItem {
        SettingItem {
            key: key.into(),
            label: key.into(),
            kind: "float".into(),
            min: Some(min),
            max: Some(max),
            step: Some(step),
            unit: "mm".into(),
            value: json!(value),
            available: true,
            ..Default::default()
        }
    }

    #[test]
    fn numbers_step_clamp_and_snap() {
        let it = num("h", 40.0, 10.0, 90.0, 5.0);
        assert_eq!(stepped(&it, &json!(40.0), 1), Some(json!(45.0)));
        assert_eq!(stepped(&it, &json!(90.0), 1), None, "already at max");
        assert_eq!(stepped(&it, &json!(41.3), 1), Some(json!(45.0)), "snaps to the grid");
        let f = num("s", 0.35, 0.1, 2.0, 0.05);
        assert_eq!(stepped(&f, &json!(0.35), 1), Some(json!(0.4)));
        assert_eq!(format_value(&f, &json!(0.4)), "0.40 mm");
    }

    #[test]
    fn bools_and_enums() {
        let b = SettingItem { kind: "bool".into(), value: json!(true), available: true, ..Default::default() };
        assert_eq!(stepped(&b, &json!(true), 1), Some(json!(false)));
        let e = SettingItem {
            kind: "enum".into(),
            options: vec!["crawl".into(), "trot".into()],
            value: json!("crawl"),
            available: true,
            ..Default::default()
        };
        assert_eq!(stepped(&e, &json!("crawl"), 1), Some(json!("trot")));
        assert_eq!(stepped(&e, &json!("trot"), 1), Some(json!("crawl")), "wraps");
    }

    #[test]
    fn rapid_presses_accumulate_from_the_pending_value() {
        let items = vec![num("h", 40.0, 10.0, 90.0, 5.0)];
        let mut m = Menu::default();
        assert_eq!(m.navigate(Dir::Right, &items), Some(("h".into(), json!(45.0))));
        // robot hasn't confirmed yet; the next press continues from 45
        assert_eq!(m.navigate(Dir::Right, &items), Some(("h".into(), json!(50.0))));
        assert_eq!(m.shown_value(&items[0]), (json!(50.0), true));
        // a refusal drops the pending value
        m.on_ack(&SettingAck { key: "h".into(), ok: false, value: json!(50.0), text: "no".into() }, &items);
        assert_eq!(m.shown_value(&items[0]), (json!(40.0), false));
        assert!(!m.toast.as_ref().unwrap().ok);
    }

    #[test]
    fn unavailable_rows_are_not_sent() {
        let mut it = num("h", 40.0, 10.0, 90.0, 5.0);
        it.available = false;
        let mut m = Menu::default();
        assert_eq!(m.navigate(Dir::Right, &[it]), None);
        assert!(m.toast.is_some());
    }

    #[test]
    fn selection_wraps() {
        let items: Vec<_> = (0..10).map(|i| num(&format!("k{i}"), 1.0, 0.0, 2.0, 1.0)).collect();
        let mut m = Menu::default();
        m.navigate(Dir::Up, &items);
        assert_eq!(m.selected, 9);
        m.navigate(Dir::Down, &items);
        assert_eq!(m.selected, 0);
    }

    #[test]
    fn bt_wifi_row_is_always_there_and_needs_two_presses() {
        let items = with_local_items(&[]);
        assert!(robot_items_missing(&items));
        assert_eq!(items.len(), 1);
        let mut m = Menu::default();
        assert_eq!(m.confirm(&items), None, "first A only arms it");
        assert_eq!(m.confirm(&items), Some((BT_WIFI_KEY.to_string(), Value::Bool(true))));
        let with_robot = with_local_items(&[num("h", 40.0, 10.0, 90.0, 5.0)]);
        assert!(!robot_items_missing(&with_robot));
        assert_eq!(with_robot.last().unwrap().key, BT_WIFI_KEY, "robot rows come first");
    }

    #[test]
    fn action_needs_two_presses_in_time() {
        let items = vec![SettingItem {
            key: "shutdown".into(),
            label: "Shut down robot".into(),
            kind: "action".into(),
            available: true,
            ..Default::default()
        }];
        let mut m = Menu::default();
        assert_eq!(m.confirm(&items), None);
        assert!(m.action_armed("shutdown"));
        assert_eq!(m.confirm(&items), Some(("shutdown".to_string(), Value::Bool(true))));
        assert!(!m.action_armed("shutdown"));
        // left/right never fire it
        assert_eq!(m.navigate(Dir::Left, &items), None);
        assert_eq!(m.navigate(Dir::Right, &items), None);
        assert_eq!(m.confirm(&items), None);
    }
}
