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
    pub selected: usize,
    pending: HashMap<String, (Value, Instant)>,
    pub toast: Option<Toast>,
}

/// A value change to send to the robot.
pub type Change = (String, Value);

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

    /// A: flip a switch / cycle an option list. Numbers are changed with left/right.
    pub fn confirm(&mut self, items: &[SettingItem]) -> Option<Change> {
        let item = items.get(self.selected)?;
        match item.kind.as_str() {
            "bool" | "enum" => self.adjust(item, 1),
            _ => None,
        }
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
        if ack.ok {
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
}
