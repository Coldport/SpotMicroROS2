//! `--demo-nav`: a fake robot in auto nav mode, for screenshots and trying the
//! nav view away from the robot. A room with a few walls is the "saved" map;
//! the live map has an extra box in it (something new); the lidar is ray-cast
//! against the live map; the robot walks the path to the goal.

use std::sync::Arc;
use std::time::Instant;

use crate::protocol::{MapData, ScanData};
use crate::shared::{ModeInfo, NavStatus, Pose, RobotState, SharedRef};

const RES: f32 = 0.05;
const W: u32 = 180; // 9 m
const H: u32 = 130; // 6.5 m
const ORIGIN: (f32, f32) = (-2.0, -1.5);

fn room(extra_box: bool) -> MapData {
    let mut cells = vec![-1i8; (W * H) as usize];
    let mut set = |x0: f32, y0: f32, x1: f32, y1: f32, v: i8| {
        let c = |x: f32| (((x - ORIGIN.0) / RES) as i32).clamp(0, W as i32 - 1);
        let r = |y: f32| (((y - ORIGIN.1) / RES) as i32).clamp(0, H as i32 - 1);
        for row in r(y0)..=r(y1) {
            for col in c(x0)..=c(x1) {
                cells[(row as u32 * W + col as u32) as usize] = v;
            }
        }
    };
    set(-1.5, -1.0, 6.5, 4.5, 0); // floor
    for (x0, y0, x1, y1) in [
        (-1.5, -1.0, 6.5, -1.0), (-1.5, 4.5, 6.5, 4.5), (-1.5, -1.0, -1.5, 4.5), (6.5, -1.0, 6.5, 4.5),
        (2.5, -1.0, 2.5, 2.4), (2.5, 3.4, 2.5, 4.5), // inner wall with a doorway
        (4.2, 1.0, 5.2, 1.6),                        // a table
        (-1.5, 2.0, 0.3, 2.0),                       // a half wall
    ] {
        set(x0, y0, x1, y1, 100);
    }
    if extra_box {
        set(0.9, 0.2, 1.3, 0.6, 100);
    }
    MapData { width: W, height: H, resolution: RES, origin_x: ORIGIN.0, origin_y: ORIGIN.1, cells }
}

fn occupied(m: &MapData, x: f32, y: f32) -> bool {
    let col = ((x - m.origin_x) / m.resolution) as i32;
    let row = ((y - m.origin_y) / m.resolution) as i32;
    if col < 0 || row < 0 || col >= m.width as i32 || row >= m.height as i32 {
        return true;
    }
    m.cells[(row as u32 * m.width + col as u32) as usize] >= 50
}

fn scan(m: &MapData, x: f32, y: f32, yaw: f32) -> ScanData {
    let n = 360;
    let inc = std::f32::consts::TAU / n as f32;
    let ranges_mm = (0..n)
        .map(|i| {
            let a = yaw - std::f32::consts::PI + i as f32 * inc;
            let (s, c) = a.sin_cos();
            let mut r = 0.15;
            while r < 8.0 {
                if occupied(m, x + c * r, y + s * r) {
                    return (r * 1000.0) as u16;
                }
                r += 0.02;
            }
            0xFFFF
        })
        .collect();
    ScanData { angle_min: -std::f32::consts::PI, angle_increment: inc, range_max: 8.0, ranges_mm }
}

struct Demo {
    live: Arc<MapData>,
    path: Vec<[f32; 2]>,
    pose: (f32, f32, f32),
    last: Instant,
}

static DEMO: std::sync::Mutex<Option<Demo>> = std::sync::Mutex::new(None);

/// Robot walks the path at a quick 0.4 m/s so a screenshot shows progress.
pub fn fill(shared: &SharedRef, _t: f32) {
    let mut guard = DEMO.lock().unwrap();
    let first = guard.is_none();
    let d = guard.get_or_insert_with(|| Demo {
        live: Arc::new(room(true)),
        path: vec![[0.0, 0.0], [0.4, 1.0], [2.5, 2.9], [4.0, 3.4], [5.6, 2.6]],
        pose: (0.0, 0.0, 0.3),
        last: Instant::now(),
    });
    let dt = d.last.elapsed().as_secs_f32();
    d.last = Instant::now();
    // advance along the path
    let mut step = 0.4 * dt;
    while step > 0.0 && d.path.len() >= 2 {
        let [tx, ty] = d.path[1];
        let (dx, dy) = (tx - d.pose.0, ty - d.pose.1);
        let dist = (dx * dx + dy * dy).sqrt();
        d.pose.2 = dy.atan2(dx);
        if dist <= step {
            d.pose.0 = tx;
            d.pose.1 = ty;
            d.path.remove(0);
            step -= dist;
        } else {
            d.pose.0 += dx / dist * step;
            d.pose.1 += dy / dist * step;
            step = 0.0;
        }
    }
    if let Some(p) = d.path.first_mut() {
        *p = [d.pose.0, d.pose.1];
    }
    let left: f32 = d.path.windows(2).map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt()).sum();
    let arrived = d.path.len() < 2;
    let mut s = shared.lock().unwrap();
    if first {
        s.static_map = Some(Arc::new(room(false)));
        s.static_map_version += 1;
        s.map = Some(d.live.clone());
        s.map_version += 1;
        s.robot_online = Some(true);
    }
    // No broker in the demo; don't show the "connecting" card over the map.
    s.broker_connected = true;
    s.robot_online = Some(true);
    s.last_error.clear();
    s.scan = Some(Arc::new(scan(&d.live, d.pose.0, d.pose.1, d.pose.2)));
    s.scan_at = Some(Instant::now());
    s.state = Some(RobotState {
        uptime_s: 754,
        pose: Some(Pose { x: d.pose.0, y: d.pose.1, yaw_deg: d.pose.2.to_degrees() }),
        mode: Some(ModeInfo { mode: "auto_nav".into(), ..Default::default() }),
        nav: Some(NavStatus {
            state: if arrived { "arrived".into() } else { "driving".into() },
            text: if arrived { "Arrived".into() } else { format!("{left:.1} m to go") },
            goal: d.path.last().copied().filter(|_| !arrived),
            path: if arrived { vec![] } else { d.path.clone() },
        }),
        gait: Some(crate::shared::GaitInfo { running: true, armed: true }),
        ..Default::default()
    });
    s.state_at = Some(Instant::now());
}

/// A goal from the demo's controls: walk there in a straight line.
pub fn set_goal(_shared: &SharedRef, x: f32, y: f32) {
    if let Some(d) = DEMO.lock().unwrap().as_mut() {
        d.path = vec![[d.pose.0, d.pose.1], [x, y]];
    }
}
