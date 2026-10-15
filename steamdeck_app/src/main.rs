//! SpotMicro Steam Deck app: live camera / depth / lidar / map from the robot,
//! the Deck's controller input sent back to it, and a settings menu (Start/☰)
//! for the robot's live-tunable gait and video settings — all over MQTT. The
//! View (⧉) button shows every control and trick trigger. In auto nav mode
//! (A×2 on the robot) the screen becomes a map to pick goals on (navview.rs).
//! A back grip button (L4/R4/L5/R5) opens the Tune screen: autotune and
//! two-point servo calibration (tune.rs).
//!
//! Usage: spotmicro-deck [--host HOST[:PORT]] [--port N] [--user U --password P]
//!                       [--prefix TOPIC_PREFIX] [--windowed] [--menu] [--controls]
//! The host can also come from $SPOTDECK_HOST or ~/.config/spotmicro-deck/host.

mod audio;
mod bt;
mod demo;
mod input;
mod menu;
mod navview;
mod net;
mod protocol;
mod shared;
mod text;
mod track;
mod tune;
mod ui;

use std::fs::File;
use std::io::BufWriter;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdl2::controller::{Axis, GameController};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::PixelFormatEnum;
use sdl2::render::Canvas;
use sdl2::video::Window;

use input::{Action, Dir, Gamepad, InputState};
use menu::Menu;
use navview::{NavCam, NavCmd};
use shared::{Shared, View};
use track::TrackPick;
use tune::{TuneCtx, TuneScreen};
use ui::{Snapshot, Ui, LOGICAL_H, LOGICAL_W};

#[derive(Default)]
struct Args {
    host: Option<String>,
    port: Option<u16>,
    prefix: Option<String>,
    user: Option<String>,
    password: Option<String>,
    windowed: bool,
    software: bool,
    /// Start with the settings menu open (handy with --screenshot).
    menu: bool,
    /// Start with the controls reference open.
    controls: bool,
    /// Start with the Tune screen open on this page (home | auto | servo).
    tune: Option<String>,
    /// Start on this view (color | depth | lidar | map).
    view: Option<View>,
    /// Save a screenshot of the current view after N seconds, then quit.
    screenshot: Option<(String, f32)>,
    /// Scripted R2 presses with a screenshot of each view saved into this directory.
    selftest: Option<String>,
    quit_after: Option<f32>,
    /// Fake robot in auto nav mode (no connection): for screenshots of the nav view.
    demo_nav: bool,
    /// Start Wi-Fi setup over Bluetooth at launch (testing / troubleshooting).
    bt_setup: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args::default();
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut value = |name: &str| it.next().ok_or_else(|| format!("{name} needs a value"));
        match arg.as_str() {
            "--host" => a.host = Some(value("--host")?),
            "--port" => a.port = Some(value("--port")?.parse().map_err(|_| "bad --port")?),
            "--prefix" => a.prefix = Some(value("--prefix")?),
            "--user" => a.user = Some(value("--user")?),
            "--password" => a.password = Some(value("--password")?),
            "--windowed" => a.windowed = true,
            "--menu" => a.menu = true,
            "--controls" => a.controls = true,
            "--tune" => a.tune = Some(value("--tune")?),
            "--view" => {
                let name = value("--view")?;
                a.view = Some(View::ALL.into_iter().find(|v| v.wire_name() == name).ok_or("bad --view")?);
            }
            "--software" => a.software = true,
            "--screenshot" => {
                let path = value("--screenshot")?;
                a.screenshot = Some((path, 8.0));
            }
            "--screenshot-after" => {
                let secs: f32 = value("--screenshot-after")?.parse().map_err(|_| "bad --screenshot-after")?;
                if let Some((_, s)) = a.screenshot.as_mut() {
                    *s = secs;
                }
            }
            "--selftest" => a.selftest = Some(value("--selftest")?),
            "--demo-nav" => a.demo_nav = true,
            "--bt-setup" => a.bt_setup = true,
            "--quit-after" => a.quit_after = Some(value("--quit-after")?.parse().map_err(|_| "bad --quit-after")?),
            "-h" | "--help" => {
                println!(
                    "spotmicro-deck [--host HOST[:PORT]] [--port N] [--user U --password P] \
                     [--prefix P] [--windowed] [--menu] [--controls]\n\
                     Start/☰ opens the robot settings menu (D-pad to select and adjust).\n\
                     View/⧉ shows all controls and trick triggers.\n\
                     Host also from $SPOTDECK_HOST or ~/.config/spotmicro-deck/host (default spotmicro.local)."
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other} (try --help)")),
        }
    }
    Ok(a)
}

fn config_host() -> Option<String> {
    let dir = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .or_else(|| std::env::var("HOME").ok().map(|h| format!("{h}/.config")))?;
    let text = std::fs::read_to_string(format!("{dir}/spotmicro-deck/host")).ok()?;
    let host = text.trim().to_string();
    (!host.is_empty()).then_some(host)
}

/// "host" or "host:port" -> (host, port)
fn split_host(spec: &str, default_port: u16) -> (String, u16) {
    match spec.rsplit_once(':') {
        Some((h, p)) if !h.is_empty() && !h.contains(':') => match p.parse() {
            Ok(port) => (h.to_string(), port),
            Err(_) => (spec.to_string(), default_port),
        },
        _ => (spec.to_string(), default_port),
    }
}

fn save_screenshot(canvas: &Canvas<Window>, path: &str) -> Result<(), String> {
    let (w, h) = canvas.output_size()?;
    let pixels = canvas.read_pixels(None, PixelFormatEnum::RGB24)?;
    let file = File::create(path).map_err(|e| format!("{path}: {e}"))?;
    let mut enc = png::Encoder::new(BufWriter::new(file), w, h);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()
        .and_then(|mut wr| wr.write_image_data(&pixels))
        .map_err(|e| format!("{path}: {e}"))
}

/// Remember `host` in ~/.config/spotmicro-deck/host for the next launch.
fn save_host(host: &str) {
    if let Some(dir) = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .or_else(|| std::env::var("HOME").ok().map(|h| format!("{h}/.config")))
    {
        let dir = format!("{dir}/spotmicro-deck");
        if let Err(e) = std::fs::create_dir_all(&dir).and_then(|_| std::fs::write(format!("{dir}/host"), format!("{host}\n"))) {
            eprintln!("couldn't save the robot address in {dir}/host: {e}");
        }
    }
}

/// Held for the app's lifetime: a second copy (e.g. Steam launching it again)
/// exits instead of fighting this one for the MQTT client ID. An abstract
/// socket, so nothing is left behind if the app crashes.
fn single_instance() -> Result<std::os::unix::net::UnixListener, String> {
    use std::os::linux::net::SocketAddrExt;
    let addr = std::os::unix::net::SocketAddr::from_abstract_name(b"spotmicro-deck").map_err(|e| e.to_string())?;
    std::os::unix::net::UnixListener::bind_addr(&addr)
        .map_err(|_| "the app is already running".to_string())
}

fn axis_event(axis: Axis, value: i16) -> Event {
    Event::ControllerAxisMotion { timestamp: 0, which: 0, axis, value }
}

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let spec = args
        .host
        .clone()
        .or_else(|| std::env::var("SPOTDECK_HOST").ok())
        .or_else(config_host)
        .unwrap_or_else(|| "spotmicro.local".to_string());
    let (host, port) = split_host(&spec, args.port.unwrap_or(1883));
    // The demo must not talk to a real robot.
    let (host, port) = if args.demo_nav { ("127.0.0.1".to_string(), 9) } else { (host, port) };
    let mut host_label = format!("{host}:{port}");

    // Screenshot / test runs (headless) may run beside the real app.
    let headless = matches!(std::env::var("SDL_VIDEODRIVER").as_deref(), Ok("dummy") | Ok("offscreen"));
    let _instance = if headless { None } else { Some(single_instance()?) };

    // Steam Input can deliver events while another window (Steam's overlay) has focus.
    sdl2::hint::set("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1");
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let game_controller = sdl.game_controller()?;
    let mut player = audio::Player::new(sdl.audio().ok());
    let mut pump = sdl.event_pump()?;
    sdl.mouse().show_cursor(false);

    let mut wb = video.window("SpotMicro", LOGICAL_W, LOGICAL_H);
    wb.position_centered().resizable();
    if !args.windowed && !headless {
        wb.fullscreen_desktop();
    }
    let window = wb.build().map_err(|e| e.to_string())?;
    let mut canvas = if args.software || headless {
        window.into_canvas().software().build()
    } else {
        window.into_canvas().accelerated().present_vsync().build()
    }
    .map_err(|e| e.to_string())?;
    canvas.set_logical_size(LOGICAL_W, LOGICAL_H).map_err(|e| e.to_string())?;
    let tc = canvas.texture_creator();
    let mut ui = Ui::new(&tc)?;

    let shared = Arc::new(Mutex::new(Shared::default()));
    let view_atomic = Arc::new(AtomicU8::new(View::Color.to_u8()));
    let input_shared = Arc::new(Mutex::new(InputState::default()));
    let commander = net::start(
        net::NetConfig {
            host: host.clone(),
            port,
            prefix: args.prefix.clone().unwrap_or_else(|| protocol::DEFAULT_PREFIX.to_string()),
            username: args.user.clone(),
            password: args.password.clone(),
        },
        shared.clone(),
        view_atomic.clone(),
        input_shared.clone(),
    );

    let provision: bt::ProvisionRef = Arc::new(Mutex::new(bt::Provision::default()));
    if args.bt_setup {
        bt::start(&provision);
    }
    let mut pad = Gamepad::default();
    let mut controllers: Vec<GameController> = Vec::new();
    for i in 0..game_controller.num_joysticks().unwrap_or(0) {
        if game_controller.is_game_controller(i) {
            if let Ok(c) = game_controller.open(i) {
                println!("controller: {}", c.name());
                controllers.push(c);
            }
        }
    }

    let mut view = args.view.unwrap_or(View::Color);
    let mut nav = NavCam::default();
    let mut pick = TrackPick::default();
    if args.demo_nav {
        // Demo screenshots of a rotated / zoomed view: SPOTDECK_DEMO_ROT (deg), SPOTDECK_DEMO_ZOOM (px/m).
        let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<f32>().ok());
        if let Some(r) = env("SPOTDECK_DEMO_ROT") {
            nav.rot = r.to_radians();
        }
        if let Some(z) = env("SPOTDECK_DEMO_ZOOM") {
            nav.zoom = z;
        }
    }
    let mut last_frame = Instant::now();
    let mut menu = Menu::default();
    let mut tune_screen = TuneScreen::default();
    if let Some(page) = &args.tune {
        tune_screen.open = true;
        tune_screen.page = match page.as_str() {
            "auto" => tune::Page::Auto,
            "servo" => tune::Page::Servo,
            _ => tune::Page::Home,
        };
        tune_screen.home_sel = usize::from(page == "servo");
        tune_screen.on_joint = page == "servo";
        tune_screen.leg = 3;
        tune_screen.joint = 2;
    }
    menu.open = args.menu;
    menu.controls_open = args.controls && !args.menu;
    let mut last_note_seq: Option<u64> = None;
    let mut depth_ok = true;
    // R2 / Tab: next view, L2 / Backspace: previous, skipping Depth when the
    // robot's camera has none.
    let step_view = |v: View, forward: bool, depth_ok: bool| {
        let step = |v: View| if forward { v.next() } else { v.prev() };
        let n = step(v);
        if n == View::Depth && !depth_ok {
            step(n)
        } else {
            n
        }
    };
    let started = Instant::now();
    let mut selftest_step = 0usize;
    let mut pending_release: Option<f32> = None;
    // (seconds, take screenshot named X, then press R2) — see below
    let selftest_dir = args.selftest.clone();
    if let Some(dir) = &selftest_dir {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    'main: loop {
        let elapsed = started.elapsed().as_secs_f32();

        // Inject scripted events for --selftest: hold each view for 6 s,
        // screenshot it at the end, then pull and release R2.
        let mut injected: Vec<Event> = Vec::new();
        if let Some(dir) = &selftest_dir {
            let phase_end = 8.0 + selftest_step as f32 * 6.0;
            if elapsed >= phase_end && pending_release.is_none() && selftest_step < 5 {
                let path = format!("{dir}/view_{}_{}.png", selftest_step, view.wire_name());
                save_screenshot(&canvas, &path)?;
                println!("selftest: saved {path}");
                if selftest_step < 4 {
                    injected.push(axis_event(Axis::TriggerRight, 32767)); // pull R2
                    pending_release = Some(elapsed + 0.3);
                }
                selftest_step += 1;
                if selftest_step >= 5 {
                    break 'main;
                }
            }
            if let Some(t) = pending_release {
                if elapsed >= t {
                    injected.push(axis_event(Axis::TriggerRight, 0)); // release R2
                    injected.push(axis_event(Axis::LeftX, -20000)); // some stick movement
                    injected.push(axis_event(Axis::LeftY, -32767));
                    pending_release = None;
                }
            }
        }

        let events: Vec<Event> = pump.poll_iter().chain(injected).collect();
        let mut actions: Vec<Action> = Vec::new();
        let mut actions_tap: Vec<(f32, f32)> = Vec::new();
        // Touch / left mouse: (0 down, 1 move while held, 2 up, x, y), for drawing a tracking box.
        let mut touches: Vec<(u8, f32, f32)> = Vec::new();
        for ev in events {
            match ev {
                Event::Quit { .. } | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => break 'main,
                // Keyboard equivalents, for testing on a PC.
                Event::KeyDown { keycode: Some(k), .. } => match k {
                    Keycode::Tab | Keycode::Space => actions.push(Action::NextView),
                    Keycode::Backspace => actions.push(Action::PrevView),
                    Keycode::M | Keycode::F1 => actions.push(Action::ToggleMenu),
                    Keycode::H | Keycode::F2 => actions.push(Action::ToggleControls),
                    Keycode::Up => actions.push(Action::Nav(Dir::Up)),
                    Keycode::Down => actions.push(Action::Nav(Dir::Down)),
                    Keycode::Left => actions.push(Action::Nav(Dir::Left)),
                    Keycode::Right => actions.push(Action::Nav(Dir::Right)),
                    Keycode::Return => actions.push(Action::Confirm),
                    Keycode::P | Keycode::Y => actions.push(Action::Place),
                    Keycode::T => actions.push(Action::ToggleTune),
                    Keycode::X => actions.push(Action::X),
                    _ => {}
                },
                // Touchscreen taps arrive as (synthesised) left clicks, in logical pixels.
                Event::MouseButtonDown { mouse_btn: sdl2::mouse::MouseButton::Left, x, y, .. } => {
                    actions_tap.push((x as f32, y as f32));
                    touches.push((0, x as f32, y as f32));
                }
                Event::MouseMotion { mousestate, x, y, .. } if mousestate.left() => {
                    touches.push((1, x as f32, y as f32));
                }
                Event::MouseButtonUp { mouse_btn: sdl2::mouse::MouseButton::Left, x, y, .. } => {
                    touches.push((2, x as f32, y as f32));
                }
                Event::ControllerDeviceAdded { which, .. } => {
                    if let Ok(c) = game_controller.open(which) {
                        println!("controller connected: {}", c.name());
                        controllers.push(c);
                    }
                }
                Event::ControllerDeviceRemoved { which, .. } => {
                    controllers.retain(|c| c.instance_id() != which);
                    if controllers.is_empty() {
                        pad = Gamepad::default();
                    }
                }
                other => {
                    if let Some(a) = pad.handle(&other) {
                        actions.push(a);
                    }
                }
            }
        }
        if let Some(d) = pad.repeat.poll(Instant::now()) {
            actions.push(Action::Nav(d));
        }

        if args.demo_nav {
            demo::fill(&shared, started.elapsed().as_secs_f32());
        }
        let now = Instant::now();
        let dt = now.duration_since(last_frame).as_secs_f32();
        last_frame = now;
        let (settings, mode, robot_xy, track, frame_size, tune_ctx) = {
            let s = shared.lock().unwrap();
            let fresh = s.state_at.map(|t| t.elapsed().as_secs_f32() < 2.0).unwrap_or(false);
            let st = s.state.as_ref().filter(|_| fresh);
            let mode = st.and_then(|st| st.mode.as_ref()).map(|m| m.mode.clone()).unwrap_or_default();
            let mode_info = st.and_then(|st| st.mode.as_ref());
            let tune_ctx = TuneCtx {
                mode: mode.clone(),
                armed: st.and_then(|st| st.gait.as_ref()).is_some_and(|g| g.armed),
                tune: mode_info.and_then(|m| m.tune.clone()),
                servo: mode_info.and_then(|m| m.servo_cal.clone()),
            };
            let track_fresh = s.track_at.map(|t| t.elapsed().as_secs_f32() < 1.5).unwrap_or(false);
            let track = s.track.clone().filter(|_| track_fresh);
            let frame_size = s.frame.as_ref().filter(|f| f.view == View::Color).map(|f| (f.width, f.height));
            (menu::with_local_items(&s.settings), mode, st.and_then(|st| st.pose.as_ref()).map(|p| (p.x, p.y)), track, frame_size, tune_ctx)
        };
        let mut tune_cmds: Vec<serde_json::Value> = Vec::new();
        let calibrating = mode == "calibrate";
        let auto_nav = mode == "auto_nav";
        let sentry = mode == "sentry";
        let tracking = track.as_ref().is_some_and(|t| t.active());
        // Sentry, on the camera view: touch / D-pad + A pick something to track.
        let video_rect = frame_size.map(|(w, h)| track::video_rect(w, h));
        let picking = sentry && view == View::Color && !menu.open && !menu.controls_open && !tune_screen.open
            && video_rect.is_some();
        let mut track_cmds: Vec<Option<[f32; 4]>> = Vec::new(); // Some = start on that box, None = stop
        if let (true, Some(r)) = (picking, video_rect) {
            for &(kind, x, y) in &touches {
                match kind {
                    0 => pick.press(r, x, y),
                    1 => pick.motion(r, x, y),
                    _ => {
                        if let Some(b) = pick.release(r, x, y) {
                            track_cmds.push(Some(b));
                        }
                    }
                }
            }
        } else {
            pick.cancel_drag();
        }
        let mut nav_cmds: Vec<NavCmd> = Vec::new();
        if auto_nav && !menu.open && !tune_screen.open {
            nav.update(dt, &pad.state, robot_xy);
            nav_cmds.extend(actions_tap.iter().filter_map(|&(x, y)| nav.tap(x, y)));
            nav_cmds.extend(nav.poll(now));
        } else {
            nav.reset_pending();
        }
        for action in actions {
            let change = match action {
                Action::ToggleTune => {
                    tune_cmds.extend(tune_screen.toggle(&tune_ctx));
                    menu.open = false;
                    menu.controls_open = false;
                    None
                }
                // The Tune screen takes the D-pad and face buttons while it is up.
                Action::Nav(_) | Action::Confirm | Action::X | Action::Place if tune_screen.open => {
                    tune_cmds.extend(tune_screen.handle(action, &tune_ctx));
                    None
                }
                Action::NextView | Action::PrevView | Action::ToggleMenu | Action::ToggleControls if tune_screen.open => None,
                // Fallback for a Deck whose back grips don't reach the app: A on the controls screen.
                Action::Confirm if menu.controls_open => {
                    menu.controls_open = false;
                    tune_cmds.extend(tune_screen.toggle(&tune_ctx));
                    None
                }
                // L2/R2 pick the joint in calibration mode and zoom the auto-nav map,
                // so they don't switch views then.
                Action::NextView | Action::PrevView if calibrating || auto_nav => None,
                Action::Confirm if auto_nav && !menu.open => {
                    nav.press_a(now);
                    None
                }
                Action::X if auto_nav && !menu.open => {
                    nav_cmds.extend(nav.press_x());
                    None
                }
                Action::Place if auto_nav && !menu.open => {
                    nav.toggle_place();
                    None
                }
                Action::Nav(d) if auto_nav && !menu.open => {
                    nav_cmds.extend(nav.dpad(d));
                    None
                }
                Action::Nav(d) if picking => {
                    pick.nudge(d);
                    None
                }
                Action::Confirm if picking => {
                    let r = video_rect.unwrap();
                    track_cmds.push(Some(pick.reticle(r.width() as f32 / r.height() as f32)));
                    None
                }
                Action::X if sentry && tracking && !menu.open => {
                    track_cmds.push(None);
                    None
                }
                Action::NextView | Action::PrevView => {
                    view = step_view(view, action == Action::NextView, depth_ok);
                    if selftest_dir.is_some() {
                        println!("selftest: R2 -> view {}", view.wire_name());
                    }
                    None
                }
                Action::ToggleMenu => {
                    menu.toggle();
                    None
                }
                Action::ToggleControls => {
                    menu.toggle_controls();
                    None
                }
                Action::Nav(d) if menu.open => menu.navigate(d, &settings),
                Action::Confirm if menu.open => menu.confirm(&settings),
                _ => None,
            };
            if let Some((key, value)) = change {
                if key == menu::BT_WIFI_KEY {
                    bt::start(&provision);
                    menu.open = false;
                } else if key == menu::EXIT_KEY {
                    break 'main;
                } else if !commander.set(&key, &value) {
                    menu.notify("Not connected — change not sent".into(), false);
                }
            }
        }
        for cmd in nav_cmds {
            let (sent, what) = match cmd {
                NavCmd::Goal(x, y) => (commander.nav_goal(x, y), format!("Walking to {x:.1}, {y:.1} m")),
                NavCmd::Cancel => (commander.nav_cancel(), "Stopping".to_string()),
                // The new speed shows in the nav panel once the robot has it.
                NavCmd::Speed(step) => (commander.nav_speed(step), String::new()),
                NavCmd::Pose(x, y, yaw) => (commander.nav_pose(x, y, yaw), "Placing the robot…".to_string()),
            };
            if args.demo_nav {
                match cmd {
                    NavCmd::Goal(x, y) => demo::set_goal(&shared, x, y),
                    NavCmd::Speed(step) => demo::step_speed(step),
                    NavCmd::Pose(x, y, yaw) => demo::set_pose(x, y, yaw),
                    NavCmd::Cancel => {}
                }
            }
            if sent || args.demo_nav {
                if !what.is_empty() {
                    menu.notify(what, true);
                }
            } else {
                menu.notify("Not connected — not sent".into(), false);
            }
        }
        for cmd in tune_cmds {
            if !commander.tune(&cmd) {
                menu.notify("Not connected — not sent".into(), false);
            }
        }
        for cmd in track_cmds {
            let (sent, what) = match cmd {
                Some(b) => (commander.track_start(b), "Tracking — X to stop"),
                None => (commander.track_stop(), "Tracking stopped"),
            };
            if sent {
                menu.notify(what.into(), true);
            } else {
                menu.notify("Not connected — not sent".into(), false);
            }
        }
        // Results from gait_node (IMU zeroed, calibration done/failed) as toasts.
        // The first note seen is old news, so it only sets the baseline.
        let note = {
            let s = shared.lock().unwrap();
            s.state.as_ref().and_then(|st| st.mode.as_ref()).and_then(|m| m.note.clone())
        };
        if let Some(n) = note {
            if last_note_seq.is_some_and(|seq| seq != n.seq) {
                menu.notify(n.text.clone(), n.ok);
            }
            last_note_seq = Some(n.seq);
        }
        let acks = std::mem::take(&mut shared.lock().unwrap().acks);
        for ack in &acks {
            if ack.key == "set_pose" || ack.key == "autotune_keep" {
                // Not settings: the result of placing the robot on the map / saving autotune's changes.
                let done = if ack.key == "set_pose" { "Robot placed" } else { "Saved" };
                menu.notify(if ack.text.is_empty() { done.into() } else { ack.text.clone() }, ack.ok);
            } else {
                menu.on_ack(ack, &settings);
            }
        }
        view_atomic.store(view.to_u8(), Ordering::Relaxed);
        // While the settings menu is open the D-pad and A drive the menu only, so
        // they mustn't also reach the robot (in calibration mode the D-pad
        // changes servo offsets there).
        let mut sent = pad.state.clone();
        if menu.open {
            for b in [protocol::BTN_A, protocol::BTN_DPAD_UP, protocol::BTN_DPAD_DOWN, protocol::BTN_DPAD_LEFT, protocol::BTN_DPAD_RIGHT] {
                sent.buttons[b] = false;
            }
        }
        if tune_screen.open {
            // Only safety and stack buttons reach the robot: B (estop), L3+R3 (arm),
            // L1/R1 (stack chord, body height). The sticks only during an autotune
            // run, so touching one stops it; otherwise the robot can't walk off
            // while the camera isn't on screen.
            let mut only = input::InputState::default();
            for b in [protocol::BTN_B, protocol::BTN_L3, protocol::BTN_R3, protocol::BTN_L1, protocol::BTN_R1] {
                only.buttons[b] = sent.buttons[b];
            }
            if tune_ctx.running() {
                only.axes[..4].copy_from_slice(&sent.axes[..4]);
            }
            sent = only;
        }
        *input_shared.lock().unwrap() = sent;

        // Robot mic: whatever arrived since the last frame goes to the speakers.
        let audio_live = {
            let mut s = shared.lock().unwrap();
            let live = s.audio_at.is_some_and(|t| t.elapsed() < Duration::from_millis(500));
            let samples = std::mem::take(&mut s.audio);
            let rate = s.audio_rate;
            drop(s);
            player.tick(&samples, rate, live);
            live
        };

        let snap = {
            let s = shared.lock().unwrap();
            let age = |t: Option<Instant>| t.map(|t| t.elapsed().as_secs_f32()).unwrap_or(f32::MAX);
            Snapshot {
                view,
                host: host_label.clone(),
                broker_connected: s.broker_connected,
                robot_online: s.robot_online,
                last_error: s.last_error.clone(),
                state: s.state.clone(),
                state_age: age(s.state_at),
                map: s.map.clone(),
                map_version: s.map_version,
                static_map: s.static_map.clone(),
                static_map_version: s.static_map_version,
                nav_cam: auto_nav.then(|| nav.clone()),
                track: track.clone(),
                track_pick: picking.then(|| pick.clone()),
                nav_a_pending: nav.a_pending(),
                audio_live,
                scan: s.scan.clone(),
                scan_age: age(s.scan_at),
                frame: s.frame.clone(),
                frame_age: age(s.frame_at),
                video_fps: s.video_fps,
                decode_ms: s.decode_ms,
                input: pad.state.clone(),
                gamepad_connected: !controllers.is_empty(),
                settings,
                settings_age: age(s.settings_at),
                tune: tune_screen.open.then(|| (tune_screen.clone(), tune_ctx.clone())),
                provision: provision.lock().unwrap().clone(),
            }
        };
        // Bluetooth setup found the robot's address: save it and reconnect to it —
        // unless we're connected and the robot didn't change networks (nothing to do).
        if let Some(ip) = snap.provision.host.clone().filter(|_| !snap.provision.running) {
            provision.lock().unwrap().host = None;
            if snap.broker_connected && !snap.provision.moved {
                menu.notify(format!("Robot is on the Deck's Wi-Fi ({ip}) — already connected"), true);
            } else {
                save_host(&ip);
                commander.set_host(&ip);
                host_label = format!("{ip}:{port}");
                println!("reconnecting to {host_label}");
                menu.notify(format!("Connecting to the robot at {ip}"), true);
            }
        }
        depth_ok = snap.depth_available();
        ui.render(&mut canvas, &tc, &snap, &menu);

        if let Some((path, after)) = &args.screenshot {
            if elapsed >= *after {
                save_screenshot(&canvas, path)?;
                println!("saved {path}");
                break 'main;
            }
        }
        if args.quit_after.map(|q| elapsed >= q).unwrap_or(false) {
            break 'main;
        }
        canvas.present();
        if args.software || headless {
            std::thread::sleep(Duration::from_millis(16));
        }
    }
    Ok(())
}
