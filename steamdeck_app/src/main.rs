//! SpotMicro Steam Deck app: live camera / depth / lidar / map from the robot,
//! the Deck's controller input sent back to it, and a settings menu (Start/☰)
//! for the robot's live-tunable gait and video settings — all over MQTT.
//!
//! Usage: spotmicro-deck [--host HOST[:PORT]] [--port N] [--user U --password P]
//!                       [--prefix TOPIC_PREFIX] [--windowed] [--menu]
//! The host can also come from $SPOTDECK_HOST or ~/.config/spotmicro-deck/host.

mod bt;
mod input;
mod menu;
mod net;
mod protocol;
mod shared;
mod text;
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
use shared::{Shared, View};
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
    /// Start on this view (color | depth | lidar | map).
    view: Option<View>,
    /// Save a screenshot of the current view after N seconds, then quit.
    screenshot: Option<(String, f32)>,
    /// Scripted L2 presses with a screenshot of each view saved into this directory.
    selftest: Option<String>,
    quit_after: Option<f32>,
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
            "--quit-after" => a.quit_after = Some(value("--quit-after")?.parse().map_err(|_| "bad --quit-after")?),
            "-h" | "--help" => {
                println!(
                    "spotmicro-deck [--host HOST[:PORT]] [--port N] [--user U --password P] \
                     [--prefix P] [--windowed] [--menu]\n\
                     Start/☰ opens the robot settings menu (D-pad to select and adjust).\n\
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

/// Remember `host` (~/.config/spotmicro-deck/host) and re-exec this app with
/// `--host host`, so every thread reconnects to it from scratch.
fn restart_with_host(host: &str) -> Result<(), String> {
    use std::os::unix::process::CommandExt;
    if let Some(dir) = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .or_else(|| std::env::var("HOME").ok().map(|h| format!("{h}/.config")))
    {
        let dir = format!("{dir}/spotmicro-deck");
        if let Err(e) = std::fs::create_dir_all(&dir).and_then(|_| std::fs::write(format!("{dir}/host"), format!("{host}\n"))) {
            eprintln!("couldn't save the robot address in {dir}/host: {e}");
        }
    }
    let mut args: Vec<String> = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        if a == "--host" {
            it.next();
        } else {
            args.push(a);
        }
    }
    args.extend(["--host".to_string(), host.to_string()]);
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    println!("restarting on {host}");
    Err(format!("restart failed: {}", std::process::Command::new(exe).args(args).exec()))
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
    let host_label = format!("{host}:{port}");

    // Steam Input can deliver events while another window (Steam's overlay) has focus.
    sdl2::hint::set("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1");
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let game_controller = sdl.game_controller()?;
    let mut pump = sdl.event_pump()?;
    sdl.mouse().show_cursor(false);

    let headless = matches!(std::env::var("SDL_VIDEODRIVER").as_deref(), Ok("dummy") | Ok("offscreen"));
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
    let mut menu = Menu::default();
    menu.open = args.menu;
    let mut last_note_seq: Option<u64> = None;
    let mut depth_ok = true;
    // L2 / Tab: next view, skipping Depth when the robot's camera has none.
    let next_view = |v: View, depth_ok: bool| {
        let n = v.next();
        if n == View::Depth && !depth_ok {
            n.next()
        } else {
            n
        }
    };
    let started = Instant::now();
    let mut selftest_step = 0usize;
    let mut pending_release: Option<f32> = None;
    // (seconds, take screenshot named X, then press L2) — see below
    let selftest_dir = args.selftest.clone();
    if let Some(dir) = &selftest_dir {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    'main: loop {
        let elapsed = started.elapsed().as_secs_f32();

        // Inject scripted events for --selftest: hold each view for 6 s,
        // screenshot it at the end, then pull and release L2.
        let mut injected: Vec<Event> = Vec::new();
        if let Some(dir) = &selftest_dir {
            let phase_end = 8.0 + selftest_step as f32 * 6.0;
            if elapsed >= phase_end && pending_release.is_none() && selftest_step < 5 {
                let path = format!("{dir}/view_{}_{}.png", selftest_step, view.wire_name());
                save_screenshot(&canvas, &path)?;
                println!("selftest: saved {path}");
                if selftest_step < 4 {
                    injected.push(axis_event(Axis::TriggerLeft, 32767)); // pull L2
                    pending_release = Some(elapsed + 0.3);
                }
                selftest_step += 1;
                if selftest_step >= 5 {
                    break 'main;
                }
            }
            if let Some(t) = pending_release {
                if elapsed >= t {
                    injected.push(axis_event(Axis::TriggerLeft, 0)); // release L2
                    injected.push(axis_event(Axis::LeftX, -20000)); // some stick movement
                    injected.push(axis_event(Axis::LeftY, -32767));
                    pending_release = None;
                }
            }
        }

        let events: Vec<Event> = pump.poll_iter().chain(injected).collect();
        let mut actions: Vec<Action> = Vec::new();
        for ev in events {
            match ev {
                Event::Quit { .. } | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => break 'main,
                // Keyboard equivalents, for testing on a PC.
                Event::KeyDown { keycode: Some(k), .. } => match k {
                    Keycode::Tab | Keycode::Space => actions.push(Action::NextView),
                    Keycode::M | Keycode::F1 => actions.push(Action::ToggleMenu),
                    Keycode::Up => actions.push(Action::Nav(Dir::Up)),
                    Keycode::Down => actions.push(Action::Nav(Dir::Down)),
                    Keycode::Left => actions.push(Action::Nav(Dir::Left)),
                    Keycode::Right => actions.push(Action::Nav(Dir::Right)),
                    Keycode::Return => actions.push(Action::Confirm),
                    Keycode::B => actions.push(Action::Bluetooth),
                    _ => {}
                },
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

        let (settings, calibrating) = {
            let s = shared.lock().unwrap();
            let calibrating = s.state_at.map(|t| t.elapsed().as_secs_f32() < 2.0).unwrap_or(false)
                && s.state.as_ref().and_then(|st| st.mode.as_ref()).map(|m| m.mode == "calibrate").unwrap_or(false);
            (s.settings.clone(), calibrating)
        };
        for action in actions {
            let change = match action {
                // L2 picks the joint in calibration mode, so it doesn't switch views then.
                Action::NextView if calibrating => None,
                Action::NextView => {
                    view = next_view(view, depth_ok);
                    if selftest_dir.is_some() {
                        println!("selftest: L2 -> view {}", view.wire_name());
                    }
                    None
                }
                Action::ToggleMenu => {
                    menu.toggle();
                    None
                }
                Action::Nav(d) if menu.open => menu.navigate(d, &settings),
                Action::Confirm if menu.open => menu.confirm(&settings),
                // Only offered while there's no link: on the same network, nothing to set up.
                Action::Bluetooth if !shared.lock().unwrap().broker_connected => {
                    bt::start(&provision);
                    None
                }
                _ => None,
            };
            if let Some((key, value)) = change {
                if !commander.set(&key, &value) {
                    menu.notify("Not connected — change not sent".into(), false);
                }
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
            menu.on_ack(ack, &settings);
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
        *input_shared.lock().unwrap() = sent;

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
                provision: provision.lock().unwrap().clone(),
            }
        };
        // Bluetooth setup found the robot's address: save it and restart on it.
        if let Some(ip) = snap.provision.host.clone().filter(|_| !snap.provision.running) {
            if format!("{ip}:{port}") != host_label {
                ui.render(&mut canvas, &tc, &snap, &menu);
                canvas.present();
                restart_with_host(&ip)?;
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
