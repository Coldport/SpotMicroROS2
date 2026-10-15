//! MQTT networking: three small threads.
//!   * poll     — drives the rumqttc connection, dispatches incoming messages
//!   * ticker   — publishes controller input (30 Hz) and the view heartbeat (1 Hz),
//!                and swaps the video subscription when the view changes
//!   * decoder  — JPEG-decodes only the newest video frame (older ones are dropped)
//! start() also hands back a Commander, which the UI thread uses to send
//! settings changes.

use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use rumqttc::{Client, Event, LastWill, MqttOptions, Packet, QoS};

use crate::input::InputState;
use crate::protocol::{self, Topics};
use crate::shared::{Frame, RobotState, SettingAck, SettingsMsg, SharedRef, TrackStatus, View};

pub struct NetConfig {
    pub host: String,
    pub port: u16,
    pub prefix: String,
    pub username: Option<String>,
    pub password: Option<String>,
}

type FrameSlot = Arc<(Mutex<Option<(View, Vec<u8>)>>, Condvar)>;

/// Sends commands to the robot from the UI thread (never blocks).
pub struct Commander {
    client: Client,
    topics: Arc<Topics>,
}

impl Commander {
    /// Ask the robot to change one setting; the reply arrives as a SettingAck.
    pub fn set(&self, key: &str, value: &serde_json::Value) -> bool {
        self.client
            .try_publish(&self.topics.deck_set, QoS::AtLeastOnce, false, protocol::encode_set(key, value))
            .is_ok()
    }

    /// Auto nav: walk to (x, y) in the map frame.
    pub fn nav_goal(&self, x: f32, y: f32) -> bool {
        self.client
            .try_publish(&self.topics.deck_nav, QoS::AtLeastOnce, false, protocol::encode_nav_goal(x, y))
            .is_ok()
    }

    /// Auto nav: stop.
    pub fn nav_cancel(&self) -> bool {
        self.client
            .try_publish(&self.topics.deck_nav, QoS::AtLeastOnce, false, protocol::encode_nav_cancel())
            .is_ok()
    }

    /// Auto nav: one speed level up (+1) or down (-1).
    pub fn nav_speed(&self, step: i32) -> bool {
        self.client
            .try_publish(&self.topics.deck_nav, QoS::AtLeastOnce, false, protocol::encode_nav_speed(step))
            .is_ok()
    }

    /// Auto nav: the robot is here (map frame); the result comes back as a "set_pose" ack.
    pub fn nav_pose(&self, x: f32, y: f32, yaw: f32) -> bool {
        self.client
            .try_publish(&self.topics.deck_nav, QoS::AtLeastOnce, false, protocol::encode_nav_pose(x, y, yaw))
            .is_ok()
    }

    /// Sentry: track what's in this normalised camera-image box.
    pub fn track_start(&self, b: [f32; 4]) -> bool {
        self.client
            .try_publish(&self.topics.deck_track, QoS::AtLeastOnce, false, protocol::encode_track_start(b))
            .is_ok()
    }

    /// Tune screen: an autotune / servo calibration command (protocol.py decode_tune).
    pub fn tune(&self, cmd: &serde_json::Value) -> bool {
        self.client
            .try_publish(&self.topics.deck_tune, QoS::AtLeastOnce, false, cmd.to_string())
            .is_ok()
    }

    /// Sentry: stop tracking.
    pub fn track_stop(&self) -> bool {
        self.client
            .try_publish(&self.topics.deck_track, QoS::AtLeastOnce, false, protocol::encode_track_stop())
            .is_ok()
    }
}

pub fn start(
    cfg: NetConfig,
    shared: SharedRef,
    view: Arc<AtomicU8>,
    input: Arc<Mutex<InputState>>,
) -> Commander {
    let topics = Arc::new(Topics::new(&cfg.prefix));

    let mut opts = MqttOptions::new("spotmicro-deck", cfg.host.clone(), cfg.port);
    opts.set_keep_alive(Duration::from_secs(5));
    opts.set_clean_session(true);
    // Video frames are tens of KB; rumqttc's default packet limit is 10 KB.
    opts.set_max_packet_size(4 * 1024 * 1024, 256 * 1024);
    opts.set_last_will(LastWill::new(&topics.deck_online, "0", QoS::AtLeastOnce, true));
    if let (Some(u), Some(p)) = (&cfg.username, &cfg.password) {
        opts.set_credentials(u.clone(), p.clone());
    }
    let (client, mut connection) = Client::new(opts, 256);

    let commander = Commander { client: client.clone(), topics: topics.clone() };
    let slot: FrameSlot = Arc::new((Mutex::new(None), Condvar::new()));
    let subscribed_video: Arc<Mutex<Option<View>>> = Arc::new(Mutex::new(None));

    // ── decoder ──────────────────────────────────────────────────────────────
    {
        let slot = slot.clone();
        let shared = shared.clone();
        thread::spawn(move || decode_loop(slot, shared));
    }

    // ── ticker ───────────────────────────────────────────────────────────────
    {
        let client = client.clone();
        let topics = topics.clone();
        let shared = shared.clone();
        let subscribed_video = subscribed_video.clone();
        thread::spawn(move || {
            let mut seq: u32 = 0;
            let mut last_view = View::Color;
            let mut last_heartbeat = Instant::now() - Duration::from_secs(10);
            loop {
                thread::sleep(Duration::from_millis(33));
                if !shared.lock().unwrap().broker_connected {
                    continue;
                }
                let v = View::from_u8(view.load(Ordering::Relaxed));

                // Only receive the video stream that is on screen.
                {
                    let mut sub = subscribed_video.lock().unwrap();
                    let wanted = if v.is_video() { Some(v) } else { None };
                    if *sub != wanted {
                        if let Some(old) = *sub {
                            let _ = client.try_unsubscribe(video_topic(&topics, old));
                        }
                        if let Some(new) = wanted {
                            let _ = client.try_subscribe(video_topic(&topics, new), QoS::AtMostOnce);
                        }
                        *sub = wanted;
                    }
                }

                seq = seq.wrapping_add(1);
                let (axes, buttons) = {
                    let s = input.lock().unwrap();
                    (s.axes, s.buttons)
                };
                let _ = client.try_publish(
                    &topics.deck_input,
                    QoS::AtMostOnce,
                    false,
                    protocol::encode_input(seq, &axes, &buttons),
                );

                if v != last_view || last_heartbeat.elapsed() >= Duration::from_secs(1) {
                    let _ = client.try_publish(
                        &topics.deck_view,
                        QoS::AtMostOnce,
                        false,
                        v.wire_name(),
                    );
                    last_view = v;
                    last_heartbeat = Instant::now();
                }
            }
        });
    }

    // ── poll ─────────────────────────────────────────────────────────────────
    thread::spawn(move || {
        for event in connection.iter() {
            match event {
                Ok(Event::Incoming(Packet::ConnAck(_))) => {
                    {
                        let mut s = shared.lock().unwrap();
                        s.broker_connected = true;
                        s.last_error.clear();
                    }
                    *subscribed_video.lock().unwrap() = None; // clean session: resubscribe
                    let _ = client.try_publish(&topics.deck_online, QoS::AtLeastOnce, true, "1");
                    let _ = client.try_subscribe(&topics.state, QoS::AtMostOnce);
                    let _ = client.try_subscribe(&topics.scan, QoS::AtMostOnce);
                    let _ = client.try_subscribe(&topics.map, QoS::AtLeastOnce);
                    let _ = client.try_subscribe(&topics.static_map, QoS::AtLeastOnce);
                    let _ = client.try_subscribe(&topics.robot_online, QoS::AtLeastOnce);
                    let _ = client.try_subscribe(&topics.settings, QoS::AtMostOnce);
                    let _ = client.try_subscribe(&topics.settings_ack, QoS::AtLeastOnce);
                    let _ = client.try_subscribe(&topics.track, QoS::AtMostOnce);
                    let _ = client.try_subscribe(&topics.audio, QoS::AtMostOnce);
                }
                Ok(Event::Incoming(Packet::Publish(p))) => {
                    dispatch(&topics, &p.topic, &p.payload, &shared, &slot);
                }
                Ok(_) => {}
                Err(e) => {
                    {
                        let mut s = shared.lock().unwrap();
                        s.broker_connected = false;
                        s.robot_online = None;
                        s.last_error = e.to_string();
                    }
                    thread::sleep(Duration::from_secs(1)); // then iter() reconnects
                }
            }
        }
    });
    commander
}

fn video_topic(topics: &Topics, view: View) -> String {
    match view {
        View::Depth => topics.video_depth.clone(),
        _ => topics.video_color.clone(),
    }
}

fn dispatch(topics: &Topics, topic: &str, payload: &[u8], shared: &SharedRef, slot: &FrameSlot) {
    if topic == topics.video_color || topic == topics.video_depth {
        let view = if topic == topics.video_depth { View::Depth } else { View::Color };
        let (lock, cv) = &**slot;
        *lock.lock().unwrap() = Some((view, payload.to_vec())); // newest wins
        cv.notify_one();
    } else if topic == topics.state {
        match serde_json::from_slice::<RobotState>(payload) {
            Ok(st) => {
                let mut s = shared.lock().unwrap();
                s.state = Some(st);
                s.state_at = Some(Instant::now());
            }
            Err(e) => shared.lock().unwrap().last_error = format!("state: {e}"),
        }
    } else if topic == topics.map {
        match protocol::parse_map(payload) {
            Ok(m) => {
                let mut s = shared.lock().unwrap();
                s.map = Some(Arc::new(m));
                s.map_version += 1;
            }
            Err(e) => shared.lock().unwrap().last_error = format!("map: {e}"),
        }
    } else if topic == topics.static_map {
        // Empty (retained) payload: no map saved on the robot.
        let parsed = if payload.is_empty() { Ok(None) } else { protocol::parse_map(payload).map(Some) };
        match parsed {
            Ok(m) => {
                let mut s = shared.lock().unwrap();
                s.static_map = m.map(Arc::new);
                s.static_map_version += 1;
            }
            Err(e) => shared.lock().unwrap().last_error = format!("static map: {e}"),
        }
    } else if topic == topics.scan {
        if let Ok(sc) = protocol::parse_scan(payload) {
            let mut s = shared.lock().unwrap();
            s.scan = Some(Arc::new(sc));
            s.scan_at = Some(Instant::now());
        }
    } else if topic == topics.settings {
        match serde_json::from_slice::<SettingsMsg>(payload) {
            Ok(m) => {
                let mut s = shared.lock().unwrap();
                s.settings = m.items;
                s.settings_at = Some(Instant::now());
            }
            Err(e) => shared.lock().unwrap().last_error = format!("settings: {e}"),
        }
    } else if topic == topics.audio {
        if let Ok((protocol::AUDIO_MULAW, rate, _seq, bytes)) = protocol::parse_audio(payload) {
            let mut s = shared.lock().unwrap();
            s.audio_rate = rate as u32;
            s.audio_at = Some(Instant::now());
            s.audio.extend(bytes.iter().map(|&b| crate::audio::mulaw_to_i16(b)));
            // The UI thread drains this every frame; if it stalls, keep the newest second.
            let cap = rate as usize;
            if s.audio.len() > cap {
                let extra = s.audio.len() - cap;
                s.audio.drain(..extra);
            }
        }
    } else if topic == topics.settings_ack {
        if let Ok(ack) = serde_json::from_slice::<SettingAck>(payload) {
            shared.lock().unwrap().acks.push(ack);
        }
    } else if topic == topics.track {
        if let Ok(t) = serde_json::from_slice::<TrackStatus>(payload) {
            let mut s = shared.lock().unwrap();
            s.track = Some(t);
            s.track_at = Some(Instant::now());
        }
    } else if topic == topics.robot_online {
        shared.lock().unwrap().robot_online = Some(payload == b"1");
    }
}

fn decode_loop(slot: FrameSlot, shared: SharedRef) {
    let seq = AtomicU64::new(0);
    let mut fps = 0.0f32;
    let mut decode_ms = 0.0f32;
    // Frames are counted over a 1 s window. (Averaging 1/dt per frame overstates
    // the rate badly, because Wi-Fi delivers frames in bursts.)
    let mut window_start = Instant::now();
    let mut window_frames = 0u32;
    loop {
        let (view, jpeg) = {
            let (lock, cv) = &*slot;
            let mut guard = lock.lock().unwrap();
            while guard.is_none() {
                guard = cv.wait(guard).unwrap();
            }
            guard.take().unwrap()
        };
        let started = Instant::now();
        let mut decoder = jpeg_decoder::Decoder::new(&jpeg[..]);
        let pixels = match decoder.decode() {
            Ok(p) => p,
            Err(e) => {
                shared.lock().unwrap().last_error = format!("jpeg: {e}");
                continue;
            }
        };
        let info = match decoder.info() {
            Some(i) => i,
            None => continue,
        };
        let rgb = match info.pixel_format {
            jpeg_decoder::PixelFormat::RGB24 => pixels,
            jpeg_decoder::PixelFormat::L8 => pixels.iter().flat_map(|&g| [g, g, g]).collect(),
            _ => continue,
        };
        let ms = started.elapsed().as_secs_f32() * 1000.0;
        decode_ms = if decode_ms == 0.0 { ms } else { decode_ms * 0.9 + ms * 0.1 };
        window_frames += 1;
        let window = window_start.elapsed().as_secs_f32();
        if window >= 1.0 {
            fps = window_frames as f32 / window;
            window_frames = 0;
            window_start = Instant::now();
        }

        let frame = Arc::new(Frame {
            seq: seq.fetch_add(1, Ordering::Relaxed) + 1,
            view,
            width: info.width as u32,
            height: info.height as u32,
            rgb,
        });
        let mut s = shared.lock().unwrap();
        s.frame = Some(frame);
        s.frame_at = Some(Instant::now());
        s.video_fps = fps;
        s.decode_ms = decode_ms;
    }
}
