//! Wi-Fi setup over Bluetooth, for when the Deck can't reach the robot's MQTT
//! broker because they're on different networks. The robot side is
//! src/spot_micro_deck/spot_micro_deck/wifi_provision.py — see there for the
//! line-delimited JSON protocol on RFCOMM channel 22.
//!
//! Steps (on a background thread; progress goes into `Provision`):
//!   1. find the robot by Bluetooth name (paired devices first, else a scan)
//!   2. pair + trust it if needed ("just works" — no PIN)
//!   3. connect RFCOMM and read its hello (its current Wi-Fi + IP)
//!   4. same network as the Deck already? just use the IP it reported.
//!      Otherwise send the Deck's Wi-Fi SSID + password (from NetworkManager,
//!      or ~/.config/spotmicro-deck/wifi: SSID on line 1, password on line 2)
//!      and wait for the robot to join and report its new IP.
//! main.rs then saves that IP as the host and restarts the app on it.
//! Uses bluetoothctl / nmcli (both on SteamOS) and a raw RFCOMM socket.

use std::io::{BufRead, BufReader, Write};
use std::os::fd::FromRawFd;
use std::os::unix::net::UnixStream;
use std::process::Command;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use serde::Deserialize;

pub const ROBOT_NAME: &str = "SpotMicro";
pub const RFCOMM_CHANNEL: u8 = 22;

#[derive(Clone, Default, Debug)]
pub struct Provision {
    pub running: bool,
    /// What it is doing now (shown on screen).
    pub step: String,
    pub error: Option<String>,
    /// Robot IP once done; main.rs reconnects to it.
    pub host: Option<String>,
}

pub type ProvisionRef = Arc<Mutex<Provision>>;

/// Start the setup unless it is already running.
pub fn start(p: &ProvisionRef) {
    {
        let mut s = p.lock().unwrap();
        if s.running {
            return;
        }
        *s = Provision { running: true, step: "Starting…".into(), ..Default::default() };
    }
    let p = p.clone();
    thread::spawn(move || {
        let result = run(&p);
        let mut s = p.lock().unwrap();
        s.running = false;
        match result {
            Ok(ip) => s.host = Some(ip),
            Err(e) => s.error = Some(e),
        }
    });
}

fn step(p: &ProvisionRef, text: impl Into<String>) {
    let text = text.into();
    println!("bluetooth: {text}");
    p.lock().unwrap().step = text;
}

#[derive(Deserialize)]
struct Hello {
    #[serde(default)]
    ssid: Option<String>,
    #[serde(default)]
    ip: Option<String>,
}

#[derive(Deserialize)]
struct Reply {
    #[serde(default)]
    progress: Option<String>,
    #[serde(default)]
    ok: Option<bool>,
    #[serde(default)]
    ip: Option<String>,
    #[serde(default)]
    text: String,
}

fn run(p: &ProvisionRef) -> Result<String, String> {
    step(p, format!("Looking for “{ROBOT_NAME}” over Bluetooth…"));
    let mac = match find_robot()? {
        Some(mac) => mac,
        None => {
            step(p, "Scanning for the robot (12 s)…");
            let _ = bluetoothctl(&["--timeout", "12", "scan", "on"]);
            find_robot()?.ok_or_else(|| {
                format!(
                    "No “{ROBOT_NAME}” found. Is the robot on? New devices can only pair in its first \
                     10 minutes after boot (or while its Wi-Fi is down)."
                )
            })?
        }
    };
    if !bluetoothctl(&["info", &mac]).map(|o| o.contains("Paired: yes")).unwrap_or(false) {
        step(p, format!("Pairing with {ROBOT_NAME} ({mac})…"));
        let out = bluetoothctl(&["--agent", "NoInputNoOutput", "--timeout", "30", "pair", &mac])?;
        if !(out.contains("Pairing successful") || out.contains("AlreadyExists")) {
            // Not fatal: the RFCOMM connect below triggers pairing itself if it's needed.
            println!("bluetooth: pair said: {out}");
        }
        let _ = bluetoothctl(&["trust", &mac]);
    }

    step(p, format!("Connecting to {ROBOT_NAME}…"));
    let stream = rfcomm_connect(&mac, RFCOMM_CHANNEL)?;
    stream.set_read_timeout(Some(Duration::from_secs(15))).map_err(|e| e.to_string())?;
    let mut writer = stream.try_clone().map_err(|e| e.to_string())?;
    let mut reader = BufReader::new(stream);
    let hello: Hello = serde_json::from_str(&read_line(&mut reader)?).map_err(|e| format!("robot said: {e}"))?;

    if let (Some(robot_ssid), Some(ip), Some(ours)) = (&hello.ssid, &hello.ip, deck_ssid()) {
        if *robot_ssid == ours {
            step(p, format!("Already on “{ours}” together — robot is {ip}"));
            return Ok(ip.clone());
        }
    }
    let (ssid, psk) = deck_wifi()?;
    step(p, format!("Sending Wi-Fi “{ssid}” to the robot…"));
    let msg = serde_json::json!({"cmd": "wifi", "ssid": ssid, "psk": psk});
    writer.write_all(format!("{msg}\n").as_bytes()).map_err(|e| format!("send failed: {e}"))?;

    let deadline = Instant::now() + Duration::from_secs(120);
    while Instant::now() < deadline {
        let reply: Reply = serde_json::from_str(&read_line(&mut reader)?).map_err(|e| format!("robot said: {e}"))?;
        if let Some(s) = reply.progress {
            step(p, format!("Robot: {s}…"));
            continue;
        }
        return match (reply.ok, reply.ip) {
            (Some(true), Some(ip)) => {
                step(p, format!("Robot joined “{ssid}” as {ip}"));
                Ok(ip)
            }
            _ => Err(if reply.text.is_empty() { "the robot couldn't join the network".into() } else { reply.text }),
        };
    }
    Err("timed out waiting for the robot to join".into())
}

fn read_line(r: &mut BufReader<UnixStream>) -> Result<String, String> {
    // The robot can take a while (netplan apply + DHCP): retry the 15 s read timeout a few times.
    for _ in 0..8 {
        let mut line = String::new();
        match r.read_line(&mut line) {
            Ok(0) => return Err("the robot closed the Bluetooth connection".into()),
            Ok(_) => return Ok(line),
            Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => continue,
            Err(e) => return Err(format!("Bluetooth read failed: {e}")),
        }
    }
    Err("the robot stopped answering".into())
}

fn bluetoothctl(args: &[&str]) -> Result<String, String> {
    let out = Command::new("bluetoothctl")
        .args(args)
        .output()
        .map_err(|e| format!("can't run bluetoothctl ({e}) — is Bluetooth on?"))?;
    Ok(format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr)))
}

/// MAC of a known device called ROBOT_NAME.
fn find_robot() -> Result<Option<String>, String> {
    Ok(parse_devices(&bluetoothctl(&["devices"])?, ROBOT_NAME))
}

/// "Device AA:BB:CC:DD:EE:FF Name" lines -> MAC of the first whose name matches.
fn parse_devices(out: &str, name: &str) -> Option<String> {
    out.lines().find_map(|line| {
        let rest = line.trim().strip_prefix("Device ")?;
        let (mac, dev_name) = rest.split_once(' ')?;
        (mac.len() == 17 && dev_name.trim().eq_ignore_ascii_case(name)).then(|| mac.to_string())
    })
}

/// nmcli -t / -g output escapes ':' and '\' with a backslash.
fn nm_unescape(s: &str) -> String {
    let mut out = String::new();
    let mut esc = false;
    for c in s.chars() {
        if esc {
            out.push(c);
            esc = false;
        } else if c == '\\' {
            esc = true;
        } else {
            out.push(c);
        }
    }
    out
}

/// Split one `nmcli -t` line on unescaped ':'.
fn nm_fields(line: &str) -> Vec<String> {
    let mut fields = vec![String::new()];
    let mut esc = false;
    for c in line.chars() {
        if esc {
            fields.last_mut().unwrap().push(c);
            esc = false;
        } else if c == '\\' {
            esc = true;
        } else if c == ':' {
            fields.push(String::new());
        } else {
            fields.last_mut().unwrap().push(c);
        }
    }
    fields
}

fn nmcli(args: &[&str]) -> Option<String> {
    let out = Command::new("nmcli").args(args).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim_end_matches('\n').to_string())
}

/// The Deck's current Wi-Fi (SSID, password; "" = open network).
fn deck_wifi() -> Result<(String, String), String> {
    if let Some(found) = nm_wifi() {
        return Ok(found);
    }
    let dir = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .or_else(|| std::env::var("HOME").ok().map(|h| format!("{h}/.config")))
        .unwrap_or_default();
    let path = format!("{dir}/spotmicro-deck/wifi");
    let text = std::fs::read_to_string(&path).map_err(|_| {
        format!("Can't read this Deck's Wi-Fi password from NetworkManager. Put the SSID and password on two lines in {path}.")
    })?;
    let mut lines = text.lines();
    let ssid = lines.next().unwrap_or("").trim().to_string();
    let psk = lines.next().unwrap_or("").trim().to_string();
    if ssid.is_empty() {
        return Err(format!("{path}: first line must be the Wi-Fi name"));
    }
    Ok((ssid, psk))
}

/// SSID the Deck is connected to right now.
fn deck_ssid() -> Option<String> {
    let out = nmcli(&["-t", "-f", "ACTIVE,SSID", "device", "wifi"])?;
    out.lines().map(nm_fields).find(|f| f.len() >= 2 && f[0] == "yes").map(|mut f| f.remove(1))
}

fn nm_wifi() -> Option<(String, String)> {
    let active = nmcli(&["-t", "-f", "NAME,TYPE", "connection", "show", "--active"])?;
    let name = active.lines().map(nm_fields).find(|f| f.len() >= 2 && f[1] == "802-11-wireless")?.remove(0);
    let get = |field: &str, secrets: bool| {
        let mut args = vec!["-g", field, "connection", "show", "id", name.as_str()];
        if secrets {
            args.insert(0, "-s");
        }
        nmcli(&args).map(|v| nm_unescape(&v))
    };
    let ssid = get("802-11-wireless.ssid", false).filter(|s| !s.is_empty())?;
    let key_mgmt = get("802-11-wireless-security.key-mgmt", false).unwrap_or_default();
    if key_mgmt.is_empty() || key_mgmt == "none" {
        return Some((ssid, String::new()));
    }
    let psk = get("802-11-wireless-security.psk", true).filter(|s| !s.is_empty())?;
    Some((ssid, psk))
}

// ── RFCOMM (no crate for it in std; plain BlueZ sockets) ─────────────────────

const BTPROTO_RFCOMM: libc::c_int = 3;

#[repr(C)]
struct SockaddrRc {
    rc_family: libc::sa_family_t,
    rc_bdaddr: [u8; 6], // little endian: reversed from the printed form
    rc_channel: u8,
}

fn parse_mac(mac: &str) -> Option<[u8; 6]> {
    let bytes: Vec<u8> = mac.split(':').map(|b| u8::from_str_radix(b, 16)).collect::<Result<_, _>>().ok()?;
    let mut out: [u8; 6] = bytes.try_into().ok()?;
    out.reverse();
    Some(out)
}

fn rfcomm_connect(mac: &str, channel: u8) -> Result<UnixStream, String> {
    let bdaddr = parse_mac(mac).ok_or_else(|| format!("bad Bluetooth address {mac}"))?;
    let addr = SockaddrRc { rc_family: libc::AF_BLUETOOTH as libc::sa_family_t, rc_bdaddr: bdaddr, rc_channel: channel };
    // SAFETY: plain socket()/connect() on an fd we own; on success the fd is
    // handed to UnixStream (any stream socket works for read/write/timeouts),
    // which closes it on drop.
    unsafe {
        let fd = libc::socket(libc::AF_BLUETOOTH, libc::SOCK_STREAM | libc::SOCK_CLOEXEC, BTPROTO_RFCOMM);
        if fd < 0 {
            return Err(format!("Bluetooth socket: {}", std::io::Error::last_os_error()));
        }
        let rc = libc::connect(
            fd,
            &addr as *const SockaddrRc as *const libc::sockaddr,
            std::mem::size_of::<SockaddrRc>() as libc::socklen_t,
        );
        if rc < 0 {
            let err = std::io::Error::last_os_error();
            libc::close(fd);
            return Err(format!(
                "couldn't connect to {ROBOT_NAME} ({err}). Is spotmicro-wifi-provision running on the robot?"
            ));
        }
        Ok(UnixStream::from_raw_fd(fd))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_robot_in_device_list() {
        let out = "Device 11:22:33:44:55:66 Steam Controller\nDevice B8:27:EB:01:02:03 SpotMicro\n";
        assert_eq!(parse_devices(out, "spotmicro"), Some("B8:27:EB:01:02:03".into()));
        assert_eq!(parse_devices(out, "Other"), None);
    }

    #[test]
    fn mac_is_reversed_for_the_socket() {
        assert_eq!(parse_mac("B8:27:EB:01:02:03"), Some([0x03, 0x02, 0x01, 0xEB, 0x27, 0xB8]));
        assert_eq!(parse_mac("nope"), None);
    }

    #[test]
    fn nmcli_escapes() {
        assert_eq!(nm_fields(r"My\:Net:802-11-wireless"), vec!["My:Net", "802-11-wireless"]);
        assert_eq!(nm_unescape(r"pa\:ss\\word"), r"pa:ss\word");
    }

    #[test]
    fn sockaddr_rc_matches_the_kernel_layout() {
        assert_eq!(std::mem::size_of::<SockaddrRc>(), 10);
    }
}
