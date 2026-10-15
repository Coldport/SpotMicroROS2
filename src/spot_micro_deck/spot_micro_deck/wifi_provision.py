"""
Wi-Fi setup over Bluetooth: lets the Steam Deck put the robot on its Wi-Fi
network when the two aren't on the same one (so MQTT can't connect).

Runs as root (it rewrites netplan), as its own systemd service
(systemd/spotmicro-wifi-provision.service), with no ROS dependency, since it
has to work exactly when the network doesn't. Needs BlueZ (bluetoothd +
bluetoothctl).

Bluetooth side:
  * Keeps a `bluetoothctl` session running as the pairing agent
    (NoInputNoOutput, i.e. "just works" pairing, every prompt answered yes)
    and sets the adapter's alias to `--name` (default "SpotMicro"), which is
    what the Deck looks for.
  * The adapter is discoverable/pairable only for `--open-window` seconds
    after start, or while the robot has no Wi-Fi connection — an
    already-paired Deck can connect at any time. That keeps strangers from
    pairing with the robot and moving it onto their network while it is
    happily on yours.
  * Listens on RFCOMM channel `--channel` (default 22; fixed, no SDP record).

Protocol: one JSON object per line, UTF-8.
  robot -> Deck on connect:
      {"v": 1, "name": "SpotMicro", "ssid": "home" | null, "ip": "192.168.1.185" | null}
  Deck -> robot:
      {"cmd": "wifi", "ssid": "...", "psk": "..."}     ("psk": "" = open network)
  robot -> Deck while joining, then the result:
      {"progress": "Joining \"...\""}
      {"ok": true,  "ssid": "...", "ip": "10.0.0.7", "text": ""}
      {"ok": false, "ssid": "...", "ip": null, "text": "why"}
A network that can't be joined is removed again, so a typo can't strand the
robot off its old network.

Joined networks are kept in /etc/netplan/60-spotmicro-wifi.yaml (merged with
the system's own netplan files, so the old networks still work too), and the
new one is selected straight away with wpa_cli.
"""

import argparse
import json
import os
import re
import select
import socket
import struct
import subprocess
import threading
import time

import yaml

NETPLAN_FILE = '/etc/netplan/60-spotmicro-wifi.yaml'
JOIN_TIMEOUT_S = 45.0
_CLIENT_TIMEOUT_S = 120.0


# ── netplan (pure, unit-tested) ──────────────────────────────────────────────

def netplan_with_network(text, iface, ssid, psk):
    """Our netplan YAML (text, may be empty) with `ssid` added/updated on `iface`."""
    doc = yaml.safe_load(text) if text and text.strip() else None
    if not isinstance(doc, dict):
        doc = {}
    net = doc.setdefault('network', {})
    net['version'] = 2
    wifi = net.setdefault('wifis', {}).setdefault(iface, {})
    wifi.setdefault('optional', True)
    wifi.setdefault('dhcp4', True)
    aps = wifi.setdefault('access-points', {})
    aps[ssid] = {'password': psk} if psk else {}
    return yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)


def netplan_without_network(text, iface, ssid):
    doc = yaml.safe_load(text) if text and text.strip() else None
    try:
        del doc['network']['wifis'][iface]['access-points'][ssid]
    except (TypeError, KeyError):
        return text
    return yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)


def valid_credentials(ssid, psk):
    """-> error text, or '' if usable (WPA-PSK: 8..63 chars, or 64 hex)."""
    if not isinstance(ssid, str) or not 1 <= len(ssid.encode()) <= 32:
        return 'SSID must be 1-32 bytes'
    if not isinstance(psk, str):
        return 'password must be text'
    if psk and not (8 <= len(psk) <= 63 or re.fullmatch(r'[0-9a-fA-F]{64}', psk)):
        return 'Wi-Fi password must be 8-63 characters'
    return ''


# ── system helpers ───────────────────────────────────────────────────────────

def _run(*cmd, timeout=30.0):
    try:
        p = subprocess.run(cmd, capture_output=True, check=False, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    return p.returncode, (p.stdout + p.stderr).strip()


def wifi_status(iface):
    """-> (ssid or None, ipv4 or None) of the current connection."""
    _, out = _run('wpa_cli', '-i', iface, 'status', timeout=5.0)
    fields = dict(ln.split('=', 1) for ln in out.splitlines() if '=' in ln)
    ssid = fields.get('ssid') if fields.get('wpa_state') == 'COMPLETED' else None
    _, out = _run('ip', '-4', '-o', 'addr', 'show', 'dev', iface, timeout=5.0)
    m = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', out)
    return ssid, (m.group(1) if m else None)


def _select_network(iface, ssid):
    """Make wpa_supplicant switch to `ssid` now instead of whichever it prefers."""
    _, out = _run('wpa_cli', '-i', iface, 'list_networks', timeout=5.0)
    for line in out.splitlines()[1:]:
        cols = line.split('\t')
        if len(cols) >= 2 and cols[1] == ssid:
            _run('wpa_cli', '-i', iface, 'select_network', cols[0], timeout=5.0)
            return


def _write_netplan(text):
    tmp = NETPLAN_FILE + '.tmp'
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)   # netplan wants 600
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    os.replace(tmp, NETPLAN_FILE)


def _apply_netplan():
    code, out = _run('netplan', 'apply', timeout=60.0)
    if code != 0:
        raise RuntimeError(f'netplan apply failed: {out[-200:]}')


def join_network(iface, ssid, psk, progress):
    """Add the network, switch to it and wait for an address. -> (ok, ip, text)."""
    old = ''
    if os.path.exists(NETPLAN_FILE):
        with open(NETPLAN_FILE, encoding='utf-8') as f:
            old = f.read()
    try:
        _write_netplan(netplan_with_network(old, iface, ssid, psk))
        progress(f'Joining "{ssid}"')
        _apply_netplan()
        time.sleep(2.0)
        _select_network(iface, ssid)
        deadline = time.monotonic() + JOIN_TIMEOUT_S
        while time.monotonic() < deadline:
            cur, ip = wifi_status(iface)
            if cur == ssid and ip:
                return True, ip, ''
            time.sleep(1.0)
        why = f'could not join "{ssid}" (wrong password, or out of range?)'
    except (OSError, RuntimeError) as exc:
        why = str(exc)
    # Put things back so the robot returns to the network it was on.
    progress('Failed — restoring the previous Wi-Fi settings')
    try:
        if old:
            _write_netplan(netplan_without_network(old, iface, ssid))
        elif os.path.exists(NETPLAN_FILE):
            os.remove(NETPLAN_FILE)
        _apply_netplan()
    except (OSError, RuntimeError) as exc:
        why += f'; restoring also failed: {exc}'
    return False, None, why


# ── Bluetooth scan windows (raw HCI) ─────────────────────────────────────────

# Listen for the Deck half the time (320 ms of every 640 ms). BlueZ's default is
# 11.25 ms every 1.28 s (page) / 2.56 s (inquiry), and the Pi shares its radio
# with Wi-Fi, so the Deck's connection attempts mostly went unheard ("Host is
# down") and its scans didn't find the robot. setup.sh sets the page scan in
# /etc/bluetooth/main.conf too; the inquiry scan only sticks when written here.
SCAN_INTERVAL = 1024        # slots of 0.625 ms
SCAN_WINDOW = 512
_HCI_WRITE_PAGE_SCAN_ACTIVITY = 0x0C1C
_HCI_WRITE_INQUIRY_SCAN_ACTIVITY = 0x0C1E


def hci_command(opcode, params):
    """An HCI command packet (type 0x01, little-endian opcode, length, params)."""
    return struct.pack('<BHB', 1, opcode, len(params)) + params


def widen_scan(dev=0, log=None):
    """Set the page + inquiry scan windows on hci`dev` (needs root)."""
    activity = struct.pack('<HH', SCAN_INTERVAL, SCAN_WINDOW)
    try:
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW,  # pylint: disable=no-member
                          socket.BTPROTO_HCI)  # pylint: disable=no-member
        try:
            s.bind((dev,))
            for op in (_HCI_WRITE_PAGE_SCAN_ACTIVITY, _HCI_WRITE_INQUIRY_SCAN_ACTIVITY):
                s.send(hci_command(op, activity))
        finally:
            s.close()
    except OSError as exc:
        if log:
            log(f'Could not set the Bluetooth scan windows: {exc}')


# ── Bluetooth agent (bluetoothctl) ───────────────────────────────────────────

class Agent:
    """A long-running `bluetoothctl` that is the pairing agent and says yes to
    every pairing/authorisation prompt; also switches discoverable on/off."""

    def __init__(self, name, log):
        self._name = name
        self._log = log
        self._proc = None
        self._lock = threading.Lock()
        self.discoverable = None

    def _send(self, line):
        try:
            self._proc.stdin.write((line + '\n').encode())
            self._proc.stdin.flush()
        except (OSError, AttributeError):
            pass

    def ensure_running(self):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return
            if self._proc is not None:
                self._log('bluetoothctl exited; restarting it')
            self._proc = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE,
                                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            threading.Thread(target=self._read, args=(self._proc,), daemon=True).start()
            for cmd in ('power on', f'system-alias {self._name}', 'agent NoInputNoOutput',
                        'default-agent'):
                self._send(cmd)
            self.discoverable = None

    def set_discoverable(self, on):
        if on == self.discoverable:
            return
        for cmd in (['discoverable-timeout 0', 'pairable on', 'discoverable on'] if on
                    else ['discoverable off', 'pairable off']):
            self._send(cmd)
        self.discoverable = on
        self._log(f'Bluetooth {"discoverable + pairable" if on else "no longer discoverable"}')

    def _read(self, proc):
        buf = b''
        fd = proc.stdout.fileno()
        while proc.poll() is None:
            r, _, _ = select.select([fd], [], [], 1.0)
            if not r:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            buf = (buf + chunk)[-2048:]
            # Prompts end without a newline, e.g. "[agent] Confirm passkey 123456 (yes/no): "
            if b'(yes/no)' in buf:
                self._log('Accepting a Bluetooth pairing/authorisation request')
                self._send('yes')
                buf = b''

    def stop(self):
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()


# ── RFCOMM server ────────────────────────────────────────────────────────────

def _send_json(conn, obj):
    conn.sendall((json.dumps(obj, separators=(',', ':')) + '\n').encode())


def handle_client(conn, args, log):
    conn.settimeout(_CLIENT_TIMEOUT_S)
    ssid, ip = wifi_status(args.iface)
    _send_json(conn, {'v': 1, 'name': args.name, 'ssid': ssid, 'ip': ip})
    buf = b''
    while True:
        try:
            chunk = conn.recv(4096)
        except socket.timeout:
            return
        if not chunk:
            return
        buf += chunk
        if len(buf) > 8192:
            return
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            try:
                msg = json.loads(line)
            except ValueError:
                _send_json(conn, {'ok': False, 'text': 'bad message'})
                continue
            if msg.get('cmd') != 'wifi':
                continue
            want, psk = msg.get('ssid'), msg.get('psk', '')
            err = valid_credentials(want, psk)
            if err:
                _send_json(conn, {'ok': False, 'ssid': want, 'ip': None, 'text': err})
                continue
            log(f'Deck asked to join "{want}"')
            ok, new_ip, text = join_network(
                args.iface, want, psk, lambda s: _send_json(conn, {'progress': s}))
            log(f'Joined "{want}" as {new_ip}' if ok else f'Join failed: {text}')
            _send_json(conn, {'ok': ok, 'ssid': want, 'ip': new_ip, 'text': text})


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--name', default='SpotMicro', help='Bluetooth name the Deck looks for')
    ap.add_argument('--channel', type=int, default=22, help='RFCOMM channel (1-30)')
    ap.add_argument('--iface', default='wlan0')
    ap.add_argument('--open-window', type=float, default=600.0,
                    help='seconds after start that new devices may pair (also whenever Wi-Fi is down)')
    args = ap.parse_args()

    def log(text):
        print(text, flush=True)

    agent = Agent(args.name, log)
    # Bluetooth constants only exist when Python is built with BlueZ (true on the
    # Pi, not on CI runners), hence the no-member disables.
    srv = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,  # pylint: disable=no-member
                        socket.BTPROTO_RFCOMM)  # pylint: disable=no-member
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((socket.BDADDR_ANY, args.channel))  # pylint: disable=no-member
    srv.listen(1)
    srv.settimeout(5.0)
    log(f'Wi-Fi provisioning: "{args.name}", RFCOMM channel {args.channel}, iface {args.iface}')
    started = time.monotonic()
    widened_at = None
    try:
        while True:
            agent.ensure_running()
            ssid, _ = wifi_status(args.iface)
            was = agent.discoverable
            agent.set_discoverable(time.monotonic() - started < args.open_window or ssid is None)
            # Again whenever discoverable changes (BlueZ rewrites the scan settings
            # then) and once a minute in case anything else reset them.
            if agent.discoverable != was or widened_at is None or time.monotonic() - widened_at > 60.0:
                widen_scan(log=log)
                widened_at = time.monotonic()
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            log(f'Deck connected from {addr[0]}')
            try:
                handle_client(conn, args, log)
            except ConnectionResetError:
                # Normal when it only wanted our IP (already on the same Wi-Fi).
                log('Deck disconnected')
            except OSError as exc:
                log(f'Connection error: {exc}')
            finally:
                conn.close()
    except KeyboardInterrupt:
        pass
    finally:
        agent.stop()
        srv.close()


if __name__ == '__main__':
    main()
