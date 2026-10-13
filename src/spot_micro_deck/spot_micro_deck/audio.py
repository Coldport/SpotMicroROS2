"""
The robot's microphone (the Anker C200 webcam's, or any USB / ALSA capture
device) streamed to the Deck: deck_bridge runs one of these while a Deck is
connected and the "Robot mic" setting is on.

ffmpeg reads the ALSA device, mixes it to mono, applies the gain, resamples and
writes 8-bit G.711 mu-law on stdout. That is 1 byte a sample (16 kHz = 128
kbit/s, a fraction of the video), and the Deck decodes it with a lookup table,
so it needs no codec library. A reader thread cuts it into `chunk_ms` pieces
and hands each one to `on_chunk`.

The bridge's user must be allowed to open the device: on Ubuntu that's the
`audio` group (the systemd unit adds it with SupplementaryGroups=audio).
No ROS imports, so it can be unit-tested.
"""

import os
import re
import subprocess
import threading

_CARD_LINE = re.compile(r'^\s*(\d+)\s+\[(\S+)\s*\]:\s*(\S+)\s+-\s+(.*)$')


class AudioError(RuntimeError):
    pass


def list_capture_cards(asound='/proc/asound'):
    """-> [(index, id, driver, name)] for ALSA cards that can record."""
    try:
        with open(os.path.join(asound, 'cards'), encoding='utf-8') as f:
            lines = f.read().splitlines()
    except OSError:
        return []
    cards = []
    for line in lines:
        m = _CARD_LINE.match(line)
        if not m:
            continue
        index = int(m.group(1))
        card_dir = os.path.join(asound, f'card{index}')
        try:
            capture = any(re.fullmatch(r'pcm\d+c', n) for n in os.listdir(card_dir))
        except OSError:
            capture = False
        if capture:
            cards.append((index, m.group(2), m.group(3), m.group(4).strip()))
    return cards


def find_device(hint='auto', asound='/proc/asound'):
    """ALSA device name for `hint`:
      'auto'          the first USB capture card (the webcam), else the first capture card
      'plughw:1,0'..  any ALSA device name (contains ':'): used as given
      'C200', 'anker' a card whose id or name contains this (any case)
    Raises AudioError if nothing matches."""
    hint = (hint or 'auto').strip()
    if ':' in hint:
        return hint
    cards = list_capture_cards(asound)
    if not cards:
        raise AudioError('no microphone found (no ALSA capture device)')
    if hint.lower() == 'auto':
        usb = [c for c in cards if c[2] == 'USB-Audio']
        pick = (usb or cards)[0]
    else:
        match = [c for c in cards if hint.lower() in c[1].lower() or hint.lower() in c[3].lower()]
        if not match:
            names = ', '.join(c[3] for c in cards)
            raise AudioError(f'no microphone matching "{hint}" (have: {names})')
        pick = match[0]
    # plughw converts format / rate / channels if the device can't do what's asked.
    return f'plughw:{pick[0]},0'


def check_access(device, dev_snd='/dev/snd'):
    """Raise AudioError if this user can't open a (plug)hw:N,M device.
    ffmpeg only says "Input/output error" when it's a permissions problem."""
    m = re.fullmatch(r'(?:plug)?hw:(\d+)(?:,(\d+))?', device)
    if not m:
        return   # some other ALSA name: let ffmpeg try
    node = os.path.join(dev_snd, f'pcmC{m.group(1)}D{m.group(2) or 0}c')
    if not os.path.exists(node):
        raise AudioError(f'mic {device} is gone (unplugged?)')
    if not os.access(node, os.R_OK | os.W_OK):
        raise AudioError('no permission to open the mic: the bridge\'s user needs the "audio" group')


def ffmpeg_command(device, rate=16000, gain_db=0.0):
    return [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
        '-fflags', 'nobuffer', '-flags', 'low_delay',
        '-f', 'alsa', '-thread_queue_size', '64', '-i', device,
        '-af', f'volume={gain_db:g}dB', '-ac', '1', '-ar', str(int(rate)),
        '-f', 'mulaw', '-flush_packets', '1', 'pipe:1',
    ]


class MicStream:
    """ffmpeg capturing `device`; on_chunk(bytes) per `chunk_ms` of mu-law
    audio, from a background thread. on_exit(reason) once if ffmpeg stops by
    itself (device gone, no permission) rather than through stop()."""

    def __init__(self, device, on_chunk, on_exit, *, rate=16000, gain_db=0.0, chunk_ms=40):
        self.device = device
        self.rate = int(rate)
        self.gain_db = float(gain_db)
        self._chunk = max(1, self.rate * int(chunk_ms) // 1000)   # 1 byte per sample
        self._on_chunk = on_chunk
        self._on_exit = on_exit
        self._proc = None
        self._stopping = False

    def start(self):
        try:
            self._proc = subprocess.Popen(
                ffmpeg_command(self.device, self.rate, self.gain_db),
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as exc:
            raise AudioError(f'cannot run ffmpeg: {exc}') from exc
        threading.Thread(target=self._read, daemon=True, name='mic').start()

    def running(self):
        return self._proc is not None and self._proc.poll() is None

    def stop(self):
        self._stopping = True
        proc, self._proc = self._proc, None
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    def _read(self):
        proc = self._proc
        out = proc.stdout
        while True:
            data = out.read(self._chunk)
            if not data:
                break
            self._on_chunk(data)
        proc.wait()
        if not self._stopping:
            err = proc.stderr.read().decode(errors='replace').strip().splitlines()
            self._on_exit(_explain(err[-1] if err else f'ffmpeg exited ({proc.returncode})'))


def _explain(line):
    """ffmpeg's last error line, with the usual causes spelled out."""
    low = line.lower()
    if 'permission denied' in low:
        return 'no permission to open the mic (the bridge user needs the "audio" group)'
    if 'no such' in low or 'cannot open audio device' in low:
        return f'mic not available: {line}'
    return line
