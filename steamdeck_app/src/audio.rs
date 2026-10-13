//! The robot's mic (its <prefix>/audio stream, see protocol.rs) played on the
//! Deck through SDL. Turned on from the settings menu (Audio > Robot mic): the
//! robot only sends while that's on.
//!
//! The network thread decodes the mu-law into Shared::audio; the UI thread
//! moves it into an SDL audio queue each frame. A little is buffered before
//! playing (PREBUFFER_MS) so Wi-Fi jitter doesn't stutter, and if the queue
//! grows past MAX_MS (a burst after a stall) it is dropped, so the sound never
//! lags far behind.

use sdl2::audio::{AudioQueue, AudioSpecDesired};
use sdl2::AudioSubsystem;

const PREBUFFER_MS: u32 = 120;
const MAX_MS: u32 = 400;

/// G.711 mu-law byte -> 16-bit sample.
pub fn mulaw_to_i16(b: u8) -> i16 {
    let b = !b;
    let sign = b & 0x80;
    let exponent = (b >> 4) & 0x07;
    let mantissa = (b & 0x0F) as i32;
    let magnitude = (((mantissa << 3) + 0x84) << exponent) - 0x84;
    (if sign != 0 { -magnitude } else { magnitude }) as i16
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum Step {
    Keep,
    /// Enough buffered: start playing.
    Start,
    /// Ran dry: pause until PREBUFFER_MS has built up again.
    Rebuffer,
    /// Too far behind: drop what's queued.
    Flush,
}

fn decide(queued: u32, rate: u32, playing: bool) -> Step {
    let ms = |n: u32| n * rate / 1000;
    if queued > ms(MAX_MS) {
        Step::Flush
    } else if !playing && queued >= ms(PREBUFFER_MS) {
        Step::Start
    } else if playing && queued == 0 {
        Step::Rebuffer
    } else {
        Step::Keep
    }
}

pub struct Player {
    sub: Option<AudioSubsystem>,
    queue: Option<AudioQueue<i16>>,
    rate: u32,
    playing: bool,
    failed: bool,
}

impl Player {
    pub fn new(sub: Option<AudioSubsystem>) -> Self {
        Player { sub, queue: None, rate: 0, playing: false, failed: false }
    }

    /// Per frame: `samples` arrived since the last call at `rate` Hz; `live` =
    /// the robot is sending (false stops playback and drops the device).
    pub fn tick(&mut self, samples: &[i16], rate: u32, live: bool) {
        if !live || rate == 0 {
            if self.queue.take().is_some() {
                self.playing = false;
            }
            return;
        }
        if self.queue.is_none() || self.rate != rate {
            self.queue = None;
            self.playing = false;
            if !self.open(rate) {
                return;
            }
        }
        let Some(q) = &self.queue else { return };
        if !samples.is_empty() {
            let _ = q.queue_audio(samples);
        }
        let queued = q.size() / 2; // bytes -> i16 samples
        match decide(queued, rate, self.playing) {
            Step::Start => {
                q.resume();
                self.playing = true;
            }
            Step::Rebuffer => {
                q.pause();
                self.playing = false;
            }
            Step::Flush => {
                q.clear();
                q.pause();
                self.playing = false;
            }
            Step::Keep => {}
        }
    }

    fn open(&mut self, rate: u32) -> bool {
        if self.failed {
            return false;
        }
        let Some(sub) = &self.sub else {
            self.failed = true;
            return false;
        };
        let spec = AudioSpecDesired { freq: Some(rate as i32), channels: Some(1), samples: Some(512) };
        match sub.open_queue::<i16, _>(None, &spec) {
            Ok(q) => {
                self.queue = Some(q);
                self.rate = rate;
                true
            }
            Err(e) => {
                eprintln!("audio: can't open an output device: {e}");
                self.failed = true;
                false
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mulaw_matches_g711() {
        assert_eq!(mulaw_to_i16(0xFF), 0);
        assert_eq!(mulaw_to_i16(0x7F), 0);
        assert_eq!(mulaw_to_i16(0x80), 32124);
        assert_eq!(mulaw_to_i16(0x00), -32124);
        assert_eq!(mulaw_to_i16(0xF0), 120);
        // Monotonic over each half.
        let pos: Vec<i16> = (0x80..=0xFFu8).rev().map(mulaw_to_i16).collect();
        assert!(pos.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn buffers_then_plays_then_rebuffers_and_flushes() {
        let r = 16000;
        assert_eq!(decide(100, r, false), Step::Keep, "not enough yet");
        assert_eq!(decide(r * PREBUFFER_MS / 1000, r, false), Step::Start);
        assert_eq!(decide(500, r, true), Step::Keep);
        assert_eq!(decide(0, r, true), Step::Rebuffer);
        assert_eq!(decide(r * MAX_MS / 1000 + 1, r, true), Step::Flush);
    }
}
