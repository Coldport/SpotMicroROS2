"""
Real PCA9685 servo driver. Only this module imports board/adafruit_motor/
adafruit_pca9685, so importing spot_micro_ik elsewhere (tests, a dev machine,
a dry run) never requires the hardware libs or an actual I2C bus.
"""

import numpy as np


class ServoDriver:
    """Writes angle_deg to a PCA9685 channel, per-leg calibration in hand."""

    def __init__(self, calibration, frequency=50, min_pulse=500, max_pulse=2500):
        import board
        from adafruit_motor import servo
        from adafruit_pca9685 import PCA9685

        i2c = board.I2C()
        self._pca = PCA9685(i2c)
        self._pca.frequency = frequency

        self._servos = {}
        for leg_cal in calibration.values():
            for joint_cal in (leg_cal.hip, leg_cal.thigh, leg_cal.knee):
                if joint_cal.channel in self._servos:
                    continue
                self._servos[joint_cal.channel] = servo.Servo(
                    self._pca.channels[joint_cal.channel],
                    min_pulse=min_pulse,
                    max_pulse=max_pulse,
                    actuation_range=joint_cal.range,
                )

    def write(self, channel, angle_deg):
        """Safely write an angle (degrees) to a servo channel.

        Skips NaN/inf and clamps to the servo's actuation range instead of
        letting adafruit_motor raise ValueError and kill the caller's loop.
        """
        if not np.isfinite(angle_deg):
            return
        servo_obj = self._servos.get(channel)
        if servo_obj is None:
            return
        clamped = float(np.clip(angle_deg, 0.0, servo_obj.actuation_range))
        servo_obj.angle = clamped

    def release(self, channel=None):
        """Stop sending a PWM pulse on one channel (or all), so it goes limp."""
        channels = [channel] if channel is not None else list(self._servos)
        for ch in channels:
            self._servos[ch].angle = None


class DryRunDriver:
    """Drop-in replacement for ServoDriver that touches no hardware.

    Stores the last commanded angle per channel so callers (tests, a gait
    node running with dry_run=True) can inspect what *would* have been sent.
    """

    def __init__(self, calibration=None):
        self.last_angles = {}
        self.released = set()

    def write(self, channel, angle_deg):
        if not np.isfinite(angle_deg):
            return
        self.last_angles[channel] = float(angle_deg)
        self.released.discard(channel)

    def release(self, channel=None):
        channels = [channel] if channel is not None else list(self.last_angles)
        for ch in channels:
            self.released.add(ch)
