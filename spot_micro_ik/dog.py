"""
Dog — all four legs plus a servo driver. This is the main entry point for
callers (the spot_micro_move gait node, test scripts, a REPL).
"""

from .hardware import DryRunDriver, ServoDriver
from .leg import LEG_NAMES, Leg

# A conservative standing pose, in the same BODY-frame convention as
# Leg.solve() (X=fore/aft, Y=lateral, Z=vertical/leg-extension, mm). Matches
# the range test_IK.py's own main() demo exercised (Z 150-200) — tune to your
# robot before trusting it as "the" neutral stance.
DEFAULT_STAND_POSE = dict(x=0.0, y=0.0, z=190.0, roll=0.0, pitch=0.0, yaw=0.0)


class Dog:
    def __init__(self, calibration, dry_run=True):
        """
        Args:
            calibration: dict[leg_name] -> LegCal, from calibration.load_calibration().
            dry_run: if True (default), no hardware/I2C is touched — servo
                writes are only recorded (see hardware.DryRunDriver). Pass
                False to actually drive the PCA9685.
        """
        self.legs = {name: Leg(name, calibration[name]) for name in LEG_NAMES}
        self.dry_run = dry_run
        self.driver = DryRunDriver(calibration) if dry_run else ServoDriver(calibration)

    def move_leg(self, leg_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Solve IK for one leg and write its servos. Returns the channel->angle dict sent."""
        targets = self.legs[leg_name].solve(x, y, z, roll, pitch, yaw)
        for channel, angle_deg in targets.items():
            self.driver.write(channel, angle_deg)
        return targets

    def move_all(self, x=0.0, y=0.0, z=None, roll=0.0, pitch=0.0, yaw=0.0):
        """Apply the same BODY-frame pose to all four legs (each leg mirrors/
        offsets its own foot position per rotational_preprocess_xyz)."""
        if z is None:
            z = DEFAULT_STAND_POSE["z"]
        return {name: self.move_leg(name, x, y, z, roll, pitch, yaw) for name in LEG_NAMES}

    def move_feet(self, foot_poses):
        """
        Args:
            foot_poses: dict[leg_name] -> (x, y, z) or (x, y, z, roll, pitch, yaw).
        """
        out = {}
        for name, pose in foot_poses.items():
            out[name] = self.move_leg(name, *pose)
        return out

    def stand(self):
        """Move to the default standing pose."""
        return self.move_all(**DEFAULT_STAND_POSE)

    def release(self):
        """Stop driving all servos (go limp). No-op's cleanly in dry_run."""
        self.driver.release()

    def set_driver(self, driver):
        """Swap the underlying servo driver at runtime (e.g. an arm/disarm gesture
        switching between DryRunDriver and a real ServoDriver that owns the I2C bus)."""
        self.driver = driver
