import numpy as np
import threading
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import board
import time



class DOG:
    """
    A class to represent the DOG robot.
    This class contains four legs and methods to control them.
    """
    
    def __init__(self):
        """Initialize the DOG robot with four legs."""
        self.legs = {
            "FL": Leg("FL"),  # Front Left
            "FR": Leg("FR"),  # Front Right
            "RL": Leg("RL"),  # Rear Left
            "RR": Leg("RR"),  # Rear Right
        }
        self.lock = threading.Lock()

    def get_leg(self, leg_name):
        """
        Get a leg instance by its name.
        
        Args:
            leg_name (str): Name of the leg ("FL", "FR", "RL", "RR")
            
        Returns:
            Leg: The corresponding Leg instance
        """
        if leg_name not in self.legs:
            raise ValueError(f"leg_name '{leg_name}' must be one of {list(self.legs.keys())}")
        return self.legs[leg_name]

    def set_leg_angles(self, leg_name, hip_angle, lower_angle, upper_angle):
        """
        Set the joint angles for a specific leg.
        
        Args:
            leg_name (str): Name of the leg ("FL", "FR", "RL", "RR")
            hip_angle (float): Hip joint angle
            lower_angle (float): Lower joint angle
            upper_angle (float): Upper joint angle
        """
        with self.lock:
            leg = self.get_leg(leg_name)
            leg.set_angles(hip_angle, lower_angle, upper_angle)

    def reset_all_legs(self):
        """Reset all legs to their safe default positions."""
        with self.lock:
            for leg in self.legs.values():
                leg.reset_to_safe_position()

    def __str__(self):
        """String representation of the DOG robot."""
        return "\n".join(str(leg) for leg in self.legs.values())

    def __repr__(self):
        """Detailed string representation of the DOG robot."""
        return self.__str__()


class Leg:
    """
    A class to represent a leg of the DOG robot.
    This class contains the angles for the hip, lower, and upper joints.
    """
    
    # Class constants
    VALID_LEG_NAMES = {"FL", "FR", "RL", "RR"}
    
    # Robot dimensions (mm)
    LEG_TO_LEG_DISTANCE = 186.0
    LEG_CENTER_TO_LEG_DISTANCE = 93.0
    LEG_LENGTH = 120.0
    HIP_TO_LEG_DISTANCE = 40.0
    LEG_WIDTH = 80.0
    
    # Hip offsets from body center to each hip joint
    # These represent the physical mounting positions of the hip joints
    HIP_OFFSETS = {
        "FL": np.array([+93.0, +40.0, 0.0]),  # Front Left: +X, +Y
        "FR": np.array([+93.0, -40.0, 0.0]),  # Front Right: +X, -Y  
        "RL": np.array([-93.0, +40.0, 0.0]),  # Rear Left: -X, +Y
        "RR": np.array([-93.0, -40.0, 0.0]),  # Rear Right: -X, -Y
    }

    def __init__(self, leg_name, enabled=True):
        """
        Initialize a leg instance.
        
        Args:
            leg_name (str): Name of the leg ("FL", "FR", "RL", "RR")
            enabled (bool): Whether the leg is enabled
        """
        if leg_name not in self.VALID_LEG_NAMES:
            raise ValueError(f"leg_name '{leg_name}' must be one of {self.VALID_LEG_NAMES}")

        self.leg_name = leg_name
        self.enabled = enabled

        # Set safe angles and conversion flag based on leg position
        self._safe_hip_angle = 90.0
        
        if leg_name in ["FL", "RL"]:  # Left legs
            self._safe_lower_angle = 60.0
            self._safe_upper_angle = 60.0
            self.from_180 = True
        else:  # Right legs (FR, RR)
            self._safe_lower_angle = 120.0
            self._safe_upper_angle = 120.0
            self.from_180 = False

        # Initialize current angles to safe positions
        self.hip_angle = self._safe_hip_angle
        self.lower_angle = self._safe_lower_angle
        self.upper_angle = self._safe_upper_angle

        # Store current calculated angles
        self.current_angles = (self.hip_angle, self.lower_angle, self.upper_angle)

        self.servos = self.Create_servo_driver()

    def inverse_kinematics(self, X, Y, Z):
        """
        Calculate the inverse kinematics for the leg based on the given coordinates.
        These coordinates should be in the HIP frame (relative to the hip joint).
        
        Args:
            X (float): X coordinate in hip frame
            Y (float): Y coordinate in hip frame  
            Z (float): Z coordinate in hip frame
            
        Returns:
            tuple: (hip_angle, lower_angle, upper_angle) in radians
        """
        # This matches your original working IK exactly
        # Adjust Y by adding the 40mm hip offset
        Y_adjusted = Y + 40
        
        # Calculate intermediate distances
        # Clamp to 0 instead of going negative: a target inside the 40mm
        # dead-zone around the hip axis would otherwise produce NaN (sqrt of
        # a negative number) instead of just being slightly unreachable.
        D_sq = np.maximum(Y_adjusted**2 + Z**2 - 1600, 0.0)  # 40^2 = 1600
        D = np.sqrt(D_sq)
        G = np.sqrt(D**2 + X**2)
        #print(self.leg_name)

        # Calculate joint angles
        # The link geometry is identical for all four legs; left/right
        # differences are handled in Move_To_L via the servo mapping.
        if self.leg_name in self.VALID_LEG_NAMES:

            hip_angle = np.arctan2(Z, Y_adjusted) + np.arctan2(40, D)
            lower_angle = np.arccos(np.clip((G**2 - 28800) / (-28800), -1.0, 1.0))  # 2 * 120^2 = 28800
            upper_angle = np.arctan2(D, X) - np.arcsin((120 * np.sin(lower_angle)) / G)
            upper_angle = np.pi/2 - upper_angle

        else:
            raise NotImplementedError(
                f"IK / servo mapping not implemented for leg '{self.leg_name}'"
            )


        # Apply angle conversion for right legs
        #if self.from_180:
        #lower_angle = np.pi - lower_angle
        
            
        
        #lower_angle = upper_angle + lower_angle
        return hip_angle, lower_angle, upper_angle


    def rotational_preprocess_xyz(self, X, Y, Z, roll, pitch, yaw, angles_in_degrees=True):
        """
        Convert a BODY-frame foot target (x,y,z) into the HIP frame of this leg,
        after applying a body orientation (roll, pitch, yaw).
        
        Args:
            x, y, z (float): Foot target in BODY frame
            roll, pitch, yaw (float): Body orientation angles
            angles_in_degrees (bool): If True, angles are in degrees
            
        Returns:
            tuple: (x_hip, y_hip, z_hip) coordinates in hip frame
        """
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)

        z_off = 93 *  np.sin(pitch_rad)
        #print(f"z_off: {z_off}")

        #pitch
        if self.leg_name in ["FL", "FR"]:
            new_Z1 = Z + z_off
        else:
            new_Z1 = Z - z_off
        
        #roll 
        y_off = new_Z1 * np.sin(roll_rad)
        #print(f"y_off: {y_off}")
        if self.leg_name in ["FL", "RL"]:
            new_Y2 = Y + y_off
        else:
            new_Y2 = Y - y_off

        #yaw
        Xoff = 43.5 * np.sin(yaw_rad)
        #Yoff = 93 * np.sin(yaw_rad)
        #print(f"Xoff: {Xoff}, Yoff: {Yoff}")
        if self.leg_name in ["FL", "FR"]:
            new_X = X + Xoff
            
        else:
            new_X = X - Xoff
            

        return float(new_X), float(new_Y2), float(new_Z1)

    def create_points(self, start, end, steps):
        """
        Create a list of points forming a straight line between two 6D poses.
        Each point contains [x, y, z, roll, pitch, yaw].

        Args:
            start (sequence of 6): [x1, y1, z1, roll1, pitch1, yaw1]
            end   (sequence of 6): [x2, y2, z2, roll2, pitch2, yaw2]  
            steps (int): number of segments (>=1). Returns steps+1 points.

        Returns:
            list[list[float]]: interpolated points from start to end
        """
        if steps < 1:
            raise ValueError("steps must be >= 1")
        if len(start) != 6 or len(end) != 6:
            raise ValueError("start and end must have 6 elements [x,y,z,roll,pitch,yaw]")

        x1, y1, z1, r1, p1, ya1 = start
        x2, y2, z2, r2, p2, ya2 = end

        points = []
        for i in range(steps + 1):  # include both endpoints
            t = i / steps  # 0.0 to 1.0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            z = z1 + t * (z2 - z1)
            r = r1 + t * (r2 - r1)
            p = p1 + t * (p2 - p1)
            ya = ya1 + t * (ya2 - ya1)
            points.append([x, y, z, r, p, ya])
        return points
    
    def _set_servo_angle(self, index, angle_deg):
        """
        Safely write an angle (degrees) to a servo channel.

        Skips NaN/inf (an unreachable IK target) and clamps to the servo's
        actuation range instead of letting adafruit_motor raise ValueError
        and kill the control loop.
        """
        if not np.isfinite(angle_deg):
            print(f"[{self.leg_name}] servo {index}: skipping non-finite angle ({angle_deg})")
            return
        servo_obj = self.servos[index]
        clamped = float(np.clip(angle_deg, 0.0, servo_obj.actuation_range))
        if clamped != angle_deg:
            print(f"[{self.leg_name}] servo {index}: clamped angle {angle_deg:.1f} -> {clamped:.1f}")
        servo_obj.angle = clamped

    def Move_To_L(self,point):
        #filler
        x, y, z, roll, pitch, yaw = point
        Xh,Yh,Zh =self.rotational_preprocess_xyz(x, y, z, roll, pitch, yaw)
        H_ang, Th_ang , An_ang =self.inverse_kinematics(Xh,Yh,Zh)
        if self.leg_name in ["RL"]:
            self._set_servo_angle(0, np.rad2deg(H_ang))
            self._set_servo_angle(2, np.rad2deg(Th_ang))
            self._set_servo_angle(1, np.rad2deg(An_ang))
        elif self.leg_name in ["RR"]:
            # Right side is a mirror of the left: same IK, servo angle = 180 - angle
            # (servo_calibration.txt: neutral 90/120/120 vs. left 90/60/60)
            self._set_servo_angle(3, 180.0 - np.rad2deg(H_ang))
            self._set_servo_angle(5, 180.0 - np.rad2deg(Th_ang))
            self._set_servo_angle(4, 180.0 - np.rad2deg(An_ang))
        elif self.leg_name in ["FL"]:
            self._set_servo_angle(15, np.rad2deg(H_ang))
            self._set_servo_angle(13, np.rad2deg(Th_ang))
            self._set_servo_angle(14, np.rad2deg(An_ang))

    def Create_servo_driver(self):
        i2c = board.I2C()  # uses board.SCL and board.SDA
        pca = PCA9685(i2c)
        pca.frequency = 50
        servos = [
        servo.Servo(pca.channels[0],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[1],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[2],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[3],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[4],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[5],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[6],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[7],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[8],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[9],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[10],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[11],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[12],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[13],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270
            ),
        servo.Servo(pca.channels[14],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=180
            ),
        servo.Servo(pca.channels[15],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=180
            )]
        return servos


    def reset_to_safe_position(self):
        """Reset all joint angles to safe default positions."""
        self.hip_angle = self._safe_hip_angle
        self.lower_angle = self._safe_lower_angle
        self.upper_angle = self._safe_upper_angle
        self.current_angles = (self.hip_angle, self.lower_angle, self.upper_angle)

    def set_angles(self, hip_angle, lower_angle, upper_angle):
        """
        Set the joint angles for this leg.
        
        Args:
            hip_angle (float): Hip joint angle
            lower_angle (float): Lower joint angle  
            upper_angle (float): Upper joint angle
        """
        self.hip_angle = hip_angle
        self.lower_angle = lower_angle
        self.upper_angle = upper_angle
        self.current_angles = (hip_angle, lower_angle, upper_angle)

    def __str__(self):
        """String representation of the leg."""
        return (f"Leg({self.leg_name}, enabled={self.enabled}, "
                f"hip={self.hip_angle:.2f}, lower={self.lower_angle:.2f}, "
                f"upper={self.upper_angle:.2f})")

    def __repr__(self):
        """Detailed string representation of the leg."""
        return self.__str__()

    def release_servos(self):
        """
        Disable all of this leg's servo channels (stop sending a PWM pulse)
        so they go limp instead of holding their last commanded position.
        """
        for servo_obj in self.servos:
            servo_obj.angle = None
        print(f"[{self.leg_name}] servos released")


def main():



    print("-" * 40)
    leg1 = Leg("RL")
    leg2 = Leg("RR")
    leg3 = Leg("FL")
    leg4 = Leg("FR")

    print("Test Trajectory interpolation")
    
    start_pose = [-50, 10, 200, 0, 0, 0]    # Start: foot below, no rotation
    end_pose = [-50, -10, 150, 0, 0, 0]   # End: foot forward/right, slight yaw
    steps = 11
    print(f"Interpolating from {start_pose} to {end_pose} in {steps} steps:")

    try:
        while True:
            '''leg1.release_servos()
            leg2.release_servos()
            leg3.release_servos()
            leg4.release_servos()'''

            # 1. start_pose → end_pose
            trajectory = leg1.create_points(start_pose, end_pose, steps)
            for point in trajectory:
                leg1.Move_To_L(point)
                leg2.Move_To_L(point)
                leg3.Move_To_L(point)
                leg4.Move_To_L(point)
                time.sleep(0.051)

            # 2. end_pose → start_pose
            trajectory = leg1.create_points(end_pose, start_pose, steps)
            for point in trajectory:
                leg1.Move_To_L(point)
                leg2.Move_To_L(point)
                leg3.Move_To_L(point)
                leg4.Move_To_L(point)
                time.sleep(0.051)

            '''leg1.release_servos()
            leg2.release_servos()
            leg3.release_servos()
            leg4.release_servos()'''


    except KeyboardInterrupt:
        print("\nLoop interrupted by Ctrl+C")

    finally:
        for leg in (leg1, leg2, leg3, leg4):
            leg.release_servos()




if __name__ == "__main__":
    main()