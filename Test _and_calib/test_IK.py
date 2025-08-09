import numpy as np
import threading
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
            self.from_180 = False
        else:  # Right legs (FR, RR)
            self._safe_lower_angle = 120.0
            self._safe_upper_angle = 120.0
            self.from_180 = True

        # Initialize current angles to safe positions
        self.hip_angle = self._safe_hip_angle
        self.lower_angle = self._safe_lower_angle
        self.upper_angle = self._safe_upper_angle

        # Store current calculated angles
        self.current_angles = (self.hip_angle, self.lower_angle, self.upper_angle)

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
        D = np.sqrt(Y_adjusted**2 + Z**2 - 1600)  # 40^2 = 1600
        G = np.sqrt(D**2 + X**2)

        # Calculate joint angles
        hip_angle = np.arctan2(Z, Y_adjusted) + np.arctan2(40, D)
        lower_angle = np.arccos((G**2 - 28800) / (-28800))  # 2 * 120^2 = 28800
        upper_angle = np.arctan2(D, X) - np.arcsin((120 * np.sin(lower_angle)) / G)

        # Apply angle conversion for right legs
        if self.from_180:
            lower_angle = np.pi - lower_angle
            upper_angle = np.pi - upper_angle
            
        self.current_angles = (hip_angle, lower_angle, upper_angle)
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
    
    

def main():

    print("-" * 40)
    leg1 = Leg("FR")
    leg2 = Leg("FR")
    leg3 = Leg("RL")
    leg4 = Leg("RR")

    print("Test Trajectory interpolation")
    
    start_pose = [0, 0, 120, -10, 0, 0]    # Start: foot below, no rotation
    end_pose = [0, 0, 90, -10, 0, 0]   # End: foot forward/right, slight yaw
    steps = 6
    
    print(f"Interpolating from {start_pose} to {end_pose} in {steps} steps:")

    trajectory = leg1.create_points(start_pose, end_pose, steps)
    for i, point in enumerate(trajectory):
        x, y, z, roll, pitch, yaw = point
        x_hip, y_hip, z_hip = leg1.rotational_preprocess_xyz(x, y, z, roll, pitch, yaw)
        hip_angle, lower_angle, upper_angle = leg1.inverse_kinematics(x_hip, y_hip, z_hip)

        print(f"  Step {i}: body=({x:.1f},{y:.1f},{z:.1f}) rot=({roll:.1f}, {pitch:.1f}, {yaw:.1f}) "
              f"-> angles=({np.rad2deg(hip_angle):.1f}, {np.rad2deg(lower_angle):.1f}, {np.rad2deg(upper_angle):.1f})°")
    
    # Test 4: All legs comparison
    print("\n" + "-" * 40)
    


if __name__ == "__main__":
    main()