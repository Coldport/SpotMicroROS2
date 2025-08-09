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
    
    # Default hip offsets for rotational preprocessing
    DEFAULT_HIP_OFFSETS = {
        "FL": np.array([+93.0, +34.5, 0.0]),
        "FR": np.array([+93.0, -34.5, 0.0]),
        "RL": np.array([-93.0, +34.5, 0.0]),
        "RR": np.array([-93.0, -34.5, 0.0]),
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

        # Calculate leg offset based on position
        self.leg_offset = self._calculate_leg_offset()

        self.current_angles = (self.hip_angle, self.lower_angle, self.upper_angle)

    def _calculate_leg_offset(self):
        """Calculate the offset vector for this leg based on its name."""
        half_leg_distance = self.LEG_TO_LEG_DISTANCE / 2
        leg_center_offset = self.LEG_CENTER_TO_LEG_DISTANCE / 2
        
        offset_map = {
            "FL": np.array([ half_leg_distance, leg_center_offset, 0.0]),
            "FR": np.array([ half_leg_distance, -leg_center_offset, 0.0]),
            "RL": np.array([-half_leg_distance, leg_center_offset, 0.0]),
            "RR": np.array([-half_leg_distance, -leg_center_offset, 0.0]),
        }
        
        return offset_map[self.leg_name]

    def inverse_kinematics(self, X,Y,Z):
        """
        Calculate the inverse kinematics for the leg based on the given coordinates.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            z (float): Z coordinate
            
        Returns:
            tuple: (hip_angle, lower_angle, upper_angle) in radians
        X = X - self.leg_offset[0]  # Adjust X by leg offset
        Y = Y - self.leg_offset[1]  # Adjust Y by leg offset
        Z = Z - self.leg_offset[2]
        """
        #X = X - self.leg_offset[0]  # Adjust X by leg offset
        #print("X after offset:", X)
        Y = Y + 40  # Adjust Y by leg offset

        #Z = Z - self.leg_offset[2]  # Adjust Z by leg offset
        #print("Z after offset:", Z)
        # Calculate intermediate distances
        D= np.sqrt((Y**2) +(Z**2)-1600)
        G = np.sqrt((D**2)+(X**2))

        Hip_angle = np.arctan2(Z,Y) + np.arctan2(40, G)
        lower_angle = np.arccos(((G**2) - 28800)/-28800)
        upper_angle = np.arctan2(D,X) - np.arcsin((120*np.sin(lower_angle))/G)


        # Apply angle conversion for right legs
        if self.from_180:
            lower_angle = np.pi - lower_angle
            upper_angle = np.pi - upper_angle

        return (Hip_angle, lower_angle, upper_angle)
    
    def rotational_preprocess_xyz(self, x, y, z, roll, pitch, yaw, 
                                 hip_offsets=None, angles_in_degrees=True):
        """
        Convert a nominal BODY-frame foot target (x,y,z) into the HIP frame,
        after applying a body orientation (roll, pitch, yaw).
        
        Args:
            x, y, z (float): Foot target in BODY frame at zero pose
            roll, pitch, yaw (float): Body orientation about BODY axes (X,Y,Z)
            hip_offsets (dict, optional): Custom hip offset mapping
            angles_in_degrees (bool): If True, roll/pitch/yaw are in degrees
            
        Returns:
            tuple: (x_hip, y_hip, z_hip) in hip frame coordinates
        """
        if hip_offsets is None:
            hip_offsets = self.DEFAULT_HIP_OFFSETS

        if self.leg_name not in hip_offsets:
            raise ValueError(f"leg_name '{self.leg_name}' not in hip_offsets")

        # Convert angles to radians if needed
        if angles_in_degrees:
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)

        # Create rotation matrices
        rotation_matrix = self._create_rotation_matrix(roll, pitch, yaw)

        # Transform the target point
        body_point = np.array([x, y, z], dtype=float)
        rotated_point = rotation_matrix.T @ body_point
        
        # Transform to hip frame
        hip_offset = hip_offsets[self.leg_name]
        hip_point = rotated_point - hip_offset

        return float(hip_point[0]), float(hip_point[1]), float(hip_point[2])

    def create_points(self, start, end, steps):
        """
        Create a list of points forming a straight line between two 3D coordinates.

        Args:
            start (sequence of 3): [x1, y1, z1]
            end   (sequence of 3): [x2, y2, z2]
            steps (int): number of segments between start and end (>=1).
                        The function returns steps+1 points (inclusive of endpoints).

        Returns:
            list[list[float]]: points along the line from start to end.
        """
        if steps < 1:
            raise ValueError("steps must be >= 1")

        x1, y1, z1 = start
        x2, y2, z2 = end

        points = []
        for i in range(steps + 1):  # include both endpoints
            t = i / steps           # 0.0 .. 1.0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            z = z1 + t * (z2 - z1)
            points.append([x, y, z])
        return points


    @staticmethod
    def _create_rotation_matrix(roll, pitch, yaw):
        """
        Create a rotation matrix from roll, pitch, yaw angles (Z-Y-X convention).
        
        Args:
            roll, pitch, yaw (float): Rotation angles in radians
            
        Returns:
            numpy.ndarray: 3x3 rotation matrix
        """
        # Roll (X-axis rotation)
        cr, sr = np.cos(roll), np.sin(roll)
        rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])

        # Pitch (Y-axis rotation)
        cp, sp = np.cos(pitch), np.sin(pitch)
        ry = np.array([[cp, 0, sp],
                       [0, 1, 0],
                       [-sp, 0, cp]])

        # Yaw (Z-axis rotation)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rz = np.array([[cy, -sy, 0],
                       [sy, cy, 0],
                       [0, 0, 1]])

        # Combined rotation (Z-Y-X order)
        return rz @ ry @ rx

    def reset_to_safe_position(self):
        """Reset all joint angles to safe default positions."""
        self.hip_angle = self._safe_hip_angle
        self.lower_angle = self._safe_lower_angle
        self.upper_angle = self._safe_upper_angle

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

    def __str__(self):
        """String representation of the leg."""
        return (f"Leg({self.leg_name}, enabled={self.enabled}, "
                f"hip={self.hip_angle:.2f}, lower={self.lower_angle:.2f}, "
                f"upper={self.upper_angle:.2f})")

    def __repr__(self):
        """Detailed string representation of the leg."""
        return self.__str__()



def main():
    fl_leg = Leg("FL")
    
    # Test parameters
    x, y, z = 0, 0, 120
    x2, y2, z2= 0, 0, 120
    start_point = [x, y, z]
    end_point = [x2, y2, z2]
    steps = 10
    roll1, pitch1, yaw1 = 0, 0, 0
    roll2, pitch2, yaw2 = 0, -0, 0

    LEG = Leg("FL")
    #print("Initial Leg State:", LEG)
    points = LEG.create_points(start_point, end_point, steps)
    for point in points:
        x, y, z = point
        hip_angle, lower_angle, upper_angle = LEG.inverse_kinematics(x, y, z)
        print(f"Point: ({x:.2f}, {y:.2f}, {z:.2f}) -> "
              f"Hip: {np.rad2deg(hip_angle):.2f}°, "
              f"Lower: {np.rad2deg(lower_angle):.2f}°, "
              f"Upper: {np.rad2deg(upper_angle):.2f}°")
        LEG.set_angles(np.rad2deg(hip_angle), np.rad2deg(lower_angle), np.rad2deg(upper_angle))
        #print("Updated Leg State:", LEG)
    print("-" * 60)    
    LEG.reset_to_safe_position()
    print("Reset Leg State:", LEG)
    print("-" * 60)
    print("Testing rotational preprocessing:")
    # Test inverse kinematics



if __name__ == "__main__":
    main()