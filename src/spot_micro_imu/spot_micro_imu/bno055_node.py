"""
ROS2 driver node for the Adafruit BNO055 9-DOF absolute orientation IMU
(https://www.adafruit.com/product/2472), connected over I2C.

The BNO055 has an onboard Cortex-M0 that runs sensor fusion itself, so this
node just polls the chip's already-fused outputs and republishes them as
standard ROS2 messages — no filtering/fusion happens here.

Publishes (namespaced under 'spotmicro_imu'):
  imu/data         sensor_msgs/Imu             fused orientation, angular
                                                velocity, linear acceleration
  imu/mag          sensor_msgs/MagneticField   magnetometer, in tesla
  imu/temperature  sensor_msgs/Temperature     onboard die temperature, in C
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature

from spot_micro_imu.bno055 import Bno055

# The BNO055 doesn't report per-sample covariance — these are rough, fixed
# placeholders (order-of-magnitude from the datasheet's noise specs), not a
# real estimate. Tune/replace if you feed this into an EKF (e.g. robot_localization).
_ORIENTATION_COVARIANCE = 0.0025          # rad^2, ~2.9 deg stddev
_ANGULAR_VELOCITY_COVARIANCE = 0.0004     # (rad/s)^2
_LINEAR_ACCELERATION_COVARIANCE = 0.001   # (m/s^2)^2
_MAGNETIC_FIELD_COVARIANCE = 4e-13        # tesla^2


def _diag(value):
    cov = [0.0] * 9
    cov[0] = cov[4] = cov[8] = value
    return cov


class Bno055Node(Node):

    def __init__(self):
        super().__init__('spotmicro_imu')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('update_rate_hz', 20.0)

        self._frame_id = self.get_parameter('frame_id').value
        address = self.get_parameter('i2c_address').value
        rate_hz = self.get_parameter('update_rate_hz').value

        self._sensor = Bno055(bus_num=1, address=address)

        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self._temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)

        self.create_timer(1.0 / rate_hz, self._publish_readings)
        self.create_timer(5.0, self._log_calibration)

        self.get_logger().info(
            f'BNO055 ready on I2C address {hex(address)}, publishing at {rate_hz} Hz. '
            'Move the robot through the air in a slow figure-8 to calibrate the '
            'gyro/accel/magnetometer (calibration status is logged every 5s until full).')

    def _publish_readings(self):
        try:
            quat = self._sensor.read_quaternion()
            gyro = self._sensor.read_gyro()
            accel = self._sensor.read_linear_acceleration()
            mag = self._sensor.read_magnetic()
            temp_c = self._sensor.read_temperature()
        except OSError as exc:
            self.get_logger().warn(
                f'BNO055 I2C read failed: {exc}', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._frame_id

        w, x, y, z = quat
        imu_msg.orientation.w = w
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation_covariance = _diag(_ORIENTATION_COVARIANCE)

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance = _diag(_ANGULAR_VELOCITY_COVARIANCE)

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance = _diag(_LINEAR_ACCELERATION_COVARIANCE)

        self._imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self._frame_id
        # Driver reports microtesla; sensor_msgs/MagneticField wants tesla.
        mag_msg.magnetic_field.x = mag[0] * 1e-6
        mag_msg.magnetic_field.y = mag[1] * 1e-6
        mag_msg.magnetic_field.z = mag[2] * 1e-6
        mag_msg.magnetic_field_covariance = _diag(_MAGNETIC_FIELD_COVARIANCE)
        self._mag_pub.publish(mag_msg)

        temp_msg = Temperature()
        temp_msg.header.stamp = now
        temp_msg.header.frame_id = self._frame_id
        temp_msg.temperature = temp_c
        temp_msg.variance = 0.0
        self._temp_pub.publish(temp_msg)

    def _log_calibration(self):
        try:
            sys_cal, gyro_cal, accel_cal, mag_cal = self._sensor.read_calibration_status()
        except OSError as exc:
            self.get_logger().warn(
                f'BNO055 I2C read failed: {exc}', throttle_duration_sec=5.0)
            return
        if (sys_cal, gyro_cal, accel_cal, mag_cal) == (3, 3, 3, 3):
            return
        self.get_logger().info(
            f'BNO055 calibration (0-3, 3=fully calibrated): '
            f'sys={sys_cal} gyro={gyro_cal} accel={accel_cal} mag={mag_cal}')


def main(args=None):
    rclpy.init(args=args)
    node = Bno055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
