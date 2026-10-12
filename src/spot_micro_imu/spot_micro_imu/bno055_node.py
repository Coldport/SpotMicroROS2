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

Everything is published in the robot BODY frame (x forward, y left, z up): the
chip's readings go through the mount correction in mount.py — `mount_quaternion`
(how the chip sits in the body) and `yaw_offset_deg` (heading that reads 0).
Both default to "the chip's axes are the body's" and are set by these services
(std_srvs/Trigger; gait_node calls them from the Deck's Y+D-pad chords), which
also save the result into `params_files` so it survives a restart:

  zero              the current attitude becomes level, the current heading 0
  axis_calib/level  } axis calibration, drone style: record gravity with the
  axis_calib/left   } robot level, left side down, right side down, nose down,
  axis_calib/right  } nose up (call them in that order); the last call works
  axis_calib/front  } out the mount from all five (mount.calibrate_mount) and
  axis_calib/back   } saves it
"""

import collections
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_srvs.srv import Trigger

from spot_micro_deck.param_file import update_file
from spot_micro_imu import mount
from spot_micro_imu.bno055 import Bno055

# Gravity is averaged over this much of the latest readings for a capture.
_CAPTURE_S = 0.5

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
        self.declare_parameter('mount_quaternion', [1.0, 0.0, 0.0, 0.0])   # w, x, y, z
        self.declare_parameter('yaw_offset_deg', 0.0)
        # ':'-separated params files to save the mount into (imu.launch.py sets it).
        self.declare_parameter('params_files', '')

        self._frame_id = self.get_parameter('frame_id').value
        address = self.get_parameter('i2c_address').value
        rate_hz = self.get_parameter('update_rate_hz').value

        try:
            self._mount = mount.quat_normalize(
                tuple(float(v) for v in self.get_parameter('mount_quaternion').value))
        except (TypeError, ValueError):
            raise ValueError('mount_quaternion must be 4 numbers [w, x, y, z], not all zero')
        if len(self._mount) != 4:
            raise ValueError('mount_quaternion must be [w, x, y, z]')
        self._yaw0 = float(self.get_parameter('yaw_offset_deg').value)
        self._params_files = [f for f in str(self.get_parameter('params_files').value).split(':') if f]
        self._raw = collections.deque()      # (time, sensor-frame quaternion)
        self._captures = {}                  # axis calibration: step -> (up vector, quaternion)

        self._sensor = Bno055(bus_num=1, address=address)

        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self._temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)

        self.create_timer(1.0 / rate_hz, self._publish_readings)
        self.create_timer(5.0, self._log_calibration)
        self.create_service(Trigger, 'zero', self._srv_zero)
        for step in mount.CALIB_POSES:
            self.create_service(Trigger, f'axis_calib/{step}',
                                lambda req, resp, step=step: self._srv_axis_calib(step, resp))

        self.get_logger().info(
            f'BNO055 ready on I2C address {hex(address)}, publishing at {rate_hz} Hz. '
            'Move the robot through the air in a slow figure-8 to calibrate the '
            'gyro/accel/magnetometer (calibration status is logged every 5s until full). '
            f'Mount: {mount.describe(self._mount)}, yaw offset {self._yaw0:+.1f} deg.')

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

        try:
            raw_q = mount.quat_normalize(quat)
        except ValueError:
            return   # all zeros: the chip's fusion hasn't started yet
        t = time.monotonic()
        self._raw.append((t, raw_q))
        while self._raw and t - self._raw[0][0] > _CAPTURE_S:
            self._raw.popleft()
        # Into the body frame (see mount.py).
        quat = mount.apply(raw_q, self._mount, self._yaw0)
        to_body = mount.quat_conj(self._mount)
        gyro = mount.quat_rotate(to_body, gyro)
        accel = mount.quat_rotate(to_body, accel)
        mag = mount.quat_rotate(to_body, mag)

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

    # ── mount correction services ──────────────────────────────────────────

    def _latest(self):
        """-> (averaged world-up in the sensor frame, newest sensor quaternion)."""
        if not self._raw or time.monotonic() - self._raw[-1][0] > 1.0:
            raise ValueError('no fresh IMU readings')
        ups = [mount.up_vector(q) for _, q in self._raw]
        avg = tuple(sum(u[i] for u in ups) / len(ups) for i in range(3))
        spread = max(mount.tilt_deg(avg, u) for u in ups)
        if spread > 3.0:
            raise ValueError(f'the robot was moving ({spread:.0f} deg in {_CAPTURE_S}s) — hold it still')
        return avg, self._raw[-1][1]

    def _srv_zero(self, request, response):
        try:
            _, q = self._latest()
            unsaved = self._set_mount(*mount.zero_mount(self._mount, q))
        except ValueError as exc:
            response.success, response.message = False, f'IMU zero failed: {exc}'
            self.get_logger().warn(response.message)
            return response
        response.success, response.message = True, 'IMU zeroed — this pose is now level' + unsaved
        self.get_logger().info(f'{response.message}. Mount: {mount.describe(self._mount)}.')
        return response

    def _srv_axis_calib(self, step, response):
        try:
            if step == 'level':
                self._captures = {}          # starts a new calibration
            self._captures[step] = self._latest()
            if step != mount.CALIB_POSES[-1]:
                response.message = 'level recorded' if step == 'level' else \
                    f'{mount.CALIB_POSE_TEXT[step]} recorded'
            else:
                captures, self._captures = self._captures, {}
                new = mount.calibrate_mount({k: up for k, (up, _) in captures.items()})
                q0 = captures['level'][1]
                unsaved = self._set_mount(new, mount.yaw_deg(mount.quat_mul(q0, new)))
                response.message = f'IMU axes calibrated: {mount.describe(new)}{unsaved}'
        except ValueError as exc:
            response.success, response.message = False, f'IMU axis calibration ({step}) failed: {exc}'
            self.get_logger().warn(response.message)
            return response
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _set_mount(self, q_sb, yaw0):
        """Use a new mount right away and write it into the params files.
        Returns '' or a ' (NOT SAVED: ...)' suffix for the reply."""
        self._mount = tuple(q_sb)
        self._yaw0 = float(yaw0)
        values = {'mount_quaternion': [round(c, 6) for c in self._mount],
                  'yaw_offset_deg': round(self._yaw0, 3)}
        errors = []
        for path in self._params_files:
            if os.path.exists(path):
                try:
                    update_file(path, values)
                except (OSError, ValueError) as exc:
                    errors.append(f'{path}: {exc}')
        if errors:
            self.get_logger().error('Saving the IMU mount failed (in use until restart): ' + '; '.join(errors))
            return ' (NOT SAVED: ' + '; '.join(errors) + ')'
        return ''

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
