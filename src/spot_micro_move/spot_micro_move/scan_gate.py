"""
scan_gate — relays lidar scans to SLAM only while the robot is steady.

  /scan  --(only scans taken while no foot was in the air)-->  /scan_gated

gait_node publishes `stable_topic` (std_msgs/Bool) every control tick: False
while any foot is lifted. A lifted leg tips the body (and the lidar) a few
degrees and the crawl's swing is the jerkiest part of the cycle, so scans
taken then don't line up with the rest and smear the map. A scan is passed
only if every stable sample over its whole sweep, [stamp, stamp + scan_time]
plus `margin_s` either side, was True.

If no stable message has arrived for `passthrough_timeout_s` (gait_node not
running, e.g. bringup on its own), every scan passes — so this is safe to
leave in the sensing stack permanently. Touches no hardware.
"""

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanGate(Node):

    def __init__(self):
        super().__init__('scan_gate')
        self.declare_parameter('scan_in', '/scan')
        self.declare_parameter('scan_out', '/scan_gated')
        self.declare_parameter('stable_topic', '/spotmicro_move/stable')
        self.declare_parameter('margin_s', 0.02)
        self.declare_parameter('passthrough_timeout_s', 1.0)
        self.declare_parameter('history_s', 3.0)

        p = self.get_parameter
        self._margin = float(p('margin_s').value)
        self._passthrough = float(p('passthrough_timeout_s').value)
        self._history = float(p('history_s').value)
        self._samples = deque()          # (ros time s, stable)
        self._passed = 0
        self._dropped = 0

        self._pub = self.create_publisher(LaserScan, p('scan_out').value, qos_profile_sensor_data)
        self.create_subscription(Bool, p('stable_topic').value, self._on_stable, 50)
        self.create_subscription(LaserScan, p('scan_in').value, self._on_scan, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'Gating {p("scan_in").value} -> {p("scan_out").value} on {p("stable_topic").value}.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_stable(self, msg):
        now = self._now()
        self._samples.append((now, bool(msg.data)))
        while self._samples and self._samples[0][0] < now - self._history:
            self._samples.popleft()

    def _steady_over(self, start, end):
        # The sample in effect at `start` is the last one at or before it.
        before = None
        for t, stable in self._samples:
            if t <= start:
                before = stable
            elif t <= end:
                if not stable:
                    return False
            else:
                break
        return before is not False

    def _on_scan(self, scan):
        if not self._samples or self._now() - self._samples[-1][0] > self._passthrough:
            self._pub.publish(scan)
            return
        start = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        duration = scan.scan_time or scan.time_increment * len(scan.ranges)
        if self._steady_over(start - self._margin, start + duration + self._margin):
            self._passed += 1
            self._pub.publish(scan)
        else:
            self._dropped += 1

    def _report(self):
        if self._passed or self._dropped:
            self.get_logger().info(
                f'last 10s: passed {self._passed} scans, dropped {self._dropped} (foot in the air).')
        self._passed = self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
