#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String

class Tst_nodeNode(LifecycleNode):
    def __init__(self):
        super().__init__('tst_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = 'testing publisher'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "' + msg.data + '"')

def main():
    rclpy.init()
    rclpy.spin(Tst_nodeNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
