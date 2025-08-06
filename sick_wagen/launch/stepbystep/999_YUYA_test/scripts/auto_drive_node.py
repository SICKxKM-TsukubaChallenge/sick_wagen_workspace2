#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')
        self.pub = self.create_publisher(Twist, '/whill/cmd_vel', 10)

        self.get_logger().info('AutoDriveNode started: publishing forward command...')
        twist = Twist()
        twist.linear.x = 0.2  # m/s

        # 1m 進むには t = d/v = 1.0 / 0.2 = 5秒
        start_time = self.get_clock().now()
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, lambda: self.publish_for_duration(twist, start_time, 5.0))

    def publish_for_duration(self, twist, start_time, duration_sec):
        now = self.get_clock().now()
        elapsed = (now - start_time).nanoseconds / 1e9

        if elapsed < duration_sec:
            self.pub.publish(twist)
        else:
            self.get_logger().info('AutoDriveNode stopping: target distance reached.')
            self.pub.publish(Twist())  # 停止
            self.timer.cancel()
            self.destroy_node()

def main():
    rclpy.init()
    node = AutoDriveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

