#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class FakeJoyNode(Node):
    def __init__(self):
        super().__init__('fake_joy_node')
        self.publisher = self.create_publisher(Joy, '/whill/controller/joy', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.send_fake_joy)
        self.duration = 1.0  # 秒間だけ前進

    def send_fake_joy(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        joy = Joy()
        joy.axes = [0.0, 1.0]  # ↑これが前進（必要に応じて [1.0, 0.0] とかに変更）
        joy.buttons = [0] * 12
        if elapsed > self.duration:
            joy.axes = [0.0, 0.0]
            self.get_logger().info("前進終了、停止命令を送信します")
            self.timer.cancel()
        self.publisher.publish(joy)

def main(args=None):
    rclpy.init(args=args)
    node = FakeJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

