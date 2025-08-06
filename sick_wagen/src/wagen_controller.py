#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class WagenController(Node):
    def __init__(self):
        super().__init__('wagen_controller')

        # Declare and get parameters
        self.declare_parameter('linearGain', 0.5)
        self.declare_parameter('angularGain', 0.6)
        self.declare_parameter('speedUpRate', 1.6)

        self.linear_gain = self.get_parameter('linearGain').value
        self.angular_gain = self.get_parameter('angularGain').value
        self.speed_up_rate = self.get_parameter('speedUpRate').value

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_move_base', self.nav_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(Joy, '/whill/states/joy', self.whill_callback, 10)

        # Storage
        self.cmd_vel = Twist()
        self.sub_cmd_vel = Twist()
        self.cmd_vel_joy = Twist()
        self.cmd_vel_whill = Twist()

        # Timer (10Hz)
        self.create_timer(0.1, self.update_loop)

    def nav_callback(self, msg):
        self.sub_cmd_vel.linear.x = msg.linear.x
        self.sub_cmd_vel.angular.z = msg.angular.z

    def joy_callback(self, msg):
        RStickX = msg.axes[2]
        RStickY = msg.axes[3]
        LB_btn = msg.buttons[4]

        self.cmd_vel_joy.linear.x = math.copysign(RStickY ** 2, RStickY) * self.linear_gain
        if LB_btn == 1:
            self.cmd_vel_joy.linear.x *= self.speed_up_rate

        self.cmd_vel_joy.angular.z = RStickX * self.angular_gain

    def whill_callback(self, msg):
        self.cmd_vel_whill.linear.x = msg.axes[1]
        self.cmd_vel_whill.angular.z = msg.axes[0]

    def update_loop(self):
        if self.cmd_vel_whill.linear.x != 0.0 or self.cmd_vel_whill.angular.z != 0.0:
            self.cmd_vel = self.cmd_vel_whill
        elif self.cmd_vel_joy.linear.x != 0.0 or self.cmd_vel_joy.angular.z != 0.0:
            self.cmd_vel = self.cmd_vel_joy
        else:
            self.cmd_vel = self.sub_cmd_vel

        self.cmd_pub.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = WagenController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
