#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import traceback

class Nav2DiagnosticNode(Node):
    def __init__(self):
        super().__init__('nav2_diagnostic')
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 10)
        self.plan_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
        
        # Publishers for testing
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # State tracking
        self.map_received = False
        self.global_costmap_received = False
        self.plan_received = False
        
        # Timer for diagnostics
        self.timer = self.create_timer(5.0, self.run_diagnostics)
        
        self.get_logger().info("Nav2 Diagnostic Node started")

    def map_callback(self, msg):
        self.map_received = True
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}")

    def global_costmap_callback(self, msg):
        self.global_costmap_received = True
        self.get_logger().info(f"Global costmap received: {msg.info.width}x{msg.info.height}")

    def plan_callback(self, msg):
        self.plan_received = True
        self.get_logger().info(f"Plan received with {len(msg.poses)} poses")

    def check_tf_transforms(self):
        """重要なTF変換をチェック"""
        transforms_to_check = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('map', 'base_link')
        ]
        
        results = {}
        for parent, child in transforms_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                results[f"{parent}->{child}"] = "OK"
                self.get_logger().info(f"TF {parent}->{child}: OK")
            except TransformException as ex:
                results[f"{parent}->{child}"] = f"FAILED: {ex}"
                self.get_logger().error(f"TF {parent}->{child}: FAILED - {ex}")
        
        return results

    def send_test_goal(self):
        """テスト用のgoal poseを送信"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # 現在位置から少し前方にgoalを設定
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            goal.pose.position.x = transform.transform.translation.x + 1.0
            goal.pose.position.y = transform.transform.translation.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal)
            self.get_logger().info(f"Test goal sent: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
            
        except TransformException as ex:
            self.get_logger().error(f"Cannot send test goal - TF failed: {ex}")

    def run_diagnostics(self):
        """診断情報を出力"""
        self.get_logger().info("=== Nav2 Diagnostic Report ===")
        
        # 1. マップとコストマップの状態
        self.get_logger().info(f"Map received: {self.map_received}")
        self.get_logger().info(f"Global costmap received: {self.global_costmap_received}")
        
        # 2. TF変換の状態
        tf_results = self.check_tf_transforms()
        
        # 3. プランの状態
        self.get_logger().info(f"Plan received: {self.plan_received}")
        
        # 4. テストgoalを送信（10秒おきに）
        if hasattr(self, 'test_goal_counter'):
            self.test_goal_counter += 1
        else:
            self.test_goal_counter = 1
            
        if self.test_goal_counter % 2 == 0:  # 10秒おき
            self.send_test_goal()
        
        self.get_logger().info("=== End Diagnostic Report ===")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2DiagnosticNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
