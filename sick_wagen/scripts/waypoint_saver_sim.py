#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import yaml
import os
from datetime import datetime

class GoalPoseWaypointSaver(Node):
    def __init__(self):
        super().__init__('goal_pose_waypoint_saver')
        now = datetime.now()
        save_dir = os.path.join(os.path.dirname(__file__), '../data')
        os.makedirs(save_dir, exist_ok=True)
        csv_filename = now.strftime("%Y-%m-%d_%H%M") + "_goal.csv"
        yaml_filename = now.strftime("%Y-%m-%d_%H%M") + "_goal_waypoints.yaml"
        self.csvfile = os.path.join(save_dir, csv_filename)
        self.yamlfile = os.path.join(save_dir, yaml_filename)
        self.waypoint_count = 0

        # CSV初期化
        with open(self.csvfile, 'w') as fc:
            pass
        # YAML初期化
        with open(self.yamlfile, 'w') as f:
            yaml.dump({'waypoints': []}, f, default_flow_style=False)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10)
        self.get_logger().info('GoalPoseWaypointSaver started. Click 2D Goal Pose in RViz!')

    def goal_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        action = 0  # 0: continue, 1: pause など用途に応じて

        # CSV保存
        waypoint_row = [x, y, 0, 0, 0, qz, qw, action]
        with open(self.csvfile, 'a') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(waypoint_row)

        # YAML保存
        self.waypoint_count += 1
        waypoint = {
            'name': f'waypoint_{self.waypoint_count}',
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': float(z)},
                'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
            },
            'action': int(action),
            'frame_id': msg.header.frame_id
        }
        with open(self.yamlfile, 'r') as f:
            data = yaml.safe_load(f)
        data['waypoints'].append(waypoint)
        with open(self.yamlfile, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f"Saved waypoint: x={x:.3f}, y={y:.3f}, qz={qz:.3f}, qw={qw:.3f}")

def main():
    rclpy.init()
    node = GoalPoseWaypointSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()