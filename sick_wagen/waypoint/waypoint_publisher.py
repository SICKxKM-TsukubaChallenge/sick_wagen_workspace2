#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import csv
import yaml
import os
import glob


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'waypoints', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # データディレクトリのパス
        self.data_dir = os.path.join(os.path.dirname(__file__), '../data')
        
        # 最新のウェイポイントファイルを読み込み
        self.waypoints = self.load_latest_waypoints()
        
        if not self.waypoints:
            self.get_logger().warning("No waypoints loaded. Please create waypoints first using waypoint_saver.")

    def load_latest_waypoints(self):
        """最新のウェイポイントファイル（YAML優先、CSV fallback）を読み込み"""
        waypoints = []
        
        # YAMLファイルを優先的に探す
        yaml_files = glob.glob(os.path.join(self.data_dir, '*_waypoints.yaml'))
        if yaml_files:
            # 最新のYAMLファイルを選択
            latest_yaml = max(yaml_files, key=os.path.getctime)
            waypoints = self.load_waypoints_from_yaml(latest_yaml)
            if waypoints:
                self.get_logger().info(f"Loaded waypoints from YAML: {os.path.basename(latest_yaml)}")
                return waypoints
        
        # YAMLが見つからない場合はCSVを探す
        csv_files = glob.glob(os.path.join(self.data_dir, '*.csv'))
        if csv_files:
            # 最新のCSVファイルを選択
            latest_csv = max(csv_files, key=os.path.getctime)
            waypoints = self.load_waypoints_from_csv(latest_csv)
            if waypoints:
                self.get_logger().info(f"Loaded waypoints from CSV: {os.path.basename(latest_csv)}")
                return waypoints
        
        self.get_logger().error(f"No waypoint files found in {self.data_dir}")
        return waypoints

    def load_waypoints_from_yaml(self, yaml_path):
        """YAMLファイルからウェイポイントを読み込み"""
        waypoints = []
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                
            if 'waypoints' in data:
                for wp in data['waypoints']:
                    if 'pose' in wp:
                        pos = wp['pose']['position']
                        ori = wp['pose']['orientation']
                        action = wp.get('action', 0)
                        waypoints.append({
                            'x': pos['x'],
                            'y': pos['y'], 
                            'z': pos['z'],
                            'qx': ori['x'],
                            'qy': ori['y'],
                            'qz': ori['z'],
                            'qw': ori['w'],
                            'action': action,
                            'name': wp.get('name', f'waypoint_{len(waypoints)+1}')
                        })
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file {yaml_path}: {str(e)}")
        
        return waypoints

    def load_waypoints_from_csv(self, csv_path):
        """CSVファイルからウェイポイントを読み込み（従来形式対応）"""
        waypoints = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader, None)  # ヘッダーをスキップ
                
                for i, row in enumerate(reader):
                    if len(row) > 0 and row[0].strip():
                        # 新形式：[({x},{y},0.0),(0.0,0.0,{qz},{qw})],{action}
                        waypoint_str = row[0].strip()
                        if waypoint_str.startswith('[') and waypoint_str.endswith(']'):
                            waypoint = self.parse_waypoint_string(waypoint_str, i+1)
                            if waypoint:
                                waypoints.append(waypoint)
        except Exception as e:
            self.get_logger().error(f"Error loading CSV file {csv_path}: {str(e)}")
        
        return waypoints

    def parse_waypoint_string(self, waypoint_str, index):
        """ウェイポイント文字列をパース"""
        try:
            # [({x},{y},0.0),(0.0,0.0,{qz},{qw})],{action} の形式をパース
            import re
            
            # アクション部分を抽出
            action_match = re.search(r',(\d+)$', waypoint_str)
            action = int(action_match.group(1)) if action_match else 0
            
            # 座標部分を抽出
            pos_match = re.search(r'\(([^,]+),([^,]+),[^)]+\)', waypoint_str)
            rot_match = re.search(r'\([^,]+,[^,]+,([^,]+),([^)]+)\)', waypoint_str)
            
            if pos_match and rot_match:
                x = float(pos_match.group(1))
                y = float(pos_match.group(2))
                qz = float(rot_match.group(1))
                qw = float(rot_match.group(2))
                
                return {
                    'x': x, 'y': y, 'z': 0.0,
                    'qx': 0.0, 'qy': 0.0, 'qz': qz, 'qw': qw,
                    'action': action,
                    'name': f'waypoint_{index}'
                }
        except Exception as e:
            self.get_logger().error(f"Error parsing waypoint string: {waypoint_str}, error: {str(e)}")
        
        return None

    def timer_callback(self):
        if not self.waypoints:
            return
            
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        continue_count = 0
        pause_count = 0
        
        for wp in self.waypoints:
            pose = Pose()
            pose.position.x = wp['x']
            pose.position.y = wp['y']
            pose.position.z = wp['z']
            pose.orientation.x = wp['qx']
            pose.orientation.y = wp['qy']
            pose.orientation.z = wp['qz']
            pose.orientation.w = wp['qw']
            msg.poses.append(pose)
            
            if wp['action'] == 0:
                continue_count += 1
            else:
                pause_count += 1
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints '
                             f'(Continue: {continue_count}, Pause: {pause_count})')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()