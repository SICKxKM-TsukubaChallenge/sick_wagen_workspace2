#!/usr/bin/env python3

import csv
import yaml
import os
import glob
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.node import Node


class WaypointPublisher(Node):
    def __init__(self, waypoint_file):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'waypoints', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # ファイルの拡張子で判定
        if waypoint_file.endswith('.yaml'):
            self.waypoints = self.load_waypoints_from_yaml(waypoint_file)
            self.get_logger().info(f"Loaded waypoints from YAML: {os.path.basename(waypoint_file)}")
        elif waypoint_file.endswith('.csv'):
            self.waypoints = self.load_waypoints_from_csv(waypoint_file)
            self.get_logger().info(f"Loaded waypoints from CSV: {os.path.basename(waypoint_file)}")
        else:
            self.get_logger().error(f"Unsupported file type: {waypoint_file}")
            self.waypoints = []

        if not self.waypoints:
            self.get_logger().warning("No waypoints loaded. Please check the specified file.")

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
        markers = MarkerArray()
        
        for idx, wp in enumerate(self.waypoints):
            pose = Pose()
            pose.position.x = wp['x']
            pose.position.y = wp['y']
            pose.position.z = wp['z']
            pose.orientation.x = wp['qx']
            pose.orientation.y = wp['qy']
            pose.orientation.z = wp['qz']
            pose.orientation.w = wp['qw']
            msg.poses.append(pose)

            # Arrow marker for waypoint position (action=0: red, action=1: yellow)
            arrow_marker = Marker()
            arrow_marker.header = msg.header
            arrow_marker.ns = 'waypoint_arrows'
            arrow_marker.id = idx
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = wp['x']
            arrow_marker.pose.position.y = wp['y']
            arrow_marker.pose.position.z = wp['z']
            arrow_marker.pose.orientation.x = wp['qx']
            arrow_marker.pose.orientation.y = wp['qy']
            arrow_marker.pose.orientation.z = wp['qz']
            arrow_marker.pose.orientation.w = wp['qw']
            arrow_marker.scale.x = 1.0  # arrow length
            arrow_marker.scale.y = 0.3  # arrow width
            arrow_marker.scale.z = 0.3  # arrow height
            arrow_marker.color.a = 1.0
            
            # Lifetime設定（永続的に表示）
            arrow_marker.lifetime.sec = 0
            arrow_marker.lifetime.nanosec = 0
            
            # action=0: continue (赤色), action=1: pause (黄色)
            if wp['action'] == 1:  # pause
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
                self.get_logger().debug(f'Waypoint {idx}: PAUSE (yellow) - action={wp["action"]}, color=(1.0, 1.0, 0.0)')
            else:  # continue (default)
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 0.0
                arrow_marker.color.b = 0.0
                self.get_logger().debug(f'Waypoint {idx}: CONTINUE (red) - action={wp["action"]}, color=(1.0, 0.0, 0.0)')
            
            markers.markers.append(arrow_marker)

            # Text marker for waypoint label
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = 'waypoint_labels'
            text_marker.id = idx
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = wp['z'] + 0.5  # lift text a bit for visibility
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.6
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0  # white text
            text_marker.text = wp.get('name', f'waypoint_{idx + 1}')
            
            # Lifetime設定（永続的に表示）
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 0
            
            markers.markers.append(text_marker)
            
            if wp['action'] == 0:
                continue_count += 1
            else:
                pause_count += 1
        
        self.publisher_.publish(msg)
        self.marker_pub.publish(markers)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints '
                             f'(Continue: {continue_count}, Pause: {pause_count})')
        self.get_logger().info(f'Published {len(markers.markers)} markers')

def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(description='Publish waypoints from a specified YAML/CSV file in data/')
    parser.add_argument('--file', '-f', dest='file', help='Full path to waypoint YAML or CSV file')
    parser.add_argument('--name', '-n', dest='name', help='Filename under data/ to use (e.g. 2025-11-08_1051_waypoints.yaml)')
    parsed = parser.parse_args()

    # precedence: --file fullpath > --name (in data dir) > env WAYPOINT_FILE > hardcoded default
    data_dir = os.path.join(os.path.dirname(__file__), '../data')
    env_file = os.environ.get('WAYPOINT_FILE', '').strip()
    if parsed.file:
        waypoint_file = parsed.file
    elif parsed.name:
        waypoint_file = os.path.join(data_dir, parsed.name)
    elif env_file:
        # allow either full path or filename
        if os.path.isabs(env_file):
            waypoint_file = env_file
        else:
            waypoint_file = os.path.join(data_dir, env_file)
    else:
        waypoint_file = os.path.join(data_dir, 'final.yaml')

    rclpy.init(args=args)
    node = WaypointPublisher(waypoint_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
