#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import csv
import yaml
import os
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from datetime import datetime

# 現在の日時を取得し、ファイル名に使用
now = datetime.now()
csv_filename = now.strftime("%Y-%m-%d_%H%M") + ".csv"
yaml_filename = now.strftime("%Y-%m-%d_%H%M") + "_waypoints.yaml"

# 保存ディレクトリ（例: このスクリプトのdataディレクトリに保存）
save_dir = os.path.join(os.path.dirname(__file__), '../data')
os.makedirs(save_dir, exist_ok=True)  # ディレクトリがなければ作成

csvfile = os.path.join(save_dir, csv_filename)
yamlfile = os.path.join(save_dir, yaml_filename)

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            Joy,
            '/whill/controller/joy',
            self.joy_callback,
            10)
        
        self.waypoints = []
        self.waypoint_count = 0
        
        # CSVファイル初期化
        with open(csvfile, 'w') as fc:
            writer = csv.writer(fc)
            writer.writerow(['waypoint_data'])
        
        # YAMLファイル初期化
        self.init_yaml_file()
        
        self.get_logger().info('Waypoint Saver Node started')
    
    def init_yaml_file(self):
        waypoints_data = {
            'waypoints': []
        }
        with open(yamlfile, 'w') as f:
            yaml.dump(waypoints_data, f, default_flow_style=False)

    def joy_callback(self, msg):
        continue_signal = msg.buttons[1]  # ボタン2 (index 1) - 進行ウェイポイント
        pause_signal = msg.buttons[2]     # ボタン3 (index 2) - 一時停止ウェイポイント

        if continue_signal or pause_signal:
            try:
                # map -> base_link の変換を取得
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                
                action = 0 if continue_signal else 1
                action_name = "continue" if continue_signal else "pause"
                
                self.get_logger().info(f"Saving {action_name} waypoint at: x={x:.3f}, y={y:.3f}, qz={qz:.3f}, qw={qw:.3f}")
                
                # CSV形式で保存（従来の形式）
                waypoint_data = f"[({x},{y},0.0),(0.0,0.0,{qz},{qw})],{action}"
                with open(csvfile, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([waypoint_data])
                
                # Nav2用YAML形式で保存
                self.save_nav2_waypoint(x, y, z, qx, qy, qz, qw, action)
                
            except Exception as e:
                self.get_logger().error(f"Failed to get transform: {str(e)}")
    
    def save_nav2_waypoint(self, x, y, z, qx, qy, qz, qw, action):
        self.waypoint_count += 1
        
        # Nav2用のウェイポイント形式
        waypoint = {
            'name': f'waypoint_{self.waypoint_count}',
            'pose': {
                'position': {
                    'x': float(x),
                    'y': float(y),
                    'z': float(z)
                },
                'orientation': {
                    'x': float(qx),
                    'y': float(qy),
                    'z': float(qz),
                    'w': float(qw)
                }
            },
            'action': int(action),  # 0: continue, 1: pause
            'frame_id': 'map'
        }
        
        # 既存のYAMLファイルを読み込み
        with open(yamlfile, 'r') as f:
            data = yaml.safe_load(f)
        
        # ウェイポイントを追加
        data['waypoints'].append(waypoint)
        
        # YAMLファイルに書き戻し
        with open(yamlfile, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def main():
    rclpy.init()
    
    waypoint_saver = WaypointSaver()
    
    try:
        rclpy.spin(waypoint_saver)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
