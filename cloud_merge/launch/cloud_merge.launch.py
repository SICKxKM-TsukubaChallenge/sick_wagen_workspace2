from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_merge',   # パッケージ名
            executable='cloud_merge_node',  # 実行するノードの名前
            name='cloud_merge_node',  # ノード名
            output='screen',          # 標準出力をスクリーンに表示
            parameters=[{
                'use_sim_time': False  # 必要に応じてシミュレーション時間を使用
            }],
            remappings=[
                ('/cloud_in1', '/merged_cloud'),  # トピック名のリマッピング
                ('/cloud_in2', '/multiScan/cloud_360'),
            ]
        )
    ])

