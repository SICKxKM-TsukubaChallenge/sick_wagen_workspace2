from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_merge',   # パッケージ名
            executable='cloud_merge_node',  # 実行するノードの名前
            name='cloud_merge_node',  # ノード名
            output='screen',          # 標準出力をスクリーンに表示
            parameters=[config],  # パラメータファイルを指定
        )
    ])

