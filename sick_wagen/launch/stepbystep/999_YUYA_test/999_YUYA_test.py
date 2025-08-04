import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('sick_wagen')

    # --- configs ---
    merger_cfg  = os.path.join(pkg_dir, 'config', 'laser_scan_merger', 'params.yaml')
    rviz_cfg    = os.path.join(pkg_dir, 'config', 'rviz', 'stepbystep', '005.rviz')
    whill_param = os.path.join(pkg_dir, 'config', 'whill', 'whill_param.yaml')

    whill_params = [whill_param] if os.path.isfile(whill_param) else []

    nodes = [

        # ---------------- Robot model & WHILL ----------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            remappings=[('/joint_states', '/whill/states/joint_state')],
            arguments=[os.path.join(pkg_dir, 'urdf', 'sick_wagen.urdf')],
            output='screen',
        ),
        Node(
            package='ros2_whill',
            executable='ros2_whill',
            namespace='whill',
            respawn=True,
            parameters=whill_params,   # 無ければ空
            output='screen',
        ),

        # ---------------- laser -> laser_link (TF) ----------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_link_to_laser',
            # x y z qx qy qz qw parent child
            arguments=['0','0','0','0','0','0','1','laser_link','laser'],
        ),

        # ---------------- SICK TIM 5xx × 2 ----------------
        Node(
            package='sick_scan_xd', executable='sick_generic_caller',
            name='sick_tim_R', namespace='sick_tim_R', output='screen',
            parameters=[
                {"scanner_type":"sick_tim_5xx"},
                {"hostname":"TIM_RIGHT_IP"},
                {"frame_id":"tim_link_R"},
                {"cloud_topic":"tim_cloud_R"},
                {"use_binary_protocol":True},
                {"range_max":100.0},
                {"min_ang":-1.832595715}, {"max_ang":1.832595715},
            ],
            remappings=[('/sick_tim_5xx/scan','tim_scan_R')],
            respawn=True, respawn_delay=2.0,
        ),
        Node(
            package='sick_scan_xd', executable='sick_generic_caller',
            name='sick_tim_L', namespace='sick_tim_L', output='screen',
            parameters=[
                {"scanner_type":"sick_tim_5xx"},
                {"hostname":"TIM_LEFT_IP"},
                {"frame_id":"tim_link_L"},
                {"cloud_topic":"tim_cloud_L"},
                {"use_binary_protocol":True},
                {"range_max":100.0},
                {"min_ang":-1.832595715}, {"max_ang":1.832595715},
            ],
            remappings=[('/sick_tim_5xx/scan','tim_scan_L')],
            respawn=True, respawn_delay=2.0,
        ),

        # ---------------- Scan マージ → PointCloud2 ----------------
        Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[merger_cfg],
            remappings=[('cloud_out', '/merged_cloud')],
            respawn=True, respawn_delay=2.0,
            output='screen',
        ),

        # ---------------- PointCloud2 → LaserScan (/scan) ----------------
        # 入力 cloud の frame をそのまま使う（TF 不要）
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'queue_size': 50,
                'target_frame': '',        # 変換しない：cloudのframeをそのまま使用
                'range_min': 0.05,
                'range_max': 30.0,
                'use_inf': True,
            }],
            remappings=[('cloud_in', '/merged_cloud'), ('scan', '/scan')],
            respawn=True, respawn_delay=2.0,
            output='screen',
        ),

        # ---------------- SLAM (slam_toolbox) ----------------
        # ライフサイクル非使用：通常の Node として起動
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'scan_topic': '/scan',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame':  'map',
                'use_sim_time': False,
                # 任意の安定化系パラメータ（必要に応じて調整）
                'resolution': 0.05,
                'minimum_time_between_update': 0.2,
            }],
            respawn=True, respawn_delay=3.0,
            output='screen',
        ),

        # ---------------- RViz2 ----------------
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
