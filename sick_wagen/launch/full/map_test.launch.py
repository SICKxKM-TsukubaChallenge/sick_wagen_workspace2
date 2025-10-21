import os
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    sick_wagen_dir = get_package_share_directory('sick_wagen')

    # PGM map (OccupancyGrid)
    map_yaml = os.path.join('/home/sick/ros2_ws/maps', 'white.yaml')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}]
    )

    # PCD map (PointCloud2) - localization.launch.pyをインクルード（2秒遅延）
    pcd_publisher = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('sick_wagen'),
                        'launch/full/localization.launch.py'
                    )
                )
            )
        ]
    )

    # RViz
    rviz_config = os.path.join(sick_wagen_dir, 'config', 'rviz/full', 'map_test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Lifecycle Manager for map_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # Delay starting map_server + lifecycle_manager so RViz can subscribe first
    delayed_map = TimerAction(
        period=2.0,
        actions=[map_server, lifecycle_manager]
    )

    return LaunchDescription([
        rviz_node,
        pcd_publisher,
        delayed_map
    ])