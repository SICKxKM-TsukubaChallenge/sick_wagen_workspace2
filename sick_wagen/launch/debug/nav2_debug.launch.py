import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Nav2のデバッグ用launchファイル
    goal poseを設定したときにルートが生成されない問題を調査するため
    """
    
    # Nav2 lifecycle manager with debug output
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    # Planner server with debug logging
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('sick_wagen'),
            'config', 'nav2', 'nav2_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # BT Navigator with debug logging
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('sick_wagen'),
            'config', 'nav2', 'nav2_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Global costmap with debug logging
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='global_costmap',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('sick_wagen'),
            'config', 'nav2', 'nav2_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/global_costmap/costmap', '/global_costmap/costmap'),
            ('/global_costmap/costmap_updates', '/global_costmap/costmap_updates')
        ]
    )

    # Map server - YAMLファイルが必要だがPCDしかないので、static transformを使用
    # とりあえずダミーのmap_serverを起動
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': '/tmp/dummy_map.yaml'  # ダミーファイル
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    # Static transform publisher for map frame (とりあえずodomとmapを同じにする)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # RViz for visualization
    rviz_config_dir = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config', 'rviz', 'debug_nav2.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir] if os.path.exists(rviz_config_dir) else [],
        output='screen'
    )

    return LaunchDescription([
        static_transform_publisher,
        # map_server,  # ダミーなのでコメントアウト
        global_costmap,
        planner_server,
        bt_navigator,
        lifecycle_manager,
        rviz_node
    ])
