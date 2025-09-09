import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    最小限のNav2テスト用launchファイル
    goal poseルート生成問題のデバッグ用
    """
    
    # 1. 既存のfull launchを起動（ただしrvizとnav2を除く）
    full_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('sick_wagen'),
            '/launch/stepbystep/100_full_test/100_full_test.launch.py'
        ])
    )

    # 2. Localizationを起動
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('sick_wagen'),
            '/launch/full/localization.launch.py'
        ])
    )
    
    delayed_localization_launch = TimerAction(
        period=3.0,
        actions=[localization_launch]
    )

    # 3. 最小限のNav2ノード（デバッグ用）
    # Lifecycle manager
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
                'bt_navigator'
            ]
        }],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Planner server（最も重要）
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('sick_wagen'),
            'config', 'nav2', 'nav2_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'INFO'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('sick_wagen'),
            'config', 'nav2', 'nav2_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'INFO'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # 4. デバッグ用のrviz
    rviz_config_dir = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config', 'rviz', 'full/full.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # 5. デバッグ情報表示ノード
    debug_info = Node(
        package='sick_wagen',
        executable='debug_nav2_info.py',
        name='debug_info',
        output='screen'
    ) if os.path.exists(os.path.join(get_package_share_directory('sick_wagen'), 'scripts', 'debug_nav2_info.py')) else None

    nodes = [
        full_launch,
        delayed_localization_launch,
        planner_server,
        bt_navigator,
        lifecycle_manager,
        rviz_node
    ]
    
    if debug_info:
        nodes.append(debug_info)

    return LaunchDescription(nodes)
