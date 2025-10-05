import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sick_wagen'),
                'launch/full/nav2.launch.py'
            )
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('autostart', 'true'),
            ('params_file', os.path.join(
                get_package_share_directory('sick_wagen'),
                'config', 'nav2', 'nav2_params.yaml'
            )),
        ]
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sick_wagen'),
                'launch/full/sensor.py'
            )
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sick_wagen'),
                'launch/full/localization.launch.py'
            )
        )
    )

    # localization_launchを遅延して起動
    delayed_localization_launch = TimerAction(
        period=3.0,
        actions=[localization_launch]
    )

    urdf_file = os.path.join(
        get_package_share_directory('sick_wagen'),
        'urdf', 'sick_wagen.urdf'
    )
    robot_description = {'robot_description': open(urdf_file).read()}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )

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

    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {'yaml_filename': '/home/sick/ros2_ws/maps/map_2024-11_white_12.yaml'},
    #         {'use_sim_time': False}
    #     ]
    # )

    return LaunchDescription([
        # map_server_node,
        robot_state_publisher,
        rviz_node,
        sensor_launch,
        delayed_localization_launch,
        nav2_launch,
    ])