import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("sick_wagen")
    rviz_config_dir = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config/rviz/stepbystep',
        '002_whill_joy.rviz'
    )
    list = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="",
            remappings=[("/joint_states", "/whill/states/joint_state")],
            arguments=[os.path.join(pkg_dir, "urdf", "sick_wagen.urdf")]
        ),
        Node(
            package="ros2_whill",
            executable="ros2_whill",
            namespace="whill",
            output="screen",
            respawn=True,
            parameters=[os.path.join(pkg_dir, "config/whill", "whill_param.yaml")],
        ),
        Node(
            name="joy_node",
            package="joy",
            executable="joy_node",
            output="screen",
            remappings=[("/joy", "/whill/controller/joy")],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ]

    return LaunchDescription(list)