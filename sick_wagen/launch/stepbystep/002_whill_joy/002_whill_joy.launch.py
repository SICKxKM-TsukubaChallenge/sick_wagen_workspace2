import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("ros2_whill")
    list = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="",
            # parameters=[{"robot_description" : os.path.join(pkg_dir, "urdf", "modelc.urdf")}],
            remappings=[("/joint_states", "/whill/states/joint_state")],
            arguments=[os.path.join(pkg_dir, "urdf", "modelc.urdf")]
        ),
        Node(
            package="ros2_whill",
            executable="ros2_whill",
            namespace="whill",
            output="screen",
            respawn=True,
            parameters=[os.path.join(pkg_dir, "params", "whill_param.yaml")],
        ),
        Node(
            name="joy_node",
            package="joy",
            executable="joy_node",
            output="screen",
            remappings=[("/joy", "/whill/controller/joy")],
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=[os.path.join("/home/sick/ros2_ws/sick_tsukuba_ws/sick_wagen/launch/stepbystep/002_whill_joy", "002_whill_joy.rviz")]
        )
    ]

    return LaunchDescription(list)