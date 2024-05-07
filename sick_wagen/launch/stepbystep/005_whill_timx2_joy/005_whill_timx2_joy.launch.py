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
        '005.rviz'
    )
    list = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="",
            # parameters=[{"robot_description" : os.path.join(pkg_dir, "urdf", "modelc.urdf")}],
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
            package = 'sick_scan_xd',
            executable = 'sick_generic_caller',
            name = 'sick_tim_R',
            namespace = 'sick_tim_R',
            output = 'screen',
            parameters = [
                {"scanner_type":"sick_tim_5xx"},
                {"min_ang":-1.832595715},
                {"max_ang":1.832595715},
                {"use_binary_protocol":True},
                {"range_max":100.0},
                {"intensity":True},
                {"hostname":"TIM_RIGHT_IP"},
                {"cloud_topic":"tim_cloud_R"},
                {"frame_id":"tim_link_R"},
                {"port":"2112"},
                {"timelimit":5},
                {"sw_pll_only_publish":True},
                {"min_intensity":0.0},
            ],
            remappings = [('/sick_tim_5xx/scan','tim_scan_R')]
        ),
        Node(
            package = 'sick_scan_xd',
            executable = 'sick_generic_caller',
            name = 'sick_tim_L',
            namespace = 'sick_tim_L',
            output = 'screen',
            parameters = [
                {"scanner_type":"sick_tim_5xx"},
                {"min_ang":-1.832595715},
                {"max_ang":1.832595715},
                {"use_binary_protocol":True},
                {"range_max":100.0},
                {"intensity":True},
                {"hostname":"TIM_LEFT_IP"},
                {"cloud_topic":"tim_cloud_L"},
                {"frame_id":"tim_link_L"},
                {"port":"2112"},
                {"timelimit":5},
                {"sw_pll_only_publish":True},
                {"min_intensity":0.0},
            ],
            remappings = [('/sick_tim_5xx/scan','tim_scan_L')]
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