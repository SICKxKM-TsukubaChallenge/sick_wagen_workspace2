import os

from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration, NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config',
        'laser_scan_merger',
        'params.yaml'
    )
    pkg_dir = get_package_share_directory("sick_wagen")
    rviz_config_dir = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config/rviz/stepbystep',
        '006.rviz'
    )
    slam_config_file = LaunchConfiguration('slam_config_file');
    
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        remappings=[("/joint_states", "/whill/states/joint_state")],
        arguments=[os.path.join(pkg_dir, "urdf", "sick_wagen.urdf")]
    )
    
    whill_node = Node(
        package="ros2_whill",
        executable="ros2_whill",
        namespace="whill",
        output="screen",
        respawn=True,
        parameters=[os.path.join(pkg_dir, "config/whill", "whill_params.yaml")],
    )
    
    tim_R_node = Node(
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
            {"frame_id":"tim_link_L"},
            {"port":"2112"},
            {"timelimit":5},
            {"sw_pll_only_publish":True},
            {"min_intensity":0.0},
        ],
        remappings = [('/sick_tim_5xx/scan','tim_scan_R')]
    )
    
    tf_R_node = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        output = 'screen',
        arguments = [
            "0.83","-0.31","0","-0.785398","0","0","base_link","tim_link_R"
        ]
    )
    
    tim_L_node = Node(
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
            {"frame_id":"tim_link_R"},
            {"port":"2112"},
            {"timelimit":5},
            {"sw_pll_only_publish":True},
            {"min_intensity":0.0},
        ],
        remappings = [('/sick_tim_5xx/scan','tim_scan_L')]
    )
    
    tf_L_node = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        output = 'screen',
        arguments = [
            "0.83", "0.31", "0", "0.785398", "0", "0", "base_link", "tim_link_L"
        ]
    )
    
    LaserScan_merger_node = launch_ros.actions.Node(
        package = "ros2_laser_scan_merger",
        executable='ros2_laser_scan_merger',
        parameters=[config],
        output='screen',
        respawn=True,
        respawn_delay=2,
        remappings = [('/lidar_1/scan','/sick_tim_L/sick_tim_5xx/scan'), ('/lidar_2/scan','/sick_tim_R/sick_tim_5xx/scan')],
    )
    
    Cloud_to_LaserScan_node = Node(
        name='pointcloud_to_laserscan',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[config]
    )
    
    joy_node = Node(
            name="joy_node",
            package="joy",
            executable="joy_node",
            output="screen",
        remappings=[("/joy", "/whill/controller/joy")],
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    declare_arg_slam_config_file = DeclareLaunchArgument(
        'slam_config_file',
        default_value=os.path.join(
            pkg_dir,
            'config',
            'slam_tools',
            'mapper_params_offline.yaml'
            # 'mapper_params_offline.yaml'
        ),
        description='The full path to the config file for SLAM'
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[slam_config_file]
    )

    ld.add_action(whill_node)
    ld.add_action(tim_R_node)
    ld.add_action(tf_R_node)
    ld.add_action(tim_L_node)
    ld.add_action(tf_L_node)
    ld.add_action(LaserScan_merger_node)
    ld.add_action(Cloud_to_LaserScan_node)
    ld.add_action(joy_node)
    ld.add_action(declare_arg_slam_config_file)
    ld.add_action(slam_node)
    ld.add_action(rviz2_node)
    ld.add_action(robot_state_publisher_node)
    
    return ld
