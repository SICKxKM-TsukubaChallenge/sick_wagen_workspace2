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
    LAPTOP_IP = "192.168.0.101"
    MULTISCAN_IP = "192.168.0.4"
    
    config = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config',
        'laser_scan_merger',
        'params.yaml'
    )
    pkg_dir = get_package_share_directory("sick_wagen")    
    
    
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
                {"frame_id":"tim_link_R"},
                # {"tf_base_frame_id":"tim_link_L"},
                {"port":"2112"},
                {"timelimit":5},
                {"sw_pll_only_publish":True},
                {"min_intensity":0.0},
            ],
            remappings = [('/sick_tim_R/sick_tim_5xx/scan','/tim_scans/tim_scan_R')]
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
                {"frame_id":"tim_link_L"},
                # {"tf_base_frame_id":"tim_link_R"},
                {"port":"2112"},
                {"timelimit":5},
                {"sw_pll_only_publish":True},
                {"min_intensity":0.0},
            ],
            remappings = [('/sick_tim_L/sick_tim_5xx/scan','/tim_scans/tim_scan_L')]
        )
        
    
    laser_merge_node = launch_ros.actions.Node(
            package = "ros2_laser_scan_merger",
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
            remappings = [('/lidar_1/scan','/tim_scans/tim_scan_L'), ('/lidar_2/scan','/tim_scans/tim_scan_R')],
        )


    ld.add_action(multiscan_node)
    # ld.add_action(cloud_merge_node)
    ld.add_action(tim_R_node)
    ld.add_action(tim_L_node)
    # ld.add_action(laser_merge_node)
    
    return ld
