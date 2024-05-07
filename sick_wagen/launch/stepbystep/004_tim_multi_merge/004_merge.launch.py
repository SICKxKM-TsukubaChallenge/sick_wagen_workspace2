import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config',
        'laser_scan_merger',
        'params.yaml'
    )
    pkg_dir = get_package_share_directory("sick_wagen")
    # rviz_config_dir = os.path.join(
    # get_package_share_directory('sick_wagen'),
    # 'config/rviz/stepbystep',
    # '005.rviz'
    # )
    
    # ↓Absolute path
    # config = "/home/sick/ros2_ws/sick_tsukuba_ws/sick_wagen/config/laser_scan_merger/params.yaml"
    # print(config)
    # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    
    # pkg_share = FindPackageShare('sick_wagen')
    # config = PathJoinSubstitution([pkg_share, 'config','laser_scan_merger','params.yaml'])
    # print(config)
    # print("----------------------------------------")
    
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
                {"frame_id":"tim_link_R"},
                {"port":"2112"},
                {"timelimit":5},
                {"sw_pll_only_publish":True},
                {"min_intensity":0.0},
            ],
            remappings = [('/sick_tim_5xx/scan','tim_scan_L')]
        ),
        
        # Node(
        #     package = 'tf2_ros',
        #     executable = 'static_transform_publisher',
        #     output = 'screen',
        #     arguments = [
        #         "0", "1", "0", "0", "0", "0", "world", "tim_link_L",
        #     ]
        # ),
        
        # Node(
        #     package = 'tf2_ros',
        #     executable = 'static_transform_publisher',
        #     output = 'screen',
        #     arguments = [
        #         "0", "-1", "0", "0", "0", "0", "world", "tim_link_R",
        #     ]
        # ),
        
        launch_ros.actions.Node(
            package = "ros2_laser_scan_merger",
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
            remappings = [('/lidar_1/scan','/sick_tim_L/sick_tim_5xx/scan'), ('/lidar_2/scan','/sick_tim_R/sick_tim_5xx/scan')],
        ),
        
        Node(
            name='pointcloud_to_laserscan',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[config]
        ),
        
        
        
        
        # Node(
        #     package="rviz2", executable="rviz2",
        #     arguments=[os.path.join(rviz_config_dir)]
        # ),
        
    ]

    return LaunchDescription(list)
