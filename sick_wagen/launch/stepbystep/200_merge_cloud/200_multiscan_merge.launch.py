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
    
    multiscan_node = Node(
            package="sick_scan_xd",
            executable="sick_generic_caller",
            name="multiScan",
            namespace="multiScan",
            output="screen",
            parameters=[
                {"scanner_type": "sick_multiscan"},
                {"hostname": MULTISCAN_IP},
                {"udp_receiver_ip": LAPTOP_IP},
                {"frame_id":"cloud"},
                {"udp_sender": ""},
                {"udp_port": 2115},
                {"segment_count": 12},
                {"publish_frame_id": "world"},
                {"publish_laserscan_segment_topic": "laserscan_segment"},
                {"publish_laserscan_fullframe_topic": "laserscan_fullframe"},
                {"udp_input_fifolength": 20},
                {"msgpack_output_fifolength": 20},
                {"verbose_level": 0},
                {"measure_timing": True},
                {"export_csv": False},
                {"export_udp_msg": False},
                {"logfolder": ""},
                {"udp_timeout_ms": 60000},
                {"scandataformat": 2},
                {"imu_enable": True},
                {"imu_udp_port": 7503},
                {"imu_latency_microsec": 0},
                {"add_transform_xyz_rpy": "0,0,0,0,0,0"},
                {"add_transform_check_dynamic_updates": False},
                {"sopas_tcp_port": "2111"},
                {"start_sopas_service": True},
                {"send_sopas_start_stop_cmd": True},
                {"sopas_cola_binary": False},
                {"sopas_timeout_ms": 5000},
                {"client_authorization_pw": "F4724744"},
                {"host_read_filtersettings": True},
                {"host_FREchoFilter": 2},
                {"host_set_FREchoFilter": True},
                {"host_LFPangleRangeFilter": "0 -180.0 +179.0 -90.0 +90.0 1"},
                {"host_set_LFPangleRangeFilter": False},
                {"host_LFPlayerFilter": "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"},
                {"host_set_LFPlayerFilter": False},
                {"msgpack_validator_enabled": False},
                {"ros_qos": -1},
                {"laserscan_layer_filter": "0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0"},
                {"custom_pointclouds": "cloud_360"},
                {
                    "cloud_360": "coordinateNotation=3 updateMethod=0 fields=x,y,z,i,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/multiScan/cloud_360 frameid=multiscan_link publish=1"
                },
                ],
        )
    
    # tim_R_node = Node(
    #         package = 'sick_scan_xd',
    #         executable = 'sick_generic_caller',
    #         name = 'sick_tim_R',
    #         namespace = 'sick_tim_R',
    #         output = 'screen',
    #         parameters = [
    #             {"scanner_type":"sick_tim_5xx"},
    #             {"min_ang":-1.832595715},
    #             {"max_ang":1.832595715},
    #             {"use_binary_protocol":True},
    #             {"range_max":100.0},
    #             {"intensity":True},
    #             {"hostname":"TIM_RIGHT_IP"},
    #             {"cloud_topic":"tim_cloud_R"},
    #             {"frame_id":"tim_link_L"},
    #             {"port":"2112"},
    #             {"timelimit":5},
    #             {"sw_pll_only_publish":True},
    #             {"min_intensity":0.0},
    #         ],
    #         remappings = [('/sick_tim_R/sick_tim_5xx/scan','/tim_scans/tim_scan_R')]
    #     )
    # tim_L_node = Node(
    #         package = 'sick_scan_xd',
    #         executable = 'sick_generic_caller',
    #         name = 'sick_tim_L',
    #         namespace = 'sick_tim_L',
    #         output = 'screen',
    #         parameters = [
    #             {"scanner_type":"sick_tim_5xx"},
    #             {"min_ang":-1.832595715},
    #             {"max_ang":1.832595715},
    #             {"use_binary_protocol":True},
    #             {"range_max":100.0},
    #             {"intensity":True},
    #             {"hostname":"TIM_LEFT_IP"},
    #             {"cloud_topic":"tim_cloud_L"},
    #             {"frame_id":"tim_link_R"},
    #             {"port":"2112"},
    #             {"timelimit":5},
    #             {"sw_pll_only_publish":True},
    #             {"min_intensity":0.0},
    #         ],
    #         remappings = [('/sick_tim_L/sick_tim_5xx/scan','/tim_scans/tim_scan_L')]
    #     )
        
    
    laser_merge_node = launch_ros.actions.Node(
            package = "ros2_laser_scan_merger",
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
            remappings = [('/lidar_1/scan','/tim_scans/tim_scan_L'), ('/lidar_2/scan','/tim_scans/tim_scan_R')],
        )
    
    cloud_merge_node = Node(
            package='cloud_merge',   # パッケージ名
            executable='cloud_merge_node',  # 実行するノードの名前
            name='cloud_merge_node',  # ノード名
            output='screen',          # 標準出力をスクリーンに表示
            parameters=[{
                'use_sim_time': False,  # 必要に応じてシミュレーション時間を使用
                'frame_id': 'base_link'
            }],
            remappings=[
                ('/cloud_in1', '/merged_cloud'),  # トピック名のリマッピング
                ('/cloud_in2', '/multiScan/cloud_360'),
                ('/cloud_out', '/cloud_concatenated'),
            ]
        )


    ld.add_action(multiscan_node)
    ld.add_action(cloud_merge_node)
    # ld.add_action(tim_R_node)
    # ld.add_action(tim_L_node)
    ld.add_action(laser_merge_node)
    
    return ld
