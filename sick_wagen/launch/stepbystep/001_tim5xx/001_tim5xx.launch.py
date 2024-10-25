from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    tim_node = Node(
        package = 'sick_scan_xd',
        executable = 'sick_generic_caller',
        name = 'sick_tim_5xx',
        namespace = 'sick_tim_5xx',
        output = 'screen',
        parameters = [
            {"scanner_type":"sick_tim_5xx"},
            {"nodename":"sick_tim_5xx"},
            {"min_ang":-2.35619449},
            {"max_ang":2.35619449},
            {"use_binary_protocol":True},
            {"range_min":0.0},
            {"range_max":100.0},
            {"range_filter_handling":0},
            {"intensity":True},
            {"hostname":"TIM_RIGHT_IP"},
            {"cloud_topic":"cloud"},
            {"laserscan_topic":"scan"},
            {"frame_id":"cloud"},
            {"port":"2112"},
            {"timelimit":5},
            {"sw_pll_only_publish":True},
            {"use_generation_timestamp":True},
            {"min_intensity":0.0},
            {"add_transform_xyz_rpy":"0,0,0,0,0,0"},
            {"add_transfrom_check_dynamic_updates":False},
            {"start_services":True},
            {"message_monitoring_enabled":True},
            {"read_timeout_millisec_default":5000},
            {"read_timeout_millisec_startup":120000},
            {"read_timeout_millisec_kill_node":150000},
            {"client_authorization_pw":"F4724744"},
            {"ros_qos":-1},
            ],
        )
    
    ld.add_action(tim_node)
    
    return ld