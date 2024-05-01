from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    sick_tim_R_node = Node(
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
    )
    
    sick_tim_L_node = Node(
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
    )
    
    static_tf_node1 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        output = 'screen',
        arguments = [
            "0", "0", "0", "0", "0", "0", "world", "tim_link_L",
        ]
    )
    
    static_tf_node2 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        output = 'screen',
        arguments = [
            "0", "0", "0", "0", "0", "0", "world", "tim_link_R",
        ]
    )

    
    ld.add_action(sick_tim_L_node)
    ld.add_action(sick_tim_R_node)
    ld.add_action(static_tf_node1)
    ld.add_action(static_tf_node2)
    return ld