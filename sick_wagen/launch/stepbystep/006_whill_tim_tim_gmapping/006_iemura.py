import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration, NotSubstitution)
from launch_ros.actions import (LifecycleNode, Node)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition

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
        '004.rviz'
    )
    
    # ↓Absolute path
    # config = "/home/sick/ros2_ws/sick_tsukuba_ws/sick_wagen/config/laser_scan_merger/params.yaml"
    # print(config)
    # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    
    # pkg_share = FindPackageShare('sick_wagen')
    # config = PathJoinSubstitution([pkg_share, 'config','laser_scan_merger','params.yaml'])
    # print(config)
    # print("----------------------------------------")
    
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
        parameters=[os.path.join(pkg_dir, "config/whill", "whill_param.yaml")],
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
        
    laserscan_merger_node = launch_ros.actions.Node(
        package = "ros2_laser_scan_merger",
        executable='ros2_laser_scan_merger',
        parameters=[config],
        output='screen',
        respawn=True,
        respawn_delay=2,
        remappings = [('/lidar_1/scan','/sick_tim_L/sick_tim_5xx/scan'), ('/lidar_2/scan','/sick_tim_R/sick_tim_5xx/scan')],
    )
        
    pointcould_to_laserscan_node = Node(
        name='pointcloud_to_laserscan',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[config]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
        
    #####################################################################
    ################## slam_toolbox #####################
    #####################################################################
    # slam_config_file = LaunchConfiguration('slam_config_file')
    
    # declare_arg_slam_config_file = DeclareLaunchArgument(
    #     'slam_config_file',
    #     default_value=os.path.join(
    #         get_package_share_directory('sick_wagen'),
    #         'config',
    #         'mapper_params_offline.yaml'),
    #     description='The full path to the config file for SLAM'
    # )
        
    # slam_node = Node(
    #     package='slam_toolbox', executable='sync_slam_toolbox_node',
    #     output='screen',
    #     parameters=[slam_config_file],
    # )
    #####################################################################
    #####################################################################
    
    ###slam_toolbox(sick_wagen_workspace2/slam_toolbox/launch/offline_launch.pyを参考)###
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("sick_wagen"),
                                   'config', 'slam_tools','mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )
    #####################################
    
    ld.add_action(tim_R_node)
    ld.add_action(tim_L_node)
    ld.add_action(laserscan_merger_node)
    ld.add_action(pointcould_to_laserscan_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(whill_node)
    
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    # ld.add_action(declare_arg_slam_config_file)
    # ld.add_action(slam_node)
    
    ld.add_action(rviz2_node)

    
    return ld
