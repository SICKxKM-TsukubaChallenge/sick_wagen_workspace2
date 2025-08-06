import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
import launch.actions
import launch.events
import launch_ros.actions
import launch_ros.events
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
import lifecycle_msgs.msg

def generate_launch_description():
    # LaunchDescriptionオブジェクトの作成
    ld = LaunchDescription()

    # --- ekf.launch.pyからのノード ---
    # robot_localizationパッケージのekf_nodeを起動
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("sick_wagen"), 'config', 'localization_params', 'ekf.yaml')],
    )
    ld.add_action(ekf_node)

    # --- navsat_transform.launch.pyからのノード ---
    # robot_localizationパッケージのnavsat_transform_nodeを起動
    #navsat_transform_node = Node(
    #    package='robot_localization',
    #    executable='navsat_transform_node',
    #    name='navsat_transform_node',
    #    output='screen',
    #    parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'navsat_transform.yaml')],
    #)
    #ld.add_action(navsat_transform_node)

    # --- lidar_localization.launch.pyからのノードと設定 ---
    # 静的なTF (base_link -> velodyne)
    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link']
    )
    ld.add_action(lidar_tf)

    # 静的なTF (base_link -> imu_link)
    imu_tf = Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
    )
    ld.add_action(imu_tf)

    # localization.yamlのパス設定
    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(get_package_share_directory('sick_wagen'), 'config', 'localization_params', 'localization.yaml'))

    # lidar_localizationのライフサイクルノード
    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir],
        remappings=[('/cloud', '/multiScan/cloud_360'), ('/imu', '/wit/imu')],
        output='screen')
    ld.add_action(lidar_localization)
    
    # ライフサイクルノードをunconfiguredからinactive状態へ遷移させるイベント発行
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld.add_action(to_inactive)

    # ライフサイクルノードの状態遷移を管理するイベントハンドラ
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    ld.add_action(from_unconfigured_to_inactive)

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    ld.add_action(from_inactive_to_active)

    return ld