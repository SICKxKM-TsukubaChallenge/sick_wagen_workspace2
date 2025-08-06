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
        parameters=[os.path.join(get_package_share_directory("sick_wagen"), 'config', 'localization_param', 'ekf.yaml')],
    )
    ld.add_action(ekf_node)

    # localization.yamlのパス設定
    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(get_package_share_directory('sick_wagen'), 'config', 'localization_param', 'localization.yaml'))

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