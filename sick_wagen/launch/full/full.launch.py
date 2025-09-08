import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/full/nav2.launch.py'
      ]),
      launch_arguments={
          # 'namespace': 'nav2',
          # this makes the bug do not use the namespace
          'use_sim_time': 'false',
          'autostart': 'true',
          'params_file': os.path.join(
              get_package_share_directory('sick_wagen'),
              'config', 'nav2', 'nav2_params.yaml'
          ),
      }.items()
  )
  
  full_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/stepbystep/100_full_test/100_full_test.launch.py'
      ])
  )

  urdf_file = os.path.join(
      get_package_share_directory('sick_wagen'),
      'urdf', 'sick_wagen.urdf'
  )
  
  rviz_config_dir = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config', 'rviz', 'full/full.rviz'
    )
  rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
  
  localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/full/localization.launch.py'
      ])
  )
    # localization_launchを3秒遅延して起動
  delayed_localization_launch = TimerAction(
      period=3.0,
      actions=[localization_launch]
  )

  # Map server to provide static map for nav2
  map_server = Node(
      package='nav2_map_server',
      executable='map_server',
      name='map_server',
      output='screen',
      parameters=[{
          'use_sim_time': False,
          'yaml_filename': os.path.join(
              get_package_share_directory('sick_wagen'),
              'maps', 'garden_small.yaml'  # 適切なマップファイルに変更
          )
      }]
  )

  return LaunchDescription([
      rviz_node,
      delayed_localization_launch,
      # robot_state_publisher,  # 100_full_test.launch.pyで起動するためコメントアウト
      full_launch,
      nav2_launch,
      map_server
  ])