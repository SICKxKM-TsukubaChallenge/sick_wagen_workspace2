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
  
  full_test_launch = IncludeLaunchDescription(
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
    # localization_launchを10秒遅延して起動（TF準備＋手動active化時間）
  delayed_localization_launch = TimerAction(
      period=10.0,
      actions=[localization_launch]
  )

  # Map server for Nav2 (OccupancyGrid format)
  map_server = Node(
      package='nav2_map_server',
      executable='map_server',
      name='map_server',
      output='screen',
      parameters=[{
          'use_sim_time': False,
          'yaml_filename': os.path.join(
              get_package_share_directory('sick_wagen'),
              'maps', 'sick_10f.yaml'
          ),
          'frame_id': 'map'
      }]
  )

  # Map server lifecycle manager
  map_lifecycle_manager = Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_mapper',
      output='screen',
      parameters=[{
          'use_sim_time': False,
          'autostart': True,
          'node_names': ['map_server'],
      }]
  )

  return LaunchDescription([
    #   map_server,  # map_serverを最初に起動
    #   map_lifecycle_manager,
      full_test_launch,  # robot_state_publisherとWHILLノードを先に起動
      rviz_node,
      delayed_localization_launch,  # TF準備後にlocalizationを起動
      # robot_state_publisher,  # 100_full_test.launch.pyで起動するためコメントアウト                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    #   nav2_launch,
  ])