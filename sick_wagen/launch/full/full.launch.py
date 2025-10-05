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
  
  pkg_dir = get_package_share_directory("sick_wagen")  
  robot_state_publisher_node = IncludeLaunchDescription(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    namespace="",
    remappings=[("/joint_states", "/whill/states/joint_state")],
    arguments=[os.path.join(pkg_dir , "urdf", "sick_wagen.urdf")]
  )
      
  sensor_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/full/sensor.py'
      ])
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
  
  # Map server node
  map_server_node = IncludeLaunchDescription(
      package='nav2_map_server',
      executable='map_server',
      name='map_server',
      parameters=[{
          'yaml_filename': '/home/sick/ros2_ws/maps/map_2024-11_white_2.yaml',
          'use_sim_time': False
      }],
      output='screen'
  )

  localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/full/localization.launch.py'
      ])
  )
    # localization_launchを遅延して起動
  delayed_localization_launch = TimerAction(
      period=3.0,
      actions=[localization_launch]
  )

  return LaunchDescription([
      map_server_node,
      robot_state_publisher_node,
      rviz_node,
      sensor_launch,
      delayed_localization_launch,
      nav2_launch,
  ])