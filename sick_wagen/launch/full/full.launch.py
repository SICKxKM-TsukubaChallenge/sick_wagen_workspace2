import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

  localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/full/localization.launch.py'
      ])
  )
  
  full_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          get_package_share_directory('sick_wagen'),
          '/launch/stepbystep/100_full_test/100_full_test.launch.py'
      ])
  ) 
  rviz_config_dir = os.path.join(get_package_share_directory('sick_wagen'), 'config/rviz/full/full.rviz')
  rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

  urdf_file = os.path.join(
      get_package_share_directory('sick_wagen'),
      'urdf', 'sick_wagen.urdf'
  )
  robot_description = {'robot_description': open(urdf_file).read()}
  robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[
          robot_description,
          {'use_sim_time': False}
      ],
      remappings=[
          ('/tf', '/tf'),
          ('/tf_static', '/tf_static')
      ]
  )


  return LaunchDescription([
    #   nav2_launch,
      localization_launch,
      full_launch,
      rviz_node,
      robot_state_publisher
  ])