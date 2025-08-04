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
          'namespace': "nav2",
          'use_sim_time': 'true',
          'autostart': 'true',
          'params_file': os.path.join(
              get_package_share_directory('sick_wagen'),
              'config', 'nav2', 'nav2_params.yaml'
          ),
          # 'map': os.path.join(
          #     get_package_share_directory('sick_wagen'),
          #     'maps', 'nakano_map.yaml'
          # )
      }.items()
  )

  return LaunchDescription([
      nav2_launch
  ])