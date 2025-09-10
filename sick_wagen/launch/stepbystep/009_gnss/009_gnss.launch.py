import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory('sick_wagen'),
        'config','gnss')
    params = os.path.join(config_directory, 'ublox.yaml')
    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[params],
                                             remappings=[
                                                 ('/ublox_gps_node/fix', '/ublox/fix'),
                                                 ('/ublox_gps_node/fix_velocity', '/ublox/fix_velocity'),
                                                 ('/ublox_gps_node/navpvt', '/ublox/navpvt')
                                             ])

    return launch.LaunchDescription([ublox_gps_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])