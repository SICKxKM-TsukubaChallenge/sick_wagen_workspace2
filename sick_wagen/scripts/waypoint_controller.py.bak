#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml

"""
Basic navigation demo to go to poses.
"""
def load_waypoints_from_yaml(yaml_path):
    """YAMLファイルからウェイポイントを読み込み"""
    waypoints = []
    try:
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            for item in data.get('waypoints', []):
                pose = Pose()
                # older formats may nest under 'pose' key
                pos = item.get('pose', {}).get('position', item.get('position', {}))
                ori = item.get('pose', {}).get('orientation', item.get('orientation', {}))
                pose.position.x = pos.get('x', 0.0)
                pose.position.y = pos.get('y', 0.0)
                pose.position.z = pos.get('z', 0.0)
                pose.orientation.x = ori.get('x', 0.0)
                pose.orientation.y = ori.get('y', 0.0)
                pose.orientation.z = ori.get('z', 0.0)
                pose.orientation.w = ori.get('w', 1.0)
                waypoints.append(pose)
    except Exception as e:
        print(f"Failed to load waypoints from YAML: {e}")
    return waypoints

def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()
    # load waypoint from yaml in arg
    import sys
    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]
        waypoints = load_waypoints_from_yaml(yaml_path)
        print(f"Loaded {len(waypoints)} waypoints from {yaml_path}")
        print(f"Total waypoints: {len(waypoints)}")
    else:
        print("No YAML file provided. Using default waypoints.")

    if not waypoints:
      print("No waypoints loaded. Exiting.")
      return

    # Set first waypoint as initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose = waypoints[0]

    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # # set our demo's goal poses to follow
    # goal_poses = []
    # goal_pose1 = PoseStamped()
    # goal_pose1.header.frame_id = 'map'
    # goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose1.pose.position.x = 10.15
    # goal_pose1.pose.position.y = -0.77
    # goal_pose1.pose.orientation.w = 1.0
    # goal_pose1.pose.orientation.z = 0.0
    # goal_poses.append(goal_pose1)

    # # additional goals can be appended
    # goal_pose2 = PoseStamped()
    # goal_pose2.header.frame_id = 'map'
    # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose2.pose.position.x = 17.86
    # goal_pose2.pose.position.y = -0.77
    # goal_pose2.pose.orientation.w = 1.0
    # goal_pose2.pose.orientation.z = 0.0
    # goal_poses.append(goal_pose2)
    # goal_pose3 = PoseStamped()
    # goal_pose3.header.frame_id = 'map'
    # goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose3.pose.position.x = 21.58
    # goal_pose3.pose.position.y = -3.5
    # goal_pose3.pose.orientation.w = 1.0
    # goal_pose3.pose.orientation.z = 0.0
    # goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    # convert waypoints to PoseStamped list
    goal_poses = []
    for wp in waypoints:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
        pose_stamped.pose = wp
        goal_poses.append(pose_stamped)

    nav_start = navigator.get_clock().now()
    follow_waypoints_task = navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete(task=follow_waypoints_task):
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback(task=follow_waypoints_task)
        # if feedback and i % 5 == 0:
            # print(
            #     'Executing current waypoint: '
            #     + str(feedback.current_waypoint + 1)
            #     + '/'
            #     + str(len(goal_poses))
            # )
            # now = navigator.get_clock().now()

            # # Some navigation timeout to demo cancellation
            # if now - nav_start > Duration(seconds=600.0):
            #     navigator.cancelTask()

            # # Some follow waypoints request change to demo preemption
            # if now - nav_start > Duration(seconds=35.0):
            #     goal_pose4 = PoseStamped()
            #     goal_pose4.header.frame_id = 'map'
            #     goal_pose4.header.stamp = now.to_msg()
            #     goal_pose4.pose.position.x = 0.0
            #     goal_pose4.pose.position.y = 0.0
            #     goal_pose4.pose.orientation.w = 1.0
            #     goal_pose4.pose.orientation.z = 0.0
            #     goal_poses = [goal_pose4]
            #     nav_start = now
            #     follow_waypoints_task = navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print('Goal failed!{error_code}:{error_msg}')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()