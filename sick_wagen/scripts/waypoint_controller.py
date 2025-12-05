#!/usr/bin/env python3
import argparse
import os
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
import yaml

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from sensor_msgs.msg import Joy

DEFAULT_YAML_PATH = os.path.join(os.path.dirname(__file__), '../data/test2.yaml')

@dataclass
class Waypoint:
    pose: Pose
    action: int = 1  # 1: auto-continue, 0: wait for button
    frame_id: str = 'map'
    name: str = ''


def load_waypoints_from_yaml(yaml_path: str) -> List[Waypoint]:
    """Load Pose + action pairs from a Nav2-style waypoint YAML file."""
    waypoints: List[Waypoint] = []
    try:
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file) or {}
    except FileNotFoundError:
        print(f"YAML file not found: {yaml_path}")
        return waypoints
    except Exception as exc:  # noqa: BLE001
        print(f"Failed to load waypoints from YAML: {exc}")
        return waypoints

    for idx, item in enumerate(data.get('waypoints', [])):
        pose = Pose()
        pose_data = item.get('pose', item)
        pos = pose_data.get('position', {})
        ori = pose_data.get('orientation', {})
        pose.position.x = float(pos.get('x', 0.0))
        pose.position.y = float(pos.get('y', 0.0))
        pose.position.z = float(pos.get('z', 0.0))
        pose.orientation.x = float(ori.get('x', 0.0))
        pose.orientation.y = float(ori.get('y', 0.0))
        pose.orientation.z = float(ori.get('z', 0.0))
        pose.orientation.w = float(ori.get('w', 1.0))

        raw_action = item.get('action', 1)
        try:
            action_val = int(raw_action)
        except (TypeError, ValueError):
            action_val = 1
        action = 0 if action_val == 0 else 1

        frame_id = item.get('frame_id', 'map')
        name = item.get('name', f'waypoint_{idx + 1}')
        waypoints.append(Waypoint(pose=pose, action=action, frame_id=frame_id, name=name))

    return waypoints


class ButtonWaiter(Node):
    """Joy listener that detects a rising edge after reset and a skip button."""

    def __init__(self, topic: str, button_indices: List[int], skip_button_index: int = 9) -> None:
        super().__init__('waypoint_button_waiter')
        self._topic = topic
        self._button_indices = button_indices
        self._skip_button_index = skip_button_index
        self._waiting = False
        self._pressed_after_reset = False
        self._armed_for_rise = {idx: False for idx in self._button_indices}
        self._last_raw = {idx: 0 for idx in self._button_indices}
        self._skip_requested = False
        self._skip_last_raw = 0
        self._skip_armed = False
        self.create_subscription(Joy, topic, self._joy_callback, 10)

    def _extract_button(self, msg: Joy, idx: int) -> int:
        if len(msg.buttons) > idx:
            return msg.buttons[idx]
        return 0

    def _joy_callback(self, msg: Joy) -> None:
        # Detect skip on rising edge of the configured button.
        skip_raw = self._extract_button(msg, self._skip_button_index)
        if skip_raw == 0:
            self._skip_armed = True
        if self._skip_armed and skip_raw == 1 and self._skip_last_raw == 0:
            self._skip_requested = True
            self._skip_armed = False
        self._skip_last_raw = skip_raw

        if not self._waiting:
            for idx in self._button_indices:
                self._last_raw[idx] = self._extract_button(msg, idx)
            return

        for idx in self._button_indices:
            raw = self._extract_button(msg, idx)
            if raw == 0:
                self._armed_for_rise[idx] = True
            if self._armed_for_rise[idx] and raw == 1 and self._last_raw[idx] == 0:
                self._pressed_after_reset = True
                self._waiting = False
                break
            self._last_raw[idx] = raw

    def reset_after_arrival(self) -> None:
        self._waiting = True
        self._pressed_after_reset = False
        self._armed_for_rise = {idx: False for idx in self._button_indices}
        self._last_raw = {idx: 0 for idx in self._button_indices}

    @property
    def pressed_since_reset(self) -> bool:
        return self._pressed_after_reset

    def consume_skip_request(self) -> bool:
        """Return True once per rising edge of skip button."""
        if self._skip_requested:
            self._skip_requested = False
            return True
        return False

    def wait_for_press(self, timeout_sec: float = 0.1) -> None:
        while rclpy.ok() and not self._pressed_after_reset:
            rclpy.spin_once(self, timeout_sec=timeout_sec)
            if self.consume_skip_request():
                # Treat skip as an immediate continue while waiting so it does not bleed into next leg.
                self._pressed_after_reset = True
                self._waiting = False

    def stop_waiting(self) -> None:
        self._waiting = False


def main() -> None:
    parser = argparse.ArgumentParser(description='Waypoint navigator with per-action button gating.')
    parser.add_argument('yaml_path', nargs='?', help='Path to waypoint YAML file')
    parser.add_argument(
        '--yaml',
        dest='yaml_override',
        help='Path to waypoint YAML file (same as positional; use this in launch files, etc.)',
    )
    parser.add_argument('--button-topic', default='/joy', help='Joy topic for continue/pause buttons (default: /joy)')
    parser.add_argument(
        '--button-indices',
        nargs='+',
        type=int,
        default=[1, 2],
        help='Button indices that are accepted for continue (default: 1 2, same as waypoint saver)',
    )
    parser.add_argument(
        '--skip-button-index',
        type=int,
        default=9,
        help='Button index used to skip the current waypoint (default: 9 / START)',
    )
    parser.add_argument(
        '--close-distance',
        type=float,
        default=0.5,
        help='Consider a goal reached if distance stays under this value (m) for close-hold time.',
    )
    parser.add_argument(
        '--close-hold-time',
        type=float,
        default=2.0,
        help='Seconds distance must stay under close-distance before forcing completion.',
    )
    args = parser.parse_args()

    yaml_path = args.yaml_override or args.yaml_path or DEFAULT_YAML_PATH
    if not yaml_path:
        print('No YAML file provided. Set DEFAULT_YAML_PATH in this file or use positional arg / --yaml to set the waypoint file.')
        return

    rclpy.init()
    navigator = BasicNavigator()
    button_waiter = ButtonWaiter(
        topic=args.button_topic,
        button_indices=args.button_indices,
        skip_button_index=args.skip_button_index,
    )

    waypoints = load_waypoints_from_yaml(yaml_path)
    if not waypoints:
        print('No waypoints loaded. Exiting.')
        button_waiter.destroy_node()
        rclpy.shutdown()
        return

    # 自己位置は既に推定済み前提なので初期ポーズは上書きしない。
    navigator.waitUntilNav2Active(localizer='lidar_localization')

    total = len(waypoints)
    for idx, waypoint in enumerate(waypoints):
        attempts = 0
        while rclpy.ok():
            goal = PoseStamped()
            goal.header.frame_id = waypoint.frame_id
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose = waypoint.pose

            navigator.goToPose(goal)
            mode = 'wait-button' if waypoint.action == 0 else 'auto'
            print(f"Navigating to waypoint {idx + 1}/{total} ({mode}), attempt {attempts + 1}")

            skipped = False
            forced_success = False
            close_enter_time: Optional[float] = None
            while not navigator.isTaskComplete():
                rclpy.spin_once(button_waiter, timeout_sec=0.1)
                if button_waiter.consume_skip_request():
                    print(f"Skip requested at waypoint {idx + 1}/{total}. Canceling and moving to next.")
                    navigator.cancelTask()
                    skipped = True
                    break
                feedback = navigator.getFeedback()
                if feedback and feedback.distance_remaining is not None:
                    distance_remaining = float(feedback.distance_remaining)
                    if distance_remaining < args.close_distance:
                        if close_enter_time is None:
                            close_enter_time = time.time()
                        elif time.time() - close_enter_time >= args.close_hold_time:
                            print(
                                f"Distance under {args.close_distance} m for {args.close_hold_time:.1f}s."
                                " Forcing completion to avoid circling."
                            )
                            navigator.cancelTask()
                            forced_success = True
                            break
                    else:
                        close_enter_time = None

            if skipped:
                while not navigator.isTaskComplete():
                    rclpy.spin_once(button_waiter, timeout_sec=0.1)
                break

            if forced_success:
                while not navigator.isTaskComplete():
                    rclpy.spin_once(button_waiter, timeout_sec=0.1)
                result = TaskResult.SUCCEEDED
            else:
                result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                print(f"Reached waypoint {idx + 1}/{total}")
                if waypoint.action == 1:
                    button_waiter.reset_after_arrival()
                    print(
                        f"Waiting for buttons {args.button_indices} on {args.button_topic} before continuing..."
                    )
                    button_waiter.wait_for_press()
                    button_waiter.stop_waiting()
                    print('Button press detected. Proceeding to next waypoint.')
                break

            attempts += 1
            try:
                error_code, error_msg = navigator.getTaskError()
                print(f"Navigation result: {result}. Error detail: {error_code}: {error_msg}")
            except Exception:
                print(f"Navigation result: {result}. No additional error detail.")

            print(f"Retrying waypoint {idx + 1}/{total} (attempt {attempts + 1}) after failure/cancel.")
            time.sleep(1.0)

    navigator.lifecycleShutdown()
    button_waiter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
