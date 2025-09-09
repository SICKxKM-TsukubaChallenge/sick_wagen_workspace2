#!/bin/bash

echo "=== Nav2 デバッグ情報 ==="
echo

echo "1. アクティブなノードを確認:"
ros2 node list | grep -E "(planner|navigator|costmap|lifecycle)"
echo

echo "2. nav2のサービスを確認:"
ros2 service list | grep -E "(navigate|plan|costmap)"
echo

echo "3. トピックを確認:"
ros2 topic list | grep -E "(goal|plan|path|costmap|map)"
echo

echo "4. TFフレームを確認:"
ros2 run tf2_tools view_frames.py
echo

echo "5. プランナーサーバーの状態を確認:"
ros2 lifecycle get /planner_server
echo

echo "6. BT Navigatorの状態を確認:"
ros2 lifecycle get /bt_navigator
echo

echo "7. Global costmapの情報を確認:"
ros2 topic info /global_costmap/costmap
echo

echo "8. マップが利用可能かを確認:"
ros2 topic info /map
echo

echo "9. ローカライゼーションの状態を確認:"
ros2 topic echo /amcl_pose --once
echo

echo "10. Goal poseトピックを監視 (5秒間):"
timeout 5s ros2 topic echo /goal_pose
echo

echo "=== デバッグ完了 ==="
