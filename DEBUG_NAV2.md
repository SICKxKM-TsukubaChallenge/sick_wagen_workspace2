# Nav2 Goal Pose デバッグガイド

## 問題
goal poseを設定した時にルートが生成されない

## 修正した問題点

### 1. nav2_params.yamlの修正
- `global_costmap`の`plugins`リストに`static_layer`を追加
- 元: `plugins: ["obstacle_layer", "inflation_layer"]`
- 修正後: `plugins: ["static_layer", "obstacle_layer", "inflation_layer"]`

### 2. full.launch.pyの修正
- 不適切なmap_server設定を削除（PCDファイルではなくYAMLファイルが必要）
- ローカライゼーションが既にマップを提供しているため

## デバッグ手順

### 1. 基本的な確認

```bash
# ROS2環境をセットアップ
source /home/sick/ros2_ws/sick_wagen_workspace2/install/setup.bash

# 1. システムを起動
ros2 launch sick_wagen full.launch.py

# 2. 別ターミナルで診断スクリプトを実行
./sick_wagen/scripts/debug_nav2.sh
```

### 2. 詳細な診断

```bash
# 診断ノードを起動
ros2 run sick_wagen nav2_diagnostic_node.py

# または最小限のnav2システムでテスト
ros2 launch sick_wagen minimal_nav2.launch.py
```

### 3. 手動でのチェック項目

#### A. TF変換の確認
```bash
# TFツリーを生成
ros2 run tf2_tools view_frames.py

# 重要な変換を確認
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

#### B. Nav2サービスの状態確認
```bash
# ライフサイクルノードの状態
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# サービスリスト
ros2 service list | grep -E "(navigate|plan)"
```

#### C. トピックの確認
```bash
# マップが利用可能か
ros2 topic info /map
ros2 topic echo /map --once

# Global costmapが正常か
ros2 topic info /global_costmap/costmap
ros2 topic echo /global_costmap/costmap --once

# Goal poseが正しく受信されるか
ros2 topic echo /goal_pose

# プランが生成されるか
ros2 topic echo /plan
```

#### D. 手動でgoal pose送信
```bash
# RVizからgoal poseを設定するか、コマンドラインから送信
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

## 一般的な問題と解決策

### 1. マップが読み込まれない
- ローカライゼーションが正しく動作しているか確認
- `/map`トピックが存在するか確認

### 2. TF変換が不完全
- `map->odom`変換: ローカライゼーションが提供
- `odom->base_link`: robot_state_publisherまたはオドメトリが提供
- すべての変換が利用可能か確認

### 3. Global costmapが初期化されない
- static_layerがpluginリストに含まれているか確認
- マップトピックが利用可能か確認

### 4. プランナーが応答しない
- プランナーサーバーがアクティブ状態か確認
- Global costmapにvalid cellsがあるか確認
- Start/goalポーズがvalid cellsにあるか確認

## ログレベルの調整

より詳細なデバッグ情報が必要な場合：

```bash
# プランナーのデバッグログを有効にして起動
ros2 launch sick_wagen full.launch.py --ros-args --log-level planner_server:=DEBUG
```

## 追加のツール

### RVizでの可視化
- Global costmap表示
- Local costmap表示  
- プランパス表示
- TF表示
- ロボット位置表示

### rqt tools
```bash
# ノードグラフ
rqt_graph

# トピック監視
rqt_topic

# サービス呼び出し
rqt_service_caller
```
