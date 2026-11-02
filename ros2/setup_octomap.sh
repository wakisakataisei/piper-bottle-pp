#!/usr/bin/env bash
# MoveIt に深度の点群（Octomap）を取り込む設定。1回だけ実行
# - MoveIt の点群プラグイン（moveit_ros_perception）を入れる
# - agx_arm_moveit の sensors_3d.yaml を、このカメラ用に書き換える（元はバックアップ）
#   格子 2 cm、範囲 1 m、ロボット自身を除く余白 3 cm、更新 2 Hz、座標は base_link
set -eo pipefail
WS=${WS:-$HOME/agx_arm_ws}
CFG_FILE="$WS/src/agx_arm_ros/src/agx_arm_moveit/config/sensors_3d.yaml"
sudo apt install -y ros-humble-moveit-ros-perception
[ -f "$CFG_FILE.orig" ] || cp "$CFG_FILE" "$CFG_FILE.orig"
cat > "$CFG_FILE" <<'YAML'
# piper-bottle-pp: Dabai DC1（手首）の点群を camera_node.py が配信
octomap_frame: base_link
octomap_resolution: 0.02
max_range: 1.0
sensors:
  - point_cloud_sensor

point_cloud_sensor:
  sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
  point_cloud_topic: /camera/depth/color/points
  max_range: 1.0
  point_subsample: 1
  padding_offset: 0.03
  padding_scale: 1.0
  max_update_rate: 2.0
  filtered_cloud_topic: filtered_cloud
YAML
source /opt/ros/humble/setup.bash
cd "$WS" && colcon build --symlink-install --packages-select agx_arm_moveit
echo "完了。launch_b.sh を起動し直したあと: ros2 param get /move_group octomap_resolution → 0.02 なら反映"
echo "元に戻すとき: cp $CFG_FILE.orig $CFG_FILE"
