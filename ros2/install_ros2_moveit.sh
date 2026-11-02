#!/usr/bin/env bash
# B（ROS2 Humble＋MoveIt2＋AgileX 公式ドライバ）を Jetson に入れる。所要 20〜40 分（ビルド含む）
# 使い方: bash ros2/install_ros2_moveit.sh
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)     # 途中で cd するので最初に絶対パスで覚える
AGX_COMMIT=${AGX_COMMIT:-286633e}        # 2026-10-21 の agx_arm_ros（動作を確認する版に固定）
WS=${WS:-$HOME/agx_arm_ws}
source /opt/ros/humble/setup.bash
set -u

echo "== 1) apt: MoveIt2・ros2_control・コントローラ"
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-moveit-ros-perception ros-humble-moveit-planners-chomp ros-humble-pilz-industrial-motion-planner \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller ros-humble-gripper-controllers \
  ros-humble-topic-tools ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro \
  python3-colcon-common-extensions python3-pip can-utils

echo "== 2) agx_arm_ctrl 用: システムの Python に pyAgxArm（ROS2 ノードは /usr/bin/python3 で動く）"
/usr/bin/python3 -m pip install --user "python-can>3.3.4" numpy scipy
/usr/bin/python3 -m pip install --user "git+https://github.com/agilexrobotics/pyAgxArm.git"

echo "== 3) agx_arm_ros（URDF・MoveIt 設定・ドライバ）を取得して版を固定、ビルド"
mkdir -p "$WS/src"
cd "$WS/src"
[ -d agx_arm_ros ] || git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
cd agx_arm_ros
git fetch --all -q || true
git checkout -q "$AGX_COMMIT"
git submodule update --init --recursive
cd "$WS"
colcon build --symlink-install
grep -q "LC_NUMERIC=en_US.UTF-8" ~/.bashrc || echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc

echo "== 4) venv_pp: pymoveit2（Python から MoveIt を使う）"
"$HOME/venv_pp/bin/pip" install "pymoveit2==5.0.0"

echo "== 5) YOLO の重みを先にダウンロード（リポジトリ直下に yolo11n.pt）"
cd "$REPO"
"$HOME/venv_pp/bin/python" -c "from ultralytics import YOLO; YOLO('yolo11n.pt'); print('YOLO OK')"

echo "== 確認"
source "$WS/install/setup.bash"
ros2 pkg list | grep -E "agx_arm_(ctrl|moveit|description)" || { echo "agx_arm_ros のビルドを確認"; exit 1; }
"$HOME/venv_pp/bin/python" -c "import pymoveit2, ultralytics; print('pymoveit2', pymoveit2.__file__); print('ultralytics', ultralytics.__version__)"
echo "完了。新しい端末で: source ~/piper-bottle-pp/ros2/env.sh"
