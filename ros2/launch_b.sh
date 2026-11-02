#!/usr/bin/env bash
# agx_arm_ctrl（実機ドライバ）＋ MoveIt2 ＋ RViz を起動する
# 注意: 起動すると腕は全関節 0（折りたたみ）へ動き、グリッパは閉じる（MoveIt 内の仮想の腕の初期値に合わせるため）。
#       先に A の 02_jog.py で h（全関節0）にしておくと動きが出ない。
source "$(dirname "$0")/env.sh"
SPEED=${SPEED:-30}
echo "speed_percent=${SPEED}（実機の追従速度の上限）。腕は全関節 0 へ動きます。非常停止に手を添えて Enter（Ctrl+C で中止）"
read -r _
exec ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can_piper arm_type:=piper effector_type:=agx_gripper \
  tcp_offset:='[0.0, 0.0, 0.145, 0.0, 0.0, 0.0]' \
  speed_percent:=${SPEED} gripper_default_effort:=2.0 \
  follow:=true auto_control_gate:=false
