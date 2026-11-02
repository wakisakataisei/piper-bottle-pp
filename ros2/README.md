# ros2/ — B：ROS2 Humble＋MoveIt2＋YOLO

A（`scripts/09_pick_place.py`）と同じ動作を MoveIt で行い、同じ形式で記録して比べる。

| ファイル | 役割 |
|---|---|
| `install_ros2_moveit.sh` | MoveIt2・ros2_control、AgileX 公式 `agx_arm_ros`（コミット 286633e に固定）、`pymoveit2==5.0.0`、YOLO の重み |
| `env.sh` | 端末ごとに `source`（ROS2・agx_arm_ws・venv_pp・pyorbbecsdk） |
| `launch_b.sh` | 実機ドライバ（agx_arm_ctrl）＋MoveIt2＋RViz を起動。**起動すると腕は全関節 0 へ動く** |
| `p1_check.py` | MoveIt の `tcp_link` と A の TCP の一致、MoveIt 完了と実機到達の遅れ |
| `goto.py` | 保存姿勢（look など）へ MoveIt で移動。`deg J1 … J6`（度）で関節角を直接指定、`now` で今の関節角を表示 |
| `save_pose.py` | 今の実機の姿勢を data/poses.json に保存（同じ名前は `<名前>_prev` に退避） |
| `camera_node.py` | カメラを開いたまま点群を配信（MoveIt の Octomap 用）し、`/bottle_vision/locate` も提供。YOLO の枠付き画像を `/bottle_vision/image` に出す（`rqt_image_view` で見る）。vision_node.py と同時に起動しない |
| `setup_octomap.sh` | Octomap の設定（点群プラグイン、格子 2 cm）。1回だけ |
| `vision_node.py` | 認識ノード。サービス `/bottle_vision/locate`（V1 = YOLO＋点群、`use_yolo:=false` で V0） |
| `task_moveit.py` | 往復タスク。`--dry`（計画のみ。RViz で軌道を再生）、`--mode joint|pose`、`--obstacle`（仮想の壁）、`--octomap`（点群の障害物を避ける。camera_node が動いていれば自動で有効）。見つからないときは2回撮り直し → J1 を右へ 5/10/15° 回して探す |
| `rosutil.py` | 共通部品（実機の関節角の受信、到達待ち） |

## 構成

```
task_moveit.py ──Trigger──▶ camera_node.py（YOLO＋点群、/feedback/joint_states から腕の姿勢）
      │ pymoveit2                 │ PointCloud2 2 Hz
      ▼                           ▼
move_group（OMPL / Pilz、計画シーン：床・台・ボトル円柱・仮想の壁＋Octomap）
      │ FollowJointTrajectory
      ▼
ros2_control（mock）── /control/joint_states ──▶ agx_arm_ctrl ── pyAgxArm move_j ──▶ PiPER
                                                      └── /feedback/joint_states（実機の関節角）
```

- URDF の `link6` は SDK のフランジと 0.1 mm 以内で一致（確認済み）。`tcp_offset:=[0,0,0.145,0,0,0]` で `tcp_link` が A の TCP になる
- `auto_control_gate:=false` のため、MoveIt の仮想の腕の関節角が常に実機へ送られる。起動直後は仮想の腕が全関節 0・グリッパ閉なので、実機もそこへ動く
- グリッパも MoveIt の `gripper` グループで動かす（仮想の腕とずれると、腕の動作中にグリッパが閉じるため）
- 実行は venv_pp の Python。ノードは colcon パッケージにしていない（`python ros2/xxx.py` で直接実行）

当日の手順は Claude の Docs「PiPER B（ROS2＋MoveIt＋YOLO）比較試験 手順と記録」を参照。
