# piper-bottle-pp

お題「ペットボトル移動チャレンジ」：AgileX PiPER と手首の RGB-D カメラ（Orbbec Dabai DC1）で、ペットボトルを2つの台（A・B）の間で往復させるピック＆プレイス。
方式 A は Jetson Orin 上の Python（pyAgxArm）だけで制御する。方式 B は同じ動作を ROS2＋MoveIt＋YOLO で行い、A と比べる。

1号機（2026-10-28）で、キャップ把持 → 10 cm 持ち上げ → 反対の台に置く → 置き位置の確認、を A↔B で繰り返し実行できた。
このリポジトリは**別の機体で同じ結果が再現できるか**を確かめるためのもの。

流れの全体は [docs/workflow.html](docs/workflow.html)（PDF: [docs/workflow.pdf](docs/workflow.pdf)）を参照。

## 構成

```
config/robot.json   機体ごとの設定（CAN名、TCP長さ、置き面高さ、把持パラメータ）
piper_pp/           共通モジュール
  robot.py            接続・イネーブル・到達待ち・J6制限
  kinematics.py       把持姿勢の逆運動学（pyAgxArm の fk() を使う平面解）
  camera.py           Orbbec 取得・表示の向き
  vision.py           深度点群 → キャップ位置（学習なし）
  handeye.py          ハンドアイ校正の計算（Park-Martin + 最小二乗）
scripts/            手順の番号順に実行するスクリプト
system/             gs_usb ドライバのビルド、CAN の udev 設定、Python 環境
tests/              実機なしで動く確認（pytest）
data/               機体ごとに測って作るファイル（Git 管理外）
docs/               流れの説明（HTML / PDF）、ArUco マーカー（A4 PDF）
```

## 必要なもの

| 項目 | 1号機の構成 |
|---|---|
| ロボット | AgileX PiPER（HW H-V1.2-1 / FW S-V1.8-2）、標準グリッパ AgxGripper |
| 制御PC | Jetson Orin 16GB、JetPack 6.2.1（L4T R36.4.7、Ubuntu 22.04、カーネル 5.15.148-tegra） |
| CAN | PiPER 付属の USB-CAN（1d50:606f、gs_usb） |
| カメラ | Orbbec Dabai DC1（グリッパ側面に上下逆に取付け、eye-in-hand） |
| 対象 | 500 ml の不透明なボトル（透明ボトルは深度が欠ける）、キャップ直径 30 mm |
| 台 | 同じ高さの台を2つ（1号機は木の台、上面 z = 21 mm） |
| その他 | ノギス、メジャー、ArUco マーカー（docs/aruco_id0_50mm_A4.pdf を等倍印刷） |

## 手順（概要）

```bash
git clone <このリポジトリ> ~/piper-bottle-pp && cd ~/piper-bottle-pp
bash system/build_gs_usb.sh        # 1. USB-CAN ドライバ（JetPack 6 に無いのでビルド）
bash system/setup_can.sh           # 2. can_piper として 1 Mbps で自動 up
bash system/install_python.sh      # 3. venv、pyAgxArm、pyorbbecsdk、piper_pp
pytest -q                          # 4. 実機なしの確認（8 件）

python scripts/00_check.py         # 5. 接続・状態（動かさない）
python scripts/00g_range.py        # 6. J6 の範囲（中心が 0° 付近か）※ずれていたら 00h_calib_j6.py
python scripts/01_first_move.py    # 7. 初回動作
python scripts/02_jog.py           # 8. TCP 長さの照合、look 姿勢の保存（s look）
python scripts/03_cam_check.py > /dev/null      # 9. カメラ
python scripts/05_handeye.py 51.0 > /dev/null   # 10. ハンドアイ校正（02_jog.py と並行）
python scripts/06_locate.py > /dev/null         # 11. ボトル位置の推定と実測照合
python scripts/07_approach.py X Y Z --dry       # 12. 接近（まず --dry）
python scripts/08_pick_test.py X Y Z            # 13. 1 cm 持ち上げ
python scripts/09_pick_place.py --setup A       # 14. 台A・B の登録
python scripts/09_pick_place.py --setup B
python scripts/09_pick_place.py 10              # 15. 往復（慣れたら --auto）
```

カメラを使うスクリプトは SDK のログが標準出力に出るため、結果は標準エラーに出している。`> /dev/null` を付けて実行する。

## B：ROS2＋MoveIt＋YOLO との比較（追加）

A と同じ往復を MoveIt で行い、同じ形式で記録して比べる。詳しくは [ros2/README.md](ros2/README.md)。
コードの流れの図解は [docs/b_code_flow.html](docs/b_code_flow.html)（PDF: [docs/b_code_flow.pdf](docs/b_code_flow.pdf)）。

```bash
bash ros2/install_ros2_moveit.sh                       # MoveIt2・公式ドライバ・pymoveit2・YOLO の重み
python scripts/09_pick_place.py 10 --auto [--yolo]     # A（区間の記録 data/segments_A.csv も出る）
python scripts/10_vision_compare.py 10 --label base    # 認識 V0 と V1 を同じ撮影データで比較
bash ros2/launch_b.sh                                  # B の起動（腕は全関節 0 へ動く）
python ros2/task_moveit.py --dry --ros-args -r joint_states:=/feedback/joint_states
python scripts/11_make_report.py --pdf                 # 記録から比較レポート（docs/report_A_vs_B.html）
```

| 記録（data/） | 内容 |
|---|---|
| `cycles_A.csv` / `cycles_B.csv` | 1サイクル1行（A と B で同じ列） |
| `segments_A.csv` / `segments_B.csv` | 区間ごとの計画時間・到達時間・手先の道のり・最終誤差 |
| `traj_A.csv` / `traj_B.csv` | 動作中の実際の関節角（約 20 Hz） |
| `vision_compare.csv` | 認識 V0 / V1 の検出率・ばらつき・処理時間 |
| `p1_check.json` | B の TCP 一致と遅れ |

### B の実機結果（1号機、2026-10-30）

| 項目 | 結果 |
|---|---|
| TCP の一致（MoveIt の tcp_link と A の TCP） | 0.09 mm / 0.000° |
| MoveIt 完了 → 実機到達の遅れ | 全区間 +0.30〜0.32 s（ros2_control の mock → agx_arm_ctrl の move_j） |
| 計画時間（OMPL RRTConnect、関節目標） | 1区間 20〜75 ms、1サイクル 8 区間で合計 230〜300 ms、すべて 1 回で計画成功 |
| 認識（V1 = YOLO11n＋点群） | 全体 0.6〜0.7 s、うち YOLO 49 ms（初回のみ 1.06 s） |
| 往復 10 サイクル（初期設定、run1） | 10/10 成功、置き位置のずれ 平均 6.3 mm・最大 10.1 mm、1サイクル約 26 s |
| 調整後（run2） | つかむ・置く位置を台ごとに補正。途中の停止（グリッパの計画失敗、見つからず）を直して継続 |

分かったこと
- 台Bは斜めから見るため、キャップの高さが実物より約 26 mm 低く出る（台の実測差は 5 mm）。台ごとの上下補正（`zoff`）で合わせた
- 点群（Octomap）を有効にすると、つかむボトル自体が障害物になる。計画シーンにボトルの円柱（キャップ上端＋40 mm まで）を置いて点群から外す
- 置く高さが低いとボトルを台に押し付けたまま止まり、グリッパを開いた瞬間に腕が下がる
- Jetson の RViz に Image 表示を足すと 3D 表示が真っ暗になる → 画像は rqt_image_view で見る

### 台ごとの補正（data/stations.json）

| キー | 意味 | 単位 |
|---|---|---|
| `adv` | 置く位置を J1 軸から外向きへ | mm |
| `adv_pick` | つかむ位置を J1 軸から外向きへ（無ければ `adv`） | mm |
| `zoff` | `{"pick": 上下, "place": 上下}`、上が正 | m |
| `surface` | 台ごとの置き面の高さ（無ければ `surface_z`） | m |

形式は `data/examples/stations.json`。A（09_pick_place.py）と B（task_moveit.py）で同じ補正を使う。

## 機体ごとに測り直すもの

| 項目 | 方法 | 1号機の値 |
|---|---|---|
| FW の版 | 00_check.py → `config/robot.json` の firmware | S-V1.8-2 → default |
| J6 の原点 | 00g_range.py。中心が大きくずれていたら 00h_calib_j6.py | −288.6° ずれ → 再校正 |
| USB ポート | setup_can.sh が自動で udev に書く | 1-2.2:1.0 |
| TCP 長さ | ノギス実測 + 約 50 mm（SDK のフランジ点は取付け面より手首側） | 0.145 m |
| 置き面の高さ | 定規で実測（ベース上面が z=0） | 0.021 m |
| look 姿勢 | 台A・B が両方写る姿勢。A は 02_jog.py の `s look`、B は `ros2/goto.py deg …` → `ros2/save_pose.py look` | [0, 2.3, −1.3, 0, 16.6, 0]°（ほぼ水平に見る） |
| ハンドアイ | 05_handeye.py（10〜15 姿勢、ばらつき平均 3 mm 以下） | 平均 2.2 / 最大 4.6 mm |
| 台の位置・補正 | 09_pick_place.py `--setup`、`--set-advance`、stations.json の `adv_pick` / `zoff` | 上の「台ごとの補正」 |

## 安全

- 速度は 10%（`config/robot.json` の speed_percent）。非常停止に手が届く位置で操作する
- J6 はカメラ配線保護のため ±90° に制限（j6_limit_deg）
- 動作中に止まった場合、腕はその場で保持される。00d_disable.py で脱力するときは腕を手で支える
- `09_pick_place.py` は既定で各段階の Enter 待ち。`--auto` は動作を確認してから使う

## ライセンスと依存

お題「ペットボトル移動チャレンジ」に取り組む一般向けの内容で、業務の情報は含まない。公開してよい。ライセンスは未設定（使う場合は作者に確認）。

| 依存 | ライセンス | 扱い |
|---|---|---|
| [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) | LGPL-3.0 | 同梱せず pip で入れる |
| [pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk) | Apache-2.0 | 同梱せずソースからビルド |
| gs_usb.c（Linux カーネル） | GPL-2.0 | 同梱せず、ビルドスクリプトのみ |
| numpy / scipy / OpenCV / python-can | BSD / Apache / LGPL | pip |
| Ultralytics YOLO（B の比較・`--yolo`） | AGPL-3.0 | 同梱せず pip。社外配布・公開時は要確認 |
| pymoveit2 | BSD-3-Clause | 同梱せず pip |
| agx_arm_ros | MIT | 同梱せず ros2/install_ros2_moveit.sh で取得 |
